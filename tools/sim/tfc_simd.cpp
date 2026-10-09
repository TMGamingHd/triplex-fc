// SPDX-License-Identifier: MIT
// The vehicle simulator on a SocketCAN bus (docs/design/VEHICLE_SIM.md, docs/design/PROTOCOL.md). It follows SYNC, publishes the sensor inputs for the next frame as soon as ACT's
// output for this one arrives (sim/vehicle/runner.hpp explains why one frame ahead), and steps the 6-DOF vehicle with the gimbal command in ACT's frame. Its time is the
// frame number: a SYNC that skips numbers (a sync-master takeover) steps the world over the skipped frames with the last command held, so sim time never drifts from the
// flight computers' frame count. A run starts at the first SYNC heard and starts over if the frame number goes backwards (the sync master was reset).
// With --pico PORT it also streams the vehicle's tilts to the Pico that drives the platform (docs/design/PICO.md): one platform frame per simulated frame, so 100 Hz.
// With --hold the vehicle stands clamped on the pad until the mission frame in SYNC passes T-zero (docs/design/LAUNCH_SEQUENCE.md), then flies; a simulator that starts after T-zero joins the flight in progress.
//   tfc_simd [--iface vcan0] [--vehicle FILE] [--hold] [--pico PORT] [--telemetry PORT] [--vehicle-true] [--wind-scale X] [--gust T,DUR,PEAK_MS] [--engine-out T[,N]] [--cg-shift M] [--frames N] [--quiet]
// With --telemetry PORT it also sends the vehicle's true state, 10 times a second, as one JSON object per UDP datagram to 127.0.0.1:PORT (the flight console, docs/design/CONSOLE.md): what the bus does not
// carry (the distance downrange, the stages, the thrust, the program the flight computers follow). It is an output only: nothing the flight computers do depends on it, and a datagram nobody reads is dropped.
// --vehicle flies the vehicle described in FILE (docs/design/VEHICLE_SPEC.md) instead of the reference vehicle; give it before the options that adjust the scenario. The flight computers carry the tables
// they were built with, which are the reference vehicle's: for another vehicle regenerate them (tfc_gen_tables --vehicle FILE firmware/app/src/flight_tables.hpp) and rebuild the firmware.
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <algorithm>
#include <array>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "design.hpp"
#include "runner.hpp"
#include "spec_io.hpp"
#include "tfc/pico_link.hpp"
#include "tfc/protocol.hpp"

namespace {

// A SYNC frame number beyond this (five minutes of frames, past the burn-out of the vehicle) is not a frame number of this run: it is ignored, so a corrupt or foreign SYNC cannot make the world step for hours.
constexpr uint32_t kMaxFrame = 30000U;
// A jump forward of more than this many frames (a minute) is a new run rather than a gap to step over.
constexpr uint32_t kMaxGap = 6000U;

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

int open_can(const std::string& iface) {
  const int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    std::perror("socket");
    return -1;
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    std::fprintf(stderr, "no CAN interface %s (sim/scripts/setup_vcan.sh)\n", iface.c_str());
    ::close(s);
    return -1;
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
    std::perror("bind");
    ::close(s);
    return -1;
  }
  can_filter filters[2] = {{tfc::id::kSync, CAN_SFF_MASK}, {tfc::id::kActOut, CAN_SFF_MASK}};
  ::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof filters);
  return s;
}

bool send_frame(int s, const tfc::Frame& f) {
  can_frame cf{};
  cf.can_id = f.id;
  cf.can_dlc = 8;
  std::memcpy(cf.data, f.data.data(), 8);
  return ::write(s, &cf, sizeof cf) == static_cast<ssize_t>(sizeof cf);
}

void send_all(int s, const sim::SimFrames& fr, unsigned& errors) {
  for (unsigned i = 0; i < fr.n; ++i) {
    if (!send_frame(s, fr.f[i])) {
      ++errors;
    }
  }
}

// The Pico's USB serial port, raw and non-blocking: a frame that cannot be written now is dropped (the Pico holds and levels by itself if the stream stops).
int open_pico(const std::string& path) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    std::perror(path.c_str());
    return -1;
  }
  termios tio{};
  if (::tcgetattr(fd, &tio) == 0) {
    ::cfmakeraw(&tio);
    ::tcsetattr(fd, TCSANOW, &tio);
  }
  return fd;
}

void send_platform(int fd, uint8_t seq, const sim::Tilts& t, unsigned& drops) {
  tfc::pico::PlatformCommand c;
  c.seq = seq;
  c.tilt_x_deg = static_cast<float>(t.x_deg);
  c.tilt_y_deg = static_cast<float>(t.y_deg);
  std::array<uint8_t, tfc::pico::kMaxFrame> b{};
  const std::size_t n = tfc::pico::encode(tfc::pico::pack_platform(c), b.data());
  if (::write(fd, b.data(), n) != static_cast<ssize_t>(n)) {
    ++drops;
  }
}

// The console's telemetry: a UDP socket to 127.0.0.1:port, and one JSON object per datagram. The state is the simulator's own (the truth), not what a sensor read.
struct Telemetry {
  int fd = -1;
  sockaddr_in to{};
};

bool open_telemetry(Telemetry& t, int port) {
  if (port <= 0 || port > 65535) {
    std::fprintf(stderr, "--telemetry: %d is not a port\n", port);
    return false;
  }
  t.fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (t.fd < 0) {
    std::perror("socket");
    return false;
  }
  t.to.sin_family = AF_INET;
  t.to.sin_port = htons(static_cast<uint16_t>(port));
  t.to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  return true;
}

void send_telemetry(const Telemetry& t, const sim::SimRunner& runner) {
  const sim::Vehicle6& v = runner.vehicle();
  const sim::Loads l = v.current_loads();
  const sim::Tilts tl = v.tilts();
  const tfc::Reference ref = runner.tables().guidance.at(runner.flight_frame());
  const std::size_t stages = std::min<std::size_t>(v.spec().stages.size(), sim::kMaxStages);
  unsigned active = 0U;
  unsigned ignited = 0U;
  for (std::size_t s = 0; s < stages; ++s) {
    active |= v.stage_active(s) ? (1U << s) : 0U;
    ignited |= v.stage_ignited(s) ? (1U << s) : 0U;
  }
  std::array<char, 640> buf{};
  int n = std::snprintf(buf.data(), buf.size(),
                        "{\"frame\":%u,\"ft\":%.2f,\"clamped\":%d,\"alt\":%.2f,\"range\":%.1f,\"speed\":%.2f,\"mach\":%.3f,\"q\":%.1f,\"mass\":%.1f,\"thrust\":%.0f,"
                        "\"tilt_p\":%.4f,\"tilt_y\":%.4f,\"ref_p\":%.4f,\"ref_y\":%.4f,\"gim_p\":%.3f,\"gim_y\":%.3f,\"stages_active\":%u,\"stages_ignited\":%u,"
                        "\"engines_on\":%d,\"engines\":%d,\"crashed\":%d,\"prop\":[",
                        static_cast<unsigned>(runner.frame()), static_cast<double>(runner.flight_frame()) * 0.01, runner.clamped() ? 1 : 0, v.altitude(), v.range(), v.speed(), l.mach,
                        l.dynamic_pressure, v.mass(), l.thrust, tl.y_deg, tl.x_deg, static_cast<double>(ref.tilt_y_deg), static_cast<double>(ref.tilt_x_deg), v.gimbal_pitch_deg(),
                        v.gimbal_yaw_deg(), active, ignited, v.engines_on(), v.engine_count(), v.crashed() ? 1 : 0);
  for (std::size_t s = 0; s < stages && n > 0 && static_cast<std::size_t>(n) + 24U < buf.size(); ++s) {
    n += std::snprintf(buf.data() + n, buf.size() - static_cast<std::size_t>(n), "%s%.1f", s == 0U ? "" : ",", v.propellant(s));
  }
  if (n > 0 && static_cast<std::size_t>(n) + 4U < buf.size()) {
    n += std::snprintf(buf.data() + n, buf.size() - static_cast<std::size_t>(n), "]}");
    (void)::sendto(t.fd, buf.data(), static_cast<std::size_t>(n), MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&t.to), sizeof t.to);
  }
}

bool split(const std::string& arg, double* out, int n) {
  std::size_t pos = 0;
  for (int i = 0; i < n; ++i) {
    const std::size_t comma = arg.find(',', pos);
    const std::string part = arg.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
    if (part.empty()) {
      return i >= 1;  // the later ones are optional
    }
    out[i] = std::atof(part.c_str());
    if (comma == std::string::npos) {
      return true;
    }
    pos = comma + 1U;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::string iface = "vcan0";
  sim::RunnerConfig cfg;
  uint32_t max_frames = 0U;
  std::string pico_port;
  int telemetry_port = 0;
  bool quiet = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s needs a value\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    double v[3] = {0.0, 0.0, 0.0};
    if (a == "--iface") {
      iface = next();
    } else if (a == "--vehicle") {
      sim::VehicleFile vf;
      std::vector<std::string> errors;
      if (!sim::load_vehicle_file(next(), vf, errors)) {
        for (const std::string& e : errors) {
          std::fprintf(stderr, "%s\n", e.c_str());
        }
        return 1;
      }
      cfg.params = vf.params;
      cfg.design = vf.params;
      cfg.scenario = vf.scenario;
      cfg.plan = vf.plan;
      std::fprintf(stderr, "tfc_simd: flying \"%s\"; the flight computers must carry the tables of this vehicle (tfc_gen_tables --vehicle)\n", vf.name.c_str());
    } else if (a == "--hold") {
      cfg.start_held = true;
    } else if (a == "--pico") {
      pico_port = next();
    } else if (a == "--telemetry") {
      telemetry_port = std::atoi(next().c_str());
    } else if (a == "--vehicle-true") {
      cfg.vehicle_true = true;
    } else if (a == "--wind-scale") {
      cfg.scenario.wind_scale = std::atof(next().c_str());
    } else if (a == "--gust" && split(next(), v, 3)) {
      sim::Gust g;
      g.t0 = v[0];
      g.duration = v[1];
      g.peak = sim::V3{0.0, 0.0, v[2]};
      cfg.scenario.gusts.push_back(g);
    } else if (a == "--engine-out" && split(next(), v, 2)) {
      cfg.scenario.engine_out_time = v[0];
      cfg.scenario.engine_out_index = v[1] > 0.0 ? static_cast<int>(v[1]) : 1;
    } else if (a == "--cg-shift") {
      cfg.scenario.dry_cg_shift = std::atof(next().c_str());
    } else if (a == "--frames") {
      max_frames = static_cast<uint32_t>(std::strtoul(next().c_str(), nullptr, 10));
    } else if (a == "--quiet") {
      quiet = true;
    } else {
      std::fprintf(stderr, "usage: tfc_simd [--iface vcan0] [--vehicle FILE] [--hold] [--pico PORT] [--telemetry PORT] [--vehicle-true] [--wind-scale X] [--gust T,DUR,PEAK_MS] [--engine-out T[,N]] [--cg-shift M] [--frames N] [--quiet]\n");
      return 2;
    }
  }
  const int sock = open_can(iface);
  if (sock < 0) {
    return 1;
  }
  int pico = -1;
  if (!pico_port.empty()) {
    pico = open_pico(pico_port);
    if (pico < 0) {
      return 1;
    }
  }
  Telemetry telemetry;
  if (telemetry_port != 0 && !open_telemetry(telemetry, telemetry_port)) {
    return 1;
  }
  unsigned pico_drops = 0U;
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  sim::SimRunner runner(cfg);
  bool started = false;
  uint32_t acted = 0xFFFFFFFFU;  // the frame whose ACT output has been applied
  unsigned tx_errors = 0U;
  uint32_t first_frame = 0U;
  uint8_t last_act_state = 0U;
  std::fprintf(stderr, "tfc_simd on %s: waiting for SYNC\n", iface.c_str());

  while (g_stop == 0) {
    pollfd pfd{sock, POLLIN, 0};
    if (::poll(&pfd, 1, 200) <= 0) {
      continue;
    }
    can_frame cf{};
    if (::read(sock, &cf, sizeof cf) != static_cast<ssize_t>(sizeof cf)) {
      continue;
    }
    tfc::Frame f;
    f.id = cf.can_id & CAN_SFF_MASK;
    f.len = cf.can_dlc;
    std::memcpy(f.data.data(), cf.data, 8);
    if (f.id == tfc::id::kSync) {
      const tfc::DecodedSync d = tfc::unpack_sync(f);
      if (!d.ok) {
        continue;
      }
      const uint32_t k = d.frame_no;
      if (k > kMaxFrame) {
        if (!quiet) {
          std::fprintf(stderr, "ignored a SYNC with frame number %u\n", static_cast<unsigned>(k));
        }
        continue;
      }
      if (!started || k < runner.frame() || k - runner.frame() > kMaxGap) {  // the first SYNC, or the frame number went backwards: a new run
        if (started && !quiet) {
          std::fprintf(stderr, "frame number went back (%u to %u): a new run\n", static_cast<unsigned>(runner.frame()), static_cast<unsigned>(k));
        }
        if (cfg.start_held && tfc::mission::in_flight(d.mission)) {
          send_all(sock, runner.start_in_flight(k, tfc::mission::flight_frames(d.mission)), tx_errors);  // joining after T-zero
        } else {
          send_all(sock, runner.start(k), tx_errors);
        }
        started = true;
        first_frame = k;
        acted = 0xFFFFFFFFU;
        continue;
      }
      if (runner.clamped() && tfc::mission::in_flight(d.mission)) {  // T-zero: the clamps open
        runner.release();
        if (!quiet) {
          std::fprintf(stderr, "T-zero at frame %u: released\n", static_cast<unsigned>(k));
        }
      }
      // frames the world has not been stepped over (ACT's frame did not come, or SYNC skipped numbers): step them with the last command held
      while (runner.frame() < k) {
        const uint32_t m = runner.frame();
        send_all(sock, runner.end_of_frame(m, nullptr), tx_errors);
        if (pico >= 0) {
          send_platform(pico, static_cast<uint8_t>(runner.frame()), runner.vehicle().tilts(), pico_drops);
        }
      }
      continue;
    }
    if (f.id == tfc::id::kActOut && started) {
      const tfc::DecodedAct d = tfc::unpack_act_out(f);
      const uint32_t k = runner.frame();
      if (!d.ok || d.seq != static_cast<uint8_t>(k) || acted == k) {
        continue;  // damaged, for another frame, or a repeat
      }
      acted = k;
      last_act_state = d.act.state;
      send_all(sock, runner.end_of_frame(k, &d.act), tx_errors);
      if (pico >= 0) {
        send_platform(pico, static_cast<uint8_t>(runner.frame()), runner.vehicle().tilts(), pico_drops);
      }
      if (telemetry.fd >= 0 && runner.frame() % 10U == 0U) {
        send_telemetry(telemetry, runner);
      }
      if (!quiet && runner.frame() % 100U == 0U) {
        const sim::Tilts t = runner.vehicle().tilts();
        std::fprintf(stderr, "t=%6.2f s  alt %8.0f m  speed %7.1f m/s  tilt pitch %7.3f yaw %7.3f deg  gimbal %6.2f %6.2f  ACT state %u  tx_err %u\n",
                     static_cast<double>(runner.frame()) * 0.01, runner.vehicle().altitude(), runner.vehicle().speed(), t.y_deg, t.x_deg,
                     runner.vehicle().gimbal_pitch_deg(), runner.vehicle().gimbal_yaw_deg(), static_cast<unsigned>(last_act_state), tx_errors);
      }
      if (max_frames != 0U && runner.frame() - first_frame >= max_frames) {
        break;
      }
    }
  }
  ::close(sock);
  if (pico >= 0) {
    ::close(pico);
  }
  if (telemetry.fd >= 0) {
    ::close(telemetry.fd);
  }
  std::fprintf(stderr, "tfc_simd: stopped at frame %u (t = %.2f s), altitude %.0f m\n", static_cast<unsigned>(runner.frame()), static_cast<double>(runner.frame()) * 0.01,
               runner.vehicle().altitude());
  return 0;
}
