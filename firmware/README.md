# Firmware (Zephyr)

Zephyr v4.4.2 app that consumes `core/` as-is. Only drivers, threads and ISRs belong here.
Status: M0 hello world builds and runs on `native_sim` and builds for `nucleo_g474re`.

```
firmware/
  west.yml       # pins Zephyr (v4.4.2) and the few modules we use
  env.sh         # source this: activates the venv, sets ZEPHYR_BASE and the SDK path
  app/           # Zephyr application (fc = flight computer, act = voter come later)
    CMakeLists.txt  prj.conf  src/main.cpp
```

## One-time setup (Ubuntu)
```bash
sudo apt install -y ninja-build device-tree-compiler gperf ccache

# 1. Zephyr SDK (minimal + ARM toolchain + host tools, ~230 MB)
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.1/zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz
tar xf zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz -C ~
~/zephyr-sdk-1.0.1/setup.sh -t arm-zephyr-eabi -h -c

# 2. west + Zephyr, from the repo root. The workspace is the repo root, so zephyr/ and
#    modules/ land inside it (git-ignored, ~1.8 GB). The venv lives next to the repo.
python3 -m venv ../.venv && ../.venv/bin/pip install west
../.venv/bin/west init -l firmware
../.venv/bin/west update --narrow -o=--depth=1
../.venv/bin/pip install -r zephyr/scripts/requirements-base.txt
```

## Every session
```bash
cd ~/SpaceX/triplex-fc
. firmware/env.sh            # once per terminal; puts `west` on PATH
```

## Build and run on the host (`native_sim`)
Runs the firmware as a normal Linux process. No hardware needed; this is what CI runs.
```bash
west build -p always -b native_sim/native/64 firmware/app -d build/native_sim
build/native_sim/zephyr/zephyr.exe -stop_at=1
```
Expected output:
```
*** Booting Zephyr OS build dccb09599635 ***
triplex-fc hello: vote_milli=1010 status=0 disagree_mask=0x04
Stopped at 1.010s
```
- `-stop_at=<s>` ends the run after `<s>` simulated seconds; without it the process idles forever (Ctrl+C).
- Drop `-p always` for faster incremental rebuilds.
- Use `native_sim/native/64` (64-bit); the plain `native_sim` needs 32-bit multilib.

## Build for the real board (`nucleo_g474re`)
```bash
west build -p always -b nucleo_g474re firmware/app -d build/nucleo_g474re
```
Flashing needs the board plugged in over USB (ST-LINK): `west flash -d build/nucleo_g474re`.
Serial console: `picocom -b 115200 /dev/ttyACM0` (first check `dmesg` for the device name).

## Gotchas
- **Include order:** in any `.cpp` that uses `core/`, include the `tfc/*.hpp` headers (and so the
  C++ standard library) *before* any `<zephyr/...>` header. Zephyr defines an `__unused` macro that
  breaks a glibc header (`struct_mutex.h`) on the native_sim host build.
- `core/` needs the full libstdc++ (`CONFIG_REQUIRES_FULL_LIBCPP=y`); Zephyr's default minimal C++
  library has no `<array>`/`<cmath>`. Exceptions and RTTI stay off.
- `printk` has no float support; print fixed-point integers, or enable picolibc float I/O deliberately.
