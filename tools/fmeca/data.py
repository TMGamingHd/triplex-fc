# SPDX-License-Identifier: MIT
"""The FMECA's data: the failure modes of the system, item by item, with their severity, probability, detection and compensation, and the rows of the fault matrix, the
campaign's fault kinds and the requirements that stand behind each. `fmeca.py` checks it against those and writes docs/verification/FMECA.md from it; edit this file, not the
document.

Method (docs/verification/FMECA.md section 2): ECSS-Q-ST-30-02C (severity categories and the suffixes R and SP, clause 4.2; critical items, 4.3 and 5.3; the criticality number
CN = SN x PN, 5.2; the hardware-software interaction analysis, clause 7), and NASA-GB-8719.13 appendix D for the software side (the data and event fault types). The ratings are
my engineering judgement (ECSS 5.2e: with no failure-rate data the qualitative approach is used); each is meant to be changed by the owner and the tool will show what changes.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Mode:
    id: str                      # FM-<item letter><number>
    item: str                    # the group the mode is listed under
    mode: str                    # how the item fails
    cause: str                   # why
    effect: str                  # the end effect, if nothing compensates (ECSS 4.2b: the severity is assigned without the compensating provisions)
    sev: int                     # 1 catastrophic, 2 critical, 3 major, 4 minor (ECSS Table 4-1, tailored in FMECA.md section 2)
    suffix: str                  # "R": the effect needs the failure of every redundant item; "SP": a single point; "SH": a safety hazard to people; "" none
    pn: int                      # probability number 1 to 4 that the end effect occurs (ECSS Table 5-2; with redundancy the probability that all the redundant items fail)
    pn_basis: str                # why that number
    detect: str                  # the failure detection method / observable symptom
    comp: str                    # the compensating provision
    faults: tuple[str, ...] = ()   # rows of the fault matrix (F01 ...)
    kinds: tuple[str, ...] = ()    # fault kinds of the campaign (tfc_peers.faults.KINDS and the campaign's groups)
    hw: bool = False             # a hardware failure that the software must answer (listed again in the hardware-software interaction analysis, ECSS clause 7)
    recs: tuple[str, ...] = ()   # recommendations (REC-nn) that this mode raised
    scope: str = ""              # "out of v1" for a stretch goal


@dataclass(frozen=True)
class Rec:
    id: str
    text: str
    status: str                  # open | done | accepted
    source: str                  # where it came from


# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
# The failure modes
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
MODES: tuple[Mode, ...] = (
    # ---- IMU ----
    Mode("FM-S1", "IMU (one of three, on its own computer)", "Output wrong but plausible: a bias step, a drift, a scale error, inverted or swapped axes, zeros, clipping, bit errors",
         "a failing part, a loose connection, an upset in a sample before its CRC", "a wrong rate reaches the attitude estimator and the gimbal command: loss of control", 1, "R", 1,
         "needs a second IMU to fail the same way in the same run", "the vote on the six sensor values; 3-of-5 persistence; the leaky count for a sparse fault",
         "mid-value select masks it in the same frame; the IMU channel is isolated and its computer keeps voting commands",
         ("F04", "F05", "F27", "F29", "F30", "F31", "F32", "F35", "F36"), ("bias_gyro", "bias_accel", "drift", "scale", "invert", "swap", "zero", "clip", "bitflip", "stuckbit")),
    Mode("FM-S2", "IMU (one of three, on its own computer)", "Noisier, vibrating or repeating samples (noise, aliasing, a data rate that does not match)", "wear, a mounting resonance, a configuration slip",
         "a noisier attitude estimate, a coarser command", 3, "R", 2, "the part is the likeliest thing on the bench to degrade slowly", "the vote, once the noise passes about a third of the tolerance; a small increase is left alone on purpose",
         "isolated when it matters (above about 3x the healthy noise); below that the estimator filters it", ("F28", "F33", "F34"), ("noise", "oscillate", "repeat")),
    Mode("FM-S3", "IMU (one of three, on its own computer)", "Output frozen at its last value", "a stuck bus, a hung sensor, a frozen buffer", "the attitude estimate stops following the vehicle", 1, "R", 2,
         "a physical fault that recurs", "the stuck detector (20 identical frames) and the vote once the vehicle moves away from the frozen value", "isolated; a second stuck latch disables the channel for the run (physical cause)",
         ("F03",), ("stuck",)),
    Mode("FM-S4", "IMU (one of three, on its own computer)", "A single wild sample or a short burst", "EMI, a bus glitch", "none if it is ignored; one bad command if it is believed", 4, "", 3,
         "common on a bench", "the persistence filter (3 of 5)", "a single sample never latches; the median masks it in the same frame", ("F06",), ("spike",)),
    Mode("FM-S5", "IMU (one of three, on its own computer)", "A wild or non-finite value, a saturated output", "a failing part, a decoding fault", "an estimator fed a number it cannot use", 1, "R", 1,
         "needs a second channel to fail", "the vote (a non-finite sample is invalid and is blamed)", "the voter drops a non-finite channel; the estimator refuses a non-finite gyro", ("F07",), ("saturate",)),
    Mode("FM-S6", "IMU (one of three, on its own computer)", "No data from the IMU while its computer is healthy", "a dead sensor, SPI wiring, a driver fault", "one channel of sensing is lost", 3, "R", 2,
         "a connector on a bench", "the missing-frame check on the sensor stream", "the IMU channel is isolated alone; its computer stays a voter for commands (the sensor/compute split, ADR-020)",
         ("F65", "F92", "F93", "F94", "F95"), ("dropout",)),
    Mode("FM-S7", "IMU (one of three, on its own computer)", "An IMU and a different computer both lost", "two independent failures", "only one channel of sensing is left", 3, "R", 1, "two independent failures",
         "both channels are missing", "the ring that re-homes an orphaned IMU (ADR-020 case 2): not built", ("F73", "F74"), (), scope="out of v1"),
    # ---- flight computer ----
    Mode("FM-C1", "Flight computer (one of three)", "Power loss, a crash, fail-silent", "a power glitch, a hard fault, a connector", "its command is missing", 1, "R", 1, "needs the other two to fail",
         "the missing-frame check (3 frames), the heartbeat, the supervisor's KICK", "isolated within 3 frames; the voted output does not move; the supervisor resets a stopped unit", ("F01", "F58"), ("dropout",), hw=True),
    Mode("FM-C2", "Flight computer (one of three)", "A hang of one task while the others keep running", "a software defect, a deadlock", "stale or missing commands from a node that still looks alive", 1, "R", 1,
         "needs the other two to fail", "a frame is serviced to the watchdog only at the end of a completed frame; the vote sees stale data", "the watchdog resets it; the other two outvote it meanwhile", ("F02", "F57"), (), hw=True),
    Mode("FM-C3", "Flight computer (one of three)", "A reset, a reboot, a reset loop", "a watchdog, a brown-out, a software fault", "its commands are missing for a while; repeated, it keeps leaving and joining", 3, "R", 2,
         "resets are the commonest fault of a young firmware", "missing frames; the reset counter in non-volatile RAM", "it rejoins in phase from SYNC and the others' state share restores its strikes; a reset loop is held out and reported",
         ("F24", "F60", "F61", "F68", "F86"), ("reboot",), hw=True),
    Mode("FM-C4", "Flight computer (one of three)", "A wrong but valid command (a frozen, inverted or offset command, a software defect)", "a software defect, an upset in the controller", "a wrong command reaches the actuator", 1, "R", 1,
         "needs a second computer to fail the same way", "the vote on the two command channels; in Duplex, continuity with the motion (ADR-017)", "outvoted in Triplex; in Duplex isolated where the motion tells, otherwise held and Safe requested",
         ("F09", "F37", "F38", "F50"), ("cmd_offset", "cmdstuck", "cmdinvert")),
    Mode("FM-C5", "Flight computer (one of three)", "Frames late, early, jittering or on a drifting clock", "a timing overrun, a clock fault, a stalled sender", "stale data is voted, or a frame misses the vote", 1, "R", 1,
         "needs a second computer to fail", "the frame number each frame carries is checked against the receiver's count (ADR-018); the supervisor measures the period", "isolated once the frames cross the deadline; a frame a few ms early inside its window is not seen (arrival-time check waits for the board)",
         ("F25", "F43", "F44", "F45", "F67"), ("late", "early", "jitter", "clockdrift"), hw=True, recs=("REC-07",)),
    Mode("FM-C6", "Flight computer (one of three)", "Only some of its frames are sent", "a partial hang, a task that stopped", "its command or a sensor stream is missing while the others arrive", 3, "R", 2,
         "a task-level fault", "the missing-frame check per stream", "the computer is isolated within 3 frames", ("F39",), ("partial",)),
    Mode("FM-C7", "Flight computer (one of three)", "The estimator state diverges from the others'", "an upset in the state, a sensor frame lost at one receiver", "its command drifts away from the others'", 1, "R", 2,
         "a lost frame at one receiver does it; real CAN loss is not known yet", "the state digest in every command; the resync's large-correction report", "the digest cross-check isolates the odd computer; every 100 frames the replicas adopt the mid-value state (ADR-030)",
         ("F10", "F51", "F88", "F89", "F90", "F91"), ("digest",), recs=("REC-08",)),
    Mode("FM-C8", "Flight computer (all three)", "A software defect common to every replica", "the same code on the same inputs", "all three commands are wrong together", 1, "SP", 2,
         "a defect that testing missed is certain to be common", "a regression is seen by the node on the previous release (ADR-021); a bug in both releases is not seen", "none for a defect present in both releases; the claim excludes it (LIMITATIONS 2)",
         ("F63", "F64"), (), recs=("REC-01",)),
    Mode("FM-C9", "Flight computer (all three)", "The guidance or navigation commands a wrong thing", "a model of the descent that is wrong, a bad fix", "the stage misses the tower or leaves its corridor", 2, "SP", 3,
         "the whole-mission Monte Carlo catches the stage in none of 100 flights under the assumed dispersions", "none on a board; the host Monte Carlo and the mission runs", "none: host only", (), (), recs=("REC-02",)),
    Mode("FM-C10", "Flight computer (all three)", "A cycle that overruns its frame on the target", "software double precision on a single-precision FPU, a predictor that runs long", "late commands from all three computers together", 1, "SP", 2,
         "the cost on the G474 is not known", "the firmware's WCET counters; the frame number check", "none until it is measured (S1)", (), (), hw=True, recs=("REC-03",)),
    # ---- CAN bus ----
    Mode("FM-B1", "CAN bus (one segment)", "A frame corrupted, lost, duplicated or replayed; a frozen frame counter", "EMI, a marginal connector, a faulty transceiver", "a sample missing or stale for one node", 3, "", 3,
         "the commonest bus fault", "CRC-8, the frame number, the missing-frame check", "one bad sample costs nothing; three in five isolate; a duplicate or replay is a sequence error", ("F08", "F19", "F40", "F41", "F42"),
         ("corrupt", "duplicate", "replay", "seqstuck", "seqgap"), hw=True, recs=("REC-04",)),
    Mode("FM-B2", "CAN bus (one segment)", "A babbling node, traffic on an unlisted identifier", "a node stuck transmitting", "the bus is flooded and every deadline slips", 1, "SP", 2, "one bus segment; a stuck transmitter is a known failure",
         "the out-of-schedule rate monitor (a bus alarm at 3 stray frames in a frame)", "reported, not blamed; ACT ignores unlisted ids; no answer for a persistent alarm beyond the report", ("F11", "F49", "F59"), ("babble",), hw=True, recs=("REC-04",)),
    Mode("FM-B3", "CAN bus (one segment)", "One node cut off from the bus", "a stub or a tap open", "its frames are missing", 3, "R", 2, "a connector", "the missing-frame check", "as for a silent computer", ("F12",), ("dropout",), hw=True),
    Mode("FM-B4", "CAN bus (one segment)", "The bus lost altogether, a short, bus-off", "a trunk cut, CANH shorted to CANL", "no frame reaches anyone", 1, "SP", 2, "one segment, no second bus", "ACT sees no valid vote; every node sees nobody",
         "ACT freezes, holds and ramps to neutral by itself (Safe); a bus-off controller recovers on its own", ("F13", "F14", "F69"), (), hw=True, recs=("REC-05",)),
    Mode("FM-B5", "CAN bus (one segment)", "The sync master lost, or SYNC wrong", "node A fails, a damaged SYNC", "the frame clock stops or jumps", 1, "R", 2, "the master is one of three", "SYNC missing for 2 frames; the mission frame is checked against the follower's own count",
         "the next computer takes over; a flying follower keeps its own count against a false SYNC", ("F15", "F84", "F85"), (), hw=True),
    Mode("FM-B6", "CAN bus (one segment)", "Inconsistent omission: one receiver misses a frame the others got", "a transceiver at its limit, an error frame", "two computers vote different sets", 1, "R", 2, "a real CAN effect nobody has measured",
         "the digest cross-check and the resync (the designed answer)", "not modelled by the peers; the answer is designed and exercised on the host only", (), (), hw=True, recs=("REC-04",)),
    # ---- redundancy manager ----
    Mode("FM-M1", "Redundancy manager (software, one per computer)", "An upset in the manager's own state or configuration, or an invalid configuration", "a cosmic ray or a stray write; a bad constant", "a healthy node believed bad, or a bad one believed healthy; a tolerance that sees nothing", 1, "R", 2,
         "RAM is not immune and the configuration is data from outside", "a complement beside each guarded byte, a checksum of the configuration, a validation at construction; all scrubbed every frame", "repaired on the safe side (node excluded, Safe requested, configuration restored), counted and reported (ADR-015)",
         ("F47", "F48", "F56"), (), recs=("REC-09",)),
    Mode("FM-M2", "Redundancy manager (software, one per computer)", "A disagreement of two computers that nobody can attribute", "a small bias, a slow drift, a digest mismatch with two voters", "a wrong command is chosen or the right one thrown away", 2, "", 2,
         "needs two voters, a small error and no way to tell", "the Duplex vote and the digest", "hold the last good output and request Safe (sticky, ADR-008); only the operator ends it", ("F16", "F20"), (), recs=("REC-06",)),
    Mode("FM-M3", "Redundancy manager (software, one per computer)", "Total loss: every computer latched", "a bus-wide outage plus one more failure", "no voted output and no way back", 2, "", 1, "needs the loss of everything", "no healthy node", "the probationers judge each other (ADR-014); ACT's own sequence meanwhile", ("F46",), ()),
    Mode("FM-M4", "Redundancy manager (software, one per computer)", "A healthy computer isolated by mistake (a false latch)", "a threshold set too tight, a lost frame, a noisy sensor", "redundancy lost for nothing", 3, "R", 2,
         "the rate is a parameter that real error rates will move", "none (it is the manager's own decision)", "persistence 3 of 5, a dwell and a probation before readmission; the campaign counts every false isolation and finds none", ("F19", "F21", "F26"), ("intermittent",), recs=("REC-10",)),
    Mode("FM-M5", "Redundancy manager (software, one per computer)", "A repeat offender, an operator who readmits a node that is still bad", "a faulty part, an impatient operator", "a bad node back in the vote", 3, "R", 2, "an operator action", "the shadow vote on probation; strikes",
         "a node that is still bad cannot pass probation; the third strike (second for a physical cause) disables it for the run", ("F22", "F23", "F72"), ()),
    Mode("FM-M6", "Redundancy manager (software, one per computer)", "A phase change or a demotion that leaves too few voters", "an operator slip", "fewer voters than the phase needs", 3, "", 2, "an operator action", "the phase table", "refused below the phase's minimum; an ARM below its nominal", ("F71",), ()),
    # ---- actuator node ----
    Mode("FM-A1", "Actuator node ACT (single)", "A reset or a brown-out", "a power glitch, a watchdog", "the output is lost for the length of a boot", 2, "SP", 2, "one node", "the reset cause and a stored output", "it restarts in Safe holding the output it had stored, so nothing steps across the reset (TFC-RESP-003)", ("F17",), (), hw=True),
    Mode("FM-A2", "Actuator node ACT (single)", "Silent or hung", "a software defect, a hard fault", "no command reaches the actuator", 1, "SP", 2, "one node, no second ACT (a stretch goal)", "the supervisor's KICK and FRAME lines on ACT", "the supervisor resets it; the actuator holds or nulls per SAFE_MODE; a hung ACT is fail-passive", (), (), hw=True, recs=("REC-11",)),
    Mode("FM-A3", "Actuator node ACT (single)", "An upset in ACT's own state: the mode byte, or a bad configuration", "a cosmic ray; a bad constant", "Safe silently undone (Safe 2 and Standby 0 differ by one bit), or every computer excluded on the first frame", 1, "SP", 2,
         "ACT is the last line and had no self-protection until the review of core/", "a complement beside the mode byte, checked every frame; the configuration validated like the manager's", "ACT goes to Safe (cause: integrity) and counts it; a bad constant is replaced by its default and reported", ("F99", "F100"), (), recs=("REC-12",)),
    Mode("FM-A4", "Actuator node ACT (single)", "Safe entered or left at the wrong time", "a hardware line, a request, an early clear", "a vehicle safed on the pad needlessly, or a Safe left with the cause still there", 2, "", 2, "an operator action or a line fault",
         "the exit conditions (100 good votes from two nodes, no request, no line)", "Safe goes straight to neutral on the pad; nothing leaves Safe without the operator's ARMed clear-safe", ("F70", "F98"), ()),
    Mode("FM-A5", "Actuator node ACT (single)", "A computer that asks for Safe when it should not, or hides a request", "a faulty computer's heartbeat", "a vehicle safed by one computer, or one that should be safed and is not", 2, "R", 2, "needs one faulty computer", "the heartbeat monitor over the fresh heartbeats of the nodes ACT has not excluded",
         "a majority decides; a lone survivor decides; a stale or excluded heartbeat does not count", ("F102",), ()),
    # ---- ground commands and the launch sequence ----
    Mode("FM-G1", "Ground command path", "A command forged, replayed, stale or damaged", "an attacker, a stuck transmitter, a corrupted frame that passes the CRC", "an operator action nobody took", 1, "R", 2, "the key is the public bench key", "the SipHash tag, the counter window, the ARM expiry",
         "dropped and counted without a trace on the bus; a dangerous command needs an ARM and then its EXECUTE", ("F52", "F53", "F81"), (), recs=("REC-13",)),
    Mode("FM-G2", "Ground command path", "A dangerous command given by mistake: clear-safe, clear-disabled, disabling nodes until redundancy is gone", "an operator slip", "protection removed", 1, "R", 2, "an operator action", "the interlock tiers",
         "a plain disable down to Duplex; an ARM below that; refused below the phase's minimum; the last voter is a loud, critical event", ("F54", "F55"), ()),
    Mode("FM-G3", "Ground command path", "An upset in the command counter or the ARM record", "a cosmic ray", "a replay accepted, or a command refused", 3, "R", 1, "needs a second fault", "the guarded bytes", "the ARM is cleared and the counter forgotten and re-established; reported", ("F56",), ()),
    Mode("FM-G4", "Ground command path", "The operator cannot tell whether the path works", "a dead link, a wrong key", "an action believed done that was not", 3, "", 2, "a bench routine", "a noop command answered like any other", "the noop proves the path end to end (TFC-FDIR-043)", ("F62",), ()),
    Mode("FM-L1", "Launch sequence and mission clock", "A launch with the system not ready, forged, twice, or a scrub after T-zero", "an operator slip, a forged frame", "an unintended or an unready launch", 1, "R", 2, "an operator action", "the go/no-go (four conditions, checked every frame of the countdown)",
         "refused unless go; a scrub is refused in flight; a second launch is refused", ("F81", "F82", "F83", "F87"), ()),
    Mode("FM-L2", "Launch sequence and mission clock", "T-zero early, stuck or lost; the sync master dies in the countdown; a computer restarts after T-zero", "a glitch on the T0 line, a failed master", "a shortened countdown or a clock that jumps", 1, "R", 2, "a line fault",
         "the T0 edge window; the follower's own count; the supervisor's plausibility check", "an edge outside the last second is refused; a restarting computer adopts the mission frame; the countdown is the fallback", ("F84", "F85", "F86", "F96"), (), hw=True),
    Mode("FM-L3", "Launch sequence and mission clock", "A launch with a hardware override engaged and not acknowledged", "mode confusion", "a run whose protections are off without anyone knowing", 1, "R", 2, "an operator action", "the supervisor's debounced sense lines", "launch is refused until the operator acknowledges", ("F97",), (), hw=True),
    # ---- supervisor ----
    Mode("FM-U1", "Supervisor (single)", "Unpowered, in reset or hung", "a software defect, a power fault", "no watchdog help for the nodes", 3, "SP", 2, "one Pico, software too", "its status and USB port stop", "every node keeps running with no relay energised and SAFE low (TFC-SUP-007); the supervisor is not in the flight path", ("F66",), (), hw=True),
    Mode("FM-U2", "Supervisor (single)", "A working unit reset or power-cycled", "a wrong judgement, a noisy KICK line", "a healthy computer taken out", 1, "R", 1, "it acts only on a unit that has stopped", "none (a design property)", "it never acts on data (TFC-SUP-010) and never on the last agreeing computer (TFC-RESP-004); three resets become a power cycle; two cycles hold it DEAD", ("F68", "F67"), (), hw=True),
    # ---- platform, injector, overrides, power ----
    Mode("FM-P1", "Platform, injector and hardware overrides", "The platform moves violently: the E-stop, a servo rail cut at an angle", "a snap to level, a stall", "a hazard to people and to the rig", 1, "SH", 2, "the only physical hazard of the rig", "the platform's tilt and the servo current", "rate and travel limits; a hold after 100 ms without a command and a level after 1 s", ("F78",), (), hw=True, recs=("REC-14",)),
    Mode("FM-P2", "Platform, injector and hardware overrides", "The injector's relay stays on after a Pico crash", "a stuck relay, a crashed driver", "a node stays cut and the test is not the one believed", 3, "", 2, "a relay", "the node is missing; the supervisor sees it", "the injector-disarm switch; the pre-session check", ("F75",), (), hw=True),
    Mode("FM-P3", "Platform, injector and hardware overrides", "An override fails to act, acts by accident, leaves a cut node half powered, or its coils pull the rail down", "a broken contact, a brush, a signal line, a coil load", "a latent loss of protection, or a lost run", 3, "R", 2, "hardware not yet ordered", "the pre-session check; the sense line; a meter on every cut signal",
         "each override is operated by P-HWO-01; the run is marked overridden", ("F76", "F77", "F79", "F80"), (), hw=True, recs=("REC-15",)),
    Mode("FM-W1", "Power", "The single 5 V adapter lost or sagging", "an adapter fault", "every node, both Picos and every relay coil lose power together", 1, "SP", 2, "one rail, one star ground", "every node and the supervisor", "none: the rail is outside the claim (LIMITATIONS 1); no override removes a failing adapter", (), (), hw=True, recs=("REC-16",)),
    # ---- recorders and the vehicle ----
    Mode("FM-R1", "Flight log and reset log", "No record of a fatal fault, or a record that corrupts the memory around it", "a fault with no handler, an upset in the ring's index", "a computer that cannot say why it reset; a stray write outside the ring", 3, "", 2, "built in ADR-055, not yet run on a board", "a CRC on every record and on the header", "a damaged record is skipped and said so; the ring index is range-checked on every write", ("F101",), (), recs=("REC-17",)),
    Mode("FM-V1", "Vehicle (the plant)", "An engine fails", "a real engine failure", "less thrust and an unbalanced cluster", 2, "", 3, "the plant's own failure, injected on purpose", "the engine controllers' health word; the attitude loop", "the guidance counts the engines left; the booster lands and the ship reaches orbit with engines out (ADR-058)", ("F18",), ()),
)

# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
# The software FMEA, by data item and fault type (NASA-GB-8719.13 appendix D: the data table). Each cell is where that fault is injected/tested, "n/a" if the fault means nothing for the item,
# or "GAP REC-nn" when nothing exercises it.
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
DATA_FAULTS = ("missing", "corrupted", "wrong value", "out of range", "late", "early / out of sequence", "duplicate / replayed", "frozen")
DATA_ITEMS: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("IMU gyro and accelerometer samples (0x100, 0x110)", ("dropout", "corrupt", "bias_gyro", "saturate", "late", "early", "replay", "stuck")),
    ("Control command (0x200)", ("dropout", "corrupt", "cmd_offset", "cmd_offset", "late", "early", "duplicate", "cmdstuck")),
    ("State digest in the command", ("dropout", "corrupt", "digest", "n/a", "late", "early", "replay", "digest")),
    ("SYNC: frame number and mission frame (0x010)", ("F15, F85", "host: decoder tests", "F84", "F84", "n/a", "F67", "F84", "GAP REC-19")),
    ("Heartbeat: mode, Safe request, ready, release (0x400)", ("host: HeartbeatMonitor ageing", "host: decoder tests", "F102", "host: decoder masks the fields", "host: HeartbeatMonitor ageing", "n/a", "host: HeartbeatMonitor ageing", "F102")),
    ("State share: strikes, command counter (0x410)", ("F60", "host: decoder tests", "host: test_state_share", "n/a", "n/a", "n/a", "n/a", "n/a")),
    ("Resync chunk (0x420)", ("F90", "host: decoder tests", "F89, F91", "n/a", "F88", "F88", "n/a", "n/a")),
    ("Ground command (0x510)", ("F62", "F52", "F52", "n/a", "n/a", "n/a", "F53", "n/a")),
    ("ACT output (0x300), as the go/no-go and the simulator read it", ("GAP REC-18", "host: decoder tests", "n/a", "n/a", "GAP REC-18", "n/a", "n/a", "GAP REC-18")),
    ("Engine health word and the simulator's frames (0x509 and the 0x50x range)", ("host: ADR-058 tests", "host: decoder tests", "host: ADR-058 tests", "host: decoder tests", "n/a", "n/a", "n/a", "n/a")),
)

# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
# The recommendations that the analysis raised
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
RECS: tuple[Rec, ...] = (
    Rec("REC-01", "A defect common to every replica is not defended by voting. Keep node C on the previous release (ADR-021), keep the claim's exclusion (LIMITATIONS 2), and ask an outside reader to attack `core/` (CODE_REVIEW_CORE.md).", "accepted", "FM-C8"),
    Rec("REC-02", "The guidance has run on the host only and the whole-mission Monte Carlo catches the stage in none of 100 flights under the assumed dispersions: it is a model, not a flight-computer function with timing; do not quote the catch as evidence of the flight computer.", "open", "FM-C9"),
    Rec("REC-03", "Measure the worst-case cycle on the board (S1), with the guidance and navigation in the image, and decide float or fixed point from the number.", "open", "FM-C10"),
    Rec("REC-04", "Measure the real bus: the loss rate (`bus_loss.py`), a babbler's real harm, and inconsistent omission. The resync period and the digest persistence were set on a simulator.", "open", "FM-B1, FM-B2, FM-B6"),
    Rec("REC-05", "A second bus is outside v1. Until then the bus is a single point; the claim says so.", "accepted", "FM-B4"),
    Rec("REC-06", "An unattributable Duplex disagreement ends in Safe and the operator. Analytical redundancy (FUTURE_WORK 2.1) would settle more of these; not built.", "accepted", "FM-M2"),
    Rec("REC-07", "The arrival-time check (a frame a few ms early inside its window) waits for the board's receive timestamps (TFC-FDIR-037).", "open", "FM-C5"),
    Rec("REC-08", "Measure how often a real receiver misses a frame the others got; the state resync is designed against it and tested only against modelled loss.", "open", "FM-C7"),
    Rec("REC-09", "The detectors (the 3-of-5 window, the leaky count, the stuck detector) hold copies of their parameters outside the integrity check: an upset in one is not seen. Rebuild them from the checked configuration in the scrub.", "open", "FM-M1, CODE_REVIEW_CORE finding 7"),
    Rec("REC-10", "The false-isolation rate of healthy computers on the real IMUs and the real bus is not known: run a fault-free soak of hours on the rig and count false latches (TS-1 asks for under one per 100 hours).", "open", "FM-M4"),
    Rec("REC-11", "ACT is a single point. A second ACT with a supervisor-driven selector is a stretch goal (FUTURE_WORK 3).", "accepted", "FM-A2"),
    Rec("REC-12", "ACT's mode byte is guarded and its configuration validated (finding 2 and 3 of the review of core/); the same discipline for ACT's other state (the held output, the lost-vote count) is the next step.", "done", "FM-A3, CODE_REVIEW_CORE findings 2 and 3"),
    Rec("REC-13", "The ground key is the public bench key and the tag comparison is not constant-time: do not use the command path for anything but the bench (FUTURE_WORK 2.4).", "accepted", "FM-G1"),
    Rec("REC-14", "The platform's E-stop shock and the servo's stall current wait for the rig (PICO_TESTS E1 to E6).", "open", "FM-P1"),
    Rec("REC-15", "Order the override parts, then run P-HWO-01 on each.", "open", "FM-P3"),
    Rec("REC-16", "One 5 V rail feeds everything. Outside the claim; if the claim is widened, a second supply and an isolating scheme are the first change.", "accepted", "FM-W1"),
    Rec("REC-17", "The flight log and the fault handler are untested on a board; check that a forced fault leaves its record and that the CRC of the image is compared with an expected value, not only reported.", "open", "FM-R1"),
    Rec("REC-18", "ACT's output frame is read by the firmware's go/no-go under a freshness rule (`k - act_seen <= 3`) that lives in `firmware/app/src/main.cpp`, not in the core, and no test exercises a missing or frozen ACT frame: move the rule into `launch_gate.hpp` and test it. Also, the campaign's virtual peers send no heartbeat, so the heartbeat-driven decisions (ACT's Safe majority, the go/no-go, release awareness) are covered by unit and live tests only: give the peers a heartbeat and a fault kind that lies in it.", "open", "software FMEA, section 6"),
    Rec("REC-19", "A SYNC whose frame number stops advancing (a stuck master) is accepted by the followers, and the resync schedule, which keys on the frame number, then stops or runs every frame. Check that the number advances (by one, or by the gap of the SYNCs missed) and test it.", "open", "software FMEA, section 6"),
)
