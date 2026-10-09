# SPDX-License-Identifier: MIT
"""The console reads what the nodes print. These tests hold the parser to the firmware's own format strings, so a new event line in `firmware/` that the parser does not know cannot go unnoticed."""
import re
import unittest

from .console_support import ROOT
from tfc_console.lines import parse_line

FIRMWARE = [ROOT / "firmware" / "app" / "src" / "main.cpp", ROOT / "firmware" / "act" / "src" / "main.cpp", ROOT / "firmware" / "supervisor" / "src" / "main.cpp"]

# printk formats that are not events and are shown as plain text on purpose (the console never hides a line, it only does not call these events)
PLAIN = ("CAN device not ready", "can_set_bitrate", "cannot add", "can_start", "CONFIG ERROR", "WARNING: the node-id straps", "[cycle %u] waiting for SYNC", "[cycle %u] RESET LOOP", "WATCHDOG: could not be started",
         "ACT: following SYNC", "overrides not yet tested")


def formats(path):
    """Every printk / snprintk format string in a firmware file: the literals (adjacent ones joined) that open the call, not the ones in its arguments."""
    text = path.read_text()
    out = []
    for m in re.finditer(r"(?:printk|snprintk)\s*\(", text):
        i, lits = m.end(), []
        while i < len(text):
            while text[i] in " \t\r\n":
                i += 1
            if text[i] != '"':
                if lits:
                    break                          # the first argument ended: the rest are values
                i = text.find(",", i) + 1          # snprintk(buffer, size, "format"...): skip the buffer and the size
                continue
            j = i + 1
            while text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            lits.append(text[i + 1:j])
            i = j + 1
        fmt = "".join(lits).replace("\\n", "")
        if fmt:
            out.append(fmt)
    return out


# formats whose conversions stand for words, and what a node prints for them
SAMPLES = {
    "[frame %u] GROUND COMMAND %s%s P%u: %s": "[frame 7] GROUND COMMAND ARM phase P3: refused: phase",
    "[frame %u] GROUND COMMAND %s%s: %s": "[frame 7] GROUND COMMAND ARM clear-safe: accepted",
    "[frame %u] GROUND COMMAND %s%s %c: %s": "[frame 7] GROUND COMMAND reintegrate B: accepted",
    "[frame %u] BUS ALARM %s: %u out-of-schedule frames in this 10 ms frame": "[frame 7] BUS ALARM RAISED: 5 out-of-schedule frames in this 10 ms frame",
    "[frame %u] MODE %s -> %s": "[frame 7] MODE TRIPLEX -> DUPLEX",
    "[frame %u] GROUND command %s": "[frame 7] GROUND command refused",
    "[frame %u] ACT %s -> %s (%s), cause: %s": "[frame 7] ACT NOMINAL -> SAFE (hold), cause: lost-votes",
    "[frame %u] ACT %s node %c": "[frame 7] ACT EXCLUDED node B",
    "[frame %u] %s %s pitch %.3f yaw %.3f  voted %u excluded %u held %u  | held_frames %u safe_entries %u refused_clears %u tx_err %u": "[frame 7] NOMINAL - pitch 0.1 yaw 0.2  voted 7 excluded 0 held 0  | held_frames 0 safe_entries 0 refused_clears 0 tx_err 0",
    "[frame %u] %s  A%c B%c C%c  | crc=%u": "[frame 7] TRIPLEX  A+ B+ C+  | crc=0 seq=0",
    "[frame %u] RESYNC: nothing adopted (%s)": "[frame 7] RESYNC: nothing adopted (the states disagree)",
    "[frame %u] LAUNCH REFUSED, no-go: %s": "[frame 7] LAUNCH REFUSED, no-go: not ready",
    "[frame %u] COUNTDOWN SCRUBBED, no-go: %s": "[frame 7] COUNTDOWN SCRUBBED, no-go: not ready",
    "[frame %u] node %c LATCHED OUT: %s": "[frame 7] node B LATCHED OUT: vote disagreement",
    "[frame %u] IMU %c LATCHED OUT (computer %c stays in the command vote): %s": "[frame 7] IMU B LATCHED OUT (computer B stays in the command vote): frame missing",
    "[frame %u] node %c FAILED PROBATION, back to latched: %s": "[frame 7] node B FAILED PROBATION, back to latched: vote",
}


def render(fmt):
    """What a node prints for a format: a hand-written sample where the conversions stand for words, otherwise numbers."""
    for prefix, line in SAMPLES.items():
        if fmt.startswith(prefix):
            return line
    for conv, sample in (("%c", "B"), ("%.3f", "1.234"), ("%u", "7"), ("%d", "7"), ("%x", "0x4"), ("%s", "WORDS")):
        fmt = fmt.replace(conv, sample)
    return fmt.replace("%%", "%")


class AgainstTheFirmware(unittest.TestCase):
    def test_every_format_in_the_firmware_is_found(self):
        for p in FIRMWARE:
            self.assertGreater(len(formats(p)), 5, p)

    def test_no_event_line_of_the_firmware_is_read_as_plain_text(self):
        unknown = []
        for p in FIRMWARE:
            for fmt in formats(p):
                if any(fmt.startswith(x) or x in fmt for x in PLAIN):
                    continue
                parsed = parse_line(render(fmt))
                if parsed is not None and parsed.kind == "text":
                    unknown.append(f"{p.parent.parent.name}: {fmt[:90]}")
        self.assertEqual(unknown, [], "teach tfc_console/lines.py these lines (or add them to PLAIN with the reason)")

    def test_a_non_empty_line_is_never_dropped(self):
        for p in FIRMWARE:
            for fmt in formats(p):
                self.assertIsNotNone(parse_line(render(fmt)), fmt)


class Kinds(unittest.TestCase):
    def test_a_latch_has_its_node_its_reason_and_its_frame(self):
        p = parse_line("[frame 402] node B LATCHED OUT: vote disagreement")
        self.assertEqual((p.kind, p.level, p.node, p.frame, p.fields["reason"]), ("latched-out", "warn", "B", 402, "vote disagreement"))

    def test_the_status_line_becomes_counters_and_the_nodes_own_view(self):
        p = parse_line("[frame 300] TRIPLEX  A+ Bp Cw  | crc=1 seq=2 missing=3 vote=4 digest=5 stuck=6 oos=7 tx_err=0 imu_err=1 imu_stale=2 wdt_refused=0 bus_off=0 err_passive=0 sync_missed=0 mission=0 ready=1 resync=12/3 far=2 wcet_step=7 wcet_vote=8 wcet_frame=9100 phase=3 warm=0x4")
        self.assertEqual(p.kind, "status")
        f = p.fields
        self.assertEqual((f["mode"], f["crc"], f["seq"], f["missing"], f["vote"], f["digest"], f["stuck"], f["oos"], f["wcet_frame"], f["phase"], f["warm"]), ("triplex", 1, 2, 3, 4, 5, 6, 7, 9100, 3, 4))
        self.assertEqual((f["resync_adopted"], f["resync_skipped"], f["far"]), (12, 3, 2))
        self.assertEqual(f["view"], {"A": "voting", "B": "probation", "C": "warm"})

    def test_acts_status_line(self):
        p = parse_line("[frame 201] NOMINAL - pitch 0.054 yaw -0.060  voted 7 excluded 2 held 1  | held_frames 4 safe_entries 1 refused_clears 2 tx_err 0")
        self.assertEqual(p.kind, "act-status")
        self.assertEqual((p.fields["mode"], p.fields["pitch"], p.fields["yaw"], p.fields["voted"], p.fields["excluded"], p.fields["held"], p.fields["safe_entries"], p.fields["refused_clears"]), ("nominal", 0.054, -0.06, 7, 2, 1, 1, 2))

    def test_ground_command_answers(self):
        a = parse_line("[frame 9] GROUND COMMAND ARM clear-disabled B: accepted")
        self.assertEqual((a.kind, a.level, a.fields["op"], a.fields["arm"], a.fields["target"], a.fields["result"]), ("ground-command", "ok", "clear-disabled", True, "B", "accepted"))
        b = parse_line("[frame 9] GROUND COMMAND disable C: refused: needs an ARM frame first")
        self.assertEqual((b.level, b.fields["result"]), ("warn", "refused: needs an ARM frame first"))
        c = parse_line("[frame 9] GROUND COMMAND phase P3: accepted")
        self.assertEqual(c.fields["target"], "P3")
        d = parse_line("[frame 9] GROUND clear-safe: REFUSED (the votes have not been good for long enough)")
        self.assertEqual((d.kind, d.level, d.fields["by"]), ("ground-command", "warn", "ACT"))

    def test_a_boot_line_is_not_an_event_even_though_it_names_a_watchdog(self):
        p = parse_line("boot 1 since power-on, last reset cause 1 (1 power-on, 2 pin, 3 watchdog, 4 software, 5 brown-out, 6 fault), 0 short boots in a row")
        self.assertEqual((p.kind, p.level), ("boot", "info"))

    def test_the_colours_of_a_terminal_are_not_the_parsers_business_but_a_blank_line_is_nothing(self):
        self.assertIsNone(parse_line(""))
        self.assertIsNone(parse_line("   \r\n"))

    def test_levels_follow_the_words(self):
        self.assertEqual(parse_line("[frame 5] MODE TRIPLEX -> DUPLEX").level, "warn")
        self.assertEqual(parse_line("[frame 5] MODE DUPLEX -> TRIPLEX").level, "ok")
        self.assertEqual(parse_line("[frame 5] SAFE REQUESTED: the two voting nodes disagree").level, "crit")
        self.assertEqual(parse_line("[frame 5] !!! CRITICAL: the last voting node was removed by operator command !!!").level, "crit")
        self.assertEqual(parse_line("[frame 5] node C REINTEGRATED into the vote (strikes on record: 1)").level, "ok")
        self.assertEqual(parse_line("[frame 5] node C DISABLED for the run (strikes: 3)").level, "warn")
        self.assertEqual(parse_line("[frame 5] RESYNC: nothing adopted (fewer than two voters)").level, "warn")
        self.assertEqual(parse_line("[frame 5] LAUNCH REFUSED, no-go: a flight computer is not ready").kind, "launch-refused")

    def test_the_supervisor(self):
        self.assertEqual(parse_line("RESET B (kicks stopped)").kind, "supervisor")
        self.assertEqual(parse_line("DEAD A: held in reset; `release A` to try again").level, "crit")


class OnARealRecording(unittest.TestCase):
    def test_every_console_line_of_the_recorded_session_is_understood(self):
        import gzip
        import json
        side = ROOT / "console" / "demo" / "launch-and-node-loss.side.jsonl.gz"
        kinds = {}
        with gzip.open(side, "rt") as fh:
            for line in fh:
                r = json.loads(line)
                if r["k"] != "line":
                    continue
                p = parse_line(r["text"])
                self.assertIsNotNone(p, r)
                kinds[p.kind] = kinds.get(p.kind, 0) + 1
        for k in ("status", "act-status", "countdown", "t-zero", "latched-out", "mode-change", "ground-command", "act-excluded"):
            self.assertIn(k, kinds, f"the recording has no {k} line")
        self.assertLess(kinds.get("text", 0), 12, kinds)


if __name__ == "__main__":
    unittest.main()
