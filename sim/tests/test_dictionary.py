# SPDX-License-Identifier: MIT
"""The command and frame dictionary (core/dictionary/dictionary.json, TFC-IF-004) against the three places that define the same things: the C++ protocol header, the Python peers and the console."""
import json
import re
import unittest
from pathlib import Path

from tfc_peers import protocol as P

ROOT = Path(__file__).resolve().parents[2]
DICT = json.loads((ROOT / "core" / "dictionary" / "dictionary.json").read_text())
HEADER = (ROOT / "core" / "include" / "tfc" / "protocol.hpp").read_text()


def cpp_ground_ops():
    body = re.search(r"enum class GroundOp : uint8_t \{(.*?)\};", HEADER, re.S).group(1)
    return {m.group(1): int(m.group(2)) for m in re.finditer(r"^\s*(\w+) = (\d+)", body, re.M)}


def kebab(name):
    return re.sub(r"(?<!^)([A-Z])", r"-\1", name).lower()


class Dictionary(unittest.TestCase):
    def test_every_ground_command_is_the_same_in_the_dictionary_the_cpp_and_the_python(self):
        cpp = {kebab(k): v for k, v in cpp_ground_ops().items()}
        d = {c["name"]: c["opcode"] for c in DICT["commands"]}
        self.assertEqual(d, cpp, "a command was added or renumbered in the C++ header and not in the dictionary (or the other way)")
        self.assertEqual(d, P.GROUND_OPS)
        self.assertEqual(DICT["arm_flag"], P.ARM_FLAG)
        self.assertEqual(DICT["phases"], list(P.PHASE_NAMES))

    def test_the_nodeless_commands_are_the_ones_whose_node_field_the_dictionary_says_is_ignored(self):
        ignored = {c["name"] for c in DICT["commands"] if c["node"] == "ignored"}
        self.assertEqual(ignored, set(P.NODELESS_OPS))

    def test_the_commands_that_always_need_an_arm_are_the_cpp_comments_that_say_so(self):
        always = {c["name"] for c in DICT["commands"] if c["arm"] == "always"}
        for name in always:
            line = next(l for l in HEADER.split("\n") if re.match(r"\s*" + "".join(w.capitalize() for w in name.split("-")) + r" = ", l))
            self.assertIn("arm", line.lower(), name)

    def test_every_frame_id_is_the_headers_and_every_header_family_is_in_the_dictionary(self):
        consts = {m.group(1): int(m.group(2), 16) for m in re.finditer(r"constexpr uint32_t (k\w+) = (0x[0-9A-Fa-f]+);", HEADER)}
        ids = {f["name"]: f["id"] for f in DICT["frames"]}
        for name, const in (("sync", "kSync"), ("gyro", "kGyroBase"), ("accel", "kAccelBase"), ("command", "kCmdBase"), ("propulsion", "kPropBase"), ("surface", "kSurfBase"), ("act_output", "kActOut"),
                            ("heartbeat", "kHeartbeat"), ("state_share", "kState"), ("resync", "kResync"), ("ground", "kGround")):
            self.assertEqual(ids[name], consts[const], name)
        # no two frames overlap
        spans = sorted((f["id"], f["id"] + f["count"] - 1, f["name"]) for f in DICT["frames"])
        for a, b in zip(spans, spans[1:]):
            self.assertLess(a[1], b[0], f"{a[2]} and {b[2]} overlap")

    def test_every_command_has_a_documented_opcode_arm_rule_and_requirements_that_exist(self):
        reqs = set(re.findall(r"^\| (TFC-[A-Z0-9]+-\d+) \|", (ROOT / "docs" / "verification" / "REQUIREMENTS.md").read_text(), re.M))
        for c in DICT["commands"]:
            self.assertTrue(c["doc"] and c["arm"] and c["node"], c["name"])
            for r in c["requirements"]:
                self.assertIn(r, reqs, f"{c['name']}: {r} is not a requirement")


if __name__ == "__main__":
    unittest.main()
