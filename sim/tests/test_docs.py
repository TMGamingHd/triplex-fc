# SPDX-License-Identifier: MIT
"""The README must document every input the tools accept. Fails when a flag, subcommand,
fault kind or fault option is added or renamed without updating sim/README.md."""
import argparse
import re
import unittest
from pathlib import Path

from tfc_peers.cli import build_parser
from tfc_peers.faults import KINDS

ROOT = Path(__file__).resolve().parents[2]
README = (ROOT / "sim" / "README.md").read_text()


def subparsers():
    action = next(a for a in build_parser()._actions if isinstance(a, argparse._SubParsersAction))
    return action.choices


class ReadmeCoversEveryInput(unittest.TestCase):
    def test_every_subcommand_is_documented(self):
        for name in subparsers():
            self.assertIn(f"python3 -m tfc_peers {name}", README, f"subcommand {name!r} missing from sim/README.md")

    def test_every_cli_flag_is_documented(self):
        for name, parser in subparsers().items():
            for action in parser._actions:
                for opt in action.option_strings:
                    if opt in ("-h", "--help"):
                        continue
                    self.assertIn(f"`{opt}", README, f"flag {opt} of `{name}` missing from sim/README.md")
                if not action.option_strings and action.dest not in ("help",):
                    self.assertIn(action.dest.upper(), README, f"argument {action.dest} of `{name}` undocumented")

    def test_every_fault_kind_and_option_is_documented(self):
        for kind, (row, _desc, params) in KINDS.items():
            self.assertIn(f"| `{kind}` | {row} |", README, f"fault kind {kind!r} missing from the fault table")
            for key in params:
                line = next(ln for ln in README.splitlines() if ln.startswith(f"| `{kind}` |"))
                self.assertIn(f"`{key}`", line, f"option {key!r} of fault {kind!r} missing from its table row")
        for common in ("start=N", "end=N"):
            self.assertIn(common, README)

    def test_every_replay_flag_is_documented(self):
        src = (ROOT / "tools" / "replay" / "replay.cpp").read_text()
        for flag in sorted(set(re.findall(r'"(--[a-z0-9-]+)"', src))):
            self.assertIn(flag, README, f"tfc_replay flag {flag} missing from sim/README.md")

    def test_every_replay_counter_is_documented(self):
        src = (ROOT / "tools" / "replay" / "replay.cpp").read_text()
        for key in re.findall(r'\{"([a-z_]+)", c\.', src):
            self.assertIn(f"`{key}`", README, f"counter {key} missing from sim/README.md")


if __name__ == "__main__":
    unittest.main()
