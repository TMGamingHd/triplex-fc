#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Record a release, so that it can be the golden one (ADR-021, TFC-ARCH-005).

    python3 tools/release/record_release.py --tag golden-1 --elf build/nucleo_flight/zephyr/zephyr.elf \
        [--elf-rebuilt /tmp/other/zephyr.elf] --campaign logs/campaign.jsonl --out releases/golden-1.json [--allow-anomalies]

The golden release is "a tagged release that passed the full fault campaign and the hardware stage exit tests, built reproducibly with a recorded toolchain and hash, and changed afterwards only for a
safety fix". This writes the record that says so: the git commit and whether the tree was clean, the release id the image reports in its heartbeat, the compiler and the Zephyr and SDK versions, the
hash of the flash image (the loadable segments of the ELF, so that file paths in the debug information do not matter), whether a second build of the same commit gave the same image, and the campaign's
scenario count and anomaly count. It refuses to record a release whose campaign has anomalies unless told, and says so in the record when told. Exit status 0 written, 1 refused, 2 usage.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def image_hash(elf: Path) -> tuple[str, int]:
    """SHA-256 over the loadable segments of an ELF file (addresses and contents, in file order) and their total size: the image that is flashed, without paths and timestamps."""
    data = elf.read_bytes()
    if data[:4] != b"\x7fELF":
        raise ValueError(f"{elf} is not an ELF file")
    is64 = data[4] == 2
    little = data[5] == 1
    e = "<" if little else ">"
    if is64:
        phoff, = struct.unpack_from(e + "Q", data, 0x20)
        phentsize, phnum = struct.unpack_from(e + "HH", data, 0x36)
    else:
        phoff, = struct.unpack_from(e + "I", data, 0x1C)
        phentsize, phnum = struct.unpack_from(e + "HH", data, 0x2A)
    h = hashlib.sha256()
    total = 0
    for i in range(phnum):
        off = phoff + i * phentsize
        if is64:
            p_type, _flags, p_offset, _vaddr, p_paddr, p_filesz = struct.unpack_from(e + "IIQQQQ", data, off)
        else:
            p_type, p_offset, _vaddr, p_paddr, p_filesz = struct.unpack_from(e + "IIIII", data, off)
        if p_type == 1 and p_filesz:  # PT_LOAD
            h.update(struct.pack("<Q", p_paddr))
            h.update(data[p_offset:p_offset + p_filesz])
            total += p_filesz
    return h.hexdigest(), total


def campaign_summary(path: Path) -> dict:
    scenarios = anomalies = 0
    codes: dict[str, int] = {}
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        scenarios += 1
        for code, _detail in r.get("anomalies", []):
            anomalies += 1
            codes[code] = codes.get(code, 0) + 1
    return {"scenarios": scenarios, "anomalies": anomalies, "codes": codes}


def git(*args: str) -> str:
    return subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True).stdout.strip()


def tool_version(cmd: list[str]) -> str:
    try:
        return (subprocess.run(cmd, capture_output=True, text=True, timeout=20).stdout.strip().splitlines() or [""])[0]
    except (OSError, subprocess.SubprocessError):
        return ""


def build_record(tag: str, elf: Path, rebuilt: Path | None, campaign: Path, allow: bool) -> tuple[dict, list[str]]:
    problems: list[str] = []
    digest, size = image_hash(elf)
    rec = {
        "tag": tag,
        "commit": git("rev-parse", "HEAD"),
        "release_id": "0x" + git("rev-parse", "--short=4", "HEAD"),
        "tree_clean": git("status", "--porcelain") == "",
        "image_sha256": digest,
        "image_bytes": size,
        "compiler": tool_version(["arm-zephyr-eabi-g++", "--version"]) or tool_version(["g++", "--version"]),
        "west": tool_version(["west", "--version"]),
        "campaign": campaign_summary(campaign),
    }
    if not rec["tree_clean"]:
        problems.append("the working tree is not clean: a release is built from a commit")
    if rebuilt is not None:
        again, _ = image_hash(rebuilt)
        rec["reproducible"] = again == digest
        if again != digest:
            problems.append("a second build of the same commit gave a different flash image: the build is not reproducible")
    else:
        rec["reproducible"] = None
        problems.append("no second build was given (--elf-rebuilt): reproducibility is not shown")
    if rec["campaign"]["scenarios"] == 0:
        problems.append("the campaign file holds no scenarios")
    if rec["campaign"]["anomalies"]:
        problems.append(f"the campaign has {rec['campaign']['anomalies']} anomalies: {rec['campaign']['codes']}")
    rec["accepted_with_problems"] = bool(problems) and allow
    rec["problems"] = problems if allow else []
    return rec, problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="record_release", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--elf", required=True, type=Path)
    ap.add_argument("--elf-rebuilt", type=Path)
    ap.add_argument("--campaign", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--allow-anomalies", action="store_true", help="record the release although something above is wrong (the record says so)")
    args = ap.parse_args(argv)
    try:
        rec, problems = build_record(args.tag, args.elf, args.elf_rebuilt, args.campaign, args.allow_anomalies)
    except (OSError, ValueError) as e:
        print(f"record_release: {e}", file=sys.stderr)
        return 2
    if problems and not args.allow_anomalies:
        for p in problems:
            print(f"REFUSED: {p}", file=sys.stderr)
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(rec, indent=2) + "\n")
    print(f"recorded {args.tag} ({rec['release_id']}, image {rec['image_sha256'][:12]}, {rec['image_bytes']} bytes) in {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
