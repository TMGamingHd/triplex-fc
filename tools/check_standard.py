# SPDX-License-Identifier: MIT
"""Mechanical checks of the flight-code rules in docs/verification/CODING_STANDARD.md that a compiler does not enforce.

    python3 tools/check_standard.py            # checks core/include and supervisor/include, exits 1 on a violation

Scans the flight core with comments and string literals removed, so a rule word in a comment is not a violation.
The deeper rules (function size, complexity, warnings, analysis) are enforced by clang-tidy and the strict build.
"""
from __future__ import annotations

import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "core" / "include"
ROOTS = [CORE, ROOT / "supervisor" / "include"]  # the supervisor follows the same mechanical rules (it shares no code with core/, TFC-SUP-001)

# rule -> (regex over code, why)
FORBIDDEN = {
    "P10-1 no goto/setjmp": (r"\b(goto|setjmp|longjmp)\b", "unstructured control flow"),
    "P10-2 no while/do loops": (r"\b(while|do)\b\s*[({]", "every loop in the core is a `for` over a compile-time bound"),
    "P10-3 no heap": (r"\b(malloc|calloc|realloc|free|new|delete)\b|\bstd::(vector|string|map|set|list|deque|unordered_\w+|unique_ptr|shared_ptr|function|any)\b",
                      "no dynamic memory, no allocating containers"),
    "NO exceptions": (r"\b(throw|try|catch)\b|\bstd::(exception|runtime_error|bad_\w+)\b", "exceptions are disabled in flight code"),
    "NO RTTI/virtual": (r"\b(dynamic_cast|typeid|virtual)\b", "no run-time type information or dynamic dispatch"),
    "P10-8 no macros": (r"^\s*#\s*(define|undef|if|ifdef|ifndef|elif)\b", "only #include and #pragma once"),
    "NO I/O in the core": (r"\b(printf|fprintf|puts|cout|cerr|std::cin|fopen|scanf)\b|#\s*include\s*<(iostream|cstdio|stdio.h|fstream)>", "the core never prints or touches files"),
    "NO function pointers/casts": (r"\breinterpret_cast\b|\bconst_cast\b|\(\*\s*\w+\)\s*\(", "no pointer reinterpretation, no function pointers"),
    "NO fast-math/volatile tricks": (r"\b(volatile|__attribute__|asm|__asm__)\b", "the core is portable C++, no inline assembly or volatile"),
}


def strip(src: str) -> str:
    """Remove comments and string/char literals, keeping line structure."""
    src = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), src, flags=re.S)
    src = re.sub(r"//[^\n]*", "", src)
    src = re.sub(r'"(?:\\.|[^"\\])*"', '""', src)
    src = re.sub(r"'(?:\\.|[^'\\])'", "''", src)
    return src


def global_state() -> list[str]:
    """P10-6: the core keeps all state in objects. Compile it and ask the object file for writable data symbols."""
    with tempfile.TemporaryDirectory() as d:
        obj = Path(d) / "strict_check.o"
        subprocess.run(["g++", "-std=c++17", "-O2", "-fno-exceptions", "-fno-rtti", f"-I{CORE}", "-c", str(ROOT / "core/check/strict_check.cpp"),
                        "-o", str(obj)], check=True)
        out = subprocess.run(["nm", "-C", str(obj)], check=True, capture_output=True, text=True).stdout
    # b/B = zero-initialised and d/D = initialised writable data; anything of ours there is a global variable
    return [ln for ln in out.splitlines() if re.match(r"^[0-9a-f]+ [bBdD] ", ln) and "tfc" in ln]


def main() -> int:
    bad = 0
    for sym in global_state():
        print(f"P10-6 no mutable globals: writable data symbol in the core object: {sym}")
        bad += 1
    headers = sorted(h for root in ROOTS for h in root.rglob("*.hpp"))
    for path in headers:
        code = strip(path.read_text())
        lines = code.splitlines()
        for rule, (pattern, why) in FORBIDDEN.items():
            for m in re.finditer(pattern, code, re.M):
                line = code.count("\n", 0, m.start()) + 1
                text = lines[line - 1].strip()
                print(f"{path.relative_to(ROOT)}:{line}: {rule}: {text}   [{why}]")
                bad += 1
    if bad:
        print(f"\n{bad} violation(s) of docs/verification/CODING_STANDARD.md", file=sys.stderr)
        return 1
    print(f"ok: {len(headers)} flight-code headers satisfy the mechanical rules")
    return 0


if __name__ == "__main__":
    sys.exit(main())
