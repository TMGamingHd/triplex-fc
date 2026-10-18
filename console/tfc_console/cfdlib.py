# SPDX-License-Identifier: MIT
"""The flow fields the 3D viewer can show: the files `tfc_cfd --field` writes (docs/design/CFD.md section 7), in `console/cfd/<vehicle>/` (a library `tools/cfd/make_library.py` makes, not in the repository) and
`console/demo/` (one or two that are). The manifest is read from each file's first line, a JSON header; a file is served only by a name the manifest has."""
from __future__ import annotations

import json
from pathlib import Path

KEYS = ("vehicle", "stage_from", "mach", "alpha_deg", "ca_skin", "cn", "x_cp_m", "cells", "iterations", "residual_drop", "h_m", "d_ref_m")


class CfdLibrary:
    def __init__(self, *dirs: Path) -> None:
        self.dirs = [Path(d) for d in dirs]
        self._cache: dict[str, tuple[float, dict]] = {}

    def _files(self) -> list[tuple[str, Path]]:
        out = []
        for d in self.dirs:
            if not d.is_dir():
                continue
            for p in sorted(d.rglob("*.cfd")):
                if p.is_file():
                    out.append((p.relative_to(d.parent).as_posix() if d.name == "cfd" else "demo/" + p.name, p))
        return out

    def _header(self, name: str, p: Path) -> dict | None:
        m = p.stat().st_mtime
        c = self._cache.get(name)
        if c is not None and c[0] == m:
            return c[1]
        try:
            with p.open("rb") as f:
                h = json.loads(f.readline())
        except (OSError, ValueError):
            return None
        if not isinstance(h, dict) or h.get("k") != "cfd":
            return None
        self._cache[name] = (m, h)
        return h

    def manifest(self) -> dict:
        cases = []
        for name, p in self._files():
            h = self._header(name, p)
            if h is None:
                continue
            row = {"name": name, "bytes": p.stat().st_size}
            for k in KEYS:
                if k in h:
                    row[k] = h[k]
            cases.append(row)
        return {"present": bool(cases), "cases": cases}

    def file_bytes(self, name: str) -> tuple[bytes, str]:
        for n, p in self._files():
            if n == name and self._header(n, p) is not None:
                return p.read_bytes(), "application/octet-stream"
        raise ValueError(f"no flow field named {name!r} (console/cfd/ and console/demo/)")
