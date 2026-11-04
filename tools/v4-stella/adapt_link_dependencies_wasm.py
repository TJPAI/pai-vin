#!/usr/bin/env python3
"""Adapt real (not stubbed) Stella OpenMP/yaml-cpp WASM link dependencies.

Pinned upstream stella_vslam e445b5452535e781fdf6777ec33a06f5f8f5e416.
The Emscripten build disables OpenMP; upstream's imported fallback accidentally
links an unresolved OpenMP_CXX_FLAGS (NOTFOUND). Keep Threads::Threads only.
Resolve yaml-cpp to the already built genuine WASM static archive.
"""
from pathlib import Path
import argparse
import subprocess

PIN = "e445b5452535e781fdf6777ec33a06f5f8f5e416"

def adapt(root: Path, prefix: Path, check=False):
    head = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if head != PIN:
        raise RuntimeError(f"Stella commit mismatch: {head}")
    yaml = prefix / "lib/libyaml-cpp.a"
    if not yaml.is_file() or yaml.stat().st_size == 0:
        raise RuntimeError(f"Real WASM yaml-cpp archive missing: {yaml}")
    replacements = {
        root / "CMakeLists.txt": [(
            "PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads",
            "PROPERTY INTERFACE_LINK_LIBRARIES Threads::Threads",
        )],
        root / "src/stella_vslam/CMakeLists.txt": [(
            "                      yaml-cpp\n",
            f"                      {yaml}\n",
        )],
    }
    pending = {}
    for path, changes in replacements.items():
        source = path.read_text()
        modified = source
        for before, after in changes:
            n = modified.count(before)
            if n != 1:
                raise RuntimeError(f"Expected one target snippet in {path}, found {n}: {before!r}")
            modified = modified.replace(before, after, 1)
        pending[path] = modified
    if not check:
        for path, modified in pending.items():
            path.write_text(modified)
    print("PAI_STELLA_REAL_WASM_LINK_DEPS_ADAPTED" if not check else "PAI_STELLA_REAL_WASM_LINK_DEPS_CHECK_PASS")
    print("No dummy libraries added. Node/Safari execution not verified.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("upstream_root", type=Path)
    parser.add_argument("wasm_prefix", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    adapt(args.upstream_root.resolve(), args.wasm_prefix.resolve(), args.check)
