#!/usr/bin/env python3
"""Adapt pinned FBoW CPU feature probe for WebAssembly without pretending x86 CPUID works.

Target: stella-cv/FBoW submodule commit c6e3c29e3332a0b0834021797e2aa4e8eb66a3c1.
WASM reports no x86 SIMD features and uses genuine generic FBoW descriptor code.
"""
from pathlib import Path
import argparse
import subprocess

UPSTREAM_SHA = "c6e3c29e3332a0b0834021797e2aa4e8eb66a3c1"
OLD_POSITIVE = "defined(__ANDROID__) || defined(__arm64__)"
NEW_POSITIVE = "defined(__EMSCRIPTEN__) || defined(__ANDROID__) || defined(__arm64__)"
OLD_NEGATIVE = "!defined(__ANDROID__) && !defined(__arm64__)"
NEW_NEGATIVE = "!defined(__EMSCRIPTEN__) && !defined(__ANDROID__) && !defined(__arm64__)"

def apply(root: Path, check: bool = False) -> None:
    actual = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if actual != UPSTREAM_SHA:
        raise RuntimeError(f"Unrecognized FBoW revision {actual}; expected {UPSTREAM_SHA}")
    path = root / "include/fbow/cpu.h"
    source = path.read_text()
    if "__EMSCRIPTEN__" in source:
        raise RuntimeError("Unexpected existing Emscripten branch; refusing double adaptation")
    for old, expected in ((OLD_POSITIVE, 2), (OLD_NEGATIVE, 3)):
        count = source.count(old)
        if count != expected:
            raise RuntimeError(f"Expected {expected} occurrences of {old!r}; found {count}")
    updated = source.replace(OLD_POSITIVE, NEW_POSITIVE)
    updated = updated.replace(OLD_NEGATIVE, NEW_NEGATIVE)
    if not check:
        path.write_text(updated)
    print(f"FBOW_WASM_CPUID_{'CHECK' if check else 'PATCH'}_PASS: pinned source; generic CPU feature branch")
    print("FBoW numerical equivalence and iPhone runtime are NOT verified.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("fbow_root", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    apply(args.fbow_root.resolve(), args.check)
