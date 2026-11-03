#!/usr/bin/env python3
"""Inspect OpenCV's installed CMake target exports, no fake libraries."""
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1]).resolve()
out = pathlib.Path(sys.argv[2])
cmakes = list((root / "lib/cmake/opencv4").glob("OpenCVModules*.cmake"))
assert cmakes, "OpenCV CMake targets missing"
missing = set()
for item in cmakes:
    text = item.read_text(encoding="utf-8")
    for relative in re.findall(r'\$\{_IMPORT_PREFIX\}(/[^"\n; ]+\.a)', text):
        if not (root / relative.lstrip("/")).exists():
            missing.add(relative)
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text("\n".join(sorted(missing)) + ("\n" if missing else ""), encoding="utf-8")
print("Missing OpenCV exported static archives:", sorted(missing))
