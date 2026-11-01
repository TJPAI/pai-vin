#!/usr/bin/env python3
"""Offline parser regression test; no COLMAP binary or video data needed."""
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from run_colmap_baseline import registered_names

HEADER = '# Image list with two lines of data per image:\n'
POSE1 = '1 1 0 0 0 0 0 0 1 mapping/000000.jpg\n'
POSE2 = '2 1 0 0 0 0 0 0 1 localization/000000.jpg\n'
with tempfile.TemporaryDirectory() as tmp:
    folder = Path(tmp)
    (folder/'images.txt').write_text(HEADER + POSE1 + '\n' + POSE2 + '11 10 20 -1\n')
    assert registered_names(folder) == ['mapping/000000.jpg', 'localization/000000.jpg']
    (folder/'images.txt').write_text(HEADER + POSE1)
    try:
        registered_names(folder)
    except RuntimeError as err:
        assert 'odd data line' in str(err)
    else:
        raise AssertionError('Expected malformed two-line record to be rejected')
print('COLMAP images.txt parser: PASS')
