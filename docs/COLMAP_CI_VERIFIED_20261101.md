# COLMAP CI smoke verification — 2026-11-01

Verified against **actual GitHub Actions job results**, not only workflow YAML:

- COLMAP environment smoke run: https://github.com/TJPAI/pai-vin/actions/runs/41288035222 — **completed, success**
- PAI-VIN Pages run: https://github.com/TJPAI/pai-vin/actions/runs/41288035213 — **completed, success**
- Ubuntu 24.04 job installed `colmap` version **3.9.1-2build2** from apt and `ffmpeg` successfully.
- Corrected a literal `\\n` embedded in `tools/run_colmap_baseline.py`, previously causing `SyntaxError` in both workflows.
- Successful smoke job produced artifact `colmap-smoke-help` (GitHub artifact ID 13075568610).

## Remaining blockers

The two real 42.03s mapping and 33.43s localization MP4s are privately attached to the ChatGPT conversation/working container, **not on the GitHub runner**. Smoke-job success cannot establish real 3D reconstruction. To run actual video in CI we must transfer video datasets to an authorized, access-controlled computation location, without publishing them in this public repository. The GitHub connector does not automatically transport local ChatGPT files to GitHub Actions.

Measured pose accuracy in meters also requires calibrated intrinsics, recovered scale, and independent surveyed ground truth. The A4 board geometry alone has not provided a measured scale yet.

## Next concrete milestone

Run `tools/run_colmap_baseline.py` with the real videos on a compute host that can access both MP4 files, inspect the COLMAP sparse model and registration coverage, then compare AirSLAM/SLAM-MER.
