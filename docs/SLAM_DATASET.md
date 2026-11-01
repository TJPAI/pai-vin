# PAI-VIN Visual SLAM dataset workflow

## Status
This repository currently has an iPhone capture frontend and a **video preprocessing utility**. It **does not** currently implement or run AirSLAM, SLAM-MER, triangulation, bundle adjustment, loop closure, or 3D map-based relocalization.

## Required inputs
1. **Mapping video**: continuously film A4 scale board at the start from multiple viewpoints, then walk a loop, returning to start.
2. **Localization video**: a separate pass along part of the same route, without requiring the board.
3. Camera calibration: image size, focal lengths, optical center, and distortion. MediaRecorder alone does not record a trustworthy calibration matrix; do not fabricate one.
4. The printed board geometry is defined in `a4-board.html` with the inner corners: (25,25), (185,25), (185,272), (25,272) millimeters. Printing at 100% must be verified.

## Local preprocessing
Install `ffmpeg` and `ffprobe` on a workstation. Keep the videos off the public GitHub repository and run:

```bash
python3 tools/prepare_slam_dataset.py \
  --mapping /path/to/mapping.mp4 \
  --localization /path/to/localization.mp4 \
  --out /private-data/pai-vin-test01 \
  --fps 10
```

Outputs:

- `mapping/images/*.jpg` and `mapping/timestamps.txt` (image sequence)
- `localization/images/*.jpg` and `localization/timestamps.txt`
- `preflight.json` (video metadata, warnings, number of extracted frames)

**Timestamps are nominal frame-sample times**, derived from the target sampling FPS. They are not original per-frame capture timestamps; clips with variable frame rate or discontinuities require proper timestamp handling before rigorous evaluation.

## Next integration gates

1. Determine actual camera intrinsics; inspect motion blur, feature overlap and loop coverage.
2. Run AirSLAM and SLAM-MER **separately** on the same mapping video. Save camera poses, keyframes, actual triangulated 3D points and loop closure statistics.
3. Detect board corners and solve calibrated board camera poses on selected mapping frames. Estimate a metric similarity alignment (Sim(3)) between board-derived poses and the SLAM trajectory, then validate residuals on withheld frames.
4. Freeze the map. Use only the **second** video for 2D–3D relocalization.
5. Evaluate horizontal position error against independently surveyed reference points. Target P95 <= 0.5 meters is a goal, **not** an achieved result.

## Data privacy and preservation

Raw videos can contain faces, signs or private interiors. Do not commit them to the public repository. Store them privately and remove unnecessary personal details before external sharing.
