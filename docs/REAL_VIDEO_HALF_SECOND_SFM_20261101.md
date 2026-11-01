# 0.5-second incremental SfM experiment · 2026-11-01

Private source data: mapping 42.03 s and localization 33.43 s, 512x910, 30fps iPhone MP4. Raw footage is **not** in the public repository.

## Reproduction context

OpenCV sparse feature matching / two-view initialization / incremental PnP + triangulation. Original 1.5-second candidate-frame schedule was modified to 0.5-second stepping, processing 10.0–41.5 s then 9.0–0.5 s around the established 9.5 s/12.5 s seed pair. Intrinsics remain **assumed** (fx=fy=650 px; cx=256, cy=455), with baseline arbitrarily normalized to 1.

| Actual measure | Prior 1.5s schedule | New 0.5s schedule |
|---|---:|---:|
| Unique registered mapping frame timestamps | 4 | **13** |
| Generated local 3D candidate points | 583 | **2,467** |
| Accepted localization sampled frames | 4/22 | **4/22** |

Registered mapping timestamps: 9.0, 9.5, 10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 40.5 seconds. The isolated 40.5-second result may be a revisited view or a visual alias and **must not** be described as verified loop closure. Continuous mapping is only supported over 9.0–14.5 seconds, not the entire video.

Query PnP sampled timestamps accepted: 2.0, 3.5, 5.0, and 32.0 seconds. No improvement in accepted coverage despite a substantially larger candidate point set.

## Scientific limits and decision

This is NOT an optimized full SLAM result: no true multi-keyframe feature-track fusion, local/global BA, loop optimization, calibrated intrinsics, metric scale, independent ground truth or verified 0.5 m error. Points may include duplicate landmarks and outliers.

**Decision**: stop iterating on this ad hoc local triangulation engine. Use real COLMAP/AirSLAM/SLAM-MER backends with proper feature tracks and BA. A COLMAP Ubuntu install smoke workflow is in `.github/workflows/colmap-smoke.yml`, but successful execution has not yet been verified. Raw private videos must not be committed to public GitHub.

Detailed local report, source script and candidate PLY point cloud were delivered as conversation artifacts.
