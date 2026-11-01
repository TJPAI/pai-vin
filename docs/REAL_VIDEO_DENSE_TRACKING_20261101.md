# Real video dense tracking experiment — 2026-11-01

## Inputs
Two private iPhone recordings: mapping 42.03 s; independent localization 33.43 s, both 512×910, 30 fps. Raw video files have not been committed.

## Measured diagnostic results
| Method | Mapping adjacent-pair support | Localization adjacent-pair support |
|---|---:|---:|
| ORB+essential matrix at ~3 Hz | 91/126 = 72.2% | 63/100 = 63.0% |
| Lucas–Kanade flow + forward/backward consistency + essential matrix at 10 Hz | **354/420 = 84.3%** | **265/334 = 79.3%** |

3 Hz gate: at least 40 ratio-test matches and 25 recoverPose inliers. 10 Hz gate: at least 70 tracked correspondences and 30 essential-matrix RANSAC inliers. These are **heuristic diagnostic gates**, not measured 6DoF tracking accuracy.

Mapping has weak geometric support around 17.5–19s; localization around 8.4–9.3s, among other gaps.

## Important limitations
- Intrinsics used for geometric tests are **assumed** (fx=fy=650 px, cx=256, cy=455); not calibrated.
- No continuous 42-second SLAM trajectory, loop closure or global BA demonstrated.
- No metric scale recovery, 0.5-meter positioning verification, or full 3D map-based second-run localization.
- High RANSAC inlier counts can result from rotation or degenerate scene geometry, and should not be interpreted as accurate translation.

A detailed downloadable experiment archive (Python scripts, time-series JSON and plot) was delivered in the conversation. The next engineering milestone is calibrated full-sequence SfM/SLAM with an actual backend, followed by held-out 2D–3D localization.
