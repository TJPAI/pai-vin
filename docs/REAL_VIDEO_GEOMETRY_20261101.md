# First real-video geometry experiment · 2026-11-01

Inputs (private user recordings; **not committed**):
- mapping: 42.03 s, 512×910 portrait, 30 fps, H.264 MP4
- localization: 33.43 s, 512×910 portrait, 30 fps, H.264 MP4

This is an exploratory OpenCV **two-view reconstruction and localization feasibility** test, **not** a completed SLAM deployment.

### Observed results

| Test | Output |
|---|---|
| Mapping frames used for a candidate local 3D map | 9.5 s and 12.5 s |
| ORB descriptor tentative correspondences | 246 |
| Essential-matrix/pose inliers | 222 |
| Candidate triangulated 3D points passing depth/parallax filter | 222 |
| Median triangulation parallax | 6.08° |
| Localization frame 2.0 s: 2D–3D candidates / PnP RANSAC inliers | 156 / 154 |
| Localization frame 3.5 s | 154 / 143 |
| Localization frame 5.0 s | 32 / 26 |
| Localization frame 8.0 s | 6 / 0 (failure) |
| Localization frame 9.5 s | 12 / 7 (insufficient) |

Camera intrinsics were **assumed**, not calibrated: fx = fy = 650 px, cx = 256, cy = 455. The relative camera baseline was normalized to arbitrary unit length. Consequently the local point cloud is **not metrically scaled**.

**Not established:** calibrated SfM, continuous tracking over 42 s, map loop closure, global bundle adjustment, recovered A4 scale, standalone PnP at every frame, 0.5 m accuracy, or iPhone real-time performance. Some pairs yielded degenerate, excessive apparent parallax and were not selected.

### Recommended next gate
1. Calibrate actual iPhone capture intrinsics (and lens distortion) for the recording resolution.
2. Run an actual full multi-frame SfM/SLAM reconstruction (COLMAP / AirSLAM / SLAM-MER), with keyframes, BA and loop closure.
3. Extract and register the printed A4 board corners to solve a valid metric alignment.
4. Freeze map; test independent second-pass 2D–3D localization, reported with success rate, time, and surveyed position error.
5. Reject false relocalization hypotheses from repeated indoor textures.

**Dataset handling**: Do not commit indoor video footage to this public repository. The user retains the two private original files.
