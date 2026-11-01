# PAI-VIN Multi-frame Monocular Geometry · 2026-11-01

## Real inputs
- 42.03 s iPhone portrait 512×910 mapping video (private).
- 33.43 s iPhone portrait 512×910 localization video (private).

## Actual experiment
Offline OpenCV ORB descriptors + ratio test, essential matrix/recoverPose, triangulation with positive-depth/parallax/reprojection gates, incremental PnP/RANSAC and landmark triangulation. Intrinsics **assumed** (fx=fy=650, cx=256, cy=455); no trustworthy physical units. Map baseline initialized to arbitrary 1.

| Measure | Result |
|---|---|
| Successfully registered mapping keyframes | **4**: 9.5, 11, 12.5, 14 s |
| Candidate 3D landmarks | **583** |
| 2nd-video sampled frames | **22** |
| 2nd-video PnP-supported sample frames | **4**: 2.0, 3.5, 5.0, 32.0 s |
| 2D–3D RANSAC inliers on those frames | 303, 217, 61, 22 |
| Remaining query samples without accepted pose | **18 of 22** |

**This is an exploratory sparse local multi-view reconstruction, not complete SLAM.** No full video tracking, no loop-closure optimization, no global bundle adjustment, no camera calibration, no physical scale from A4 board, and no absolute position error measurement. A PnP-supported pose is not proof of an accurate pose; repetitive indoor patterns can create aliases.

## Diagnosis and next experiments
1. Frame spacing was 1.5 s; after 14 s, the local map frequently lacks enough 2D–3D correspondences to continue. Switch to denser sequential tracking and adaptive keyframes (parallax / survival criteria) instead of attempting distant relocalization every 1.5 s.
2. Calibrate iPhone camera intrinsics at actual recorded 512×910 orientation, and handle crop, stabilization and rolling shutter.
3. Compare a real full SfM/SLAM backend (COLMAP, AirSLAM, SLAM-MER), enable local/global BA and verify loop closure.
4. Prefer printed corner board and independent scale checks before metric claims.
5. Hold out the second video for relocalization evaluation with failure rate and independently measured 2D position errors.

The exact Python experiment, PLY point cloud, report, and preview were delivered as conversation artifacts. Raw videos were **not** committed to this public GitHub repo.
