# PAI-VIN V3.0: iPhone Safari VIO/SLAM Decision

Status 2026-11-03: sensor interface and inertial rotation integration developed; complete VIO/SLAM not yet implemented.

## Acceptance goal
A4 board calibration, one loop indoor mapping on iPhone, verified metric sparse map projected to 2D floor XY; later restart Safari, reopen camera, relocalize into saved map and draw a truthful live walking trajectory. Handle stationary operation, turning, loops and tracking loss.

## Real mobile failure evidence
User exported a V2.7 diagnostic: 104 checks; 14 accepted, 90 rejected. Breakdown: 47 few_correspondences, 30 epipolar_unreliable, 7 insufficient_baseline, 4 pure_rotation_or_no_parallax, 2 accumulating_baseline. Approximately 115 features detected per frame, but median only 16.5 valid descriptor correspondences. A normalized-step trajectory based on isolated two-frame pose estimation has no stable metric meaning. Retire this estimator as a product source; retain as explicitly labeled legacy diagnostic only.

## VIO engine design
M0 (currently coded): monotonic browser timestamps for device motion and camera observations; synchronized windows, device gyroscope rotation preintegration, sampling diagnostics. Browser event timestamps are approximate receipt times, not exposure times.
M1: native C++/WASM long-lived KLT tracks, robust geometric outlier handling and stable feature IDs. Needs independent test against Nov 3 private source footage.
M2: initialize gyroscope/accelerometer biases, gravity and camera–IMU extrinsics/time offset; run EKF/MSCKF or sliding-window VIO estimating orientation, position, velocity, IMU bias and covariance. Test on target iPhone Safari.
M3: sparse triangulated map, visual reprojection residual minimization/local BA, A4 scale constraint and map/trajectory serialization.
M4: candidate retrieval and geometric 2D-to-3D PnP/RANSAC, optimized loop closure and saved-map relocalization across different app sessions.
M5: verify stationary drift, turn behavior, loop closure, ATE and RPE against independently measured ground truth. Only then discuss the 0.5m accuracy target.

## Browser limitations and safeguards
Safari device motion provides browser-transformed IMU measurements; physical sensor axis calibration, camera-IMU extrinsics and capture-time offset still need verification. A4 plane homography alone does not calibrate full camera intrinsics or recover global SLAM scale. WASM and memory/performance require actual iPhone tests. Recording a rotation-only IMU delta does not constitute visual-inertial state fusion. Do not fabricate map position while VIO state is unavailable.

## Candidate references
OpenVINS: https://docs.openvins.com/ ; project https://github.com/rpng/open_vins
ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
Review licenses, particularly GPL, before code integration. No claim that any native engine is already ported to Safari.
