# PAI-VIN V3.5 — 2026-11-03

Active browser build contains the one-photo four-fiducial A4 detector, a stable-ID sparse feature tracker, browser camera/IMU stream, stationary gyro bias/gravity initialization, and gyro-only orientation propagation.

New V3.5 modules: `vio-rotation.js` estimates robust relative CAMERA rotation from matched stable-ID features when validated camera intrinsics are provided; `vio-fusion.js` conditionally corrects the gyro attitude using this relative rotation only when all calibration flags (intrinsics, camera–IMU extrinsics, temporal offset) are verified. Synthetic geometry, corruption/rejection and calibrated orientation correction tests run in Pages CI.

The live Safari app has **NOT enabled visual IMU correction**: intrinsics/extrinsics/time offset have not been measured on the target iPhone. The new modules are shipped and tested but not connected to live pose output. No complete VIO state estimator or metric trajectory exists.

Next: obtain defensible camera intrinsics/extrinsics/time offset on-device; then apply visual observation corrections to an IMU state with covariance, initialize translation and local 3D features, then mapping / loop closure. Never draw arbitrary normalised-step or metric positions prior to successful validation.

Historical modules remain archived under `archive/legacy-v3-1-experiments-20261103`.
