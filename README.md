# PAI-VIN V3.2 — iPhone Safari Visual-Inertial SLAM Research

**Current published entry:** https://tjpai.github.io/pai-vin/vin-app.html

The production-facing UI intentionally contains only:

1. **A4 physical plane reference**: capture printed four-marker A4 sheet; save four-corner plane homography (160 mm × 247 mm marker centers) in Safari. Not camera intrinsics or SLAM scale calibration.
2. **Real camera + IMU collection and persistent feature tracking**: device motion, monotonic reception timestamps, gyro delta rotation, tracked image features with stable IDs, timing/quality diagnostics. This is **not yet six-degree-of-freedom pose or fused VIO**.
3. **Mapping and localization status**: truthful pending stages only. No fabricated positions, no arbitrary-step trajectory plots or misleading synthetic floor plans.

## Status
- Implemented: A4 planar reference UI, PWA offline caching, camera/IMU acquisition, gyro rotation preintegration, sparse image feature tracking, diagnostics export, synthetic regression tests.
- **Not implemented:** calibrated camera–IMU time/extrinsic alignment, full VIO state estimator (position, velocity, gravity, biases and covariance), optimized sparse metric map, loop closure, cross-session re-localization, live metric 2D path.
- No navigation or 0.5 m precision claim.

## Test
On iPhone Safari, open the link above. Under section ① capture A4 and select four marker centers. Under section ② start VIO capture and inspect tracked-feature longevity, camera/IMU sample rates and timing coverage. Export sensor diagnostics if needed. Video and photos are not uploaded.

## Repository cleanup
The obsolete browser keyframe-path/normalized-step-VO demos, related scripts and tests were removed from main when V3.2 was introduced. A historical snapshot is retained in branch `archive/legacy-v3-1-experiments-20261103`. Legacy proof-of-concept UI must not be reintroduced as a substitute for an estimator. Original private video experiments are untouched.

See `docs/VIO_V3_ARCHITECTURE.md` for the engineering acceptance gates. Separate offline COLMAP tooling in `tools/` is not part of the published Safari application.
