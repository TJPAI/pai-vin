# PAI-VIN V3.7 — 2026-11-03

V3.7 targets two P0 problems diagnosed from real iPhone V3.6 JSON: 16/194 visual rotation accepted, 106 insufficient tracks, 72 rotation geometry unreliable, and 0 visually accepted windows with a valid IMU bracket. Observed IMU rate ~51 Hz, camera reads ~9.93 Hz, and 360 duplicate/reordered receipt timestamps.

Implemented:
- `vio-sensor-core.js`: strict bracketing IMU samples and interpolation at start and end of frame observation window, reject large internal gaps, separately count duplicate and reordered event timestamps. These are browser receipt timestamps and **do not** provide physical camera exposure synchronization.
- `vio-sensor-probe.js`: defer vision/IMU comparison until the post-frame IMU event has arrived; export per-window reasons, count valid IMU windows and truly paired valid observations. Sensor capture listener avoids heavy visual processing.
- `vio-tracker.js`: shared three-level pyramid with a quarter-resolution search for larger inter-frame movement; retain backward matching consistency and fail-closed rejects.
- Regression tests: interpolated frame bracketing, missing first/last events and IMU data gaps; translated image tracking with larger displacement.

This release is NOT a working visual-inertial pose estimator; it does not claim measured phone performance improvement, corrected camera time offset, or metric SLAM. Next gate is a fresh iPhone diagnostic comparing genuinely paired sample counts, visual track lifespan, and CPU throughput before any full calibration.
