# PAI-VIN

iPhone camera + motion sensor PWA feasibility demo.

## Status

V0.1 is a real device testing interface, **not** verified SLAM or meter-level positioning.

- Camera permission and live capture
- Motion sensor permission and simple step threshold
- 2D relative track based on *unvalidated* constant step length and approximate gyro integration
- Simple 8x8 brightness image descriptor; matched visual anchors are **not** 3D relocalization
- Local export and service-worker cache

## Test

Open the GitHub Pages HTTPS URL on an iPhone Safari, select Enable IMU, then Open Camera. Save an anchor and return to the same viewpoint for a crude image comparison. Pocket mode closes video, but Safari must remain active. Use airplane mode to test cached shell after successful initial visit.

## Critical limitations

No native Core Motion pressure/magnetic field access, no 3D SLAM, no validated global map, no path planning, no true coordinate or reliability guarantee. Do not demonstrate simulated tracks as real tracks. No photos are uploaded by this app.

## Target

A more complete offline visual-inertial prototype for the November 7, 2026 demonstration.
