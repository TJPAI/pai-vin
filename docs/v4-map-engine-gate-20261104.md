# PAI-VIN V4 — persistent visual map engineering gate (2026-11-04)

## Outcome
Do **not** label AlvaAR tracking as persistent visual-map localization. Native iPhone browser WASM 6DoF was observed, but poses are arbitrary scale; native resets change the coordinate frame; no serialization or map reload API exists in the pinned release.

### Audited source: AlvaAR 7796af500ee92001ac2a9888363ff64d7a3bee75 (GPL-3.0)
- `src/slam/src/map_manager.hpp`: `mapKeyframes_`, `mapMapPoints_`, `pointCloud_` (only points are insufficient).
- `src/slam/src/frame.hpp`: `Twc_`, `Tcw_`, `mapKeypoints_`, co-visibility, camera calibration.
- `src/slam/src/map_point.hpp`: 3D points, observation keyframe IDs, descriptor(s), inverse depth, anchor keyframe.
- `src/slam/src/system.hpp`, `src/slam/src/embind.cpp`: only configure, reset, findCameraPose, findCameraPoseWithIMU, findPlane, getFramePoints. **No save/load/relocalize.**
- `src/slam/src/visual_frontend.cpp`: initialization requires >=50 tracked 2D keypoints and sufficient parallax. Native reset observed.
- New full persistent map support therefore requires C++ serializer, graph reconstruction, index/descriptor restoration, explicit *localization mode*, camera-model handling, and Emscripten rebuild. File export alone cannot restore location.
- User-facing AlvaAR license GPLv3; transitive components must be inventoried.

### Alternative: stella_vslam
- Upstream: https://github.com/stella-cv/stella_vslam
- README explicitly states saved maps can be loaded and new images localized.
- Includes C++ and native dependencies (OpenCV, optimization, FBoW, etc.); **no Safari WASM build or iPhone measurements verified**.
- Before an expensive port: GitHub Actions (Ubuntu) dependency configure; isolate filesystem, threads, memory, exception and module-build restrictions; produce a standalone WASM load and camera-frame processing test.
- Keep GPL AlvaAR validation separate from a potentially BSD-based stella port.

### Acceptance gate for selecting engine
1. Native module returns distinct **map session ID and frame pose ID**; resets invalidate previous coordinate frame.
2. Map export records intrinsic model, keyframes, 3D landmarks, observations, descriptors, map/index version; encrypted/optional visual assets are separately governed.
3. Saved artifact survives complete browser close and offline reload; corrupt/incompatible artifacts are rejected.
4. Same mapped place is recognized in new session using **2D–3D PnP geometry**, and camera pose has the **same stored map frame**. A tracking-only pose is not relocalization.
5. Scale remains explicitly **unverified** until A4 geometry is linked into optimized map and checked independently; no user-visible meters otherwise.
6. Final iPhone measurements: P50/P95 latency, coverage, reinit, bad-map false acceptance, offline persistence, daylight/night queries.

## Engineering decision
Keep AlvaAR as the browser 6DoF feasibility/reference engine. Before modifying its C++, perform the minimal stella WASM dependency audit. The map protocol below is intentionally implementation-independent; schema validation does **not** mean an engine can populate it.

## Current PAI-VIN Map V1 status
The `v4-map-contract.js` validator enforces a conservative serialization envelope. **This is protocol scaffolding, not a map exporter, importable SLAM state or relocalizer.**
