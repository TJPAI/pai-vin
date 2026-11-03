# PAI-VIN V4: stella_vslam iPhone Safari WASM feasibility experiment

- Upstream pinned: https://github.com/stella-cv/stella_vslam/commit/e445b5452535e781fdf6777ec33a06f5f8f5e416
- Status at creation: **not compiled for WASM**. Do not call this port complete.
- Workflow: `.github/workflows/v4-stella-wasm-probe.yml`, separate from production Pages deployment, auto-runs on probe changes.
- Emscripten 3.1.74 smoke builds a **trivial C executable** to wasm + Node; this tests infrastructure only, not the actual SLAM code.
- Then a real `emcmake cmake` configure of the upstream code is executed without supplying fake libraries. Exit code and log become uploaded Actions artifacts.
- Upstream base `CMakeLists.txt` has hard requirements `find_package(Threads REQUIRED)`, `find_package(OpenMP REQUIRED)`, Eigen3, yaml-cpp, OpenCV core/imgcodecs/videoio/features2d/calib3d/highgui. Dependencies/optimizers/FBoW need further review.
- A configuration failure is expected to reveal initial blockers. This is a diagnostic run, **not a successful port**, nor a guarantee that a compatible mobile Safari build will be achieved.
- Proceed with a dependency-reduced modular wasm port only if the blockers can be removed with bounded effort; otherwise reevaluate alternative portable localization approaches.

## Decision boundary
AlvaAR can output 6DoF in Safari but does not have persistence/relocalization and has recurrent tracking resets. Avoid pretending its per-segment arbitrary-scale trajectory is metric.
