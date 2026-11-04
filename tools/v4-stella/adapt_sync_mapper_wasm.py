#!/usr/bin/env python3
"""Pinned Stella WASM cooperative (single-thread) mapper adapter.

Stella's native mono initialization enqueues keyframes and blocks on a future
processed by mapping_module::run in a second std::thread. Browser builds here
have NO pthreads. This adapter advances the REAL map queue after every frame,
and avoids blocking on a non-existent worker; it does not synthesize poses.

Experimental: numerical correctness and Safari performance require validation.
Only touches ephemeral pinned upstream clone in GitHub Actions.
"""
from pathlib import Path
import argparse
import subprocess

UPSTREAM_SHA = "e445b5452535e781fdf6777ec33a06f5f8f5e416"

def main(root: Path, check: bool) -> None:
    head = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if head != UPSTREAM_SHA:
        raise RuntimeError(f"Unexpected Stella SHA {head}; required {UPSTREAM_SHA}")
    files = {
        "src/stella_vslam/mapping_module.h": [
            ("    //! Run main loop of the mapping module\n    void run();",
             "    //! Run main loop of the mapping module\n    void run();\n#ifdef __EMSCRIPTEN__\n    void process_pending_keyframes_sync();\n    void reset_sync();\n#endif")
        ],
        "src/stella_vslam/mapping_module.cc": [
            ("void mapping_module::run() {",
             """#ifdef __EMSCRIPTEN__
void mapping_module::process_pending_keyframes_sync() {
    while (keyframe_is_queued()) {
        mapping_with_new_keyframe();
    }
}
void mapping_module::reset_sync() {
    auto future = async_reset();
    reset();
    future.get();
}
#endif

void mapping_module::run() {"""),
            ('enable_interruption_of_landmark_generation_(yaml_node["enable_interruption_of_land_generation"].as<bool>(true)),', '')
        ],
        "src/stella_vslam/tracking_module.cc": [
            ("""    if (global_optimizer_) {
        auto future_mapper_reset = mapper_->async_reset();
        auto future_global_optimizer_reset = global_optimizer_->async_reset();
        future_mapper_reset.get();
        future_global_optimizer_reset.get();
    }
    else {
        auto future_mapper_reset = mapper_->async_reset();
        future_mapper_reset.get();
    }""",
             """#ifdef __EMSCRIPTEN__
    // Synchronous WASM build has no mapper thread to fulfill async_reset().
    mapper_->reset_sync();
#else
    if (global_optimizer_) {
        auto future_mapper_reset = mapper_->async_reset();
        auto future_global_optimizer_reset = global_optimizer_->async_reset();
        future_mapper_reset.get();
        future_global_optimizer_reset.get();
    }
    else {
        auto future_mapper_reset = mapper_->async_reset();
        future_mapper_reset.get();
    }
#endif"""),
            ("""        auto future = mapper_->async_add_keyframe(keyfrm);
        future.get();""",
             """#ifdef __EMSCRIPTEN__
        mapper_->async_add_keyframe(keyfrm);
#else
        auto future = mapper_->async_add_keyframe(keyfrm);
        future.get();
#endif""")
        ],
        "src/stella_vslam/system.cc": [
            ("    const auto cam_pose_wc = tracker_->feed_frame(frm);",
             """    const auto cam_pose_wc = tracker_->feed_frame(frm);
#ifdef __EMSCRIPTEN__
    // Consume REAL queued mapping work without depending on background threads.
    mapper_->process_pending_keyframes_sync();
#endif""")
        ]
    }
    # Turn off std::async landmark generation for WASM, not the actual
    # triangulation itself; this keeps all map creation/BA algorithms intact.
    files["src/stella_vslam/mapping_module.cc"][1] = (
        'enable_interruption_of_landmark_generation_(yaml_node["enable_interruption_of_landmark_generation"].as<bool>(true)),',
        'enable_interruption_of_landmark_generation_(false),'
    )
    changed = {}
    for name, replacements in files.items():
        path = root / name
        original = path.read_text()
        result = original
        for old, new in replacements:
            count = result.count(old)
            if count != 1:
                raise RuntimeError(f"{name}: expected exactly one match for {old[:80]!r}; saw {count}")
            result = result.replace(old, new, 1)
        changed[path] = result
    if not check:
        for path, result in changed.items():
            path.write_text(result)
    print("PAI_STELLA_SYNC_WASM_" + ("CHECK_PASS" if check else "PATCH_PASS"))
    print("Real map algorithm retained; no pose or trajectory simulation. Runtime not validated.")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("stella_root", type=Path)
    ap.add_argument("--check", action="store_true")
    ns = ap.parse_args()
    main(ns.stella_root.resolve(), ns.check)
