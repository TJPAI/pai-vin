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
    # Stella monocular perspective initializer normally launches two std::threads
    # for homography/fundamental RANSAC. Non-pthread Emscripten aborts on frame 2.
    # Preserve the same algorithms and RANSAC parameters, execute sequentially.
    files["src/stella_vslam/initialize/perspective.cc"] = [(
        """    std::thread thread_for_H(&solve::homography_solver::find_via_ransac, &homography_solver, num_ransac_iters_, false);
    std::thread thread_for_F(&solve::fundamental_solver::find_via_ransac, &fundamental_solver, num_ransac_iters_, false);
    thread_for_H.join();
    thread_for_F.join();""",
        """#ifdef __EMSCRIPTEN__
    homography_solver.find_via_ransac(num_ransac_iters_, false);
    fundamental_solver.find_via_ransac(num_ransac_iters_, false);
#else
    std::thread thread_for_H(&solve::homography_solver::find_via_ransac, &homography_solver, num_ransac_iters_, false);
    std::thread thread_for_F(&solve::fundamental_solver::find_via_ransac, &fundamental_solver, num_ransac_iters_, false);
    thread_for_H.join();
    thread_for_F.join();
#endif"""
    )]
    # Instrument only the WASM clone for actionable native initialization
    # diagnostics: match counts and geometric reconstruction outcomes.
    files["src/stella_vslam/module/initializer.cc"] = [
        (
            "    if (num_matches < min_num_valid_pts_) {",
            """#ifdef __EMSCRIPTEN__
    spdlog::info("PAI_STELLA_INIT_MATCHES frame={} matches={} min_required={}",
                 curr_frm.id_, num_matches, min_num_valid_pts_);
#endif
    if (num_matches < min_num_valid_pts_) {"""
        ),
        (
            "    return initializer_->initialize(curr_frm, init_matches_);",
            """    const bool initialized = initializer_->initialize(curr_frm, init_matches_);
#ifdef __EMSCRIPTEN__
    spdlog::info("PAI_STELLA_INIT_GEOMETRY frame={} result={}", curr_frm.id_, initialized ? 1 : 0);
#endif
    return initialized;"""
        )
    ]
    # Diagnose WHY a geometrically valid-looking match set is rejected.
    # Diagnostic-only logs, never alter thresholds or fabricate geometry.
    files["src/stella_vslam/initialize/base.cc"] = [
        ("#include \"stella_vslam/solve/triangulator.h\"",
         "#include \"stella_vslam/solve/triangulator.h\"\n#include <spdlog/spdlog.h>"),
        ("    if (*max_num_valid_pts_iter < min_num_valid_pts_) {",
         """#ifdef __EMSCRIPTEN__
    spdlog::info("PAI_STELLA_POSE_HYPOTHESES max_valid={} min_valid={} similar={} parallax_cos={} parallax_thr={} triangulated={} min_triangulated={}",
                 *max_num_valid_pts_iter, min_num_valid_pts_,
                 std::count_if(nums_valid_pts.begin(), nums_valid_pts.end(),
                     [max_num_valid_pts_iter](unsigned int v){ return 0.8 * (*max_num_valid_pts_iter) < v; }),
                 init_parallax.at(max_num_valid_index),
                 std::cos(parallax_deg_thr_ / 180.0 * M_PI),
                 num_triangulated_pts.at(max_num_valid_index), min_num_triangulated_);
#endif
    if (*max_num_valid_pts_iter < min_num_valid_pts_) {""")
    ]
    # Upstream automatic relocalization is gated on bow_db_. Browser tests
    # currently load no BoW vocabulary, so a turn-induced Lost state can
    # otherwise never recover. Reuse REAL upstream robust descriptor matching,
    # PnP/RANSAC, pose and local-map refinement against bounded recent keyframes.
    # No dead reckoning and no fabricated pose are used in this branch.
    files["src/stella_vslam/tracking_module.cc"].extend([
        (
            '#include "stella_vslam/data/landmark.h"',
            '#include "stella_vslam/data/landmark.h"\n#include "stella_vslam/data/keyframe.h"'
        ),
        (
            """    // update the local map and optimize current camera pose
    unsigned int fixed_keyframe_id_threshold = map_db_->get_fixed_keyframe_id_threshold();""",
            """#ifdef __EMSCRIPTEN__
    else if (!bow_db_ && enable_auto_relocalization_) {
        auto candidates = map_db_->get_all_keyframes();
        std::sort(candidates.begin(), candidates.end(),
                  [](const std::shared_ptr<data::keyframe>& a,
                     const std::shared_ptr<data::keyframe>& b) {
                      return a->id_ > b->id_;
                  });
        if (candidates.size() > 12) candidates.resize(12);
        // Use Stella's REAL robust feature matches and PnP pose solution.
        succeeded = relocalizer_.reloc_by_candidates(curr_frm_, candidates, true);
        spdlog::info("PAI_STELLA_WASM_RELOC frame={} candidates={} recovered={}",
                     curr_frm_.id_, candidates.size(), succeeded ? 1 : 0);
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
            last_reloc_frm_timestamp_ = curr_frm_.timestamp_;
        }
    }
#endif

    // update the local map and optimize current camera pose
    unsigned int fixed_keyframe_id_threshold = map_db_->get_fixed_keyframe_id_threshold();"""
        )
    ])
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
