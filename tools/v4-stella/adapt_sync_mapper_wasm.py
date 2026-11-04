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
    const auto queued_before = get_num_queued_keyframes();
    while (keyframe_is_queued()) {
        mapping_with_new_keyframe();
    }
    if (queued_before > 0) {
        spdlog::info("PAI_STELLA_SYNC_MAPPER queued={} keyframes={} landmarks={}",
                     queued_before, map_db_->get_num_keyframes(),
                     map_db_->get_num_landmarks());
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
        // First, a genuine robust match to the last confirmed tracking frame.
        // Fall back to multi-keyframe PnP only if this fails.
        if (last_frm_.pose_is_valid() && curr_frm_.ref_keyfrm_
            && curr_frm_.timestamp_ - last_frm_.timestamp_ < 1.5) {
            twist_is_valid_ = false;
            succeeded = frame_tracker_.robust_match_based_track(
                curr_frm_, last_frm_, curr_frm_.ref_keyfrm_);
            spdlog::info("PAI_STELLA_TEMPORAL_RECOVER frame={} success={}",
                         curr_frm_.id_, succeeded ? 1 : 0);
        }
        if (!succeeded) {
        auto candidates = map_db_->get_all_keyframes();
        std::sort(candidates.begin(), candidates.end(),
                  [](const std::shared_ptr<data::keyframe>& a,
                     const std::shared_ptr<data::keyframe>& b) {
                      return a->id_ > b->id_;
                  });
        // Fast path: latest 12 keyframes. Use native descriptor matching,
        // PnP-RANSAC and local-map refinement, not an extrapolated pose.
        const size_t recent_count = std::min<size_t>(12, candidates.size());
        std::vector<std::shared_ptr<data::keyframe>> recent(
            candidates.begin(), candidates.begin() + recent_count);
        if (!recent.empty()) {
            succeeded = relocalizer_.reloc_by_candidates(curr_frm_, recent, true);
        }
        spdlog::info("PAI_STELLA_WASM_RELOC_RECENT frame={} candidates={} recovered={}",
                     curr_frm_.id_, recent.size(), succeeded ? 1 : 0);
        // Turn-around recovery: the place last seen may be older than the
        // newest 12 keyframes. Search up to 12 evenly spaced historical
        // keyframes only after the fast path fails.
        if (!succeeded && candidates.size() > recent_count) {
            const size_t older = candidates.size() - recent_count;
            const size_t count = std::min<size_t>(12, older);
            std::vector<std::shared_ptr<data::keyframe>> historical;
            historical.reserve(count);
            // Sweep every historical keyframe over successive Lost frames.
            // A fixed uniformly spaced subset could miss the exact return view
            // forever. No BoW dictionary exists in this browser build.
            const size_t start = (static_cast<size_t>(curr_frm_.id_) * count) % older;
            for (size_t k = 0; k < count; ++k) {
                historical.push_back(candidates.at(recent_count + (start + k) % older));
            }
            succeeded = relocalizer_.reloc_by_candidates(curr_frm_, historical, true);
            spdlog::info("PAI_STELLA_WASM_RELOC_HISTORY frame={} total={} tested={} recovered={}",
                         curr_frm_.id_, candidates.size(), historical.size(), succeeded ? 1 : 0);
        }
        }
        // Do not mark a relocalization as complete until the ordinary
        // local-map acceptance check has also succeeded. Otherwise the
        // post-reloc 40-landmark gate may repeatedly reject 30–39 point
        // candidate solutions, continuously refreshing its own 1s timer.
    }
#endif

    // update the local map and optimize current camera pose
    unsigned int fixed_keyframe_id_threshold = map_db_->get_fixed_keyframe_id_threshold();"""
        )
    ])
    files["src/stella_vslam/tracking_module.cc"].extend([
        (
            "    if (curr_frm_.pose_is_valid()) {\n        last_cam_pose_from_ref_keyfrm_ = curr_frm_.get_pose_cw() * curr_frm_.ref_keyfrm_->get_pose_wc();",
            "    if (succeeded && curr_frm_.pose_is_valid()) {\n        last_cam_pose_from_ref_keyfrm_ = curr_frm_.get_pose_cw() * curr_frm_.ref_keyfrm_->get_pose_wc();"
        ),
        (
            "        last_frm_ = curr_frm_;\n    }\n    SPDLOG_TRACE(\"tracking_module: finish tracking\");",
            "#ifdef __EMSCRIPTEN__\n        if (succeeded && curr_frm_.pose_is_valid()) last_frm_ = curr_frm_;\n#else\n        last_frm_ = curr_frm_;\n#endif\n    }\n    SPDLOG_TRACE(\"tracking_module: finish tracking\");"
        )
    ])
    # The earlier branch only runs in Lost mode. Confirm the result after
    # all local-map checks, not immediately after robust/PnP candidate matching.
    files["src/stella_vslam/tracking_module.cc"].append((
        """    // update the frame statistics
    SPDLOG_TRACE("tracking_module: update_frame_statistics (curr_frm_={})", curr_frm_.id_);""",
        """#ifdef __EMSCRIPTEN__
    if (relocalization_is_needed && !bow_db_ && succeeded) {
        last_reloc_frm_id_ = curr_frm_.id_;
        last_reloc_frm_timestamp_ = curr_frm_.timestamp_;
        spdlog::info("PAI_STELLA_RELOC_ACCEPTED frame={} local_map=pass",
                     curr_frm_.id_);
    }
#endif
    // update the frame statistics
    SPDLOG_TRACE("tracking_module: update_frame_statistics (curr_frm_={})", curr_frm_.id_);"""
    ))
    # The normal turn fallback uses an Essential-matrix solver, which can be
    # degenerate during in-place yaw. Before entering Lost, try real 2D-3D
    # descriptor/PnP-RANSAC against the current reference keyframe. This is
    # independent of camera translation and keeps the original map origin.
    files["src/stella_vslam/tracking_module.cc"].append((
        """        succeeded = track_current_frame();
    }
    else if (bow_db_ && enable_auto_relocalization_) {""",
        """        succeeded = track_current_frame();
#ifdef __EMSCRIPTEN__
        if (!succeeded && curr_frm_.ref_keyfrm_ && !curr_frm_.ref_keyfrm_->will_be_erased()) {
            std::vector<std::shared_ptr<data::keyframe>> reference_only{curr_frm_.ref_keyfrm_};
            succeeded = relocalizer_.reloc_by_candidates(curr_frm_, reference_only, true);
            spdlog::info("PAI_STELLA_TURN_PNP frame={} success={}", curr_frm_.id_, succeeded ? 1 : 0);
        }
#endif
    }
    else if (bow_db_ && enable_auto_relocalization_) {"""
    ))
    # Upstream relocalizer marks a successful PnP pose but has a TODO to
    # update the reference keyframe. This matters after a turn: local-map
    # tracking must use the actual recovered candidate, not a stale reference.
    files["src/stella_vslam/module/relocalizer.cc"] = [(
        """            // TODO: should set the reference keyframe of the current frame
            return true;""",
        """#ifdef __EMSCRIPTEN__
            curr_frm.ref_keyfrm_ = candidate_keyfrm;
#else
            // TODO: should set the reference keyframe of the current frame
#endif
            return true;"""
    )]
    # Upstream frame_tracker::robust_match_based_track instantiates
    # robust(0.8, true), which rejects ORB keypoint orientation differences
    # above 30 degrees BEFORE geometric verification. On abrupt phone turns
    # this hard gate blocks real feature correspondences. Only the WASM turn
    # recovery matcher disables this prefilter; Essential RANSAC, pose
    # optimization and true 3D landmark thresholds remain unchanged.
    files["src/stella_vslam/module/frame_tracker.cc"] = [(
        """    match::robust robust_matcher(0.8, true);

    // Search 2D-2D matches between the ref keyframes and the current frame""",
        """#ifdef __EMSCRIPTEN__
    match::robust robust_matcher(0.8, false);
#else
    match::robust robust_matcher(0.8, true);
#endif

    // Search 2D-2D matches between the ref keyframes and the current frame"""
    )]
    # A pure-rotation / short-baseline turn makes the essential-matrix
    # prerequisite geometrically degenerate. For LOST-mode 2D-3D re-localization,
    # obtain actual ORB correspondences and let the existing EPnP + RANSAC,
    # reprojection, pose optimization and local-map gates verify the result.
    # Keep essential-matrix filtering for all ordinary tracking call sites.
    files["src/stella_vslam/match/robust.h"] = [(
        """                                          bool use_fixed_seed = false) const;""",
        """                                          bool use_fixed_seed = false,
                                          bool validate_essential = true) const;"""
    )]
    files["src/stella_vslam/match/robust.cc"] = [
        (
            """                                              bool use_fixed_seed) const {
    // Initialization
    const auto num_frm_keypts = frm.frm_obs_.undist_keypts_.size();""",
            """                                              bool use_fixed_seed, bool validate_essential) const {
    // Initialization
    const auto num_frm_keypts = frm.frm_obs_.undist_keypts_.size();"""
        ),
        (
            """    // Extract only inliers with RANSAC
    solve::essential_solver solver(frm.frm_obs_.bearings_, keyfrm->frm_obs_.bearings_, matches, use_fixed_seed);
    solver.find_via_ransac(1000, true);
    if (!solver.solution_is_valid()) {
        return 0;
    }
    const auto is_inlier_matches = solver.get_inlier_matches();

    // Save the information
    for (unsigned int i = 0; i < matches.size(); ++i) {
        if (!is_inlier_matches.at(i)) {
            continue;
        }
        const auto frm_idx = matches.at(i).first;
        const auto keyfrm_idx = matches.at(i).second;

        matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
        ++num_inlier_matches;
    }

    return num_inlier_matches;
}

unsigned int robust::brute_force_match""",
            """    // In existing-map relocalization, Essential RANSAC may fail for pure
    // rotation even when the map's actual 3D points can be matched.
    if (!validate_essential) {
        for (const auto& pair : matches) {
            const auto& lm = keyfrm_lms.at(pair.second);
            if (!lm || lm->will_be_erased()) continue;
            matched_lms_in_frm.at(pair.first) = lm;
            ++num_inlier_matches;
        }
        return num_inlier_matches;
    }
    // Ordinary tracking still retains its original Essential RANSAC gate.
    solve::essential_solver solver(frm.frm_obs_.bearings_, keyfrm->frm_obs_.bearings_, matches, use_fixed_seed);
    solver.find_via_ransac(1000, true);
    if (!solver.solution_is_valid()) {
        return 0;
    }
    const auto is_inlier_matches = solver.get_inlier_matches();
    for (unsigned int i = 0; i < matches.size(); ++i) {
        if (!is_inlier_matches.at(i)) continue;
        const auto frm_idx = matches.at(i).first;
        const auto keyfrm_idx = matches.at(i).second;
        matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
        ++num_inlier_matches;
    }
    return num_inlier_matches;
}

unsigned int robust::brute_force_match"""
        )
    ]
    files["src/stella_vslam/module/relocalizer.cc"].append((
        """    const auto num_matches = use_robust_matcher ? robust_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, matched_landmarks)
                                                : bow_matcher_.match_frame_and_keyframe(candidate_keyfrm, curr_frm, matched_landmarks);""",
        """#ifdef __EMSCRIPTEN__
    // Bypass only the degeneracy-prone Essential gate in map relocalization.
    // EPnP-RANSAC and all downstream geometric/landmark gates stay intact.
    const auto num_matches = use_robust_matcher ? robust_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, matched_landmarks, false, false)
                                                : bow_matcher_.match_frame_and_keyframe(candidate_keyfrm, curr_frm, matched_landmarks);
#else
    const auto num_matches = use_robust_matcher ? robust_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, matched_landmarks)
                                                : bow_matcher_.match_frame_and_keyframe(candidate_keyfrm, curr_frm, matched_landmarks);
#endif"""
    ))
    # Genuine upstream bug for map relocalization: after EPnP and descriptor
    # matching have produced valid 3D associations, local-map refinement
    # unconditionally returns false when no *additional* landmark can be
    # projected. This rejects a valid candidate merely for lacking extra
    # observations, especially when turning at the edge of the mapped FOV.
    # Still execute the pose optimizer and final inlier-ratio acceptance.
    files["src/stella_vslam/module/relocalizer.cc"].append((
        """        if (!found_proj_candidate) {
            return false;
        }

        // acquire more 2D-3D matches by projecting the local landmarks to the current frame""",
        """#ifdef __EMSCRIPTEN__
        // No additional map candidates is not itself a failed pose.
        // Existing 2D-3D associations still undergo optimization and
        // the unchanged final inlier-ratio gate below.
        if (!found_proj_candidate) {
            spdlog::info("PAI_STELLA_RELOC_NO_EXTRA_LMS iter={} remaining_existing_matches={}",
                         i, curr_frm.get_landmarks().size());
        }
#else
        if (!found_proj_candidate) {
            return false;
        }
#endif

        // acquire more 2D-3D matches by projecting the local landmarks to the current frame"""
    ))
    # Upstream keyframe insertion gates mature maps on camera TRANSLATION.
    # During an in-place turn translation stays ~zero, so the changing
    # viewing direction fails to gain any new keyframe coverage.
    # Allow genuine map-backed rotation keyframes while tracking is stable.
    # This does not create points from pure rotation or bypass pose checks.
    files["src/stella_vslam/module/keyframe_inserter.cc"] = [(
        """    return (max_interval_elapsed || max_distance_traveled || view_changed || not_enough_lms)
           && (!enough_keyfrms || (min_interval_elapsed && min_distance_traveled))
           && !tracking_is_unstable
           && !almost_all_lms_are_tracked
           && !mapper_is_skipping_localBA;""",
        """#ifdef __EMSCRIPTEN__
    bool rotation_keyframe_needed = false;
    if (last_inserted_keyfrm && num_tracked_lms >= 20
        && curr_frm.timestamp_ >= last_inserted_keyfrm->timestamp_ + 0.25) {
        // Compare optical axes in the same world coordinates. A 12-degree
        // yaw/pitch change captures a new viewing direction at zero translation.
        const double optical_axis_cos =
            curr_frm.get_rot_wc().col(2).dot(last_inserted_keyfrm->get_rot_cw().row(2).transpose());
        rotation_keyframe_needed = optical_axis_cos < 0.9781476007338057;
    }
    if (rotation_keyframe_needed && !mapper_is_skipping_localBA) {
        spdlog::info("PAI_STELLA_ROTATION_KEYFRAME frame={} tracked={} cos_angle={}",
                     curr_frm.id_, num_tracked_lms,
                     curr_frm.get_rot_wc().col(2).dot(last_inserted_keyfrm->get_rot_cw().row(2).transpose()));
        return true;
    }
#endif
    return (max_interval_elapsed || max_distance_traveled || view_changed || not_enough_lms)
           && (!enough_keyfrms || (min_interval_elapsed && min_distance_traveled))
           && !tracking_is_unstable
           && !almost_all_lms_are_tracked
           && !mapper_is_skipping_localBA;"""
    )]
    # A second premature-rejection bug in NORMAL (not just relocalization)
    # tracking: search_local_landmarks() returns false when it cannot ADD
    # further projected landmarks, even when the frame already has enough
    # confirmed 2D-3D matches for pose optimization. This produces sharp
    # turn-onset tracking dropouts at the boundary of local map visibility.
    # Keep the 20-point gate and all downstream PnP/pose inlier checks.
    files["src/stella_vslam/tracking_module.cc"].append((
        """    if (!found_proj_candidate) {
        spdlog::warn("projection candidate not found");
        return false;
    }

    // acquire more 2D-3D matches by projecting the local landmarks to the current frame""",
        """    if (!found_proj_candidate) {
#ifdef __EMSCRIPTEN__
        if (curr_landmark_ids.size() < 20) {
            spdlog::info("PAI_STELLA_LOCAL_PROJ_MISS insufficient_existing={}",
                         curr_landmark_ids.size());
            return false;
        }
        spdlog::info("PAI_STELLA_LOCAL_PROJ_FALLBACK existing={}",
                     curr_landmark_ids.size());
#else
        spdlog::warn("projection candidate not found");
        return false;
#endif
    }

    // acquire more 2D-3D matches by projecting the local landmarks to the current frame"""
    ))
    # Distinguish failure to REQUEST a new keyframe from failure to PROCESS
    # the synchronous mapper queue. Do not change geometry or thresholds.
    files["src/stella_vslam/tracking_module.cc"].append((
        """            keyfrm_inserter_.insert_new_keyframe(map_db_, curr_frm_);
        }""",
        """            keyfrm_inserter_.insert_new_keyframe(map_db_, curr_frm_);
#ifdef __EMSCRIPTEN__
            spdlog::info("PAI_STELLA_KF_INSERT frame={} tracked={} reliable={} keyframes_before_sync={}",
                         curr_frm_.id_, num_tracked_lms, num_reliable_lms,
                         map_db_->get_num_keyframes());
#endif
        }"""
    ))
    # Deterministic time-zero bug: upstream initializes last_reloc timestamp
    # and ID to ZERO. Consequently the '1 second since relocalization' special
    # gates fire for the ENTIRE first second even if NO relocalization happened.
    # In #86 the first stable map has just two keyframes, frame 26 fails with
    # 39 (<40) inlier landmarks before turning even starts, and the keyframe
    # insertion gate prevents any coverage growth before frame 30.
    # Apply strict gates only to a genuine recorded relocalization. The true
    # 40-landmark geometric threshold remains unchanged.
    files["src/stella_vslam/tracking_module.cc"].extend([
        (
            """    if (curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0 && num_tracked_lms < 2 * num_tracked_lms_thr) {""",
            """#ifdef __EMSCRIPTEN__
    if (last_reloc_frm_id_ > 0 && curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0
        && num_tracked_lms < 2 * num_tracked_lms_thr) {
#else
    if (curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0 && num_tracked_lms < 2 * num_tracked_lms_thr) {
#endif"""
        ),
        (
            """    if (curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0) {
        return false;
    }

    // check the new keyframe is needed""",
            """#ifdef __EMSCRIPTEN__
    if (last_reloc_frm_id_ > 0 && curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0) {
        spdlog::info("PAI_STELLA_KF_BLOCKED_AFTER_RELOC frame={} since_reloc={}",
                     curr_frm_.id_, curr_frm_.timestamp_ - last_reloc_frm_timestamp_);
        return false;
    }
#else
    if (curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0) {
        return false;
    }
#endif

    // check the new keyframe is needed"""
        )
    ])
    # Diagnostic only: log real matcher stage and map acceptance per frame.
    files["src/stella_vslam/tracking_module.cc"].append((
        "    return succeeded;\n}\n\nbool tracking_module::relocalize_by_pose",
        """#ifdef __EMSCRIPTEN__
    unsigned int associated = 0;
    for (const auto& lm : curr_frm_.get_landmarks()) {
        if (lm && !lm->will_be_erased()) ++associated;
    }
    spdlog::info("PAI_STELLA_FRAME_MATCH frame={} success={} associated={}",
                 curr_frm_.id_, succeeded ? 1 : 0, associated);
#endif
    return succeeded;
}

bool tracking_module::relocalize_by_pose"""
    ))
    # Rotational tracking: when the predicted optical flow exceeds the
    # normal projection window, retry with a wider geometry-centered search.
    # This affects candidate discovery only. Pose optimization and the
    # original minimum inlier count remain mandatory.
    files["src/stella_vslam/module/frame_tracker.cc"].append((
        """    if (num_matches < num_matches_thr_) {
        spdlog::debug("motion based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }""",
        """#ifdef __EMSCRIPTEN__
    if (num_matches < num_matches_thr_) {
        curr_frm.erase_landmarks();
        num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, 4 * margin_);
        spdlog::info("PAI_STELLA_MOTION_WIDE frame={} matches={} threshold={}",
                     curr_frm.id_, num_matches, num_matches_thr_);
    }
#endif
    if (num_matches < num_matches_thr_) {
        spdlog::info("PAI_STELLA_MOTION_REJECT frame={} matches={} threshold={}",
                     curr_frm.id_, num_matches, num_matches_thr_);
        return false;
    }"""
    ))
    # Read-only per-frame internal state exported to iPhone diagnostics.
    # Values: tracker state (0 init/1 tracking/2 lost), keyframe count,
    # landmark count, observed ORB count, associated 3D landmark count.
    files["src/stella_vslam/system.h"] = [
        ("    //-----------------------------------------\n    // config\n",
         """#ifdef __EMSCRIPTEN__
    void get_wasm_tracking_diagnostics(int* out5) const;
#endif

    //-----------------------------------------
    // config
""")
    ]
    files["src/stella_vslam/system.cc"].append((
        "std::shared_ptr<Mat44_t> system::feed_frame(const data::frame& frm, const cv::Mat& img, const double extraction_time_elapsed_ms) {",
        """#ifdef __EMSCRIPTEN__
void system::get_wasm_tracking_diagnostics(int* out5) const {
    if (!out5 || !tracker_ || !map_db_) return;
    const auto state = tracker_->tracking_state_;
    out5[0] = state == tracker_state_t::Tracking ? 1 : (state == tracker_state_t::Lost ? 2 : 0);
    out5[1] = static_cast<int>(map_db_->get_num_keyframes());
    out5[2] = static_cast<int>(map_db_->get_num_landmarks());
    out5[3] = static_cast<int>(tracker_->curr_frm_.frm_obs_.undist_keypts_.size());
    int associated = 0;
    for (const auto& lm : tracker_->curr_frm_.get_landmarks()) if (lm) ++associated;
    out5[4] = associated;
}
#endif

std::shared_ptr<Mat44_t> system::feed_frame(const data::frame& frm, const cv::Mat& img, const double extraction_time_elapsed_ms) {"""
    ))
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
