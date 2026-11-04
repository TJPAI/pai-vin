// PAI-VIN Stella browser-facing C ABI (experimental).
// Real upstream stella_vslam, NOT a simulated pose pipeline.
// Deliberately no system::startup() in the no-pthread build; full concurrent
// mapping is a separate, unpassed Safari acceptance gate.
#include "stella_vslam/config.h"
#include "stella_vslam/system.h"
#include "stella_vslam/data/frame.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <exception>
#include <cmath>
#include <emscripten/emscripten.h>

namespace {
std::shared_ptr<stella_vslam::config> cfg;
std::unique_ptr<stella_vslam::system> engine;
std::string error;
double last_timestamp = -1.0;
void set_error(const char* e) { error = e ? e : "unknown"; }
bool valid_image(const uint8_t* bytes, int w, int h, double timestamp) {
    return engine && bytes && w > 0 && h > 0 && w <= 2048 && h <= 2048
        && std::isfinite(timestamp) && timestamp > last_timestamp;
}
} // namespace

extern "C" {
EMSCRIPTEN_KEEPALIVE const char* pai_stella_last_error() { return error.c_str(); }
EMSCRIPTEN_KEEPALIVE int pai_stella_get_diagnostics(int* out5) {
    if (!engine || !out5) return -1;
    try {
        engine->get_wasm_tracking_diagnostics(out5);
        return 0;
    } catch (const std::exception& e) {
        set_error(e.what());
        return -2;
    }
}
EMSCRIPTEN_KEEPALIVE int pai_stella_create(const char* yaml) {
    if (engine) { set_error("already_created"); return -1; }
    if (!yaml || std::strlen(yaml) > 65536) { set_error("invalid_config"); return -2; }
    try {
        cfg = std::make_shared<stella_vslam::config>(YAML::Load(yaml));
        engine.reset(new stella_vslam::system(cfg, ""));
        last_timestamp = -1;
        error.clear();
        return 0;
    } catch (const std::exception& e) {
        engine.reset();
        cfg.reset();
        set_error(e.what());
        return -3;
    }
}
EMSCRIPTEN_KEEPALIVE void pai_stella_destroy() {
    engine.reset();
    cfg.reset();
    last_timestamp = -1;
    error.clear();
}
// Process a REAL image through Stella ORB extraction. No fake map or pose.
// Returns keypoint count; negative numbers indicate input/runtime failure.
EMSCRIPTEN_KEEPALIVE int pai_stella_features_rgba(
    const uint8_t* bytes, int width, int height, double timestamp) {
    if (!valid_image(bytes, width, height, timestamp)) {
        set_error("invalid_frame_or_timestamp"); return -1;
    }
    try {
        cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t*>(bytes));
        cv::Mat rgb;
        cv::cvtColor(rgba, rgb, cv::COLOR_RGBA2RGB);
        auto frame = engine->create_monocular_frame(rgb, timestamp);
        last_timestamp = timestamp;
        error.clear();
        return static_cast<int>(frame.frm_obs_.undist_keypts_.size());
    } catch (const std::exception& e) {
        set_error(e.what()); return -2;
    }
}
// Feed a REAL image through the native tracking module. Only a native pose is
// emitted; return 0 if initialization/tracking has no valid pose.
// The non-threaded build is not yet a validated continuous mapping engine.
// Returns 1 on native pose, 0 on none, <0 on failure.
EMSCRIPTEN_KEEPALIVE int pai_stella_track_rgba(
    const uint8_t* bytes, int width, int height, double timestamp,
    double* pose_wc_column_major_16) {
    if (!pose_wc_column_major_16 || !valid_image(bytes, width, height, timestamp)) {
        set_error("invalid_frame_or_timestamp"); return -1;
    }
    try {
        cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t*>(bytes));
        cv::Mat rgb;
        cv::cvtColor(rgba, rgb, cv::COLOR_RGBA2RGB);
        auto pose = engine->feed_monocular_frame(rgb, timestamp);
        last_timestamp = timestamp;
        error.clear();
        if (!pose) return 0;
        for (int col = 0; col < 4; ++col)
            for (int row = 0; row < 4; ++row)
                pose_wc_column_major_16[col * 4 + row] = (*pose)(row, col);
        return 1;
    } catch (const std::exception& e) {
        set_error(e.what()); return -2;
    }
}
}
