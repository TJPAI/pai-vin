// Genuine Stella WebAssembly frame-extraction and link/runtime smoke.
// This deliberately constructs the real SLAM system but does NOT start its
// background mapping threads or claim pose, map persistence, or browser SLAM.
#include "stella_vslam/config.h"
#include "stella_vslam/system.h"
#include "stella_vslam/data/frame.h"
#include <opencv2/core.hpp>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <iostream>
#include <exception>

int main() {
    try {
        const auto node = YAML::Load(R"YAML(
Camera:
  name: "PAI-VIN WASM link smoke"
  setup: "monocular"
  model: "perspective"
  fx: 500.0
  fy: 500.0
  cx: 160.0
  cy: 120.0
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
  fps: 30.0
  cols: 320
  rows: 240
  color_order: "RGB"
Preprocessing:
  min_size: 320
Feature:
  name: "PAI-VIN probe ORB"
  scale_factor: 1.2
  num_levels: 4
  ini_fast_threshold: 20
  min_fast_threshold: 7
)YAML");
        auto cfg = std::make_shared<stella_vslam::config>(node);
        {
            stella_vslam::system system(cfg, "");
            // Test REAL OpenCV -> Stella ORB frame extraction, not a fabricated pose.
            // Deliberately DO NOT call startup(): it spawns std::threads.
            cv::Mat image(240, 320, CV_8UC3);
            unsigned int seed = 0x9e3779b9u;
            for (int row = 0; row < image.rows; ++row) {
                for (int col = 0; col < image.cols; ++col) {
                    seed = seed * 1664525u + 1013904223u;
                    const unsigned char v = static_cast<unsigned char>(seed >> 24);
                    image.at<cv::Vec3b>(row,col) = cv::Vec3b(v, v, v);
                }
            }
            const auto frm = system.create_monocular_frame(image, 0.0);
            const auto features = frm.frm_obs_.undist_keypts_.size();
            const auto descriptor_rows = frm.frm_obs_.descriptors_.rows;
            std::cout << "PAI_STELLA_REAL_ORB_FRAME features=" << features
                      << " descriptor_rows=" << descriptor_rows << std::endl;
            if (features == 0 || descriptor_rows != static_cast<int>(features)) {
                std::cerr << "PAI_STELLA_ORB_FRAME_REJECTED" << std::endl;
                return 3;
            }
        }
        std::cout << "PAI_STELLA_REAL_SYSTEM_CONSTRUCTOR_LINK_PASS" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "PAI_STELLA_REAL_SYSTEM_CONSTRUCTOR_FAIL: " << e.what() << std::endl;
        return 2;
    }
}
