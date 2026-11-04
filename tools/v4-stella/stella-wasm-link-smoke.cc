// Minimal genuine Stella WebAssembly link/runtime smoke.
// This deliberately constructs the real SLAM system but does NOT start its
// background mapping threads or claim pose, map persistence, or browser SLAM.
#include "stella_vslam/config.h"
#include "stella_vslam/system.h"

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
            // Deliberately DO NOT call startup(): it spawns std::threads.
        }
        std::cout << "PAI_STELLA_REAL_SYSTEM_CONSTRUCTOR_LINK_PASS" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "PAI_STELLA_REAL_SYSTEM_CONSTRUCTOR_FAIL: " << e.what() << std::endl;
        return 2;
    }
}
