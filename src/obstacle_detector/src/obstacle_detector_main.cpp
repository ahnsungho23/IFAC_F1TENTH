// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Entry point only. The node itself lives in obstacle_detector_core so node-level tests can link
// it without colliding with this main(), matching how local_planning separates
// local_planner_main.cpp from local_planner_core.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "obstacle_detector/obstacle_detector_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obstacle_detector::ObstacleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
