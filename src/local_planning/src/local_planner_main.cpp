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

#include "local_planning/local_planner_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

// ============================================================================
// local_planner_node 실행 진입점
// ============================================================================
// 계획 콜백과 odometry 콜백을 서로 다른 callback group에서 처리하므로 스레드 2개를 사용한다.
// 노드 내부 mutex가 최신 odometry 스냅샷의 일관성을 보장한다.
int main(int argc, char ** argv)
{
  // ROS 2 통신 계층과 명령행의 --ros-args를 초기화한다.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::LocalPlannerNode>();

  // 계획이 진행 중이어도 고주기 Frenet odometry 콜백을 받을 수 있게 병렬 executor를 사용한다.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();

  // SIGINT 또는 executor 종료 후 ROS 리소스를 정상 해제한다.
  rclcpp::shutdown();
  return 0;
}
