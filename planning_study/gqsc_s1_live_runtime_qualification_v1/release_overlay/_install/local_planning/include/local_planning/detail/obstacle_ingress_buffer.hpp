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

#ifndef LOCAL_PLANNING__DETAIL__OBSTACLE_INGRESS_BUFFER_HPP_
#define LOCAL_PLANNING__DETAIL__OBSTACLE_INGRESS_BUFFER_HPP_

#include <cstdint>
#include <mutex>
#include <optional>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <rclcpp/time.hpp>

namespace local_planning::detail
{

// 비-lockstep 운용에서 검출 콜백이 계획 계산에 막히지 않도록 최신 스냅샷 1건만 보관한다.
// 유효성 검사와 플래너 상태 변경은 계획 콜백이 가져간 뒤 수행하므로 이 클래스는 수신 시각,
// 메시지 포인터, 순번 이외의 의미 상태를 갖지 않는다.
class ObstacleIngressBuffer
{
public:
  struct Snapshot
  {
    f110_msgs::msg::ObstacleArray::SharedPtr message;
    rclcpp::Time receipt_time{0, 0, RCL_ROS_TIME};
    std::uint64_t sequence{0};
  };

  std::uint64_t store(
    const f110_msgs::msg::ObstacleArray::SharedPtr & message,
    const rclcpp::Time & receipt_time)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++latest_sequence_;
    latest_ = Snapshot{message, receipt_time, latest_sequence_};
    return latest_sequence_;
  }

  std::optional<Snapshot> latestAfter(std::uint64_t processed_sequence) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_.has_value() || latest_->sequence <= processed_sequence) {
      return std::nullopt;
    }
    return latest_;
  }

  std::uint64_t latestSequence() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_sequence_;
  }

private:
  mutable std::mutex mutex_;
  std::optional<Snapshot> latest_;
  std::uint64_t latest_sequence_{0};
};

}  // namespace local_planning::detail

#endif  // LOCAL_PLANNING__DETAIL__OBSTACLE_INGRESS_BUFFER_HPP_
