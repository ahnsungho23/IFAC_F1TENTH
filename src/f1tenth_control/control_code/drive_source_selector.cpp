#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

std::int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
}

}  // namespace

// ============================================================================
// drive_source_selector — control_map_node의 자율 명령 포워더 (sim/real 공용)
// ============================================================================
// 실차는 조이스틱 수동/자율/E-stop Mux를 팀 공용 f1tenth_stack의 drive_mode_manager +
// ackermann_mux가 담당한다(우리 joy_teleop_monitor는 실차 런치에서 제외됨). 하지만
// 그 스택의 자율 입력은 mux의 navigation 채널 'drive' 하나뿐이라, control_map_node가
// 발행하는 /drive_autonomous를 그 채널로 흘려보내는 노드가 필요하다. 이 노드가 그것만 한다.
//
// (2026-08-01: MPPI 노드/솔버 전체 제거와 함께 MAP/MPPI 셀렉터 기능도 걷어냈다 — 대회
//  준비 기간 동안 MAP 하나에만 집중하기로 함. 조이스틱 RB 토글·/mppi_active 발행 등은
//  더 이상 없다. 되살리려면 git 이력에서 이 파일의 이전 버전을 참고할 것.)
//
// 구독: /drive_autonomous(control_map_node)
// 발행: /drive(= ackermann_mux navigation 채널, 우선순위 10 — 자율모드에서 teleop 침묵 시 통과.
//       시뮬은 gym_bridge가 직접 구독)
// ============================================================================

class DriveSourceSelector : public rclcpp::Node {
public:
    DriveSourceSelector() : Node("drive_source_selector") {
        timing_diagnostics_enable_ =
            this->declare_parameter<bool>("timing_diagnostics_enable", false);
        timing_diagnostics_topic_ = this->declare_parameter<std::string>(
            "timing_diagnostics_topic", "/cma_timing/events");
        auto_drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_autonomous", 10,
            std::bind(&DriveSourceSelector::auto_drive_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
        if (timing_diagnostics_enable_) {
            timing_diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>(
                timing_diagnostics_topic_, rclcpp::QoS(100).reliable());
        }

        RCLCPP_INFO(this->get_logger(),
            "drive_source_selector 시작 — /drive_autonomous를 /drive로 포워딩합니다.");
    }

private:
    // 재스탬프만 하고 그대로 포워딩. E-stop/수동 게이트는 여기서 하지 않는다 —
    // drive_mode_manager + ackermann_mux 담당.
    void auto_drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        ++forward_sequence_;
        const std::int64_t input_stamp_ns = stamp_ns(msg->header.stamp);
        auto drive_msg = *msg;
        drive_msg.header.stamp = this->now();
        const std::int64_t publish_steady_ns = steady_now_ns();
        drive_pub_->publish(drive_msg);
        if (timing_diagnostics_enable_ && timing_diagnostics_pub_ != nullptr) {
            std_msgs::msg::String event;
            std::ostringstream json;
            json << std::setprecision(17)
                 << "{\"schema\":\"cma_timing_event/1\","
                 << "\"event\":\"T7_DRIVE\","
                 << "\"node\":\"drive_source_selector\","
                 << "\"steady_time_ns\":" << publish_steady_ns << ','
                 << "\"ros_time_ns\":" << this->now().nanoseconds() << ','
                 << "\"forward_sequence\":" << forward_sequence_ << ','
                 << "\"input_drive_stamp_ns\":" << input_stamp_ns << ','
                 << "\"drive_stamp_ns\":" << stamp_ns(drive_msg.header.stamp) << ','
                 << "\"steering_angle_rad\":" << drive_msg.drive.steering_angle << ','
                 << "\"command_speed_mps\":" << drive_msg.drive.speed << '}';
            event.data = json.str();
            timing_diagnostics_pub_->publish(event);
        }
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_drive_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_diagnostics_pub_;
    bool timing_diagnostics_enable_{false};
    std::string timing_diagnostics_topic_{"/cma_timing/events"};
    std::uint64_t forward_sequence_{0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveSourceSelector>());
    rclcpp::shutdown();
    return 0;
}
