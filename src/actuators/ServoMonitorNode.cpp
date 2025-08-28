#include <array>
#include <rclcpp/rclcpp.hpp>
#include "robofer/msg/servo_goal.hpp"
using std::placeholders::_1;

class ServoMonitorNode : public rclcpp::Node {
public:
  ServoMonitorNode() : Node("servo_monitor") {
    sub_ = create_subscription<robofer::msg::ServoGoal>(
        "servo_angles", 10,
        std::bind(&ServoMonitorNode::angleCallback, this, _1));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ServoMonitorNode::printAngles, this));
  }
private:
  void angleCallback(const robofer::msg::ServoGoal::SharedPtr msg) {
    if(msg->id >= 0 && msg->id < static_cast<int>(angles_.size()))
      angles_[msg->id] = msg->angle;
  }
  void printAngles() {
    RCLCPP_INFO(get_logger(), "Servo0: %.2f deg | Servo1: %.2f deg",
                angles_[0], angles_[1]);
  }
  std::array<float,2> angles_{0.0f,0.0f};
  rclcpp::Subscription<robofer::msg::ServoGoal>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
