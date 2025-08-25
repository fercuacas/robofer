#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/uint8.hpp>

#include "robofer/servos.hpp"
#include "robofer/eyes.hpp"

using robo_servos::ControlServo;
using robo_eyes::Mood;

class StateHandler : public rclcpp::Node {
public:
  StateHandler()
  : Node("state_handler"),
    servos_(this->declare_parameter<std::string>("gpiochip", "gpiochip0"),
            this->declare_parameter<int>("servo1_offset", -1),
            this->declare_parameter<int>("servo2_offset", -1))
  {
    mood_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/eyes/mood", 10);

    sub_happy_ = this->create_subscription<std_msgs::msg::Empty>(
      "/mode/happy", 10, [&](const std_msgs::msg::Empty&){ handle_mode(Mood::HAPPY); });
    sub_angry_ = this->create_subscription<std_msgs::msg::Empty>(
      "/mode/angry", 10, [&](const std_msgs::msg::Empty&){ handle_mode(Mood::ANGRY); });
    sub_sad_ = this->create_subscription<std_msgs::msg::Empty>(
      "/mode/sad", 10, [&](const std_msgs::msg::Empty&){ handle_mode(Mood::FROWN); });
  }

private:
  ControlServo servos_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mood_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_happy_, sub_angry_, sub_sad_;

  void handle_mode(Mood m){
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(m);
    mood_pub_->publish(msg);
    switch(m){
      case Mood::HAPPY:
        servos_.set_speed(0, 90.0f);
        servos_.set_speed(1,-90.0f);
        break;
      case Mood::ANGRY:
        servos_.move_to(0, 30.0f, 120.0f);
        servos_.move_to(1,150.0f, 120.0f);
        break;
      case Mood::FROWN:
      default:
        servos_.set_idle(0);
        servos_.set_idle(1);
        break;
    }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

