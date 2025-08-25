#include <functional>
#include <chrono>
#include "robofer/state_handler.hpp"

using namespace std::chrono_literals;

namespace robofer {

// --- State implementations -------------------------------------------------

class StateHandler::HappyState : public StateHandler::State {
public:
  void on_enter(StateHandler &ctx) override {
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::HAPPY);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.set_speed(0, 90.0f);
    ctx.servos_.set_speed(1,-90.0f);
  }
};

class StateHandler::AngryState : public StateHandler::State {
public:
  void on_enter(StateHandler &ctx) override {
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::ANGRY);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.move_to(0, 30.0f, 120.0f);
    ctx.servos_.move_to(1,150.0f, 120.0f);
  }
};

class StateHandler::SadState : public StateHandler::State {
public:
  void on_enter(StateHandler &ctx) override {
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::FROWN);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.set_idle(0);
    ctx.servos_.set_idle(1);
  }
};

} // namespace robofer

using robofer::StateHandler;

// --- StateHandler methods ---------------------------------------------------

StateHandler::StateHandler()
: Node("state_handler"),
  servos_(declare_parameter<std::string>("gpiochip", "gpiochip0"),
          declare_parameter<int>("servo1_offset", -1),
          declare_parameter<int>("servo2_offset", -1))
{
  mood_pub_ = create_publisher<std_msgs::msg::UInt8>("/eyes/mood", 10);
  mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/mode", 10,
      std::bind(&StateHandler::mode_callback, this, std::placeholders::_1));
  timer_ = create_wall_timer(50ms, std::bind(&StateHandler::update, this));
  set_state(Mood::FROWN);
}

void StateHandler::update() {
  if (current_state_) current_state_->on_update(*this);
}

void StateHandler::mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  set_state(static_cast<Mood>(msg->data));
}

void StateHandler::set_state(Mood m) {
  if (current_state_) current_state_->on_exit(*this);
  switch (m) {
    case Mood::HAPPY:
      current_state_ = std::make_unique<HappyState>();
      break;
    case Mood::ANGRY:
      current_state_ = std::make_unique<AngryState>();
      break;
    case Mood::FROWN:
    default:
      current_state_ = std::make_unique<SadState>();
      break;
  }
  current_state_->on_enter(*this);
}

// --- main ------------------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

