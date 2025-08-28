#include <functional>
#include <chrono>
#include "robofer/control/state_handler.hpp"

using namespace std::chrono_literals;

namespace robofer {

// --- State implementations -------------------------------------------------

class StateHandler::HappyState : public StateHandler::State {
public:
  void on_enter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering HAPPY state");
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::HAPPY);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.set_speed(0, 360.0f);
    ctx.servos_.set_speed(1,-360.0f);
    if(ctx.audio_ && !ctx.happy_sound_.empty())
      ctx.audio_->play(ctx.happy_sound_);
  }
};

class StateHandler::AngryState : public StateHandler::State {
public:
  void on_enter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering ANGRY state");
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::ANGRY);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.move_to(0, 30.0f, 120.0f);
    ctx.servos_.move_to(1,150.0f, 120.0f);
    if(ctx.audio_ && !ctx.angry_sound_.empty())
      ctx.audio_->play(ctx.angry_sound_);
  }
};

class StateHandler::SadState : public StateHandler::State {
public:
  void on_enter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering SAD state");
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::FROWN);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.set_idle(0);
    ctx.servos_.set_idle(1);
    if(ctx.audio_ && !ctx.sad_sound_.empty())
      ctx.audio_->play(ctx.sad_sound_);
  }
};

} // namespace robofer

using robofer::StateHandler;

// --- StateHandler methods ---------------------------------------------------

StateHandler::StateHandler()
: Node("state_handler"),
  servos_(this,
          declare_parameter<std::string>("gpiochip", "gpiochip0"),
          declare_parameter<int>("servo1_offset", -1),
          declare_parameter<int>("servo2_offset", -1),
          declare_parameter<bool>("sim", false)),
  audio_(nullptr)
{
  bool sim = get_parameter("sim").as_bool();
  audio_ = std::make_unique<robo_audio::AudioPlayer>(sim);
  audio_->reindex();
  happy_sound_ = declare_parameter<std::string>("happy_sound", "");
  angry_sound_ = declare_parameter<std::string>("angry_sound", "");
  sad_sound_ = declare_parameter<std::string>("sad_sound", "");
  RCLCPP_INFO(get_logger(), "State handler starting (sim=%s)", sim ? "true" : "false");
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
  auto m = static_cast<Mood>(msg->data);
  RCLCPP_INFO(get_logger(), "Received mode request: %u", static_cast<unsigned>(msg->data));
  set_state(m);
}

void StateHandler::set_state(Mood m) {
  RCLCPP_INFO(get_logger(), "Changing state to %u", static_cast<unsigned>(m));
  if (audio_) audio_->stop();
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

