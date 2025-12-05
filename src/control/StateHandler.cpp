#include <functional>
#include <chrono>
#include <std_msgs/msg/bool.hpp>
#include "robofer/control/StateHandler.hpp"

using namespace std::chrono_literals;

namespace robofer {

// --- State implementations -------------------------------------------------

class StateHandler::HappyState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering HAPPY state");
    ctx.publishMood(Mood::HAPPY);
    ctx.publishIdle(true);
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
    ctx.servos_.setSpeed(0, 360.0f);
    ctx.servos_.setSpeed(1,-360.0f);
    if(ctx.audio_ && !ctx.happy_sound_.empty())
      ctx.audio_->play(ctx.happy_sound_);
  }
};

class StateHandler::AngryState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering ANGRY state");
    ctx.publishMood(Mood::ANGRY);
    ctx.publishIdle(true);
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
    ctx.servos_.moveTo(0, 30.0f, 120.0f);
    ctx.servos_.moveTo(1,150.0f, 120.0f);
    if(ctx.audio_ && !ctx.angry_sound_.empty())
      ctx.audio_->play(ctx.angry_sound_);
  }
};

class StateHandler::SadState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering SAD state");
    ctx.publishMood(Mood::FROWN);
    ctx.publishIdle(true);
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
    ctx.servos_.setIdle(0);
    ctx.servos_.setIdle(1);
    if(ctx.audio_ && !ctx.sad_sound_.empty())
      ctx.audio_->play(ctx.sad_sound_);
  }
};

class StateHandler::LoveState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering LOVE state");
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::LOVE);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.moveTo(0, 90.0f, 60.0f);
    ctx.servos_.moveTo(1, 90.0f, 60.0f);
    if(ctx.audio_ && !ctx.love_sound_.empty())
      ctx.audio_->play(ctx.love_sound_);
  }
};

class StateHandler::BailoteoWaitingState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering BAILOTEO_WAIT state");
    ctx.publishMood(Mood::BAILOTEO_WAIT);
    ctx.publishIdle(false);
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
  }
  void onExit(StateHandler &ctx) override {
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
  }
};

class StateHandler::BailoteoState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering BAILOTEO state");
    ctx.publishMood(Mood::BAILOTEO);
    ctx.publishIdle(false);
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
    last_switch_ = std::chrono::steady_clock::now();
    side_left_ = false;
  }

  void onUpdate(StateHandler &ctx) override {
    auto now = std::chrono::steady_clock::now();
    if(now - last_switch_ < std::chrono::milliseconds(400)) return;
    last_switch_ = now;
    side_left_ = !side_left_;
    ctx.publishEyePos(side_left_ ? robo_eyes::Pos::W : robo_eyes::Pos::E);
  }

  void onExit(StateHandler &ctx) override {
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
  }

private:
  bool side_left_{false};
  std::chrono::steady_clock::time_point last_switch_{};
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
  love_sound_ = declare_parameter<std::string>("love_sound", "");
  RCLCPP_INFO(get_logger(), "State handler starting (sim=%s)", sim ? "true" : "false");
  mood_pub_ = create_publisher<std_msgs::msg::UInt8>("/eyes/mood", 10);
  eye_pos_pub_ = create_publisher<std_msgs::msg::UInt8>("/eyes/pos", 10);
  eye_idle_pub_ = create_publisher<std_msgs::msg::Bool>("/eyes/idle", 10);
  mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/mode", 10,
      std::bind(&StateHandler::modeCallback, this, std::placeholders::_1));
  timer_ = create_wall_timer(50ms, std::bind(&StateHandler::update, this));
  setState(Mood::FROWN);
}

void StateHandler::update() {
  if (current_state_) current_state_->onUpdate(*this);
}

void StateHandler::modeCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
  auto m = static_cast<Mood>(msg->data);
  RCLCPP_INFO(get_logger(), "Received mode request: %u", static_cast<unsigned>(msg->data));
  setState(m);
}

void StateHandler::setState(Mood m) {
  RCLCPP_INFO(get_logger(), "Changing state to %u", static_cast<unsigned>(m));

  if(!isBailoteo(m)){
    last_regular_mood_ = m;
  }

  if (audio_ && !isBailoteo(m)) audio_->stop();
  if (current_state_) current_state_->onExit(*this);

  switch (m) {
    case Mood::HAPPY:
      current_state_ = std::make_unique<HappyState>();
      break;
    case Mood::ANGRY:
      current_state_ = std::make_unique<AngryState>();
      break;
    case Mood::LOVE:
      current_state_ = std::make_unique<LoveState>();
      break;
    case Mood::BAILOTEO:
      current_state_ = std::make_unique<BailoteoState>();
      break;
    case Mood::BAILOTEO_WAIT:
      current_state_ = std::make_unique<BailoteoWaitingState>();
      break;
    case Mood::FROWN:
    default:
      current_state_ = std::make_unique<SadState>();
      break;
  }

  if (current_state_) current_state_->onEnter(*this);
}

void StateHandler::publishMood(Mood m){
  if(!mood_pub_) return;
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(m);
  mood_pub_->publish(msg);
}

void StateHandler::publishEyePos(robo_eyes::Pos pos){
  if(!eye_pos_pub_) return;
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(pos);
  eye_pos_pub_->publish(msg);
}

void StateHandler::publishIdle(bool enabled){
  if(!eye_idle_pub_) return;
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  eye_idle_pub_->publish(msg);
}

// --- main ------------------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
