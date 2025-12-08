#include <functional>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <std_msgs/msg/bool.hpp>
#include "robofer/control/StateHandler.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace robofer {

// --- State implementations -------------------------------------------------

class StateHandler::HappyState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering HAPPY state");
    ctx.publishMood(Mood::HAPPY);
    ctx.publishIdle(true);
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
    last_switch_ = std::chrono::steady_clock::now();
    forward_ = true;
    // Desplaza un poco desde el neutro para girar en sentidos opuestos
    float delta = 10.0f; // ajuste de “velocidad”
    ctx.servos_.setAngleFromNeutral(0, +delta);
    ctx.servos_.setAngleFromNeutral(1, -delta);
    if(ctx.audio_ && !ctx.happy_sound_.empty())
      ctx.audio_->play(ctx.happy_sound_);
  }

  void onUpdate(StateHandler &ctx) override {
    auto now = std::chrono::steady_clock::now();
    if(now - last_switch_ < std::chrono::milliseconds(800)) return;
    last_switch_ = now;
    forward_ = !forward_;
    float delta = 10.0f;
    if(forward_){
      ctx.servos_.setAngleFromNeutral(0, +delta);
      ctx.servos_.setAngleFromNeutral(1, -delta);
    } else {
      ctx.servos_.setAngleFromNeutral(0, -delta);
      ctx.servos_.setAngleFromNeutral(1, +delta);
    }
  }

  void onExit(StateHandler &ctx) override {
    ctx.servos_.setIdle(0);
    ctx.servos_.setIdle(1);
  }

private:
  bool forward_{true};
  std::chrono::steady_clock::time_point last_switch_{};
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

  void onExit(StateHandler &ctx) override {
    ctx.servos_.setIdle(0);
    ctx.servos_.setIdle(1);
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
    ctx.servos_.setIdle(0);
    ctx.servos_.setIdle(1);
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

class StateHandler::PuxaineState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering PUXAINE state");
    bailoteo_.onEnter(ctx);
    loadPlaylist(ctx);
    if(playlist_.empty()){
      RCLCPP_WARN(ctx.get_logger(), "No se encontraron audios en %s", target_dir_.c_str());
      return;
    }
    index_ = 0;
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudo iniciar la reproducción en Puxaine");
    }
  }

  void onUpdate(StateHandler &ctx) override {
    bailoteo_.onUpdate(ctx);
    if(playlist_.empty()) return;
    if(!ctx.audio_) return;
    if(ctx.audio_->isPlaying()) return;
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudieron reproducir las pistas de Puxaine");
    }
  }

  void onExit(StateHandler &ctx) override {
    bailoteo_.onExit(ctx);
    if(ctx.audio_) ctx.audio_->stop();
  }

private:
  void loadPlaylist(StateHandler &ctx){
    playlist_.clear();
    const char* home = std::getenv("HOME");
    if(!home){
      RCLCPP_WARN(ctx.get_logger(), "HOME no definido: sin playlist Puxaine");
      return;
    }

    target_dir_ = (fs::path(home) / "Music" / "Pux").string();
    std::error_code ec;
    fs::path dir(target_dir_);
    if(!fs::exists(dir, ec) || !fs::is_directory(dir, ec)){
      RCLCPP_WARN(ctx.get_logger(), "Carpeta Puxaine no encontrada: %s", target_dir_.c_str());
      return;
    }

    std::vector<fs::path> files;
    for(fs::directory_iterator it(dir, ec), end; it != end; it.increment(ec)){
      if(ec) break;
      const auto& entry = *it;
      if(!entry.is_regular_file(ec)) continue;
      fs::path p = entry.path();
      if(!ctx.audio_ || !ctx.audio_->isSupportedFile(p.string())) continue;
      files.push_back(p);
    }

    std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b){
      return a.filename().string() < b.filename().string();
    });

    for(const auto& p : files){
      auto canonical = fs::canonical(p, ec);
      if(ec) continue;
      playlist_.push_back(canonical.string());
    }
  }

  bool startNext(StateHandler &ctx){
    if(!ctx.audio_ || playlist_.empty()) return false;

    for(size_t attempt=0; attempt<playlist_.size(); ++attempt){
      const std::string& path = playlist_[index_];
      index_ = (index_ + 1) % playlist_.size();
      if(ctx.audio_->play(path)) return true;
      RCLCPP_WARN(ctx.get_logger(), "Fallo al reproducir %s", path.c_str());
    }
    return false;
  }

  std::vector<std::string> playlist_{};
  size_t index_{0};
  std::string target_dir_{};
  BailoteoState bailoteo_{};
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
  float neutral1 = declare_parameter<float>("servo1_neutral_deg", 90.0f);
  float neutral2 = declare_parameter<float>("servo2_neutral_deg", 90.0f);
  servos_.setNeutralAngle(0, neutral1);
  servos_.setNeutralAngle(1, neutral2);
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
    case Mood::PUXAINE:
      current_state_ = std::make_unique<PuxaineState>();
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
