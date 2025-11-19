#include <functional>
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <vector>
#include <cstdlib>
#include "robofer/control/StateHandler.hpp"

using namespace std::chrono_literals;

namespace robofer {

namespace fs = std::filesystem;

// --- State implementations -------------------------------------------------

class StateHandler::HappyState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering HAPPY state");
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::HAPPY);
    ctx.mood_pub_->publish(msg);
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
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::ANGRY);
    ctx.mood_pub_->publish(msg);
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
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::FROWN);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.setIdle(0);
    ctx.servos_.setIdle(1);
    if(ctx.audio_ && !ctx.sad_sound_.empty())
      ctx.audio_->play(ctx.sad_sound_);
  }
};

class StateHandler::PuxaineState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering PUXAINE state");
    std_msgs::msg::UInt8 msg; msg.data = static_cast<uint8_t>(Mood::BAILONGO);
    ctx.mood_pub_->publish(msg);
    ctx.servos_.setSpeed(0, 360.0f);
    ctx.servos_.setSpeed(1,-360.0f);
    loadPlaylist(ctx);
    if(playlist_.empty()){
      RCLCPP_WARN(ctx.get_logger(),
                  "Modo Puxaine: no se encontraron pistas en %s",
                  root_dir_.empty() ? "(desconocido)" : root_dir_.c_str());
      return;
    }
    current_index_ = 0;
    if(!startCurrent(ctx)){
      advance(ctx);
    }
  }

  void onUpdate(StateHandler &ctx) override {
    if(playlist_.empty() || !ctx.audio_) return;
    if(ctx.audio_->isPlaying()) return;
    advance(ctx);
  }

  void onExit(StateHandler &ctx) override {
    (void)ctx;
    playlist_.clear();
  }

private:
  std::vector<std::string> playlist_;
  size_t current_index_{0};
  std::string root_dir_;

  static std::string toLower(const std::string& s){
    std::string out(s);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return out;
  }

  void loadPlaylist(StateHandler &ctx){
    playlist_.clear();
    root_dir_.clear();
    const char* home = std::getenv("HOME");
    if(!home){
      RCLCPP_WARN(ctx.get_logger(), "Modo Puxaine: variable HOME no definida");
      return;
    }
    fs::path dir = fs::path(home) / "Music" / "Pux";
    root_dir_ = dir.string();
    std::error_code ec;
    if(!fs::exists(dir, ec) || !fs::is_directory(dir, ec)){
      RCLCPP_WARN(ctx.get_logger(), "Modo Puxaine: directorio inexistente: %s", root_dir_.c_str());
      return;
    }

    static const std::vector<std::string> kExts{".wav", ".mp3"};
    for(fs::recursive_directory_iterator it(dir, ec), end; it != end; it.increment(ec)){
      if(ec){ ec.clear(); continue; }
      if(!it->is_regular_file()) continue;
      std::string ext = toLower(it->path().extension().string());
      if(std::find(kExts.begin(), kExts.end(), ext) == kExts.end()) continue;
      playlist_.push_back(it->path().string());
    }
    std::sort(playlist_.begin(), playlist_.end());
  }

  bool startCurrent(StateHandler &ctx){
    if(!ctx.audio_ || playlist_.empty()) return false;
    const std::string& track = playlist_[current_index_];
    if(ctx.audio_->play(track)){
      RCLCPP_INFO(ctx.get_logger(), "Modo Puxaine: reproduciendo %s", track.c_str());
      return true;
    }
    RCLCPP_WARN(ctx.get_logger(), "Modo Puxaine: no se pudo reproducir %s", track.c_str());
    return false;
  }

  void advance(StateHandler &ctx){
    if(playlist_.empty()) return;
    size_t attempts = playlist_.size();
    while(attempts-- > 0){
      current_index_ = (current_index_ + 1) % playlist_.size();
      if(startCurrent(ctx)) return;
    }
    RCLCPP_WARN(ctx.get_logger(), "Modo Puxaine: ninguna pista reproducible disponible");
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
  if (audio_) audio_->stop();
  if (current_state_) current_state_->onExit(*this);
  switch (m) {
    case Mood::HAPPY:
      current_state_ = std::make_unique<HappyState>();
      break;
    case Mood::ANGRY:
      current_state_ = std::make_unique<AngryState>();
      break;
    case Mood::BAILONGO:
      current_state_ = std::make_unique<PuxaineState>();
      break;
    case Mood::FROWN:
    default:
      current_state_ = std::make_unique<SadState>();
      break;
  }
  current_state_->onEnter(*this);
}

// --- main ------------------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

