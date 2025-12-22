#include <functional>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <random>
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
    entered_ = std::chrono::steady_clock::now();
    last_switch_ = std::chrono::steady_clock::now();
    forward_ = true;
    // Desplaza un poco desde el neutro para girar en sentidos opuestos
    float delta = 10.0f; // ajuste de velocidad
    ctx.servos_.setAngleFromNeutral(0, +delta);
    ctx.servos_.setAngleFromNeutral(1, -delta);
    if(ctx.audio_ && !ctx.happy_sound_.empty())
      ctx.audio_->play(ctx.happy_sound_);
  }

  void onUpdate(StateHandler &ctx) override {
    auto now = std::chrono::steady_clock::now();
    if(now - entered_ >= std::chrono::seconds(10)){
      ctx.setState(Mood::FROWN);
      return;
    }
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
  std::chrono::steady_clock::time_point entered_{};
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
    loadPlaylist(ctx);
    index_ = 0;
    remaining_ = playlist_.size();
    done_ = false;
    if(remaining_ == 0){
      RCLCPP_WARN(ctx.get_logger(), "No se encontraron audios en %s", target_dir_.c_str());
      done_ = true;
      return;
    }
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudo iniciar la reproduccion en Angry");
      done_ = true;
    }
  }

  void onUpdate(StateHandler &ctx) override {
    if(done_){
      ctx.setState(Mood::ESPERA);
      return;
    }
    if(!ctx.audio_) return;
    if(ctx.audio_->isPlaying()) return;
    if(remaining_ == 0){
      done_ = true;
      ctx.setState(Mood::ESPERA);
      return;
    }
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudieron reproducir las pistas de Angry");
      done_ = true;
      ctx.setState(Mood::ESPERA);
    }
  }

  void onExit(StateHandler &ctx) override {
    ctx.servos_.setIdle(0);
    ctx.servos_.setIdle(1);
  }

private:
  void loadPlaylist(StateHandler &ctx){
    playlist_.clear();
    const char* home = std::getenv("HOME");
    if(!home){
      RCLCPP_WARN(ctx.get_logger(), "HOME no definido: sin playlist Angry");
      return;
    }

    target_dir_ = (fs::path(home) / "Music" / "Angry").string();
    std::error_code ec;
    fs::path dir(target_dir_);
    if(!fs::exists(dir, ec) || !fs::is_directory(dir, ec)){
      RCLCPP_WARN(ctx.get_logger(), "Carpeta Angry no encontrada: %s", target_dir_.c_str());
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
    if(!ctx.audio_ || playlist_.empty() || remaining_ == 0) return false;

    size_t attempts = 0;
    while(attempts < playlist_.size() && remaining_ > 0){
      const std::string& path = playlist_[index_];
      index_ = (index_ + 1) % playlist_.size();
      ++attempts;
      --remaining_;
      if(ctx.audio_->play(path)) return true;
      RCLCPP_WARN(ctx.get_logger(), "Fallo al reproducir %s", path.c_str());
    }
    return false;
  }

  std::vector<std::string> playlist_{};
  size_t index_{0};
  size_t remaining_{0};
  std::string target_dir_{};
  bool done_{false};
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
    loadPlaylist(ctx);
    index_ = 0;
    remaining_ = playlist_.size();
    done_ = false;
    if(remaining_ == 0){
      RCLCPP_WARN(ctx.get_logger(), "No se encontraron audios en %s", target_dir_.c_str());
      done_ = true;
      return;
    }
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudo iniciar la reproduccion en Sad");
      done_ = true;
    }
  }

  void onUpdate(StateHandler &ctx) override {
    if(done_){
      ctx.setState(Mood::ESPERA);
      return;
    }
    if(!ctx.audio_) return;
    if(ctx.audio_->isPlaying()) return;
    if(remaining_ == 0){
      done_ = true;
      ctx.setState(Mood::ESPERA);
      return;
    }
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudieron reproducir las pistas de Sad");
      done_ = true;
      ctx.setState(Mood::ESPERA);
    }
  }

private:
  void loadPlaylist(StateHandler &ctx){
    playlist_.clear();
    const char* home = std::getenv("HOME");
    if(!home){
      RCLCPP_WARN(ctx.get_logger(), "HOME no definido: sin playlist Sad");
      return;
    }

    target_dir_ = (fs::path(home) / "Music" / "Sad").string();
    std::error_code ec;
    fs::path dir(target_dir_);
    if(!fs::exists(dir, ec) || !fs::is_directory(dir, ec)){
      RCLCPP_WARN(ctx.get_logger(), "Carpeta Sad no encontrada: %s", target_dir_.c_str());
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
    if(!ctx.audio_ || playlist_.empty() || remaining_ == 0) return false;

    size_t attempts = 0;
    while(attempts < playlist_.size() && remaining_ > 0){
      const std::string& path = playlist_[index_];
      index_ = (index_ + 1) % playlist_.size();
      ++attempts;
      --remaining_;
      if(ctx.audio_->play(path)) return true;
      RCLCPP_WARN(ctx.get_logger(), "Fallo al reproducir %s", path.c_str());
    }
    return false;
  }

  std::vector<std::string> playlist_{};
  size_t index_{0};
  size_t remaining_{0};
  std::string target_dir_{};
  bool done_{false};
};

class StateHandler::EsperaState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering ESPERA state");
    ctx.publishMood(Mood::ESPERA);
    ctx.publishIdle(true);
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
    ctx.servos_.setIdle(0);
    ctx.servos_.setIdle(1);
    next_action_ = std::chrono::steady_clock::now() + std::chrono::seconds(10);

    if(ctx.pending_return_){
      ctx.pending_return_ = false;
      if(ctx.playRandomReturnTrack()) return;
    }
    if(!ctx.welcome_played_){
      ctx.welcome_played_ = true;
      ctx.playModesTrack("bienvenido.mp3");
    }
  }

  void onUpdate(StateHandler &ctx) override {
    if(!ctx.audio_) return;
    if(ctx.audio_->isPlaying()) return;
    auto now = std::chrono::steady_clock::now();
    if(now < next_action_) return;

    auto track = ctx.randomTrackInDir(idle_dir(ctx));
    if(track && ctx.audio_->play(*track)){
      triggerActionFor(ctx, *track);
    }
    next_action_ = now + std::chrono::seconds(10);
  }

private:
  std::chrono::steady_clock::time_point next_action_{};

  static std::string idle_dir(StateHandler &ctx){
    const char* home = std::getenv("HOME");
    if(!home){
      RCLCPP_WARN(ctx.get_logger(), "HOME no definido: sin audios IDLE");
      return {};
    }
    return (fs::path(home) / "Music" / "IDLE").string();
  }

  static void triggerActionFor(StateHandler &ctx, const std::string& path){
    std::string name = fs::path(path).filename().string();
    if(name == "bostezo.mp3"){
      ctx.sendEyeAction(EyeAction::TIRED);
    } else if(name == "fart.mp3"){
      ctx.sendEyeAction(EyeAction::FROWN);
    } else if(name == "rock.mp3"){
      ctx.sendEyeAction(EyeAction::CURIOUS);
    } else if(name == "yipi.mp3"){
      ctx.sendEyeAction(EyeAction::LAUGH);
    } else if(name == "ah.mp3"){
      ctx.sendEyeAction(EyeAction::CONFUSED);
    }
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
    stage_ = 0;
    done_ = !playNext(ctx);
  }

  void onUpdate(StateHandler &ctx) override {
    if(!ctx.audio_){
      done_ = true;
    } else if(ctx.audio_->isPlaying()){
      return;
    }
    if(!done_ && playNext(ctx)) return;
    ctx.setState(Mood::ESPERA);
  }

  void onExit(StateHandler &ctx) override {
    if(ctx.audio_) ctx.audio_->stop();
  }

private:
  bool playNext(StateHandler &ctx){
    while(stage_ < tracks_.size()){
      if(ctx.playModesTrack(tracks_[stage_])){
        ++stage_;
        return true;
      }
      ++stage_;
    }
    return false;
  }

  std::vector<std::string> tracks_{"love.mp3", "lovefer.mp3"};
  size_t stage_{0};
  bool done_{false};
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
    index_ = 0;
    remaining_ = playlist_.size();
    intro_pending_ = ctx.playModesTrack("puxaine.mp3");
    if(intro_pending_){
      if(playlist_.empty()){
        RCLCPP_WARN(ctx.get_logger(), "No se encontraron audios en %s", target_dir_.c_str());
      }
      return;
    }
    if(remaining_ == 0){
      RCLCPP_WARN(ctx.get_logger(), "No se encontraron audios en %s", target_dir_.c_str());
      ctx.setState(Mood::ESPERA);
      return;
    }
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudo iniciar la reproducción en Puxaine");
      ctx.setState(Mood::ESPERA);
    }
  }

  void onUpdate(StateHandler &ctx) override {
    bailoteo_.onUpdate(ctx);
    if(!ctx.audio_) return;
    if(ctx.audio_->isPlaying()) return;
    if(intro_pending_){
      intro_pending_ = false;
      if(remaining_ == 0){
        ctx.setState(Mood::ESPERA);
        return;
      }
      if(!startNext(ctx)){
        RCLCPP_WARN(ctx.get_logger(), "No se pudieron reproducir las pistas de Puxaine");
        ctx.setState(Mood::ESPERA);
      }
      return;
    }
    if(remaining_ == 0){
      ctx.setState(Mood::ESPERA);
      return;
    }
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudieron reproducir las pistas de Puxaine");
      ctx.setState(Mood::ESPERA);
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
    if(!ctx.audio_ || playlist_.empty() || remaining_ == 0) return false;

    size_t attempts = 0;
    while(attempts < playlist_.size() && remaining_ > 0){
      const std::string& path = playlist_[index_];
      index_ = (index_ + 1) % playlist_.size();
      ++attempts;
      --remaining_;
      if(ctx.audio_->play(path)) return true;
      RCLCPP_WARN(ctx.get_logger(), "Fallo al reproducir %s", path.c_str());
    }
    return false;
  }

  std::vector<std::string> playlist_{};
  size_t index_{0};
  size_t remaining_{0};
  std::string target_dir_{};
  BailoteoState bailoteo_{};
  bool intro_pending_{false};
};

class StateHandler::PeoState : public StateHandler::State {
public:
  void onEnter(StateHandler &ctx) override {
    RCLCPP_INFO(ctx.get_logger(), "Entering PEO state");
    ctx.publishMood(Mood::PEO);
    ctx.publishIdle(true);
    ctx.publishEyePos(robo_eyes::Pos::CENTER);
    ctx.servos_.setIdle(0);
    ctx.servos_.setIdle(1);
    loadPlaylist(ctx);
    index_ = 0;
    remaining_ = playlist_.size();
    intro_pending_ = ctx.playModesTrack("peo.mp3");
    if(intro_pending_){
      if(playlist_.empty()){
        RCLCPP_WARN(ctx.get_logger(), "No se encontraron audios en %s", target_dir_.c_str());
      }
      return;
    }
    if(remaining_ == 0){
      RCLCPP_WARN(ctx.get_logger(), "No se encontraron audios en %s", target_dir_.c_str());
      ctx.setState(Mood::ESPERA);
      return;
    }
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudo iniciar la reproducción en Peo");
      ctx.setState(Mood::ESPERA);
    }
  }

  void onUpdate(StateHandler &ctx) override {
    if(!ctx.audio_) return;
    if(ctx.audio_->isPlaying()) return;
    if(intro_pending_){
      intro_pending_ = false;
      if(remaining_ == 0){
        ctx.setState(Mood::ESPERA);
        return;
      }
      if(!startNext(ctx)){
        RCLCPP_WARN(ctx.get_logger(), "No se pudieron reproducir las pistas de Peo");
        ctx.setState(Mood::ESPERA);
      }
      return;
    }
    if(remaining_ == 0){
      ctx.setState(Mood::ESPERA);
      return;
    }
    if(!startNext(ctx)){
      RCLCPP_WARN(ctx.get_logger(), "No se pudieron reproducir las pistas de Peo");
      ctx.setState(Mood::ESPERA);
    }
  }

  void onExit(StateHandler &ctx) override {
    if(ctx.audio_) ctx.audio_->stop();
  }

private:
  void loadPlaylist(StateHandler &ctx){
    playlist_.clear();
    const char* home = std::getenv("HOME");
    if(!home){
      RCLCPP_WARN(ctx.get_logger(), "HOME no definido: sin playlist Peo");
      return;
    }

    target_dir_ = (fs::path(home) / "Music" / "Peo").string();
    std::error_code ec;
    fs::path dir(target_dir_);
    if(!fs::exists(dir, ec) || !fs::is_directory(dir, ec)){
      RCLCPP_WARN(ctx.get_logger(), "Carpeta Peo no encontrada: %s", target_dir_.c_str());
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
    if(!ctx.audio_ || playlist_.empty() || remaining_ == 0) return false;

    size_t attempts = 0;
    while(attempts < playlist_.size() && remaining_ > 0){
      const std::string& path = playlist_[index_];
      index_ = (index_ + 1) % playlist_.size();
      ++attempts;
      --remaining_;
      if(ctx.audio_->play(path)) return true;
      RCLCPP_WARN(ctx.get_logger(), "Fallo al reproducir %s", path.c_str());
    }
    return false;
  }

  std::vector<std::string> playlist_{};
  size_t index_{0};
  std::string target_dir_{};
  size_t remaining_{0};
  bool intro_pending_{false};
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
  eye_action_pub_ = create_publisher<std_msgs::msg::UInt8>("/eyes/action", 10);
  mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/mode", 10,
      std::bind(&StateHandler::modeCallback, this, std::placeholders::_1));
  timer_ = create_wall_timer(50ms, std::bind(&StateHandler::update, this));
  setState(Mood::ESPERA);
}

void StateHandler::update() {
  if (current_state_) current_state_->onUpdate(*this);
}

std::optional<std::string> StateHandler::modesTrackPath(const std::string& filename) const {
  const char* home = std::getenv("HOME");
  if(!home){
    RCLCPP_WARN(get_logger(), "HOME no definido: no se puede reproducir %s", filename.c_str());
    return std::nullopt;
  }
  fs::path p = fs::path(home) / "Music" / "Modos" / filename;
  return p.string();
}

bool StateHandler::playModesTrack(const std::string& filename){
  if(!audio_) return false;
  auto path = modesTrackPath(filename);
  if(!path) return false;
  return audio_->play(*path);
}

std::optional<std::string> StateHandler::randomTrackInDir(const std::string& dir) const {
  if(!audio_) return std::nullopt;
  std::error_code ec;
  fs::path root(dir);
  if(!fs::exists(root, ec) || !fs::is_directory(root, ec)){
    return std::nullopt;
  }
  std::vector<std::string> files;
  for(fs::directory_iterator it(root, ec), end; it != end; it.increment(ec)){
    if(ec) break;
    const auto& entry = *it;
    if(!entry.is_regular_file(ec)) continue;
    fs::path p = entry.path();
    if(!audio_->isSupportedFile(p.string())) continue;
    auto canonical = fs::canonical(p, ec);
    if(ec) continue;
    files.push_back(canonical.string());
  }
  if(files.empty()) return std::nullopt;
  static std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<size_t> dist(0, files.size() - 1);
  return files[dist(rng)];
}

bool StateHandler::playRandomReturnTrack(){
  if(!audio_) return false;
  const char* home = std::getenv("HOME");
  if(!home){
    RCLCPP_WARN(get_logger(), "HOME no definido: sin audios Return");
    return false;
  }
  auto path = randomTrackInDir((fs::path(home) / "Music" / "Return").string());
  if(!path) return false;
  return audio_->play(*path);
}

void StateHandler::sendEyeAction(EyeAction action){
  if(!eye_action_pub_) return;
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(action);
  eye_action_pub_->publish(msg);
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

  if(current_state_ && m == Mood::ESPERA && current_mood_ != Mood::ESPERA){
    pending_return_ = true;
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
    case Mood::PEO:
      current_state_ = std::make_unique<PeoState>();
      break;
    case Mood::ESPERA:
      current_state_ = std::make_unique<EsperaState>();
      break;
    case Mood::BAILOTEO:
      current_state_ = std::make_unique<BailoteoState>();
      break;
    case Mood::BAILOTEO_WAIT:
      current_state_ = std::make_unique<BailoteoWaitingState>();
      break;
    case Mood::FROWN:
      current_state_ = std::make_unique<SadState>();
      break;
    default:
      current_state_ = std::make_unique<EsperaState>();
      break;
  }

  if (current_state_) current_state_->onEnter(*this);
  current_mood_ = m;
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
