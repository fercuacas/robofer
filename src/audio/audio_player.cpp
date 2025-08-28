#include "robofer/audio/audio_player.hpp"

#include <algorithm>
#include <cctype>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <optional>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace robo_audio {

AudioPlayer::AudioPlayer(bool sim) {
  paths_ = {"/opt/robofer/sounds", "/home/orangepi/Music", "/home/pi/Music"};
  exts_  = {".wav", ".mp3"};
  if(!sim) {
    alsa_dev_ = "hw:0,0";
  }
}

AudioPlayer::~AudioPlayer() {
  stop();
}

void AudioPlayer::set_search_paths(const std::vector<std::string>& paths){ paths_ = paths; }
void AudioPlayer::set_extensions(const std::vector<std::string>& exts){ exts_ = exts; }
void AudioPlayer::set_alsa_device(const std::string& dev){ alsa_dev_ = dev; }

std::string AudioPlayer::to_lower(std::string s) const{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  return s;
}

void AudioPlayer::reindex() {
  index_.clear();
  for(const auto& base : paths_){
    fs::path root(base);
    std::error_code ec;
    if(!fs::exists(root, ec)) continue;
    for(fs::recursive_directory_iterator it(root, ec), end; it != end; it.increment(ec)){
      if(ec) continue;
      const auto& p = it->path();
      if(!fs::is_regular_file(p, ec)) continue;
      std::string ext = to_lower(p.extension().string());
      if(std::find(exts_.begin(), exts_.end(), ext) == exts_.end()) continue;
      std::string key = to_lower(p.stem().string());
      index_[key] = fs::canonical(p, ec).string();
    }
  }
  std::cerr << "[AudioPlayer] Indexados " << index_.size() << " archivos.\n";
}

std::optional<std::string> AudioPlayer::resolve_key_or_path(const std::string& s) const {
  fs::path p(s);
  std::error_code ec;
  if(fs::exists(p, ec) && fs::is_regular_file(p, ec)){
    return fs::canonical(p, ec).string();
  }
  auto key = to_lower(s);
  auto it = index_.find(key);
  if(it != index_.end()) return it->second;
  return std::nullopt;
}

bool AudioPlayer::spawn_player(const std::string& filepath){
  std::string ext = to_lower(fs::path(filepath).extension().string());
  bool is_wav = (ext == ".wav");
  bool is_mp3 = (ext == ".mp3");

  if(!is_wav && !is_mp3){
    std::cerr << "[AudioPlayer] Extensión no soportada: " << ext << "\n";
    return false;
  }

  stop();

  child_pid_ = fork();
  if(child_pid_ < 0){
    std::perror("fork");
    child_pid_ = -1;
    return false;
  }

  if(child_pid_ == 0){
    if(is_wav){
      if(alsa_dev_.empty()){
        execlp("aplay", "aplay", filepath.c_str(), (char*)nullptr);
      }else{
        execlp("aplay", "aplay", "-D", alsa_dev_.c_str(), filepath.c_str(), (char*)nullptr);
      }
    } else {
      if(alsa_dev_.empty()){
        execlp("mpg123", "mpg123", filepath.c_str(), (char*)nullptr);
      } else {
        execlp("mpg123", "mpg123", "-a", alsa_dev_.c_str(), filepath.c_str(), (char*)nullptr);
      }
    }
    std::perror("execlp");
    _exit(127);
  }

  std::cerr << "[AudioPlayer] Reproduciendo: " << filepath
            << " (pid=" << child_pid_ << ")\n";
  return true;
}

bool AudioPlayer::play(const std::string& key_or_path){
  auto resolved = resolve_key_or_path(key_or_path);
  if(!resolved){
    std::cerr << "[AudioPlayer] No encontrado: " << key_or_path << "\n";
    return false;
  }
  return spawn_player(*resolved);
}

void AudioPlayer::stop(){
  if(child_pid_ <= 0) return;

  if(kill(child_pid_, SIGINT) == 0){
    int status = 0;
    for(int i=0;i<15;++i){
      pid_t r = waitpid(child_pid_, &status, WNOHANG);
      if(r == child_pid_) break;
      usleep(100*1000);
    }
  }
  if(kill(child_pid_, 0) == 0){
    kill(child_pid_, SIGKILL);
    int status = 0;
    waitpid(child_pid_, &status, 0);
  }
  std::cerr << "[AudioPlayer] Reproducción detenida.\n";
  child_pid_ = -1;
}

bool AudioPlayer::is_playing() const {
  if(child_pid_ <= 0) return false;
  return (kill(child_pid_, 0) == 0);
}

} // namespace robo_audio

