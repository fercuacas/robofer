#include "robofer/audio/AudioPlayer.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <filesystem>
#include <iostream>
#include <optional>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace fs = std::filesystem;

namespace robo_audio {

AudioPlayer::AudioPlayer(bool sim) {
  // Default search locations: built-in sounds plus a generic $HOME/Music
  paths_.push_back("/opt/robofer/sounds");
  if(const char* home = std::getenv("HOME")){
    paths_.push_back(std::string(home) + "/Music");
  }
  // Legacy paths for specific boards
  paths_.push_back("/home/orangepi/Music");
  paths_.push_back("/home/pi/Music");

  exts_  = {".wav", ".mp3"};
  // Use default ALSA device unless explicitly configured
  if(!sim){
    alsa_dev_.clear();
  }
}

AudioPlayer::~AudioPlayer() {
  stop();
}

void AudioPlayer::setSearchPaths(const std::vector<std::string>& paths){ paths_ = paths; }
void AudioPlayer::setExtensions(const std::vector<std::string>& exts){ exts_ = exts; }
void AudioPlayer::setAlsaDevice(const std::string& dev){ alsa_dev_ = dev; }
void AudioPlayer::setVolumePercent(int volume){
  if(volume < 0) volume = 0;
  if(volume > 100) volume = 100;
  volume_percent_ = volume;
}

std::string AudioPlayer::toLower(std::string s) const{
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
      std::string ext = toLower(p.extension().string());
      if(std::find(exts_.begin(), exts_.end(), ext) == exts_.end()) continue;
      std::string key = toLower(p.stem().string());
      index_[key] = fs::canonical(p, ec).string();
    }
  }
  std::cerr << "[AudioPlayer] Indexados " << index_.size() << " archivos.\n";
}

std::optional<std::string> AudioPlayer::resolveKeyOrPath(const std::string& s) const {
  fs::path p(s);
  std::error_code ec;
  if(fs::exists(p, ec) && fs::is_regular_file(p, ec)){
    return fs::canonical(p, ec).string();
  }
  auto key = toLower(s);
  auto it = index_.find(key);
  if(it != index_.end()) return it->second;
  return std::nullopt;
}

std::optional<std::string> AudioPlayer::findExecutable(const std::string& name) const {
  auto is_accessible = [](const std::string& path){
    return !path.empty() && access(path.c_str(), X_OK) == 0;
  };

  if(name.find('/') != std::string::npos){
    if(is_accessible(name)) return name;
    return std::nullopt;
  }

  const char* path_env = std::getenv("PATH");
  if(!path_env) return std::nullopt;

  std::string path(path_env);
  size_t start = 0;
  while(start <= path.size()){
    size_t end = path.find(':', start);
    std::string dir = (end == std::string::npos) ? path.substr(start) : path.substr(start, end - start);
    if(dir.empty()) dir = ".";
    fs::path candidate = fs::path(dir) / name;
    if(is_accessible(candidate.string())) return candidate.string();
    if(end == std::string::npos) break;
    start = end + 1;
  }
  return std::nullopt;
}

bool AudioPlayer::spawnPlayer(const std::string& filepath){
  std::string ext = toLower(fs::path(filepath).extension().string());
  bool is_wav = (ext == ".wav");
  bool is_mp3 = (ext == ".mp3");

  if(!is_wav && !is_mp3){
    std::cerr << "[AudioPlayer] Extensión no soportada: " << ext << "\n";
    return false;
  }

  std::vector<std::string> cmd;
  auto push_common = [&](const std::string& exe){
    cmd.clear();
    cmd.push_back(exe);
  };

  if(is_wav){
    if(auto exe = findExecutable("aplay")){
      push_common(*exe);
      if(!alsa_dev_.empty()){
        cmd.push_back("-D");
        cmd.push_back(alsa_dev_);
      }
      cmd.push_back(filepath);
    } else if(auto ffplay = findExecutable("ffplay")){
      push_common(*ffplay);
      cmd.push_back("-autoexit");
      cmd.push_back("-nodisp");
      cmd.push_back("-loglevel");
      cmd.push_back("error");
      cmd.push_back("-volume");
      cmd.push_back(std::to_string(volume_percent_));
      cmd.push_back(filepath);
    } else {
      std::cerr << "[AudioPlayer] No se encontró reproductor WAV (aplay/ffplay).\n";
      return false;
    }
  } else if(is_mp3){
    if(auto exe = findExecutable("mpg123")){
      push_common(*exe);
      int scale = static_cast<int>(32768.0 * (static_cast<double>(volume_percent_) / 100.0));
      if(scale < 0) scale = 0;
      if(scale > 32768) scale = 32768;
      cmd.push_back("-f");
      cmd.push_back(std::to_string(scale));
      if(!alsa_dev_.empty()){
        cmd.push_back("-a");
        cmd.push_back(alsa_dev_);
      }
      cmd.push_back(filepath);
    } else if(auto ffplay = findExecutable("ffplay")){
      push_common(*ffplay);
      cmd.push_back("-autoexit");
      cmd.push_back("-nodisp");
      cmd.push_back("-loglevel");
      cmd.push_back("error");
      cmd.push_back("-volume");
      cmd.push_back(std::to_string(volume_percent_));
      cmd.push_back(filepath);
    } else {
      std::cerr << "[AudioPlayer] No se encontró reproductor MP3 (mpg123/ffplay).\n";
      return false;
    }
// >>>>>>> Stashed changes
  }

  stop();

  child_pid_ = fork();
  if(child_pid_ < 0){
    std::perror("fork");
    child_pid_ = -1;
    return false;
  }

  if(child_pid_ == 0){
    std::vector<char*> argv;
    argv.reserve(cmd.size() + 1);
    for(auto& s : cmd){
      argv.push_back(const_cast<char*>(s.c_str()));
    }
    argv.push_back(nullptr);
    execv(argv[0], argv.data());
    std::perror(argv[0]);
    _exit(127);
  }

  std::cerr << "[AudioPlayer] Reproduciendo: " << filepath
            << " (pid=" << child_pid_ << ")\n";
  return true;
}

bool AudioPlayer::play(const std::string& key_or_path){
  auto resolved = resolveKeyOrPath(key_or_path);
  if(!resolved){
    std::cerr << "[AudioPlayer] No encontrado: " << key_or_path << "\n";
    return false;
  }
  if(!spawnPlayer(*resolved)) return false;
  current_file_ = *resolved;
  start_time_ = std::chrono::steady_clock::now();
  paused_ = false;
  return true;
}

void AudioPlayer::pollChildExit(){
  if(child_pid_ <= 0) return;
  int status = 0;
  pid_t r = waitpid(child_pid_, &status, WNOHANG);
  if(r == 0) return;
  if(r < 0){
    if(errno == ECHILD){
      child_pid_ = -1;
      paused_ = false;
      current_file_.clear();
      start_time_ = {};
    }
    return;
  }

  std::cerr << "[AudioPlayer] Reproducción finalizada.\n";
  child_pid_ = -1;
  paused_ = false;
  current_file_.clear();
  start_time_ = {};
}

void AudioPlayer::stop(){
  if(child_pid_ <= 0) return;

  const pid_t pid = child_pid_;
  child_pid_ = -1;
  paused_ = false;
  current_file_.clear();

  auto wait_with_timeout = [&](std::chrono::milliseconds timeout){
    int status = 0;
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while(std::chrono::steady_clock::now() < deadline){
      pid_t r = waitpid(pid, &status, WNOHANG);
      if(r == pid) return true;
      if(r < 0) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  };

  bool terminated = false;
  if(kill(pid, SIGINT) == 0){
    terminated = wait_with_timeout(std::chrono::milliseconds(250));
  } else if(errno == ESRCH){
    terminated = true;
  }

  if(!terminated){
    kill(pid, SIGKILL);
    int status = 0;
    waitpid(pid, &status, 0);
  }

  std::cerr << "[AudioPlayer] Reproducción detenida.\n";
}

bool AudioPlayer::isPlaying(){
  pollChildExit();
  if(child_pid_ <= 0) return false;
  if(kill(child_pid_, 0) == 0) return true;
  pollChildExit();
  return child_pid_ > 0;
}

bool AudioPlayer::pause(){
  pollChildExit();
  if(child_pid_ <= 0) return false;
  if(paused_) return true;
  if(kill(child_pid_, SIGSTOP) == 0){
    paused_ = true;
    return true;
  }
  return false;
}

bool AudioPlayer::resume(){
  pollChildExit();
  if(child_pid_ <= 0) return false;
  if(!paused_) return true;
  if(kill(child_pid_, SIGCONT) == 0){
    paused_ = false;
    start_time_ = std::chrono::steady_clock::now();
    return true;
  }
  return false;
}

std::vector<std::string> AudioPlayer::listTracks() const {
  std::vector<std::string> keys;
  keys.reserve(index_.size());
  for(const auto& kv : index_) keys.push_back(kv.first);
  std::sort(keys.begin(), keys.end());
  return keys;
}

bool AudioPlayer::isSupportedFile(const std::string& path) const {
  std::string ext = toLower(fs::path(path).extension().string());
  return std::find(exts_.begin(), exts_.end(), ext) != exts_.end();
}

double AudioPlayer::getDuration(const std::string& key_or_path){
  auto resolved = resolveKeyOrPath(key_or_path);
  if(!resolved) return -1.0;
  std::string cmd = std::string("ffprobe -v error -show_entries format=duration -of "
                               "default=noprint_wrappers=1:nokey=1 \"") +
                    *resolved + "\"";
  FILE* fp = popen(cmd.c_str(), "r");
  if(!fp) return -1.0;
  char buf[128];
  if(!fgets(buf, sizeof(buf), fp)){ pclose(fp); return -1.0; }
  pclose(fp);
  return std::atof(buf);
}

} // namespace robo_audio
