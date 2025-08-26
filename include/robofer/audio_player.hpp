#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <sys/types.h>

namespace robo_audio {

class AudioPlayer {
public:
  explicit AudioPlayer(bool sim = false);
  ~AudioPlayer();

  void set_search_paths(const std::vector<std::string>& paths);
  void set_extensions(const std::vector<std::string>& exts);
  void set_alsa_device(const std::string& dev);

  void reindex();

  bool play(const std::string& key_or_path);
  void stop();
  bool is_playing() const;

private:
  std::string to_lower(std::string s) const;
  std::optional<std::string> resolve_key_or_path(const std::string& s) const;
  bool spawn_player(const std::string& filepath);

  std::vector<std::string> paths_;
  std::vector<std::string> exts_;
  std::unordered_map<std::string, std::string> index_;
  std::string alsa_dev_;
  pid_t child_pid_{-1};
};

} // namespace robo_audio

