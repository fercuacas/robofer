#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <sys/types.h>

namespace robo_audio {

/**
 * @brief Simple audio playback utility.
 *
 * AudioPlayer searches for audio files in provided directories and
 * spawns an external player (e.g. `aplay`) to reproduce them. The
 * component can run in simulation mode where playback is skipped but
 * paths are still resolved.
 */
class AudioPlayer {
public:
  /**
   * @brief Construct a new AudioPlayer.
   * @param sim When true, no external process is launched.
   */
  explicit AudioPlayer(bool sim = false);

  /**
   * @brief Destructor. Stops any running playback.
   */
  ~AudioPlayer();

  /**
   * @brief Define directories where audio files are searched.
   * @param paths List of directories to scan.
   */
  void setSearchPaths(const std::vector<std::string>& paths);

  /**
   * @brief Define file extensions that are considered audio files.
   * @param exts Allowed filename suffixes.
   */
  void setExtensions(const std::vector<std::string>& exts);

  /**
   * @brief Set ALSA device name used by the external player.
   * @param dev Device identifier.
   */
  void setAlsaDevice(const std::string& dev);

  /** @brief Rebuild the lookup table of available audio files. */
  void reindex();

  /**
   * @brief Start playback of an audio file.
   * @param key_or_path Key or full path to the audio file.
   * @return true if playback was started.
   */
  bool play(const std::string& key_or_path);

  /**
   * @brief Stop current playback, if any.
   */
  void stop();

  /**
   * @brief Check whether an audio file is currently playing.
   * @return true if a child process is active.
   */
  bool isPlaying() const;

private:
  /**
   * @brief Convert string to lowercase for case-insensitive matching.
   * @param s Input string.
   * @return Lowercase copy.
   */
  std::string toLower(std::string s) const;

  /**
   * @brief Resolve key or path to a concrete audio file on disk.
   * @param s Key or path.
   * @return Resolved path if found.
   */
  std::optional<std::string> resolveKeyOrPath(const std::string& s) const;

  /**
   * @brief Launch the external player process for the given file.
   * @param filepath Path to audio file.
   * @return true if process spawned.
   */
  bool spawnPlayer(const std::string& filepath);

  std::vector<std::string> paths_;
  std::vector<std::string> exts_;
  std::unordered_map<std::string, std::string> index_;
  std::string alsa_dev_;
  pid_t child_pid_{-1};
};

} // namespace robo_audio

