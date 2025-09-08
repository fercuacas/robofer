#pragma once
#include <vector>
#include <string>
#include <chrono>
#include "robofer/audio/AudioPlayer.hpp"
#include "robofer/screen/UiMenu.hpp"
#include <opencv2/core.hpp>

namespace robo_ui {

class MusicMenu {
public:
  explicit MusicMenu(robo_audio::AudioPlayer& player);

  void setFontScale(double s);
  void draw(cv::Mat& canvas);
  void onKey(UiKey key);

private:
  enum class Mode { MENU, TRACKS };
  Mode mode_{Mode::MENU};

  void drawMenu(cv::Mat& canvas);
  void drawTrackList(cv::Mat& canvas);

  robo_audio::AudioPlayer& player_;
  std::vector<std::string> tracks_;
  int track_sel_{0};
  int track_offset_{0};
  int row_sel_{1};
  double font_scale_{0.15};

  bool playing_{false};
  bool paused_{false};
  std::string current_;
  double duration_{0.0};
  std::chrono::steady_clock::time_point start_time_;
  double paused_elapsed_{0.0};
};

} // namespace robo_ui
