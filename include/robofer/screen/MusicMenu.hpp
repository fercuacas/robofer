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
  std::string formatTime(double sec) const;
  void drawControls(cv::Mat& img, int x, int y, int w, int h, int pad);

  robo_audio::AudioPlayer& player_;
  std::vector<std::string> tracks_;
  int sel_{0};
  int offset_{0};
  double font_scale_{0.15};

  bool playing_{false};
  bool paused_{false};
  std::string current_;
  double duration_{0.0};
  std::chrono::steady_clock::time_point start_time_;
  double paused_elapsed_{0.0};
};

} // namespace robo_ui
