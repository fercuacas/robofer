#pragma once
#include <opencv2/core.hpp>
#include "robofer/screen/UiMenu.hpp"

namespace robo_ui {

class VolumeMenu {
public:
  VolumeMenu();

  void setFontScale(double s);
  void enter();
  void draw(cv::Mat& canvas);
  void onKey(UiKey key);

  bool takeExit();

private:
  bool refreshVolume();
  bool readVolume(int& out);
  bool writeVolume(int volume);
  void setVolume(int volume);

  int volume_{0};
  bool volume_valid_{false};
  bool editing_{false};
  bool exit_requested_{false};
  double font_scale_{0.15};
};

} // namespace robo_ui
