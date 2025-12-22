#include "robofer/screen/VolumeMenu.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace robo_ui {

namespace {

std::string runCommand(const std::string& cmd){
  std::array<char, 128> buf{};
  std::string out;
  FILE* pipe = popen(cmd.c_str(), "r");
  if(!pipe) return out;
  while(fgets(buf.data(), static_cast<int>(buf.size()), pipe)){
    out += buf.data();
  }
  pclose(pipe);
  return out;
}

bool extractPercent(const std::string& text, int& out){
  size_t pos = text.find('%');
  if(pos == std::string::npos) return false;
  size_t start = pos;
  while(start > 0 && std::isdigit(static_cast<unsigned char>(text[start - 1]))){
    --start;
  }
  if(start == pos) return false;
  try {
    int val = std::stoi(text.substr(start, pos - start));
    out = std::clamp(val, 0, 100);
    return true;
  } catch(...) {
    return false;
  }
}

} // namespace

VolumeMenu::VolumeMenu() = default;

void VolumeMenu::setFontScale(double s){
  font_scale_ = std::clamp(s, 0.1, 2.0);
}

void VolumeMenu::enter(){
  editing_ = false;
  exit_requested_ = false;
  refreshVolume();
}

bool VolumeMenu::takeExit(){
  if(!exit_requested_) return false;
  exit_requested_ = false;
  return true;
}

bool VolumeMenu::refreshVolume(){
  int val = 0;
  volume_valid_ = readVolume(val);
  if(volume_valid_) volume_ = val;
  return volume_valid_;
}

bool VolumeMenu::readVolume(int& out){
  std::string output = runCommand("amixer sget 'LINEOUT volume' 2>/dev/null");
  if(output.empty()) return false;
  return extractPercent(output, out);
}

bool VolumeMenu::writeVolume(int volume){
  std::string cmd = "amixer sset 'LINEOUT volume' " + std::to_string(volume) + "% >/dev/null 2>&1";
  int rc = std::system(cmd.c_str());
  return rc == 0;
}

void VolumeMenu::setVolume(int volume){
  int clamped = std::clamp(volume, 0, 100);
  if(writeVolume(clamped)){
    volume_ = clamped;
    volume_valid_ = true;
  }
}

void VolumeMenu::onKey(UiKey key){
  switch(key){
    case UiKey::OK:
      if(!editing_) editing_ = true;
      break;
    case UiKey::BACK:
      exit_requested_ = true;
      editing_ = false;
      break;
    case UiKey::UP:
      if(editing_) setVolume(volume_ + 5);
      break;
    case UiKey::DOWN:
      if(editing_) setVolume(volume_ - 5);
      break;
    default:
      break;
  }
}

void VolumeMenu::draw(cv::Mat& canvas){
  const int W = canvas.cols;
  const int H = canvas.rows;
  canvas.setTo(cv::Scalar(40,40,40));

  int baseline = 0;
  cv::Size sample_sz = cv::getTextSize("Ag", cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
  int text_pad = std::max(2, static_cast<int>(std::round(4 * font_scale_)));
  int line_h = std::max(8, sample_sz.height + baseline + text_pad * 2);

  std::string title = "Volumen";
  cv::putText(canvas, title, cv::Point(text_pad, text_pad + sample_sz.height),
              cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar(255,255,255), 1, cv::LINE_8);

  std::string vol_line = volume_valid_
                           ? ("Vol: " + std::to_string(volume_) + "%")
                           : "Vol: --";
  cv::Scalar vol_color = editing_ ? cv::Scalar(0,255,0) : cv::Scalar(255,255,255);
  cv::putText(canvas, vol_line, cv::Point(text_pad, text_pad + sample_sz.height + line_h),
              cv::FONT_HERSHEY_SIMPLEX, font_scale_, vol_color, 1, cv::LINE_8);

  std::string hint1 = editing_ ? "UP/DOWN ajusta" : "OK editar";
  std::string hint2 = "BACK salir";
  int hint_y = H - line_h * 2;
  cv::putText(canvas, hint1, cv::Point(text_pad, hint_y + sample_sz.height),
              cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar(200,200,200), 1, cv::LINE_8);
  cv::putText(canvas, hint2, cv::Point(text_pad, hint_y + line_h + sample_sz.height),
              cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar(200,200,200), 1, cv::LINE_8);
}

} // namespace robo_ui
