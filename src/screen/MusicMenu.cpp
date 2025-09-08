#include "robofer/screen/MusicMenu.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace robo_ui {

MusicMenu::MusicMenu(robo_audio::AudioPlayer& player)
  : player_(player) {
  tracks_ = player_.listTracks();
}

void MusicMenu::setFontScale(double s){ font_scale_ = std::clamp(s, 0.1, 2.0); }

void MusicMenu::onKey(UiKey key){
  switch(key){
    case UiKey::UP:
      if(sel_ > 0) sel_--;
      if(sel_ < offset_) offset_ = sel_;
      break;
    case UiKey::DOWN:
      if(sel_ + 1 < (int)tracks_.size()) sel_++;
      break;
    case UiKey::OK:
      if(tracks_.empty()) break;
      {
        std::string track = tracks_[sel_];
        if(!playing_ || track != current_){
          if(player_.play(track)){
            playing_ = true;
            paused_ = false;
            current_ = track;
            duration_ = player_.getDuration(track);
            start_time_ = std::chrono::steady_clock::now();
            paused_elapsed_ = 0.0;
          }
        } else {
          if(paused_){
            if(player_.resume()){
              paused_ = false;
              start_time_ = std::chrono::steady_clock::now() -
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(paused_elapsed_));
            }
          } else {
            if(player_.pause()){
              paused_ = true;
              paused_elapsed_ = std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - start_time_).count();
            }
          }
        }
      }
      break;
    case UiKey::BACK:
      if(playing_){
        player_.stop();
        playing_ = false;
        paused_ = false;
        current_.clear();
      }
      break;
    default:
      break;
  }
}

std::string MusicMenu::formatTime(double sec) const{
  if(sec < 0) return "--:--";
  int s = static_cast<int>(sec + 0.5);
  int m = s/60; s %= 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d:%02d", m, s);
  return buf;
}

void MusicMenu::drawControls(cv::Mat& img, int x, int y, int w, int h, int pad){
  int icon_size = h - pad*2;
  int cx = x + pad;
  cv::Rect r_play(cx, y + pad, icon_size, icon_size);
  if(playing_ && !paused_){
    // pause icon
    int bar_w = icon_size/3;
    cv::rectangle(img, cv::Rect(r_play.x, r_play.y, bar_w, icon_size),
                  cv::Scalar(255,255,255), cv::FILLED);
    cv::rectangle(img, cv::Rect(r_play.x + icon_size - bar_w, r_play.y, bar_w, icon_size),
                  cv::Scalar(255,255,255), cv::FILLED);
  } else {
    // play icon
    cv::Point pts[3] = {
      {r_play.x, r_play.y},
      {r_play.x, r_play.y + icon_size},
      {r_play.x + icon_size, r_play.y + icon_size/2}
    };
    cv::fillConvexPoly(img, pts, 3, cv::Scalar(255,255,255));
  }
  cx += icon_size + pad;
  cv::Rect r_stop(cx, y + pad, icon_size, icon_size);
  cv::rectangle(img, r_stop, cv::Scalar(255,255,255), cv::FILLED);
  cx += icon_size + pad;
  cv::Rect r_next(cx, y + pad, icon_size, icon_size);
  cv::Point pts2[3] = {
    {r_next.x, r_next.y},
    {r_next.x, r_next.y + icon_size},
    {r_next.x + icon_size/2, r_next.y + icon_size/2}
  };
  cv::fillConvexPoly(img, pts2, 3, cv::Scalar(255,255,255));
  cv::rectangle(img, cv::Rect(r_next.x + icon_size/2 + 1, r_next.y, icon_size/2 -1, icon_size),
                cv::Scalar(255,255,255), cv::FILLED);

  std::string info;
  if(playing_){
    double elapsed = paused_ ? paused_elapsed_ :
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    info = std::string("Reproduciendo: ") + current_ + " " +
      formatTime(elapsed) + " / " + formatTime(duration_);
  }
  int baseline = 0;
  cv::Size sz = cv::getTextSize(info, cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
  int tx = x + pad*2 + icon_size*3 + pad*2;
  int ty = y + h - pad;
  cv::putText(img, info, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
              font_scale_, cv::Scalar(255,255,255), 1, cv::LINE_8);
}

void MusicMenu::draw(cv::Mat& canvas){
  const int W = canvas.cols, H = canvas.rows;
  int baseline = 0;
  int max_w = 0;
  for(const auto& t : tracks_){
    int w = cv::getTextSize(t, cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline).width;
    max_w = std::max(max_w, w);
  }
  const int outer_pad = std::max(4, W/64);
  const int panel_w = std::min(W - 2*outer_pad, std::max(max_w + 16, W*3/5));
  cv::Size sample_sz = cv::getTextSize("Ag", cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
  const int text_pad = std::max(2, static_cast<int>(std::round(4 * font_scale_)));
  const int line_h = std::max(8, sample_sz.height + baseline + text_pad*2);
  const int controls_h = line_h;
  cv::Size title_sz = cv::getTextSize("Music", cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
  const int header_h = title_sz.height + text_pad*2;
  const int panel_h = H;
  const int max_visible = std::max(1, (panel_h - header_h - text_pad - controls_h) / line_h);
  const int visible = std::min(max_visible, (int)tracks_.size());

  if(sel_ < offset_) offset_ = sel_;
  if(sel_ >= offset_ + visible) offset_ = sel_ - visible + 1;
  offset_ = std::clamp(offset_, 0, std::max(0, (int)tracks_.size() - visible));

  int x = (W - panel_w)/2;
  int y = 0;
  cv::Mat roi = canvas(cv::Rect(x, y, panel_w, panel_h));
  roi.setTo(cv::Scalar(40,40,40));
  cv::rectangle(canvas, cv::Rect(x, y, panel_w, panel_h), cv::Scalar(200,200,200), 1, cv::LINE_8);

  // header
  cv::rectangle(canvas, cv::Rect(x+1, y+1, panel_w-2, header_h-2), cv::Scalar(200,200,200), cv::FILLED);
  cv::putText(canvas, "Music", cv::Point(x + text_pad +2, y + title_sz.height + text_pad),
              cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar(0,0,0), 1, cv::LINE_8);

  // items
  for(int row=0; row<visible && offset_+row < (int)tracks_.size(); ++row){
    int i = offset_ + row;
    int row_y = y + header_h + text_pad + row*line_h;
    bool sel = (i==sel_);
    if(sel){
      cv::rectangle(canvas, cv::Rect(x+2, row_y, panel_w-4, line_h-2),
                    cv::Scalar(200,200,200), cv::FILLED);
    }
    int tx = x + text_pad + 4;
    int ty = row_y + std::min(line_h - text_pad, sample_sz.height + text_pad);
    cv::putText(canvas, tracks_[i], cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
                font_scale_, cv::Scalar(255,255,255), 1, cv::LINE_8);
  }

  // controls at bottom
  int ctrl_y = y + panel_h - controls_h;
  drawControls(canvas, x, ctrl_y, panel_w, controls_h, text_pad);
}

} // namespace robo_ui
