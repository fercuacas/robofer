#include "robofer/screen/MusicMenu.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace robo_ui {

MusicMenu::MusicMenu(robo_audio::AudioPlayer& player)
  : player_(player) {
  tracks_ = player_.listTracks();
}

void MusicMenu::setFontScale(double s){
  font_scale_ = std::clamp(s, 0.1, 2.0);
}

void MusicMenu::onKey(UiKey key){
  switch(mode_){
    case Mode::MENU:
      switch(key){
        case UiKey::UP:
          if(row_sel_ > 1) row_sel_--;
          break;
        case UiKey::DOWN:
          if(row_sel_ < 3) row_sel_++;
          break;
        case UiKey::OK:
          if(row_sel_ == 1){
            tracks_ = player_.listTracks();
            mode_ = Mode::TRACKS;
          } else if(row_sel_ == 2){
            if(playing_){
                if(paused_){
                  if(player_.resume()){
                    paused_ = false;
                    start_time_ = std::chrono::steady_clock::now() -
                      std::chrono::duration_cast<std::chrono::steady_clock::time_point::duration>(
                        std::chrono::duration<double>(paused_elapsed_));
                  }
                } else {
                  if(player_.pause()){
                    paused_ = true;
                    paused_elapsed_ = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - start_time_).count();
                  }
                }
            } else if(!current_.empty()){
              if(player_.play(current_)){
                playing_ = true;
                paused_ = false;
                duration_ = player_.getDuration(current_);
                start_time_ = std::chrono::steady_clock::now();
                paused_elapsed_ = 0.0;
              }
            }
          } else if(row_sel_ == 3){
            if(playing_){
              player_.stop();
              playing_ = false;
              paused_ = false;
              current_.clear();
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
        default: break;
      }
      break;
    case Mode::TRACKS:
      switch(key){
        case UiKey::UP:
          if(track_sel_ > 0) track_sel_--;
          if(track_sel_ < track_offset_) track_offset_ = track_sel_;
          break;
        case UiKey::DOWN:
          if(track_sel_ + 1 < static_cast<int>(tracks_.size())) track_sel_++;
          break;
        case UiKey::OK:
          if(track_sel_ >= 0 && track_sel_ < static_cast<int>(tracks_.size())){
            std::string track = tracks_[track_sel_];
            if(player_.play(track)){
              playing_ = true;
              paused_ = false;
              current_ = track;
              duration_ = player_.getDuration(track);
              start_time_ = std::chrono::steady_clock::now();
              paused_elapsed_ = 0.0;
            }
          }
          mode_ = Mode::MENU;
          break;
        case UiKey::BACK:
          mode_ = Mode::MENU;
          break;
        default: break;
      }
      break;
  }
}

void MusicMenu::drawMenu(cv::Mat& canvas){
  const int W = canvas.cols, H = canvas.rows;
  canvas.setTo(cv::Scalar(40,40,40));
  int baseline = 0;
  cv::Size sample_sz = cv::getTextSize("Ag", cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
  int text_pad = std::max(2, static_cast<int>(std::round(4 * font_scale_)));
  int line_h = std::max(8, sample_sz.height + baseline + text_pad*2);

  // status row
  std::string status = "nothing";
  cv::Scalar st_color(200,200,200);
  if(playing_){
    if(paused_){
      status = "paused";
      st_color = cv::Scalar(0,0,255);
    } else {
      status = "playing";
      st_color = cv::Scalar(0,255,0);
    }
  }
  cv::putText(canvas, status, cv::Point(text_pad, text_pad + sample_sz.height),
              cv::FONT_HERSHEY_SIMPLEX, font_scale_, st_color, 1, cv::LINE_8);

  // current track row
  int song_y = line_h;
  if(row_sel_ == 1){
    cv::rectangle(canvas, cv::Rect(0, song_y, W, line_h), cv::Scalar(80,80,80), cv::FILLED);
  }
  std::string song = current_.empty() ? "select song" : current_;
  cv::putText(canvas, song,
              cv::Point(text_pad, song_y + text_pad + sample_sz.height),
              cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar(255,255,255), 1, cv::LINE_8);

  // controls row at bottom
  int ctrl_y = H - line_h;
  int icon_size = line_h - text_pad*2;
  int total_w = icon_size*2 + text_pad;
  int cx = (W - total_w)/2;

  // play/pause button (row 2)
  cv::Rect r_play(cx, ctrl_y + text_pad, icon_size, icon_size);
  bool show_pause = playing_ && !paused_;
  cv::Scalar play_color = show_pause ? cv::Scalar(128,128,128) : cv::Scalar(0,255,0);
  if(row_sel_ == 2){
    cv::rectangle(canvas, r_play, cv::Scalar(200,200,200), 1);
  }
  if(show_pause){
    int bar_w = icon_size/3;
    cv::rectangle(canvas, cv::Rect(r_play.x, r_play.y, bar_w, icon_size), play_color, cv::FILLED);
    cv::rectangle(canvas, cv::Rect(r_play.x + icon_size - bar_w, r_play.y, bar_w, icon_size), play_color, cv::FILLED);
  } else {
    cv::Point pts[3] = {
      {r_play.x, r_play.y},
      {r_play.x, r_play.y + icon_size},
      {r_play.x + icon_size, r_play.y + icon_size/2}
    };
    cv::fillConvexPoly(canvas, pts, 3, play_color);
  }

  // stop button (row 3)
  cx += icon_size + text_pad;
  cv::Rect r_stop(cx, ctrl_y + text_pad, icon_size, icon_size);
  if(row_sel_ == 3){
    cv::rectangle(canvas, r_stop, cv::Scalar(200,200,200), 1);
  }
  cv::rectangle(canvas, r_stop, cv::Scalar(0,0,255), cv::FILLED);
}

void MusicMenu::drawTrackList(cv::Mat& canvas){
  const int W = canvas.cols, H = canvas.rows;
  canvas.setTo(cv::Scalar(40,40,40));
  int baseline = 0;
  cv::Size sample_sz = cv::getTextSize("Ag", cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
  int text_pad = std::max(2, static_cast<int>(std::round(4 * font_scale_)));
  int line_h = std::max(8, sample_sz.height + baseline + text_pad*2);
  int max_visible = std::max(1, H / line_h);
  if(track_sel_ < track_offset_) track_offset_ = track_sel_;
  if(track_sel_ >= track_offset_ + max_visible) track_offset_ = track_sel_ - max_visible + 1;
  track_offset_ = std::clamp(track_offset_, 0, std::max(0, static_cast<int>(tracks_.size()) - max_visible));

  for(int row=0; row<max_visible && track_offset_ + row < static_cast<int>(tracks_.size()); ++row){
    int i = track_offset_ + row;
    int row_y = row * line_h;
    bool sel = (i == track_sel_);
    if(sel){
      cv::rectangle(canvas, cv::Rect(0, row_y, W, line_h), cv::Scalar(80,80,80), cv::FILLED);
    }
    int tx = text_pad;
    int ty = row_y + text_pad + sample_sz.height;
    cv::putText(canvas, tracks_[i], cv::Point(tx, ty),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar(255,255,255), 1, cv::LINE_8);
  }
}

void MusicMenu::draw(cv::Mat& canvas){
  if(mode_ == Mode::MENU){
    drawMenu(canvas);
  } else {
    drawTrackList(canvas);
  }
}

} // namespace robo_ui

