/*
 * FluxGarage RoboEyes — Desktop/ROS2 C++ port (simulation only)
 *
 * Based on: FluxGarage RoboEyes for OLED Displays V1.0.1 — GPL-3.0-or-later
 * Copyright (C) 2024 Dennis Hoelscher
 * Ported for ROS 2 + OpenCV by <you>, 2025 — GPL-3.0-or-later
 */

#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <random>
#include <chrono>
#include <cstdint>

namespace robo_eyes
{

// Colors (monochrome) — map to 8-bit gray: 0 or 255
constexpr int BGCOLOR = 0;   // background/overlays
constexpr int MAINCOLOR = 255; // drawings

// Moods
enum Mood : uint8_t { DEFAULT = 0, TIRED = 1, ANGRY = 2, HAPPY = 3, FROWN = 4 };

// Predefined positions
enum Pos : uint8_t { CENTER=0, N=1, NE=2, E=3, SE=4, S=5, SW=6, W=7, NW=8 };

class RoboEyes
{
public:
  RoboEyes();

  void begin(int width, int height, int fps);

  // one tick: updates animation state and renders to internal canvas
  void update();

  // current frame (CV_8UC1)
  const cv::Mat& frame() const { return canvas_; }

  // setters
  void setFramerate(int fps);
  void setSize(int left_w, int left_h, int right_w, int right_h);
  void setBorderRadius(int left_r, int right_r);
  void setSpaceBetween(int px);

  void setMood(Mood m);
  void setPosition(Pos p);

  void setAutoblinker(bool active, int interval_s, int variation_s);
  void setIdle(bool active, int interval_s, int variation_s);
  void setCurious(bool on) { curious_ = on; }
  void setCyclops(bool on) { cyclops_ = on; }

  // basic animations
  void closeBoth();
  void openBoth();
  void blinkBoth();

  // macro (single-shot)
  void anim_confused();
  void anim_laugh();

  // === Aliases con los nombres de la librería original ===
  void setWidth(int leftEye, int rightEye);
  void setHeight(int leftEye, int rightEye);
  void setBorderradius(int leftEye, int rightEye);
  void setSpacebetween(int space);
  void setCuriosity(bool curiousBit) { setCurious(curiousBit); }

  // === Overloads “cómodos” ===
  void setAutoblinker(bool active);                 // usa valores actuales de s/var
  void setIdle(bool active);                        // usa valores actuales de s/var

  // === Control por ojo ===
  void close();                                     // ambos
  void open();                                      // ambos
  void blink();                                     // ambos
  void close(bool left, bool right);
  void open(bool left, bool right);
  void blink(bool left, bool right);

  // === Flickers directos ===
  void setHFlicker(bool flickerBit, int amplitude);
  void setHFlicker(bool flickerBit);                // mantiene amp actual
  void setVFlicker(bool flickerBit, int amplitude);
  void setVFlicker(bool flickerBit);                // mantiene amp actual

  // === Getters con el nombre original ===
  int getScreenConstraint_X() const;
  int getScreenConstraint_Y() const;
    uint64_t frownFlickerLastChange_ = 0;
  int frownFlickerIntervalMs_ = 100; // cambia cada 100ms (~10 Hz)
private:
  // time helpers
  static uint64_t now_ms();
  int rnd(int max_inclusive);

  // geometry helpers
  int screenConstraintX() const;
  int screenConstraintY() const;

  // drawing helpers
  static void clear(cv::Mat& img, int gray=0);
  static void fillRoundRect(cv::Mat& img, int x, int y, int w, int h, int r, int gray);
  static void fillTriangle(cv::Mat& img, cv::Point a, cv::Point b, cv::Point c, int gray);
  static void fillRect(cv::Mat& img, int x, int y, int w, int h, int gray);
  static void fillCircle(cv::Mat& img, int cx, int cy, int r, int gray);

  void drawEyes();

private:
  // canvas
  cv::Mat canvas_{}; // CV_8UC1
  int screen_w_=128, screen_h_=64;

  // FPS control
  int frame_interval_ms_ = 20; // 50 fps
  uint64_t fps_timer_ms_ = 0;

  // mood flags
  bool tired_=false, angry_=false, happy_=false;
  bool frown_=false; // <— ceño fruncido
  bool curious_=false; // outer eye grows when looking sideways
  bool cyclops_=false; // single eye
  bool eyeL_open_=false, eyeR_open_=false;

  // dimensions
  int spaceBetweenDefault_=10;
  int spaceBetweenCurrent_=10;
  int spaceBetweenNext_=10;



  int eyeL_w_def_=36, eyeL_h_def_=36;
  int eyeL_w_cur_=36, eyeL_h_cur_=1; // start closed
  int eyeL_w_next_=36, eyeL_h_next_=36;
  int eyeL_h_offset_=0;
  int eyeL_r_def_=8, eyeL_r_cur_=8, eyeL_r_next_=8;

  int eyeR_w_def_=36, eyeR_h_def_=36;
  int eyeR_w_cur_=36, eyeR_h_cur_=1;
  int eyeR_w_next_=36, eyeR_h_next_=36;
  int eyeR_h_offset_=0;
  int eyeR_r_def_=8, eyeR_r_cur_=8, eyeR_r_next_=8;

  // coordinates (left eye origin)
  int eyeLx_def_=0, eyeLy_def_=0;
  int eyeLx_=0, eyeLy_=0;
  int eyeLx_next_=0, eyeLy_next_=0;

  // right eye coords are derived
  int eyeRx_def_=0, eyeRy_def_=0;
  int eyeRx_=0, eyeRy_=0;
  int eyeRx_next_=0, eyeRy_next_=0;

  // eyelids (top/bottom)
  int eyelidsHeightMax_=18; // recomputed in begin()
  int eyelidsTiredH_=0, eyelidsTiredH_next_=0;
  int eyelidsAngryH_=0, eyelidsAngryH_next_=0;
  int eyelidsHappyBottomOffset_=0, eyelidsHappyBottomOffset_next_=0;
  int eyelidsFrownH_=0, eyelidsFrownH_next_=0; // <— profundidad del ceño


  // macro animations
  bool hFlicker_=false, hFlickerAlt_=false; int hFlickAmp_=2;
  bool vFlicker_=false, vFlickerAlt_=false; int vFlickAmp_=10;

  bool autoblinker_=false; int blinkInterval_s_=1, blinkVar_s_=4; uint64_t blinkTimer_=0;
  bool idle_=false; int idleInterval_s_=1, idleVar_s_=3; uint64_t idleTimer_=0;
  bool confused_=false; uint64_t confusedTimer_=0; int confusedDur_ms_=500; bool confusedToggle_=true;
  bool laugh_=false; uint64_t laughTimer_=0; int laughDur_ms_=500; bool laughToggle_=true;

  // RNG
  std::mt19937 rng_;
};

} // namespace robo_eyes