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

/** Colors (monochrome) — map to 8-bit gray: 0 or 255 */
constexpr int BGCOLOR = 0;   /**< Background/overlays. */
constexpr int MAINCOLOR = 255; /**< Drawings. */

/** Mood presets used to adjust eye expression */
enum Mood : uint8_t { DEFAULT = 0, TIRED = 1, ANGRY = 2, HAPPY = 3, FROWN = 4 };

/** Predefined gaze positions */
enum Pos : uint8_t { CENTER=0, N=1, NE=2, E=3, SE=4, S=5, SW=6, W=7, NW=8 };

/**
 * @brief Animated cartoon eyes renderer.
 *
 * RoboEyes produces frames representing expressive eyes. The class manages
 * timing, mood transitions and basic animations such as blinking or
 * looking around.
 */
class RoboEyes
{
public:
  /**
   * @brief Construct the eye renderer.
   */
  RoboEyes();

  /**
   * @brief Initialise canvas dimensions and frame rate.
   * @param width Width of each eye canvas.
   * @param height Height of each eye canvas.
   * @param fps Desired frames per second.
   */
  void begin(int width, int height, int fps);

  /**
   * @brief Update animation state and render to internal canvas.
   */
  void update();

  /**
   * @brief Get current 8-bit frame.
   * @return Rendered canvas.
   */
  const cv::Mat& frame() const { return canvas_; }

  // setters
  /**
   * @brief Adjust target framerate.
   * @param fps Desired frames per second.
   */
  void setFramerate(int fps);
  /**
   * @brief Set eye dimensions.
   * @param left_w Width of left eye.
   * @param left_h Height of left eye.
   * @param right_w Width of right eye.
   * @param right_h Height of right eye.
   */
  void setSize(int left_w, int left_h, int right_w, int right_h);
  /**
   * @brief Set corner radii for left and right eyes.
   * @param left_r Radius for left eye corners.
   * @param right_r Radius for right eye corners.
   */
  void setBorderRadius(int left_r, int right_r);
  /**
   * @brief Space between eyes in pixels.
   * @param px Separation distance.
   */
  void setSpaceBetween(int px);

  /**
   * @brief Change overall mood expression.
   * @param m Desired mood.
   */
  void setMood(Mood m);
  /**
   * @brief Set gaze position.
   * @param p New gaze position.
   */
  void setPosition(Pos p);

  /**
   * @brief Configure automatic blinking behaviour.
   * @param active Whether blinking is active.
   * @param interval_s Base interval in seconds.
   * @param variation_s Random variation in seconds.
   */
  void setAutoblinker(bool active, int interval_s, int variation_s);
  /**
   * @brief Configure idle animation behaviour.
   * @param active Whether idle behaviour is active.
   * @param interval_s Base interval in seconds.
   * @param variation_s Random variation in seconds.
   */
  void setIdle(bool active, int interval_s, int variation_s);
  /**
   * @brief Enable or disable curious behaviour.
   * @param on True to enable.
   */
  void setCurious(bool on) { curious_ = on; }
  /**
   * @brief Render as a single cyclops eye when true.
   * @param on True for single eye.
   */
  void setCyclops(bool on) { cyclops_ = on; }

  // basic animations
  /**
   * @brief Close both eyes.
   */
  void closeBoth();
  /**
   * @brief Open both eyes.
   */
  void openBoth();
  /**
   * @brief Blink with both eyes.
   */
  void blinkBoth();

  // macro (single-shot)
  /**
   * @brief Play "confused" animation once.
   */
  void anim_confused();
  /**
   * @brief Play "laugh" animation once.
   */
  void anim_laugh();

  // === Aliases con los nombres de la librería original ===
  /**
   * @brief Alias of setSize for backward compatibility.
   * @param leftEye Width/height for left eye.
   * @param rightEye Width/height for right eye.
   */
  void setWidth(int leftEye, int rightEye);
  /**
   * @brief Alias of setSize for backward compatibility.
   * @param leftEye Height for left eye.
   * @param rightEye Height for right eye.
   */
  void setHeight(int leftEye, int rightEye);
  /**
   * @brief Alias of setBorderRadius for backward compatibility.
   * @param leftEye Radius for left eye.
   * @param rightEye Radius for right eye.
   */
  void setBorderradius(int leftEye, int rightEye);
  /**
   * @brief Alias of setSpaceBetween for backward compatibility.
   * @param space Separation distance.
   */
  void setSpacebetween(int space);
  /**
   * @brief Alias of setCurious.
   * @param curiousBit Enable curiosity flag.
   */
  void setCuriosity(bool curiousBit) { setCurious(curiousBit); }

  // === Overloads “cómodos” ===
  /**
   * @brief Enable autoblinker using existing timing parameters.
   * @param active Whether blinking is active.
   */
  void setAutoblinker(bool active);
  /**
   * @brief Enable idle behaviour using existing timing parameters.
   * @param active Whether idle behaviour is active.
   */
  void setIdle(bool active);

  // === Control por ojo ===
  /**
   * @brief Close both eyes (alias).
   */
  void close();
  /**
   * @brief Open both eyes (alias).
   */
  void open();
  /**
   * @brief Blink both eyes (alias).
   */
  void blink();
  /**
   * @brief Close specific eyes.
   * @param left Close left eye.
   * @param right Close right eye.
   */
  void close(bool left, bool right);
  /**
   * @brief Open specific eyes.
   * @param left Open left eye.
   * @param right Open right eye.
   */
  void open(bool left, bool right);
  /**
   * @brief Blink specific eyes.
   * @param left Blink left eye.
   * @param right Blink right eye.
   */
  void blink(bool left, bool right);

  // === Flickers directos ===
  /**
   * @brief Horizontal flicker with custom amplitude.
   * @param flickerBit Enable flag.
   * @param amplitude Flicker amplitude.
   */
  void setHFlicker(bool flickerBit, int amplitude);
  /**
   * @brief Horizontal flicker using current amplitude.
   * @param flickerBit Enable flag.
   */
  void setHFlicker(bool flickerBit);
  /**
   * @brief Vertical flicker with custom amplitude.
   * @param flickerBit Enable flag.
   * @param amplitude Flicker amplitude.
   */
  void setVFlicker(bool flickerBit, int amplitude);
  /**
   * @brief Vertical flicker using current amplitude.
   * @param flickerBit Enable flag.
   */
  void setVFlicker(bool flickerBit);

  // === Getters con el nombre original ===
  /**
   * @brief Get horizontal screen constraint.
   * @return Constraint in pixels.
   */
  int getScreenConstraint_X() const;
  /**
   * @brief Get vertical screen constraint.
   * @return Constraint in pixels.
   */
  int getScreenConstraint_Y() const;
    uint64_t frownFlickerLastChange_ = 0;
  int frownFlickerIntervalMs_ = 100; // cambia cada 100ms (~10 Hz)
private:
  // time helpers
  /**
   * @brief Get current time in milliseconds.
   * @return Timestamp in ms.
   */
  static uint64_t now_ms();
  /**
   * @brief Generate a random integer in [0, max_inclusive].
   * @param max_inclusive Maximum value.
   * @return Random number.
   */
  int rnd(int max_inclusive);

  // geometry helpers
  /**
   * @brief Horizontal screen constraint based on current layout.
   * @return Constraint in pixels.
   */
  int screenConstraintX() const;
  /**
   * @brief Vertical screen constraint based on current layout.
   * @return Constraint in pixels.
   */
  int screenConstraintY() const;

  // drawing helpers
  /**
   * @brief Clear image with a gray value.
   * @param img Image to clear.
   * @param gray Gray value.
   */
  static void clear(cv::Mat& img, int gray=0);
  /**
   * @brief Draw a filled rounded rectangle.
   * @param img Target image.
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param w Width.
   * @param h Height.
   * @param r Corner radius.
   * @param gray Gray value.
   */
  static void fillRoundRect(cv::Mat& img, int x, int y, int w, int h, int r, int gray);
  /**
   * @brief Draw a filled triangle.
   * @param img Target image.
   * @param a Vertex A.
   * @param b Vertex B.
   * @param c Vertex C.
   * @param gray Gray value.
   */
  static void fillTriangle(cv::Mat& img, cv::Point a, cv::Point b, cv::Point c, int gray);
  /**
   * @brief Draw a filled rectangle.
   * @param img Target image.
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param w Width.
   * @param h Height.
   * @param gray Gray value.
   */
  static void fillRect(cv::Mat& img, int x, int y, int w, int h, int gray);
  /**
   * @brief Draw a filled circle.
   * @param img Target image.
   * @param cx Center X.
   * @param cy Center Y.
   * @param r Radius.
   * @param gray Gray value.
   */
  static void fillCircle(cv::Mat& img, int cx, int cy, int r, int gray);

  /**
   * @brief Render the eyes into the internal canvas.
   */
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