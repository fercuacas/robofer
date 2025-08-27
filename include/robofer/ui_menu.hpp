#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

namespace robo_ui {

/**
 * @brief Navigation events generated from physical buttons.
 */
enum class UiKey : int { UP=0, DOWN=1, BACK=2, OK=3 };

/**
 * @brief Actions that the menu can request from external components.
 */
enum class MenuAction {
  NONE,
  SET_ANGRY,
  SET_SAD,
  SET_HAPPY,
  POWEROFF,
};

/**
 * @brief Menu controller responsible for navigation and drawing.
 *
 * MenuController maintains a menu tree, reacts to key events and renders
 * itself as an overlay onto an 8-bit grayscale canvas. The menu becomes
 * inactive after a configurable timeout without user interaction.
 */
class MenuController {
public:
  /**
   * @brief Construct a new MenuController.
   * @param on_action Callback invoked when an item triggers an action.
   */
  explicit MenuController(std::function<void(MenuAction)> on_action);

  /**
   * @brief Process a key event and reset the inactivity timer.
   * @param key Navigation input.
   */
  void on_key(UiKey key);

  /**
   * @brief Whether the menu is currently visible/active.
   * @return true if the menu is being displayed.
   */
  bool is_active() const;

  /**
   * @brief Draw the menu overlay onto the given canvas if active.
   * @param canvas Target image.
   */
  void draw(cv::Mat& canvas);

  /**
   * @brief Configure inactivity timeout in milliseconds (default 5000).
   * @param ms Timeout duration.
   */
  void set_timeout_ms(int ms);

  /**
   * @brief Set OpenCV font scale used for drawing (default 0.15).
   * @param s Font scale factor.
   */
  void set_font_scale(double s);

private:
  struct Item {
    std::string label;
    bool is_submenu{false};
    MenuAction action{MenuAction::NONE};
    std::vector<Item> children;
    cv::Scalar color{255,255,255};
  };

  /**
   * @brief Build the default menu hierarchy.
   * @return Root item of the tree.
   */
  static Item build_default_tree();

  // navigation
  std::vector<int> path_;
  int sel_{0};
  int offset_{0};

  /**
   * @brief Retrieve the currently selected menu.
   * @return Pointer to const menu item.
   */
  const Item* current_menu() const;

  /**
   * @brief Retrieve the currently selected menu.
   * @return Pointer to mutable menu item.
   */
  Item* current_menu();

  /**
   * @brief Enter the currently highlighted submenu or trigger its action.
   */
  void enter();

  /**
   * @brief Navigate back to the parent menu.
   */
  void back();

  /**
   * @brief Move selection up.
   */
  void up();

  /**
   * @brief Move selection down.
   */
  void down();

  /**
   * @brief Dispatch a menu action through the callback.
   * @param a Action to send.
   */
  void dispatch(MenuAction a);

  // drawing helpers
  /**
   * @brief Draw the menu panel background.
   * @param canvas Target image.
   */
  void draw_panel(cv::Mat& canvas);

  /**
   * @brief Draw menu header text.
   * @param img Target image.
   * @param title Header title.
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param w Width of header area.
   */
  void draw_header(cv::Mat& img, const std::string& title, int x, int y, int w, int pad);

  /**
   * @brief Draw individual menu items.
   * @param img Target image.
   * @param items Items to draw.
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param w Width of drawing area.
   * @param line_h Height of a line.
   * @param start First item index to show.
   * @param max_items Maximum number of items to draw.
   */
  void draw_items(cv::Mat& img, const std::vector<Item>& items, int x, int y,
                   int w, int line_h, int start, int max_items, int pad);

  Item root_;
  std::function<void(MenuAction)> on_action_;
  int timeout_ms_{5000};
  double font_scale_{0.15};

  using clock = std::chrono::steady_clock;
  clock::time_point last_key_time_;
};

} // namespace robo_ui

