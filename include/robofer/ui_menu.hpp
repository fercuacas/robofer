#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

namespace robo_ui {

// Eventos de navegación desde los botones
enum class UiKey : int { UP=0, DOWN=1, BACK=2, OK=3 };

// Acciones que el menú puede solicitar al mundo exterior
enum class MenuAction {
  NONE,
  SET_ANGRY,
  SET_SAD,
  SET_HAPPY,
  POWEROFF,
};

// Controlador de menú: mantiene estructura y estado; dibuja sobre canvas 8UC1
class MenuController {
public:
  explicit MenuController(std::function<void(MenuAction)> on_action);

  void on_key(UiKey key);       // recibe teclas, reinicia TTL
  bool is_active() const;       // se muestra si hubo pulsación en timeout_ms
  void draw(cv::Mat& canvas);   // dibuja overlay si activo

  void set_timeout_ms(int ms);  // por defecto 5000
  void set_font_scale(double s);// tamaño de fuente OpenCV (0.4 por defecto)

private:
  struct Item {
    std::string label;
    bool is_submenu{false};
    MenuAction action{MenuAction::NONE};
    std::vector<Item> children;
  };

  static Item build_default_tree();

  // navegación
  std::vector<int> path_;
  int sel_{0};
  int offset_{0};
  const Item* current_menu() const;
  Item* current_menu();

  void enter();
  void back();
  void up();
  void down();
  void dispatch(MenuAction a);

  // dibujo
  void draw_panel(cv::Mat& canvas);
  void draw_header(cv::Mat& img, const std::string& title, int x, int y, int w);
  void draw_items(cv::Mat& img, const std::vector<Item>& items, int x, int y,
                  int w, int line_h, int start, int max_items);

  Item root_;
  std::function<void(MenuAction)> on_action_;
  int timeout_ms_{5000};
  double font_scale_{0.45};

  using clock = std::chrono::steady_clock;
  clock::time_point last_key_time_;
};

} // namespace robo_ui

