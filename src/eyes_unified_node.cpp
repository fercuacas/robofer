#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <mutex>
#include <algorithm>

#include "robofer/eyes.hpp"
#include "robofer/display.hpp"
#include "robofer/ui_menu.hpp"

using robo_eyes::RoboEyes;
using robo_eyes::Mood;
using robo_ui::MenuController;
using robo_ui::MenuAction;
using robo_ui::UiKey;

static const char* NODE_NAME = "robo_eyes";

struct ActionDispatcher {
  rclcpp::Logger log;
  RoboEyes& eyes;
  void operator()(MenuAction a){
    switch(a){
      case MenuAction::SET_ANGRY:
        eyes.setMood(Mood::ANGRY);
        RCLCPP_INFO(log, "MenuAction: ANGRY");
        break;
      case MenuAction::SET_SAD:
        eyes.setMood(Mood::FROWN);
        RCLCPP_INFO(log, "MenuAction: SAD(FROWN)");
        break;
      case MenuAction::SET_HAPPY:
        eyes.setMood(Mood::HAPPY);
        RCLCPP_INFO(log, "MenuAction: HAPPY");
        break;
      case MenuAction::POWEROFF:
        RCLCPP_WARN(log, "MenuAction: POWEROFF (llamando a sudo poweroff)");
        std::system("sudo poweroff &");
        break;
      case MenuAction::NONE:
      default:
        break;
    }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(NODE_NAME);
  auto log = node->get_logger();

  std::string backend = node->declare_parameter<std::string>("backend", "st7735");
  const int   eyes_w  = node->declare_parameter<int>("eyes_width",  128);
  const int   eyes_h  = node->declare_parameter<int>("eyes_height", 64);
  const int   fps     = node->declare_parameter<int>("fps", 30);
  const int   menu_timeout_ms = node->declare_parameter<int>("menu_timeout_ms", 5000);

  auto display = robo_eyes::make_display(backend);
  if(!display->init(*node)){
    RCLCPP_FATAL(log, "Display init failed (backend=%s).", backend.c_str());
    return 1;
  }

  RoboEyes eyes;
  eyes.begin(eyes_w, eyes_h, fps);
  eyes.setIdle(true);
  eyes.setAutoblinker(true);
  eyes.setCuriosity(false);
  eyes.setCyclops(false);
  eyes.setMood(Mood::DEFAULT);

  ActionDispatcher dispatch{ log, eyes };
  MenuController   menu([&](MenuAction a){ dispatch(a); });
  menu.set_timeout_ms(menu_timeout_ms);

  std::mutex ui_mtx;
  auto sub_ui = node->create_subscription<std_msgs::msg::Int32>(
    "/ui/button", 10,
    [&](const std_msgs::msg::Int32::SharedPtr msg){
      std::lock_guard<std::mutex> lk(ui_mtx);
      int v = msg->data;
      if(v < 0 || v > 3) return;
      menu.on_key(static_cast<UiKey>(v));
    });

  rclcpp::Rate rate(fps);
  RCLCPP_INFO(log, "Eyes+Menu @ %d FPS, backend=%s, display=%dx%d, eyes=%dx%d",
              fps, backend.c_str(), display->width(), display->height(), eyes_w, eyes_h);

  int DW = display->width();   if(DW <= 0) DW = eyes_w;
  int DH = display->height();  if(DH <= 0) DH = eyes_h;
  menu.set_font_scale(std::clamp(DH / 100.0, 0.4, 1.0));
  cv::Mat canvas(DH, DW, CV_8UC1, cv::Scalar(0));

  while(rclcpp::ok()){
    rclcpp::spin_some(node);

    eyes.update();
    const cv::Mat& m = eyes.frame();

    canvas.setTo(cv::Scalar(0));
    int ox = std::max(0, (DW - m.cols)/2);
    int oy = std::max(0, (DH - m.rows)/2);
    cv::Rect roi(ox, oy, std::min(m.cols, DW-ox), std::min(m.rows, DH-oy));
    if(roi.width > 0 && roi.height > 0){
      m(cv::Rect(0,0,roi.width,roi.height)).copyTo(canvas(roi));
    }

    {
      std::lock_guard<std::mutex> lk(ui_mtx);
      menu.draw(canvas);
    }

    display->pushMono8(canvas);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

