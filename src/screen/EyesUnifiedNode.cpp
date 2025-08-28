#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <mutex>
#include <algorithm>

#include "robofer/screen/Eyes.hpp"
#include "robofer/screen/Display.hpp"
#include "robofer/screen/UiMenu.hpp"
#include "robofer/msg/wifi_status.hpp"

using robo_eyes::RoboEyes;
using robo_eyes::Mood;
using robo_ui::MenuController;
using robo_ui::MenuAction;
using robo_ui::UiKey;

static const char* NODE_NAME = "robo_eyes";

struct ActionDispatcher {
  rclcpp::Logger log;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bt_client;
  void operator()(MenuAction a){
    std_msgs::msg::UInt8 m;
    switch(a){
      case MenuAction::SET_ANGRY:
        m.data = static_cast<uint8_t>(Mood::ANGRY);
        mode_pub->publish(m);
        RCLCPP_INFO(log, "MenuAction -> mode ANGRY");
        break;
      case MenuAction::SET_SAD:
        m.data = static_cast<uint8_t>(Mood::FROWN);
        mode_pub->publish(m);
        RCLCPP_INFO(log, "MenuAction -> mode SAD");
        break;
      case MenuAction::SET_HAPPY:
        m.data = static_cast<uint8_t>(Mood::HAPPY);
        mode_pub->publish(m);
        RCLCPP_INFO(log, "MenuAction -> mode HAPPY");
        break;
      case MenuAction::POWEROFF:
        RCLCPP_WARN(log, "MenuAction: POWEROFF (llamando a sudo poweroff)");
        std::system("sudo poweroff &");
        break;
      case MenuAction::BT_CONNECT:
        RCLCPP_INFO(log, "MenuAction: BT_CONNECT");
        if(bt_client){
          auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
          bt_client->async_send_request(req);
        }
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

  auto mode_pub = node->create_publisher<std_msgs::msg::UInt8>("/mode", 10);
  auto bt_client = node->create_client<std_srvs::srv::Trigger>("/wifi_prov/start");
  ActionDispatcher dispatch{ log, mode_pub, bt_client };
  MenuController   menu([&](MenuAction a){ dispatch(a); });
  menu.setTimeoutMs(menu_timeout_ms);

  std::mutex ui_mtx;
  auto sub_ui = node->create_subscription<std_msgs::msg::Int32>(
    "/ui/button", 10,
    [&](const std_msgs::msg::Int32::SharedPtr msg){
      std::lock_guard<std::mutex> lk(ui_mtx);
      int v = msg->data;
      if(v < 0 || v > 3) return;
      menu.onKey(static_cast<UiKey>(v));
    });

  auto sub_mood = node->create_subscription<std_msgs::msg::UInt8>(
    "/eyes/mood", 10,
    [&](const std_msgs::msg::UInt8::SharedPtr msg){
      eyes.setMood(static_cast<Mood>(msg->data));
    });

  rclcpp::Rate rate(fps);
  RCLCPP_INFO(log, "Eyes+Menu @ %d FPS, backend=%s, display=%dx%d, eyes=%dx%d",
              fps, backend.c_str(), display->width(), display->height(), eyes_w, eyes_h);

  int DW = display->width();   if(DW <= 0) DW = eyes_w;
  int DH = display->height();  if(DH <= 0) DH = eyes_h;
  menu.setFontScale(std::clamp(DH / 200.0, 0.1, 1.0));
  cv::Mat canvas(DH, DW, CV_8UC3, cv::Scalar(0,0,0));

  auto wifi_sub = node->create_subscription<robofer::msg::WifiStatus>(
    "/wifi/status", 10,
    [&](const robofer::msg::WifiStatus::SharedPtr msg){
      std::lock_guard<std::mutex> lk(ui_mtx);
      menu.setWifiStatus(msg->connected, msg->ssid);
    });

  while(rclcpp::ok()){
    rclcpp::spin_some(node);

    eyes.update();
    const cv::Mat& m = eyes.frame();

    canvas.setTo(cv::Scalar(0,0,0));
    int ox = std::max(0, (DW - m.cols)/2);
    int oy = std::max(0, (DH - m.rows)/2);
    cv::Rect roi(ox, oy, std::min(m.cols, DW-ox), std::min(m.rows, DH-oy));
    if(roi.width > 0 && roi.height > 0){
      cv::Mat src = m(cv::Rect(0,0,roi.width,roi.height));
      cv::cvtColor(src, canvas(roi), cv::COLOR_GRAY2BGR);
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

