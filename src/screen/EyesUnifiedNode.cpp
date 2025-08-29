#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <regex>
#include "robofer/bluetoothctl_agent.hpp"
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

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(NODE_NAME);
  auto log = node->get_logger();

  std::string backend = node->declare_parameter<std::string>("backend", "st7735");
  const int   eyes_w  = node->declare_parameter<int>("eyes_width",  160);
  const int   eyes_h  = node->declare_parameter<int>("eyes_height", 128);
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
  auto bt_power_client = node->create_client<std_srvs::srv::SetBool>("/bluetooth/power");
  auto bt_pair_client  = node->create_client<std_srvs::srv::SetBool>("/bluetooth/pair_response");

  BluetoothctlAgent bt_agent;
  std::string last_passkey;
  enum class BtUiState { IDLE, STARTING, WAITING_CONFIRM, PAIRED, SPP_READY, CONNECTED, ERROR_ };
  BtUiState bt_state = BtUiState::IDLE;
  MenuController* menu_ptr = nullptr;
  auto update_bt_menu = [&](){
    if(!menu_ptr) return;
    std::string st; uint32_t code = 0;
    switch(bt_state){
      case BtUiState::IDLE: st = "IDLE"; break;
      case BtUiState::STARTING: st = "STARTING"; break;
      case BtUiState::WAITING_CONFIRM: st = std::string("CONFIRM_CODE:") + last_passkey; code = static_cast<uint32_t>(std::stoul(last_passkey)); break;
      case BtUiState::PAIRED: st = "PAIRED"; break;
      case BtUiState::SPP_READY: st = "SPP_READY"; break;
      case BtUiState::CONNECTED: st = "CONNECTED"; break;
      case BtUiState::ERROR_: st = "ERROR"; break;
    }
    menu_ptr->setBtState(st, code);
  };

  MenuController menu([&](MenuAction a){
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
      case MenuAction::BT_CONNECT: {
        RCLCPP_INFO(log, "MenuAction: BT_CONNECT");
        bt_state = BtUiState::STARTING;
        update_bt_menu();
        bool ok = bt_agent.start([&](const std::string& line){
          static const std::regex re_passkey(".*Confirm passkey\\s+(\\d{6}).*yes/no.*");
          std::smatch m;
          if(std::regex_match(line, m, re_passkey)){
            last_passkey = m[1];
            bt_state = BtUiState::WAITING_CONFIRM;
            update_bt_menu();
            return;
          }
          if(line.find("Pairing successful") != std::string::npos ||
             line.find(" Paired: yes") != std::string::npos){
            bt_agent.disableProvisionWindow();
            std::system("sdptool add --channel=3 SP >/dev/null 2>&1");
            bt_state = BtUiState::PAIRED;
            update_bt_menu();
            return;
          }
          if(line.find("Failed") != std::string::npos || line.find("Error") != std::string::npos){
            bt_state = BtUiState::ERROR_;
            update_bt_menu();
          }
        });
        if(!ok){
          bt_state = BtUiState::ERROR_;
          update_bt_menu();
          break;
        }
        bt_agent.powerOn();
        bt_agent.enableProvisionWindow("Robofer", 180);
        break; }
      case MenuAction::BT_STOP:
        RCLCPP_INFO(log, "MenuAction: BT_STOP");
        bt_agent.disableProvisionWindow();
        bt_agent.stop();
        bt_state = BtUiState::IDLE;
        update_bt_menu();
        break;
      case MenuAction::BT_ACCEPT:
        RCLCPP_INFO(log, "MenuAction: BT_ACCEPT");
        if(bt_state == BtUiState::WAITING_CONFIRM) bt_agent.send("yes");
        break;
      case MenuAction::BT_REJECT:
        RCLCPP_INFO(log, "MenuAction: BT_REJECT");
        if(bt_state == BtUiState::WAITING_CONFIRM) bt_agent.send("no");
        break;
      case MenuAction::BT_ON:
        RCLCPP_INFO(log, "MenuAction: BT_ON");
        if(bt_power_client){ auto req = std::make_shared<std_srvs::srv::SetBool::Request>(); req->data = true; bt_power_client->async_send_request(req); }
        break;
      case MenuAction::BT_OFF:
        RCLCPP_INFO(log, "MenuAction: BT_OFF");
        if(bt_power_client){ auto req = std::make_shared<std_srvs::srv::SetBool::Request>(); req->data = false; bt_power_client->async_send_request(req); }
        break;
      case MenuAction::BT_PAIR_ACCEPT:
        RCLCPP_INFO(log, "MenuAction: BT_PAIR_ACCEPT");
        if(bt_pair_client){ auto req = std::make_shared<std_srvs::srv::SetBool::Request>(); req->data = true; bt_pair_client->async_send_request(req); }
        break;
      case MenuAction::BT_PAIR_REJECT:
        RCLCPP_INFO(log, "MenuAction: BT_PAIR_REJECT");
        if(bt_pair_client){ auto req = std::make_shared<std_srvs::srv::SetBool::Request>(); req->data = false; bt_pair_client->async_send_request(req); }
        break;
      case MenuAction::NONE:
      default:
        break;
    }
  });
  menu_ptr = &menu;
  menu.setTimeoutMs(menu_timeout_ms);
  update_bt_menu();

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

  auto bt_state_sub = node->create_subscription<std_msgs::msg::String>(
    "/bluetooth/state", 10,
    [&](const std_msgs::msg::String::SharedPtr msg){
      std::lock_guard<std::mutex> lk(ui_mtx);
      bool enabled = false;
      std::string dev;
      std::string s = msg->data;
      if(s == "ON"){
        enabled = true;
      } else if(s.rfind("REQUEST:",0) == 0){
        enabled = true;
        dev = s.substr(8);
      }
      menu.setBluetoothState(enabled, dev);
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

