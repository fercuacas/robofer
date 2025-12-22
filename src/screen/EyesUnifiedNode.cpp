#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cstdlib>
#include <chrono>
#include <regex>
#include <atomic>
#include "robofer/bluetoothctl_agent.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <mutex>
#include <algorithm>

#include "robofer/screen/Eyes.hpp"
#include "robofer/screen/Display.hpp"
#include "robofer/screen/UiMenu.hpp"
#include "robofer/screen/MusicMenu.hpp"
#include "robofer/audio/AudioPlayer.hpp"
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

  const char* home_env = std::getenv("HOME");
  std::string music_dir = std::string(home_env ? home_env : "") + "/Music";
  robo_audio::AudioPlayer audio_player;
  audio_player.setSearchPaths({music_dir});
  audio_player.reindex();
  robo_ui::MusicMenu music_menu(audio_player);
  bool music_mode = false;
  std::atomic<Mood> current_mood{Mood::DEFAULT};
  std::atomic<Mood> base_mood{Mood::DEFAULT};
  Mood last_regular_mood = Mood::ESPERA;
  Mood last_sent_mode = Mood::ESPERA;
  bool bailoteo_active = false;
  bool bailoteo_playing = false;
  bool bailoteo_paused = false;
  enum class EyeAction : uint8_t { NONE=0, TIRED=1, FROWN=2, CURIOUS=3, LAUGH=4, CONFUSED=5 };
  EyeAction eye_action = EyeAction::NONE;
  std::chrono::steady_clock::time_point action_until{};
  std::chrono::steady_clock::time_point action_tick{};
  auto action_duration = [](EyeAction action){
    return (action == EyeAction::CURIOUS)
      ? std::chrono::seconds(6)
      : std::chrono::seconds(3);
  };

  auto send_mode = [&](Mood mood){
    if(last_sent_mode == mood) return;
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(mood);
    mode_pub->publish(msg);
    last_sent_mode = mood;
  };

  auto update_bailoteo_state = [&](){
    Mood target = bailoteo_active && bailoteo_playing && !bailoteo_paused
                    ? Mood::BAILOTEO
                    : (bailoteo_active ? Mood::BAILOTEO_WAIT : last_regular_mood);
    send_mode(target);
  };

  music_menu.setBailoteoHandler([&](bool active, bool playing, bool paused){
    bailoteo_active = active;
    bailoteo_playing = playing;
    bailoteo_paused = paused;
    update_bailoteo_state();
  });

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
    switch(a){
      case MenuAction::SET_ANGRY:
        last_regular_mood = Mood::ANGRY;
        update_bailoteo_state();
        if(menu_ptr) menu_ptr->hide();
        RCLCPP_INFO(log, "MenuAction -> mode ANGRY");
        break;
      case MenuAction::SET_SAD:
        last_regular_mood = Mood::FROWN;
        update_bailoteo_state();
        if(menu_ptr) menu_ptr->hide();
        RCLCPP_INFO(log, "MenuAction -> mode SAD");
        break;
      case MenuAction::SET_HAPPY:
        last_regular_mood = Mood::HAPPY;
        update_bailoteo_state();
        if(menu_ptr) menu_ptr->hide();
        RCLCPP_INFO(log, "MenuAction -> mode HAPPY");
        break;
      case MenuAction::SET_PUXAINE:
        bailoteo_active = false;
        bailoteo_playing = false;
        bailoteo_paused = false;
        last_regular_mood = Mood::PUXAINE;
        update_bailoteo_state();
        if(menu_ptr) menu_ptr->hide();
        RCLCPP_INFO(log, "MenuAction -> mode PUXAINE");
        break;
      case MenuAction::SET_PEO:
        last_regular_mood = Mood::PEO;
        update_bailoteo_state();
        if(menu_ptr) menu_ptr->hide();
        RCLCPP_INFO(log, "MenuAction -> mode PEO");
        break;
      case MenuAction::SET_LOVE: {
        std_msgs::msg::UInt8 msg;
        msg.data = static_cast<uint8_t>(Mood::LOVE);
        mode_pub->publish(msg);
        if(menu_ptr) menu_ptr->hide();
        RCLCPP_INFO(log, "MenuAction -> mode LOVE");
        break; }
      case MenuAction::SET_ESPERA:
        last_regular_mood = Mood::ESPERA;
        update_bailoteo_state();
        if(menu_ptr) menu_ptr->hide();
        RCLCPP_INFO(log, "MenuAction -> mode ESPERA");
        break;
      case MenuAction::POWEROFF:
        RCLCPP_WARN(log, "MenuAction: POWEROFF (llamando a sudo poweroff)");
        std::system("sudo poweroff &");
        break;
      case MenuAction::MUSIC_MENU:
        RCLCPP_INFO(log, "MenuAction: MUSIC_MENU");
        music_mode = true;
        break;
      case MenuAction::BT_CONNECT: {
        RCLCPP_INFO(log, "MenuAction: BT_CONNECT");
        bt_state = BtUiState::STARTING;
        update_bt_menu();
        bool ok = bt_agent.start([&](const std::string& line){
          static const std::regex re_passkey(
              "(?:confirm|request).*?(?:passkey|pin(?:\\s+code)?)\\D*(\\d{4,6})",
              std::regex::icase);
          std::smatch m;
          if(std::regex_search(line, m, re_passkey)){
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
      UiKey key = static_cast<UiKey>(v);
      if(music_mode){
        bool bailoteo_before = bailoteo_active;
        music_menu.onKey(key);
        bool bailoteo_after = bailoteo_active;
        if(key == UiKey::BACK && !bailoteo_after && !audio_player.isPlaying()){
          music_mode = false;
          menu.onKey(UiKey::BACK);
          update_bailoteo_state();
        }
      } else {
        menu.onKey(key);
      }
    });

  auto sub_mood = node->create_subscription<std_msgs::msg::UInt8>(
    "/eyes/mood", 10,
    [&](const std_msgs::msg::UInt8::SharedPtr msg){
      Mood m_val = static_cast<Mood>(msg->data);
      current_mood.store(m_val, std::memory_order_relaxed);
      base_mood.store(m_val, std::memory_order_relaxed);
      std::lock_guard<std::mutex> lk(ui_mtx);
      eyes.setMood(m_val);
    });

  auto sub_action = node->create_subscription<std_msgs::msg::UInt8>(
    "/eyes/action", 10,
    [&](const std_msgs::msg::UInt8::SharedPtr msg){
      auto action = static_cast<EyeAction>(msg->data);
      auto now = std::chrono::steady_clock::now();
      eye_action = action;
      action_until = now + action_duration(action);
      action_tick = now;
      std::lock_guard<std::mutex> lk(ui_mtx);
      switch(action){
        case EyeAction::TIRED:
          eyes.setMood(Mood::TIRED);
          break;
        case EyeAction::FROWN:
          eyes.setMood(Mood::FROWN);
          break;
        case EyeAction::CURIOUS:
          eyes.setCuriosity(true);
          eyes.setPosition(robo_eyes::Pos::E);
          break;
        case EyeAction::LAUGH:
          eyes.anim_laugh();
          break;
        case EyeAction::CONFUSED:
          eyes.anim_confused();
          break;
        case EyeAction::NONE:
        default:
          break;
      }
    });

  auto eye_pos_sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/eyes/pos", 10,
    [&](const std_msgs::msg::UInt8::SharedPtr msg){
      if(msg->data > static_cast<uint8_t>(robo_eyes::Pos::NW)) return;
      std::lock_guard<std::mutex> lk(ui_mtx);
      eyes.setPosition(static_cast<robo_eyes::Pos>(msg->data));
    });

  auto eye_idle_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/eyes/idle", 10,
    [&](const std_msgs::msg::Bool::SharedPtr msg){
      std::lock_guard<std::mutex> lk(ui_mtx);
      eyes.setIdle(msg->data);
    });

  rclcpp::Rate rate(fps);
  RCLCPP_INFO(log, "Eyes+Menu @ %d FPS, backend=%s, display=%dx%d, eyes=%dx%d",
              fps, backend.c_str(), display->width(), display->height(), eyes_w, eyes_h);

  int DW = std::max(display->width(), eyes_w);
  int DH = std::max(display->height(), eyes_h);
  double font_scale = std::clamp(DH / 200.0, 0.1, 1.0);
  menu.setFontScale(font_scale);
  music_menu.setFontScale(font_scale);
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

    {
      std::lock_guard<std::mutex> lk(ui_mtx);
      auto now = std::chrono::steady_clock::now();
      if(eye_action != EyeAction::NONE){
        if(now >= action_until){
          eye_action = EyeAction::NONE;
          eyes.setCuriosity(false);
          eyes.setMood(base_mood.load(std::memory_order_relaxed));
        } else {
          switch(eye_action){
            case EyeAction::TIRED:
              eyes.setMood(Mood::TIRED);
              break;
            case EyeAction::FROWN:
              eyes.setMood(Mood::FROWN);
              break;
            case EyeAction::CURIOUS:
              eyes.setCuriosity(true);
              eyes.setPosition(robo_eyes::Pos::E);
              break;
            case EyeAction::LAUGH:
              if(now - action_tick >= std::chrono::milliseconds(400)){
                eyes.anim_laugh();
                action_tick = now;
              }
              break;
            case EyeAction::CONFUSED:
              if(now - action_tick >= std::chrono::milliseconds(400)){
                eyes.anim_confused();
                action_tick = now;
              }
              break;
            case EyeAction::NONE:
            default:
              break;
          }
        }
      }

      eyes.update();
      const cv::Mat& m = eyes.frame();
      canvas.setTo(cv::Scalar(0,0,0));
      int ox = std::max(0, (DW - m.cols)/2);
      int oy = std::max(0, (DH - m.rows)/2);
      cv::Rect roi(ox, oy, std::min(m.cols, DW-ox), std::min(m.rows, DH-oy));
      if(roi.width > 0 && roi.height > 0){
        cv::Mat src = m(cv::Rect(0,0,roi.width,roi.height));
        cv::Mat dst = canvas(roi);
        cv::cvtColor(src, dst, cv::COLOR_GRAY2BGR);
        if(current_mood.load(std::memory_order_relaxed) == Mood::LOVE){
          cv::Mat mask_inner, mask_border;
          cv::inRange(src, 200, 255, mask_inner);      // blanco -> rosa claro
          cv::inRange(src, 1, 199, mask_border);       // gris -> rosa oscuro
          cv::Mat pink_light(dst.size(), CV_8UC3, cv::Scalar(220, 160, 230));
          cv::Mat pink_dark(dst.size(), CV_8UC3, cv::Scalar(150, 90, 170));
          pink_dark.copyTo(dst, mask_border);
          pink_light.copyTo(dst, mask_inner);
        }
      }

      if(music_mode){
        music_menu.draw(canvas);
      } else {
        menu.draw(canvas);
      }
    }

    display->pushMono8(canvas);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
