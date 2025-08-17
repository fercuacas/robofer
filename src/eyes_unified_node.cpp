#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/core.hpp>

#include "robofer/eyes.hpp"
#include "robofer/display.hpp"

using robo_eyes::RoboEyes;
using robo_eyes::Mood;

static const char* NODE_NAME = "robo_eyes";

// Mapa de IDs -> acciones/estados.
// 0: DEFAULT
// 1: TIRED
// 2: ANGRY
// 3: FROWN
// 4: HAPPY
// 5: LAUGH anim
// 6: CONFUSED anim
// 7: BLINK
// 8: OPEN
// 9: CLOSE
// 10: toggle CURIOUS
// 11: toggle IDLE
// 12: toggle CYCLOPS
// 13: toggle H_FLICKER
// 14: toggle V_FLICKER
struct MoodController {
  explicit MoodController(RoboEyes& e): eyes(e) {}
  void apply(int id){
    switch(id){
      case 0: eyes.setMood(Mood::DEFAULT); break;
      case 1: eyes.setMood(Mood::TIRED);   break;
      case 2: eyes.setMood(Mood::ANGRY);   break;
      case 3: eyes.setMood(Mood::FROWN);   break;
      case 4: eyes.setMood(Mood::HAPPY);   break;
      case 5: eyes.anim_laugh();           break;
      case 6: eyes.anim_confused();        break;
      case 7: eyes.blink();                break;
      case 8: eyes.open(true,true);        break;
      case 9: eyes.close(true,true);       break;
      case 10: curious = !curious; eyes.setCuriosity(curious); break;
      case 11: idle    = !idle;    eyes.setIdle(idle);         break;
      case 12: cyclops = !cyclops; eyes.setCyclops(cyclops);   break;
      case 13: hflip   = !hflip;   eyes.setHFlicker(hflip);    break;
      case 14: vflip   = !vflip;   eyes.setVFlicker(vflip);    break;
      default: break; // ignora IDs fuera de rango
    }
  }
  RoboEyes& eyes;
  bool curious=false, idle=true, cyclops=false, hflip=false, vflip=false;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(NODE_NAME);
  auto log = node->get_logger();

  // Parámetros genéricos
  std::string backend = node->declare_parameter<std::string>("backend", "st7735"); // "sim" | "st7735"
  const int   eyes_w  = node->declare_parameter<int>("eyes_width",  128);
  const int   eyes_h  = node->declare_parameter<int>("eyes_height", 64);
  const int   fps     = node->declare_parameter<int>("fps", 30);

  // Display
  auto display = robo_eyes::make_display(backend);
  if(!display->init(*node)){
    RCLCPP_FATAL(log, "Display init failed (backend=%s).", backend.c_str());
    return 1;
  }

  // Ojos
  RoboEyes eyes;
  eyes.begin(eyes_w, eyes_h, fps);
  eyes.setIdle(true);
  eyes.setAutoblinker(true);
  eyes.setCuriosity(false);
  eyes.setCyclops(false);
  eyes.setMood(Mood::DEFAULT);

  MoodController moodctl(eyes);

  // Subs: ID de estado de ánimo
  auto sub_mood = node->create_subscription<std_msgs::msg::Int32>(
    "eyes/mood_id", 10,
    [&](const std_msgs::msg::Int32::SharedPtr msg){
      moodctl.apply(msg->data);
      RCLCPP_INFO(log, "mood_id=%d aplicado", msg->data);
    });

  // Render loop
  rclcpp::Rate rate(fps);
  RCLCPP_INFO(log, "Eyes running @ %d FPS, backend=%s, display=%dx%d, eyes=%dx%d",
              fps, backend.c_str(), display->width(), display->height(), eyes_w, eyes_h);

  while(rclcpp::ok()){
    rclcpp::spin_some(node);

    eyes.update();             // produce frame 8UC1
    const cv::Mat& m = eyes.frame();
    display->pushMono8(m);     // cada backend centra/convierte si hace falta

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
