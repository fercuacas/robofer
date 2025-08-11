#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "robofer/eyes.hpp"

using robo_eyes::RoboEyes; 
using robo_eyes::Mood; 
using robo_eyes::Pos;

static Pos pos_from_string(const std::string& s){
  if(s=="N") return Pos::N; if(s=="NE") return Pos::NE; if(s=="E") return Pos::E; if(s=="SE") return Pos::SE;
  if(s=="S") return Pos::S; if(s=="SW") return Pos::SW; if(s=="W") return Pos::W; if(s=="NW") return Pos::NW;
  return Pos::CENTER;
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robo_eyes_sim");

  // Básicos
  const int   width   = node->declare_parameter<int>("width", 128);
  const int   height  = node->declare_parameter<int>("height", 64);
  const int   fps     = node->declare_parameter<int>("fps", 60);
  int         scale   = node->declare_parameter<int>("scale", 6);

  // Modos/flags
  bool idle       = node->declare_parameter<bool>("idle", true);
  bool autoblink  = node->declare_parameter<bool>("autoblink", true);
  bool curious    = node->declare_parameter<bool>("curious", false);
  bool cyclops    = node->declare_parameter<bool>("cyclops", false);
  std::string mood_s = node->declare_parameter<std::string>("mood", "default");
  std::string pos_s  = node->declare_parameter<std::string>("pos",  "CENTER");

  // Geometría y estilo
  int eye_l_w = node->declare_parameter<int>("eye_l_w", 36);
  int eye_l_h = node->declare_parameter<int>("eye_l_h", 36);
  int eye_r_w = node->declare_parameter<int>("eye_r_w", 36);
  int eye_r_h = node->declare_parameter<int>("eye_r_h", 36);
  int radius_l = node->declare_parameter<int>("radius_l", 8);
  int radius_r = node->declare_parameter<int>("radius_r", 8);
  int space_between = node->declare_parameter<int>("space_between", 10);

  // Timings (como el original)
  int blink_interval = node->declare_parameter<int>("blink_interval", 1);
  int blink_var      = node->declare_parameter<int>("blink_var", 4);
  int idle_interval  = node->declare_parameter<int>("idle_interval", 1);
  int idle_var       = node->declare_parameter<int>("idle_var", 3);

  // Flickers
  bool h_flicker = node->declare_parameter<bool>("h_flicker", false);
  int  h_amp     = node->declare_parameter<int>("h_amp", 2);
  bool v_flicker = node->declare_parameter<bool>("v_flicker", false);
  int  v_amp     = node->declare_parameter<int>("v_amp", 10);

  RoboEyes eyes;
  eyes.begin(width, height, fps);
  eyes.setSpacebetween(space_between);
  eyes.setWidth(eye_l_w, eye_r_w);
  eyes.setHeight(eye_l_h, eye_r_h);
  eyes.setBorderradius(radius_l, radius_r);
  eyes.setPosition(pos_from_string(pos_s));
  eyes.setIdle(idle, idle_interval, idle_var);
  eyes.setAutoblinker(autoblink, blink_interval, blink_var);
  eyes.setCuriosity(curious);
  eyes.setCyclops(cyclops);
  eyes.setHFlicker(h_flicker, h_amp);
  eyes.setVFlicker(v_flicker, v_amp);

  // Mood
  if(mood_s=="tired") eyes.setMood(Mood::TIRED);
  else if(mood_s=="angry") eyes.setMood(Mood::ANGRY);
  else if(mood_s=="happy") eyes.setMood(Mood::HAPPY);
  else eyes.setMood(Mood::DEFAULT);

  // Abrimos inicialmente (opcional)
  eyes.open();

  const std::string win = "RoboEyes SIM";
  cv::namedWindow(win, cv::WINDOW_NORMAL);
  cv::resizeWindow(win, width*scale, height*scale);

  RCLCPP_INFO(node->get_logger(), "RoboEyes SIM up (w=%d h=%d fps=%d)", width, height, fps);

  rclcpp::Rate rate(fps);
  bool running = true;
  while(rclcpp::ok() && running){
    rclcpp::spin_some(node);

    eyes.update();
    cv::Mat view; 
    cv::resize(eyes.frame(), view, cv::Size(width*scale, height*scale), 0, 0, cv::INTER_NEAREST);
    cv::imshow(win, view);

    int key = cv::waitKey(1);
    switch(key){
      case 'q': case 27: running=false; break;
      case 'b': eyes.blink(); break;
      case '1': eyes.setMood(Mood::TIRED);  break;
      case '2': eyes.setMood(Mood::ANGRY);  break;
      case '3': eyes.setMood(Mood::FROWN);  break;
      case '0': eyes.setMood(Mood::DEFAULT);break;
      case 'i': eyes.setIdle(true);  break;
      case 'I': eyes.setIdle(false); break;
      case 'c': curious=!curious; eyes.setCuriosity(curious); break;
      case 'y': eyes.anim_laugh(); break;
      case 'x': eyes.anim_confused(); break;
      case 'h': h_flicker=!h_flicker; eyes.setHFlicker(h_flicker); break;
      case 'v': v_flicker=!v_flicker; eyes.setVFlicker(v_flicker); break;
      case '+': scale = std::min(scale+1, 16); cv::resizeWindow(win, width*scale, height*scale); break;
      case '-': scale = std::max(scale-1, 1);  cv::resizeWindow(win, width*scale, height*scale); break;
      // posiciones rápidas
      case 'w': eyes.setPosition(Pos::N);      break;
      case 'e': eyes.setPosition(Pos::NE);     break;
      case 'd': eyes.setPosition(Pos::E);      break;
      //case 'c': /* ya se usa para curiosity */ break;
      //case 'x': /* ya se usa para confused */  break;
      case 's': eyes.setPosition(Pos::S);      break;
      case 'z': eyes.setPosition(Pos::SW);     break;
      case 'a': eyes.setPosition(Pos::W);      break;
      //case 'q': /* ya se usa para salir */     break;
      case 'r': eyes.setPosition(Pos::CENTER); break;
      case 'o': eyes.open(true,true);  break;
      case 'p': eyes.close(true,true); break;
      case 'k': cyclops=!cyclops; eyes.setCyclops(cyclops); break;
      default: break;
    }

    rate.sleep();
  }

  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}