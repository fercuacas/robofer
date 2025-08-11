#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "robo_eyes/robo_eyes.hpp"

using robo_eyes::RoboEyes; using robo_eyes::Mood; using robo_eyes::Pos;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robo_eyes_sim");

  // Parameters
  int width = node->declare_parameter<int>("width", 128);
  int height = node->declare_parameter<int>("height", 64);
  int fps = node->declare_parameter<int>("fps", 60);
  int scale = node->declare_parameter<int>("scale", 6); // visual scale for tiny OLED sizes
  bool idle = node->declare_parameter<bool>("idle", true);
  bool autoblink = node->declare_parameter<bool>("autoblink", true);
  bool curious = node->declare_parameter<bool>("curious", false);
  bool cyclops = node->declare_parameter<bool>("cyclops", false);
  std::string mood = node->declare_parameter<std::string>("mood", "default");

  RoboEyes eyes;
  eyes.begin(width, height, fps);
  eyes.setIdle(idle);
  eyes.setAutoblinker(autoblink);
  eyes.setCurious(curious);
  eyes.setCyclops(cyclops);
  if(mood=="tired") eyes.setMood(Mood::TIRED);
  else if(mood=="angry") eyes.setMood(Mood::ANGRY);
  else if(mood=="happy") eyes.setMood(Mood::HAPPY);
  else eyes.setMood(Mood::DEFAULT);

  const std::string win = "RoboEyes SIM";
  cv::namedWindow(win, cv::WINDOW_NORMAL);
  cv::resizeWindow(win, width*scale, height*scale);

  rclcpp::Rate rate(fps);
  while(rclcpp::ok()){
    rclcpp::spin_some(node);

    eyes.update();
    cv::Mat view; cv::resize(eyes.frame(), view, cv::Size(width*scale, height*scale), 0, 0, cv::INTER_NEAREST);
    cv::imshow(win, view);

    // handle keys
    int key = cv::waitKey(1);
    if(key == 'q' || key == 27) break; // q or ESC
    if(key == 'b') eyes.blinkBoth();
    if(key == '1') eyes.setMood(Mood::TIRED);
    if(key == '2') eyes.setMood(Mood::ANGRY);
    if(key == '3') eyes.setMood(Mood::HAPPY);
    if(key == '0') eyes.setMood(Mood::DEFAULT);
    if(key == 'i') eyes.setIdle(true);
    if(key == 'I') eyes.setIdle(false);
    if(key == 'c') eyes.setCurious(!curious), curious=!curious;
    if(key == 'y') eyes.anim_laugh();
    if(key == 'x') eyes.anim_confused();

    // window closed?
    if(cv::getWindowProperty(win, cv::WND_PROP_VISIBLE) < 1) break;

    rate.sleep();
  }

  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}