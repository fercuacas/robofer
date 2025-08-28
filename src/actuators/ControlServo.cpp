#include "robofer/actuators/ControlServo.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace robo_servos {

ControlServo::ControlServo(rclcpp::Node *node,
                           const std::string &chip_name,
                           int s1, int s2,
                           bool sim) {
  sim_ = sim;
  if(node) {
    angle_pub_ = node->create_publisher<robofer::msg::ServoGoal>("servo_angles", 10);
  }
  for(size_t i = 0; i < servos_.size(); ++i){
    servos_[i].id_ = static_cast<int>(i);
  }
  if(!sim_){
    chip_ = gpiod_chip_open_by_name(chip_name.c_str());
    if(!chip_) throw std::runtime_error("gpiod_chip_open_by_name failed");
    int offs[2] = {s1, s2};
    for(size_t i = 0; i < servos_.size(); ++i){
      if(offs[i] < 0) continue;
      servos_[i].line_ = gpiod_chip_get_line(chip_, offs[i]);
      if(!servos_[i].line_) throw std::runtime_error("gpiod_chip_get_line failed");
      if(gpiod_line_request_output(servos_[i].line_, "servo", 0) < 0)
        throw std::runtime_error("gpiod_line_request_output failed");
    }
  }
  for(auto &s : servos_){
    s.running_ = true;
    s.thread_ = std::thread([this,&s]{ threadFunc(s); });
  }
}

ControlServo::~ControlServo(){
  for(auto &s : servos_){
    if(s.running_){
      s.running_ = false;
      if(s.thread_.joinable()) s.thread_.join();
    }
    if(!sim_ && s.line_){
      gpiod_line_set_value(s.line_, 0);
      gpiod_line_release(s.line_);
    }
  }
  if(!sim_ && chip_) gpiod_chip_close(chip_);
}

void ControlServo::setSpeed(int id, float deg_per_sec){
  if(id < 0 || id >= (int)servos_.size()) return;
  servos_[id].speed_ = deg_per_sec;
  servos_[id].has_target_ = false;
}

void ControlServo::moveTo(int id, float angle_deg, float speed_deg_per_sec){
  if(id < 0 || id >= (int)servos_.size()) return;
  servos_[id].target_angle_ = angle_deg;
  servos_[id].speed_ = std::abs(speed_deg_per_sec);
  servos_[id].has_target_ = true;
}

void ControlServo::stop(int id){
  if(id < 0 || id >= (int)servos_.size()) return;
  servos_[id].speed_ = 0.0f;
  servos_[id].has_target_ = false;
}

void ControlServo::setIdle(int id){
  moveTo(id, 0.0f, 90.0f);
}

void ControlServo::threadFunc(Servo &s){
  using clock = std::chrono::steady_clock;
  const float min_pw = 1000.0f; // microseconds
  const float max_pw = 2000.0f; // microseconds
  while(s.running_){
    auto start = clock::now();
    float ang = s.current_angle_.load();
    if(s.has_target_){
      float tgt = s.target_angle_.load();
      float step = s.speed_.load() * 0.02f; // 20ms frame
      if(std::fabs(tgt - ang) <= step){
        ang = tgt;
        s.has_target_ = false;
        s.speed_ = 0.0f;
      } else {
        ang += (tgt > ang ? 1.f : -1.f) * step;
      }
      s.current_angle_ = ang;
    } else {
      ang += s.speed_.load() * 0.02f;
      s.current_angle_ = ang;
    }

    if(angle_pub_){
      robofer::msg::ServoGoal msg;
      msg.id = static_cast<int8_t>(s.id_);
      msg.angle = s.current_angle_.load();
      angle_pub_->publish(msg);
    }

    float clamped = std::clamp(ang, 0.0f, 180.0f);
    float pw = min_pw + (clamped / 180.0f) * (max_pw - min_pw);
    if(!sim_ && s.line_){
      gpiod_line_set_value(s.line_, 1);
      std::this_thread::sleep_for(std::chrono::microseconds((int)pw));
      gpiod_line_set_value(s.line_, 0);
    }
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - start);
    auto sleep = std::chrono::microseconds(20000) - elapsed;
    if(sleep.count() > 0) std::this_thread::sleep_for(sleep);
  }
}

} // namespace robo_servos

