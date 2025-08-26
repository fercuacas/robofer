#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "robofer/servos.hpp"
#include "robofer/eyes.hpp"
#include "robofer/audio_player.hpp"

namespace robofer {

using robo_servos::ControlServo;
using robo_eyes::Mood;

class StateHandler : public rclcpp::Node {
public:
  StateHandler();

private:
  class State {
  public:
    virtual ~State() = default;
    virtual void on_enter(StateHandler &ctx) {}
    virtual void on_update(StateHandler &ctx) {}
    virtual void on_exit(StateHandler &ctx) {}
  };

  class HappyState;
  class AngryState;
  class SadState;

  friend class HappyState;
  friend class AngryState;
  friend class SadState;

  void set_state(Mood m);
  void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void update();

  ControlServo servos_;
  std::unique_ptr<robo_audio::AudioPlayer> audio_;
  std::string happy_sound_;
  std::string angry_sound_;
  std::string sad_sound_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mood_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<State> current_state_;
};

} // namespace robofer

