#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "robofer/actuators/ControlServo.hpp"
#include "robofer/screen/Eyes.hpp"
#include "robofer/audio/AudioPlayer.hpp"

namespace robofer {

using robo_servos::ControlServo;
using robo_eyes::Mood;

/**
 * @brief ROS node that manages robot moods and actions.
 *
 * StateHandler coordinates servos, eyes and audio feedback according to
 * the current mood. It listens for mode requests and periodically
 * updates the active state.
 */
class StateHandler : public rclcpp::Node {
public:
  /**
   * @brief Construct the StateHandler node.
   */
  StateHandler();

private:
  /** @brief Base class for concrete mood states. */
  class State {
  public:
    virtual ~State() = default;
    /**
     * @brief Called when the state becomes active.
     * @param ctx Parent context.
     */
    virtual void onEnter(StateHandler &ctx) {}
    /**
     * @brief Called periodically while the state is active.
     * @param ctx Parent context.
     */
    virtual void onUpdate(StateHandler &ctx) {}
    /**
     * @brief Called when the state is about to be replaced.
     * @param ctx Parent context.
     */
    virtual void onExit(StateHandler &ctx) {}
  };

  // Concrete states implementing different moods
  class HappyState;
  class AngryState;
  class SadState;

  friend class HappyState;
  friend class AngryState;
  friend class SadState;

  /**
   * @brief Change the active mood/state.
   * @param m Desired mood.
   */
  void setState(Mood m);

  /**
   * @brief Handle external mode requests from a topic.
   * @param msg Incoming message with the desired mood.
   */
  void modeCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  /**
   * @brief Periodic update tick invoked by a timer.
   */
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

