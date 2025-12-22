#pragma once

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>

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
  class LoveState;
  class EsperaState;
  class BailoteoState;
  class BailoteoWaitingState;
  class PuxaineState;
  class PeoState;

  friend class HappyState;
  friend class AngryState;
  friend class SadState;
  friend class LoveState;
  friend class EsperaState;
  friend class BailoteoState;
  friend class BailoteoWaitingState;
  friend class PuxaineState;
  friend class PeoState;

  /**
   * @brief Change the active mood/state.
   * @param m Desired mood.
   */
  void setState(Mood m);

  /**
   * @brief Publish a mood update to the eyes node.
   */
  void publishMood(Mood m);

  /**
   * @brief Publish a gaze position update to the eyes node.
   */
  void publishEyePos(robo_eyes::Pos pos);

  /**
   * @brief Enable or disable idle wandering on the eyes node.
   */
  void publishIdle(bool enabled);

  bool isBailoteo(Mood m) const {
    return m == Mood::BAILOTEO || m == Mood::BAILOTEO_WAIT;
  }

  /**
   * @brief Handle external mode requests from a topic.
   * @param msg Incoming message with the desired mood.
   */
  void modeCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void poweroffCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Periodic update tick invoked by a timer.
   */
  void update();

  /**
   * @brief Build full path to a modes audio track.
   * @param filename Audio file name inside ~/Music/Modos.
   * @return Full path if HOME is defined.
   */
  std::optional<std::string> modesTrackPath(const std::string& filename) const;

  /**
   * @brief Play a track from ~/Music/Modos.
   * @param filename Audio file name inside ~/Music/Modos.
   * @return true if playback started.
   */
  bool playModesTrack(const std::string& filename);

  /**
   * @brief Pick a random track from a directory.
   * @param dir Absolute directory path.
   * @return Full path if a supported track exists.
   */
  std::optional<std::string> randomTrackInDir(const std::string& dir) const;

  /**
   * @brief Play a random track from ~/Music/Return.
   * @return true if playback started.
   */
  bool playRandomReturnTrack();

  enum class EyeAction : uint8_t {
    NONE = 0,
    TIRED = 1,
    FROWN = 2,
    CURIOUS = 3,
    LAUGH = 4,
    CONFUSED = 5
  };

  void sendEyeAction(EyeAction action);

  ControlServo servos_;
  std::unique_ptr<robo_audio::AudioPlayer> audio_;
  std::string happy_sound_;
  std::string angry_sound_;
  std::string sad_sound_;
  std::string love_sound_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mood_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr eye_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr eye_idle_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr eye_action_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr poweroff_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<State> current_state_;
  robo_eyes::Mood last_regular_mood_{robo_eyes::Mood::ESPERA};
  robo_eyes::Mood current_mood_{robo_eyes::Mood::ESPERA};
  bool welcome_played_{false};
  bool pending_return_{false};
  bool poweroff_active_{false};
};

} // namespace robofer
