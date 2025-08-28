#pragma once

#include <gpiod.h>
#include <thread>
#include <atomic>
#include <string>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include "robofer/msg/servo_goal.hpp"

namespace robo_servos {

/**
 * @brief Control interface for two hardware servomotors.
 *
 * The class wraps libgpiod access and provides simple speed and position
 * commands for a pair of servos. It also publishes goal angles to a ROS
 * topic for other components to monitor.
 */
class ControlServo {
public:
  /**
   * @brief Construct a new ControlServo instance.
   * @param node Parent ROS node.
   * @param chip_name Name of the GPIO chip used to drive the servos.
   * @param servo1_offset Offset applied to servo 1 neutral position.
   * @param servo2_offset Offset applied to servo 2 neutral position.
   * @param sim When true, no hardware access is performed.
   */
  ControlServo(rclcpp::Node *node,
               const std::string &chip_name,
               int servo1_offset,
               int servo2_offset,
               bool sim = false);

  /**
   * @brief Destructor that stops control threads.
   */
  ~ControlServo();

  /**
   * @brief Set continuous rotation speed for a servo.
   * @param id Servo identifier.
   * @param deg_per_sec Speed in degrees per second (positive = clockwise).
   */
  void setSpeed(int id, float deg_per_sec);

  /**
   * @brief Move servo to a target angle at a given speed.
   * @param id Servo identifier.
   * @param angle_deg Target angle in degrees.
   * @param speed_deg_per_sec Speed in degrees per second.
   */
  void moveTo(int id, float angle_deg, float speed_deg_per_sec);

  /**
   * @brief Stop any motion on the specified servo.
   * @param id Servo identifier.
   */
  void stop(int id);

  /**
   * @brief Move servo back to the idle (zero) angle.
   * @param id Servo identifier.
   */
  void setIdle(int id);

private:
  struct Servo {
    int id_{0};
    gpiod_line *line_{nullptr};
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<float> current_angle_{0.0f};
    std::atomic<float> target_angle_{0.0f};
    std::atomic<float> speed_{0.0f};
    std::atomic<bool> has_target_{false};
  };

  gpiod_chip *chip_{nullptr};
  std::array<Servo, 2> servos_;
  bool sim_{false};
  rclcpp::Publisher<robofer::msg::ServoGoal>::SharedPtr angle_pub_;

  /**
   * @brief Worker thread that drives a single servo.
   * @param s Servo control block.
   */
  void threadFunc(Servo &s);
};

} // namespace robo_servos

