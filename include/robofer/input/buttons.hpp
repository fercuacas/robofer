#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <gpiod.h>
#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <string>

namespace robo_input {

/**
 * @brief Helper class that watches a GPIO line for button presses.
 */
struct ButtonWatcher {
  std::string chip_name{"gpiochip0"};
  int offset{-1};
  int ui_code{0};
  bool rising_on_press{true};
  int debounce_ms{150};

  rclcpp::Logger logger;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;

  gpiod_chip* chip{nullptr};
  gpiod_line* line{nullptr};
  std::thread th;
  std::atomic<bool> running{false};

  ButtonWatcher(rclcpp::Node& node,
                const std::string& chip,
                int off, int code, bool rising, int debounce,
                rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher);
  ~ButtonWatcher();

  bool start();
  void stop();
};

/**
 * @brief ROS node that publishes button events to /ui/button.
 */
class ButtonsNode : public rclcpp::Node {
public:
  ButtonsNode();

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  std::vector<std::unique_ptr<ButtonWatcher>> watchers_;
};

} // namespace robo_input

