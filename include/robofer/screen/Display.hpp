#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <memory>
#include <string>

namespace robo_eyes {

/**
 * @brief Common interface for video output backends.
 *
 * Implementations provide a physical display or a simulation window that
 * can render monochrome frames.
 */
class Display {
public:
  /**
   * @brief Virtual destructor for proper cleanup.
   */
  virtual ~Display() = default;

  /**
   * @brief Initialise the display backend using node parameters.
   * @param node ROS node providing configuration.
   * @return true on success.
   */
  virtual bool init(rclcpp::Node& node) = 0;

  /**
   * @brief Width of the target canvas in pixels.
   * @return Width in pixels.
   */
  virtual int width() const = 0;

  /**
   * @brief Height of the target canvas in pixels.
   * @return Height in pixels.
   */
  virtual int height() const = 0;

  /**
   * @brief Display a single 8-bit grayscale frame.
   * @param mono8 Image to render.
   */
  virtual void pushMono8(const cv::Mat& mono8) = 0;
};

/**
 * @brief Factory for display backends.
 * @param backend Identifier of the backend ("sim" or "st7735").
 * @return Unique pointer to the selected backend.
 */
std::unique_ptr<Display> make_display(const std::string& backend);

} // namespace robo_eyes
