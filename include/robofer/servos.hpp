#pragma once

#include <gpiod.h>
#include <thread>
#include <atomic>
#include <string>
#include <vector>

namespace robo_servos {

class ControlServo {
public:
  ControlServo(const std::string& chip_name, int servo1_offset, int servo2_offset);
  ~ControlServo();

  // continuous speed in degrees per second (positive = clockwise)
  void set_speed(int id, float deg_per_sec);

  // move until target angle reached at given speed (deg per second)
  void move_to(int id, float angle_deg, float speed_deg_per_sec);

  // stop any motion
  void stop(int id);

  // return to idle (angle 0)
  void set_idle(int id);

private:
  struct Servo {
    gpiod_line* line{nullptr};
    std::thread th;
    std::atomic<bool> running{false};
    std::atomic<float> current_angle{0.0f};
    std::atomic<float> target_angle{0.0f};
    std::atomic<float> speed{0.0f};
    std::atomic<bool> has_target{false};
  };

  gpiod_chip* chip_{nullptr};
  std::vector<Servo> servos_;

  void thread_func(Servo& s);
};

} // namespace robo_servos

