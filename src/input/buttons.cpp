#include "robofer/input/buttons.hpp"

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

namespace robo_input {

ButtonWatcher::ButtonWatcher(rclcpp::Node& node,
                             const std::string& chip,
                             int off, int code, bool rising, int debounce,
                             rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher)
  : chip_name(chip), offset(off), ui_code(code), rising_on_press(rising),
    debounce_ms(debounce), logger(node.get_logger()), pub(std::move(publisher)) {}

ButtonWatcher::~ButtonWatcher(){ stop(); }

bool ButtonWatcher::start(){
  if(offset < 0) return true;
  chip = gpiod_chip_open_by_name(chip_name.c_str());
  if(!chip){
    RCLCPP_ERROR(logger, "gpiod_chip_open_by_name('%s') fallo", chip_name.c_str());
    return false;
  }
  line = gpiod_chip_get_line(chip, offset);
  if(!line){
    RCLCPP_ERROR(logger, "gpiod_chip_get_line(offset=%d) fallo", offset);
    return false;
  }
  unsigned flags = 0;
  int rc = rising_on_press
    ? gpiod_line_request_rising_edge_events_flags(line, "buttons_node", flags)
    : gpiod_line_request_falling_edge_events_flags(line, "buttons_node", flags);
  if(rc < 0){
    RCLCPP_ERROR(logger, "gpiod_line_request_*_events(offset=%d) fallo", offset);
    return false;
  }

  running = true;
  th = std::thread([this]{
    using clock = std::chrono::steady_clock;
    auto last_emit = clock::now() - std::chrono::milliseconds(debounce_ms);
    while(running.load()){
      timespec ts{1,0};
      int wait_rc = gpiod_line_event_wait(line, &ts);
      if(wait_rc <= 0) continue;
      gpiod_line_event ev{};
      if(gpiod_line_event_read(line, &ev) < 0){
        RCLCPP_WARN(logger, "gpiod_line_event_read fallo (offset=%d)", offset);
        continue;
      }
      bool press = false;
      if(rising_on_press && ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) press = true;
      if(!rising_on_press && ev.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) press = true;
      if(press){
        auto now = clock::now();
        if(now - last_emit >= std::chrono::milliseconds(debounce_ms)){
          last_emit = now;
          std_msgs::msg::Int32 m; m.data = ui_code;
          pub->publish(m);
          RCLCPP_INFO(logger, "Btn offset=%d -> /ui/button=%d", offset, ui_code);
        }
      }
    }
  });

  RCLCPP_INFO(logger, "Watcher: chip=%s offset=%d ui_code=%d rising=%s debounce=%dms",
              chip_name.c_str(), offset, ui_code, rising_on_press?"true":"false", debounce_ms);
  return true;
}

void ButtonWatcher::stop(){
  if(!running.exchange(false)) return;
  if(th.joinable()) th.join();
  if(line){ gpiod_line_release(line); line=nullptr; }
  if(chip){ gpiod_chip_close(chip); chip=nullptr; }
}

ButtonsNode::ButtonsNode() : Node("buttons_node")
{
  const auto chip   = this->declare_parameter<std::string>("gpiochip", "gpiochip0");
  const bool rising = this->declare_parameter<bool>("rising_on_press", true);
  const int  debounce = this->declare_parameter<int>("debounce_ms", 150);

  const int btn1_off = this->declare_parameter<int>("btn1_offset", -1);
  const int btn2_off = this->declare_parameter<int>("btn2_offset", -1);
  const int btn3_off = this->declare_parameter<int>("btn3_offset", -1);
  const int btn4_off = this->declare_parameter<int>("btn4_offset", -1);

  const int btn1_code = this->declare_parameter<int>("btn1_code", 0);
  const int btn2_code = this->declare_parameter<int>("btn2_code", 1);
  const int btn3_code = this->declare_parameter<int>("btn3_code", 2);
  const int btn4_code = this->declare_parameter<int>("btn4_code", 3);

  pub_ = this->create_publisher<std_msgs::msg::Int32>("/ui/button", 10);

  auto make = [&](int off, int code){
    return std::make_unique<ButtonWatcher>(*this, chip, off, code, rising, debounce, pub_);
  };

  watchers_.emplace_back(make(btn1_off, btn1_code));
  watchers_.emplace_back(make(btn2_off, btn2_code));
  watchers_.emplace_back(make(btn3_off, btn3_code));
  watchers_.emplace_back(make(btn4_off, btn4_code));

  bool ok = true;
  for(auto& w : watchers_){
    if(w->offset >= 0) ok = w->start() && ok;
  }
  if(!ok){
    RCLCPP_FATAL(this->get_logger(), "Fallo inicializando botones. Revisa gpiochip/offsets.");
    throw std::runtime_error("buttons init failed");
  }
  RCLCPP_INFO(this->get_logger(), "ButtonsNode listo. Publica /ui/button (0=UP,1=DOWN,2=BACK,3=OK).");
}

} // namespace robo_input

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  try {
    auto n = std::make_shared<robo_input::ButtonsNode>();
    rclcpp::spin(n);
  } catch(...) {}
  rclcpp::shutdown();
  return 0;
}

