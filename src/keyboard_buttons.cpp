#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <unordered_map>

class RawTerminalGuard {
public:
  RawTerminalGuard(){
    tcgetattr(STDIN_FILENO, &orig_);
    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  ~RawTerminalGuard(){ tcsetattr(STDIN_FILENO, TCSANOW, &orig_); }
private:
  termios orig_{};
};

class KeyboardButtonsNode : public rclcpp::Node {
public:
  KeyboardButtonsNode() : Node("keyboard_buttons_node")
  {
    key_up_   = this->declare_parameter<std::string>("key_up", "w");
    key_down_ = this->declare_parameter<std::string>("key_down", "s");
    key_back_ = this->declare_parameter<std::string>("key_back", "a");
    key_ok_   = this->declare_parameter<std::string>("key_ok", "d");
    repeat_ms_= this->declare_parameter<int>("repeat_ms", 0);

    pub_ = this->create_publisher<std_msgs::msg::Int32>("/ui/button", 10);

    map_[norm(key_up_)]   = 0;
    map_[norm(key_down_)] = 1;
    map_[norm(key_back_)] = 2;
    map_[norm(key_ok_)]   = 3;

    RCLCPP_INFO(this->get_logger(),
      "Teclado activo: [%s]=UP(0), [%s]=DOWN(1), [%s]=BACK(2), [%s]=OK(3), repeat_ms=%d",
      key_up_.c_str(), key_down_.c_str(), key_back_.c_str(), key_ok_.c_str(), repeat_ms_);

    running_.store(true);
    th_ = std::thread([this]{ this->loop(); });
  }

  ~KeyboardButtonsNode() override {
    running_.store(false);
    if(th_.joinable()) th_.join();
  }

private:
  std::string key_up_, key_down_, key_back_, key_ok_;
  int repeat_ms_{0};
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  std::thread th_;
  std::atomic<bool> running_{false};
  std::unordered_map<int,int> map_;

  static int norm(const std::string& s){
    if(s.empty()) return 0;
    return static_cast<int>(::tolower(s[0]) & 0xFF);
  }

  void loop(){
    RawTerminalGuard rtg;
    auto last_sent = std::unordered_map<int, rclcpp::Time>{};
    while(rclcpp::ok() && running_.load()){
      fd_set rfds; FD_ZERO(&rfds); FD_SET(STDIN_FILENO, &rfds);
      timeval tv{0,20000};
      int rv = select(STDIN_FILENO+1, &rfds, nullptr, nullptr, &tv);
      if(rv > 0 && FD_ISSET(STDIN_FILENO, &rfds)){
        char c; ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if(n == 1){
          int code = static_cast<int>(::tolower(c) & 0xFF);
          auto it = map_.find(code);
          if(it != map_.end()){
            int ui_code = it->second;
            auto now = this->now();
            bool can_send = true;
            if(repeat_ms_ > 0){
              auto jt = last_sent.find(code);
              if(jt != last_sent.end()){
                auto dt = (now - jt->second).nanoseconds() / 1000000;
                can_send = dt >= repeat_ms_;
              }
            }
            if(can_send){
              std_msgs::msg::Int32 m; m.data = ui_code;
              pub_->publish(m);
              last_sent[code] = now;
              RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Key '%c' -> /ui/button=%d", c, ui_code);
            }
          }
        }
      }
    }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardButtonsNode>());
  rclcpp::shutdown();
  return 0;
}

