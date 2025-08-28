#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <array>
#include <memory>
#include <mutex>
#include <sstream>
#include <algorithm>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "robofer/msg/wifi_status.hpp"
#include "robofer/msg/wifi_network.hpp"
#include "robofer/srv/wifi_get_status.hpp"
#include "robofer/srv/wifi_set_credentials.hpp"
#include "robofer/srv/wifi_scan.hpp"

using namespace std::chrono_literals;

class WifiManager : public rclcpp::Node {
public:
  WifiManager() : Node("wifi_manager") {
    status_pub_ = create_publisher<robofer::msg::WifiStatus>("/wifi/status", 10);
    get_status_srv_ = create_service<robofer::srv::WifiGetStatus>(
        "/wifi/get_status",
        std::bind(&WifiManager::handleGetStatus, this, std::placeholders::_1, std::placeholders::_2));
    set_credentials_srv_ = create_service<robofer::srv::WifiSetCredentials>(
        "/wifi/set_credentials",
        std::bind(&WifiManager::handleSetCredentials, this, std::placeholders::_1, std::placeholders::_2));
    scan_srv_ = create_service<robofer::srv::WifiScan>(
        "/wifi/scan",
        std::bind(&WifiManager::handleScan, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(5s, std::bind(&WifiManager::updateStatus, this));
    updateStatus();
  }

private:
  robofer::msg::WifiStatus queryStatus(){
    robofer::msg::WifiStatus st;
    std::array<char,128> buf{};
    std::unique_ptr<FILE, decltype(&pclose)> pipe(
        popen("nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes' | cut -d: -f2", "r"), pclose);
    if(pipe && fgets(buf.data(), buf.size(), pipe.get())){
      std::string ssid = buf.data();
      ssid.erase(std::remove(ssid.begin(), ssid.end(), '\n'), ssid.end());
      st.connected = !ssid.empty();
      st.ssid = ssid;
    } else {
      st.connected = false;
      st.ssid = "";
    }
    return st;
  }

  void updateStatus(){
    auto st = queryStatus();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_status_ = st;
    }
    status_pub_->publish(st);
  }

  void handleGetStatus(const std::shared_ptr<robofer::srv::WifiGetStatus::Request> req,
                       std::shared_ptr<robofer::srv::WifiGetStatus::Response> res){
    (void)req;
    std::lock_guard<std::mutex> lk(mtx_);
    res->connected = last_status_.connected;
    res->ssid = last_status_.ssid;
  }

  void handleSetCredentials(const std::shared_ptr<robofer::srv::WifiSetCredentials::Request> req,
                            std::shared_ptr<robofer::srv::WifiSetCredentials::Response> res){
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ssid_ = req->ssid;
      pass_ = req->password;
    }
    // Execute nmcli without invoking a shell to avoid command injection
    pid_t pid = fork();
    int ret = -1;
    if(pid == 0){
      execlp("nmcli", "nmcli", "dev", "wifi", "connect", req->ssid.c_str(),
             "password", req->password.c_str(), (char*)nullptr);
      _exit(127); // exec only returns on failure
    } else if(pid > 0){
      int status = 0;
      if(waitpid(pid, &status, 0) > 0 && WIFEXITED(status)){
        ret = WEXITSTATUS(status);
      }
    }
    res->success = (ret == 0);
    res->message = ret==0 ? "OK" : "nmcli failed";
    updateStatus();
  }

  void handleScan(const std::shared_ptr<robofer::srv::WifiScan::Request> req,
                  std::shared_ptr<robofer::srv::WifiScan::Response> res){
    (void)req;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(
        popen("nmcli -t -f SSID,SIGNAL,SECURITY dev wifi", "r"), pclose);
    if(!pipe) return;
    char line[256];
    while(fgets(line, sizeof(line), pipe.get())){
      std::string l(line);
      l.erase(std::remove(l.begin(), l.end(), '\n'), l.end());
      std::stringstream ss(l);
      std::string ssid, signal_str, sec;
      std::getline(ss, ssid, ':');
      std::getline(ss, signal_str, ':');
      std::getline(ss, sec, ':');
      robofer::msg::WifiNetwork net;
      net.ssid = ssid;
      net.rssi = signal_str.empty() ? 0 : std::stoi(signal_str);
      net.security = sec;
      res->networks.push_back(net);
    }
  }

  rclcpp::Publisher<robofer::msg::WifiStatus>::SharedPtr status_pub_;
  rclcpp::Service<robofer::srv::WifiGetStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<robofer::srv::WifiSetCredentials>::SharedPtr set_credentials_srv_;
  rclcpp::Service<robofer::srv::WifiScan>::SharedPtr scan_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string ssid_;
  std::string pass_;
  robofer::msg::WifiStatus last_status_;
  std::mutex mtx_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiManager>());
  rclcpp::shutdown();
  return 0;
}
