// Este nodo controla el estado del adaptador Bluetooth utilizando un agente
// persistente de `bluetoothctl` para poder aceptar/rechazar emparejamientos.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <regex>
#include <mutex>

#include "robofer/bluetoothctl_agent.hpp"

using namespace std::placeholders;

class BluetoothManager : public rclcpp::Node {
public:
  BluetoothManager() : Node("bluetooth_manager") {
    state_pub_ = create_publisher<std_msgs::msg::String>("/bluetooth/state", 10);
    power_srv_ = create_service<std_srvs::srv::SetBool>(
        "/bluetooth/power", std::bind(&BluetoothManager::handlePower, this, _1, _2));
    pair_srv_ = create_service<std_srvs::srv::SetBool>(
        "/bluetooth/pair_response", std::bind(&BluetoothManager::handlePair, this, _1, _2));
    publishState();
  }

private:
  void publishState(){
    std::lock_guard<std::mutex> lk(mtx_);
    std_msgs::msg::String msg;
    if(!enabled_){
      msg.data = "OFF";
    } else if(pending_code_.empty()){
      msg.data = "ON";
    } else {
      msg.data = std::string("REQUEST:") + pending_code_;
    }
    state_pub_->publish(msg);
  }

  void handlePower(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res){
    enabled_ = req->data;
    if(enabled_){
      bool ok = agent_.start([this](const std::string& line){
        static const std::regex re_passkey(
            "(?:confirm|request).*?(?:passkey|pin(?:\\s+code)?)\\D*(\\d{4,6})",
            std::regex::icase);
        std::smatch m;
        if(std::regex_search(line, m, re_passkey)){
          {
            std::lock_guard<std::mutex> lk(mtx_);
            pending_code_ = m[1];
          }
          publishState();
          return;
        }
        if(line.find("Pairing successful") != std::string::npos ||
           line.find(" Paired: yes") != std::string::npos){
          {
            std::lock_guard<std::mutex> lk(mtx_);
            pending_code_.clear();
          }
          publishState();
        }
      });
      if(ok){
        agent_.powerOn();
        agent_.enableProvisionWindow();
      }
    } else {
      agent_.disableProvisionWindow();
      agent_.powerOff();
      agent_.stop();
      {
        std::lock_guard<std::mutex> lk(mtx_);
        pending_code_.clear();
      }
    }
    publishState();
    res->success = true;
    res->message = enabled_ ? "ON" : "OFF";
  }

  void handlePair(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res){
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if(pending_code_.empty()){
        res->success = false;
        res->message = "No request";
        return;
      }
    }
    agent_.send(req->data ? "yes" : "no");
    {
      std::lock_guard<std::mutex> lk(mtx_);
      pending_code_.clear();
    }
    publishState();
    res->success = true;
    res->message = req->data ? "paired" : "rejected";
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pair_srv_;
  BluetoothctlAgent agent_;
  bool enabled_{false};
  std::string pending_code_;
  std::mutex mtx_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BluetoothManager>());
  rclcpp::shutdown();
  return 0;
}

