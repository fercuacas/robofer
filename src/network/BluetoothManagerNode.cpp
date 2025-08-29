#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

using namespace std::placeholders;

class BluetoothManager : public rclcpp::Node {
public:
  BluetoothManager() : Node("bluetooth_manager") {
    state_pub_ = create_publisher<std_msgs::msg::String>("/bluetooth/state", 10);
    power_srv_ = create_service<std_srvs::srv::SetBool>(
        "/bluetooth/power", std::bind(&BluetoothManager::handlePower, this, _1, _2));
    pair_srv_ = create_service<std_srvs::srv::SetBool>(
        "/bluetooth/pair_response", std::bind(&BluetoothManager::handlePair, this, _1, _2));
    pair_sub_ = create_subscription<std_msgs::msg::String>(
        "/bluetooth/pair_request", 10,
        std::bind(&BluetoothManager::onPairRequest, this, _1));
    publishState();
  }

private:
  void publishState(){
    std_msgs::msg::String msg;
    if(!enabled_){
      msg.data = "OFF";
    } else if(pending_device_.empty()){
      msg.data = "ON";
    } else {
      msg.data = std::string("REQUEST:") + pending_device_;
    }
    state_pub_->publish(msg);
  }

  void onPairRequest(const std_msgs::msg::String::SharedPtr msg){
    pending_device_ = msg->data;
    publishState();
  }

  void handlePower(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res){
    enabled_ = req->data;
    if(enabled_){
      std::system("bluetoothctl power on >/dev/null 2>&1");
    } else {
      std::system("bluetoothctl power off >/dev/null 2>&1");
      pending_device_.clear();
    }
    publishState();
    res->success = true;
    res->message = enabled_ ? "ON" : "OFF";
  }

  void handlePair(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res){
    if(pending_device_.empty()){
      res->success = false;
      res->message = "No request";
      return;
    }
    if(req->data){
      std::string cmd = std::string("bluetoothctl pair ") + pending_device_ + " >/dev/null 2>&1";
      std::system(cmd.c_str());
    } else {
      std::string cmd = std::string("bluetoothctl reject ") + pending_device_ + " >/dev/null 2>&1";
      std::system(cmd.c_str());
    }
    pending_device_.clear();
    publishState();
    res->success = true;
    res->message = req->data ? "paired" : "rejected";
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pair_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pair_sub_;
  bool enabled_{false};
  std::string pending_device_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BluetoothManager>());
  rclcpp::shutdown();
  return 0;
}

