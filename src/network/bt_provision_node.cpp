#include <rclcpp/rclcpp.hpp>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <string>
#include <algorithm>
#include <chrono>

#include "robofer/srv/wifi_set_credentials.hpp"

class BtProvisionNode : public rclcpp::Node {
public:
  BtProvisionNode() : Node("bt_provision_server") {
    wifi_client_ = create_client<robofer::srv::WifiSetCredentials>("/wifi/set_credentials");
    bt_thread_ = std::thread(&BtProvisionNode::server_loop_, this);
  }
  ~BtProvisionNode(){
    running_ = false;
    if(server_sock_ >= 0){
      shutdown(server_sock_, SHUT_RDWR);
      close(server_sock_);
    }
    if(bt_thread_.joinable()) bt_thread_.join();
  }
private:
  void server_loop_(){
    int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if(sock < 0){
      RCLCPP_ERROR(get_logger(), "Cannot create Bluetooth socket");
      return;
    }
    server_sock_ = sock;
    sockaddr_rc loc = {0};
    loc.rc_family = AF_BLUETOOTH;
    bdaddr_t any = {0,0,0,0,0,0};
    loc.rc_bdaddr = any;
    loc.rc_channel = (uint8_t)3;
    if(bind(server_sock_, (struct sockaddr*)&loc, sizeof(loc)) < 0){
      RCLCPP_ERROR(get_logger(), "Bind failed");
      close(server_sock_);
      server_sock_ = -1;
      return;
    }
    listen(server_sock_, 1);
    while(running_){
      sockaddr_rc rem = {0};
      socklen_t opt = sizeof(rem);
      int client = accept(server_sock_, (struct sockaddr*)&rem, &opt);
      if(client < 0){
        if(!running_) break;
        continue;
      }
      char buf[1024] = {0};
      int bytes = read(client, buf, sizeof(buf)-1);
      if(bytes > 0){
        std::string msg(buf, bytes);
        if(msg.rfind("HELLO",0) == 0){
          std::string resp = "ROBOFER\n";
          write(client, resp.c_str(), resp.size());
        } else if(msg.rfind("SET:",0) == 0){
          auto ssid_pos = msg.find("ssid=");
          auto pass_pos = msg.find(";pass=");
          if(ssid_pos != std::string::npos && pass_pos != std::string::npos){
            std::string ssid = msg.substr(ssid_pos+5, pass_pos - (ssid_pos+5));
            std::string pass = msg.substr(pass_pos+6);
            pass.erase(std::remove(pass.begin(), pass.end(), '\n'), pass.end());
            auto req = std::make_shared<robofer::srv::WifiSetCredentials::Request>();
            req->ssid = ssid;
            req->password = pass;
            if(wifi_client_->wait_for_service(std::chrono::seconds(2))){
              auto fut = wifi_client_->async_send_request(req);
              fut.wait();
              write(client, "OK\n", 3);
            } else {
              write(client, "ERROR:no_service\n", 18);
            }
          }
        }
      }
      close(client);
    }
    if(server_sock_ >= 0){
      close(server_sock_);
      server_sock_ = -1;
    }
  }
  rclcpp::Client<robofer::srv::WifiSetCredentials>::SharedPtr wifi_client_;
  std::thread bt_thread_;
  std::atomic<bool> running_{true};
  int server_sock_{-1};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtProvisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
