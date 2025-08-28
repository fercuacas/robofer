#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <string>
#include <sstream>
#include <algorithm>
#include <random>

#include "robofer/srv/wifi_set_credentials.hpp"
#include "robofer/srv/wifi_scan.hpp"
#include "robofer/srv/wifi_get_status.hpp"
#include "robofer/srv/set_bool_with_code.hpp"

using namespace std::chrono_literals;

class BtProvisionNode : public rclcpp::Node {
public:
  BtProvisionNode() : Node("bt_provision_server") {
    state_pub_ = create_publisher<std_msgs::msg::String>("/wifi_prov/state", 10);
    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "/wifi_prov/start",
        std::bind(&BtProvisionNode::handleStart, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        "/wifi_prov/stop",
        std::bind(&BtProvisionNode::handleStop, this, std::placeholders::_1, std::placeholders::_2));
    confirm_srv_ = create_service<robofer::srv::SetBoolWithCode>(
        "/wifi_prov/confirm",
        std::bind(&BtProvisionNode::handleConfirm, this, std::placeholders::_1, std::placeholders::_2));

    wifi_set_client_ = create_client<robofer::srv::WifiSetCredentials>("/wifi/set_credentials");
    wifi_scan_client_ = create_client<robofer::srv::WifiScan>("/wifi/scan");
    wifi_status_client_ = create_client<robofer::srv::WifiGetStatus>("/wifi/get_status");

    publishState("IDLE");
  }

  ~BtProvisionNode(){ stopServer(); }

private:
  enum class State {IDLE, WAITING_PAIRING, SPP_READY, CONNECTED};

  void publishState(const std::string& st){
    std_msgs::msg::String msg; msg.data = st; state_pub_->publish(msg);
  }

  bool sendCredentials(const std::string &ssid, const std::string &pass) {
    auto req = std::make_shared<robofer::srv::WifiSetCredentials::Request>();
    req->ssid = ssid;
    req->password = pass;
    if (wifi_set_client_->wait_for_service(2s)) {
      auto fut = wifi_set_client_->async_send_request(req);
      fut.wait();
      auto res = fut.get();
      return res->success;
    }
    return false;
  }

  std::string currentSsid(){
    auto req = std::make_shared<robofer::srv::WifiGetStatus::Request>();
    if(wifi_status_client_->wait_for_service(2s)){
      auto fut = wifi_status_client_->async_send_request(req);
      fut.wait();
      auto res = fut.get();
      if(res->connected) return res->ssid;
    }
    return "ROBOFER";
  }

  void sendScanResults(int client){
    auto req = std::make_shared<robofer::srv::WifiScan::Request>();
    if(!wifi_scan_client_->wait_for_service(2s)) return;
    auto fut = wifi_scan_client_->async_send_request(req);
    fut.wait();
    auto res = fut.get();
    for(const auto &net : res->networks){
      std::ostringstream oss;
      oss << "NET:ssid=" << net.ssid << ";rssi=" << net.rssi << ";sec=" << net.security << "\n";
      auto s = oss.str();
      ::write(client, s.c_str(), s.size());
    }
    ::write(client, "END\n", 4);
  }

  void handleStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res){
    (void)req;
    if(state_ != State::IDLE){
      res->success = false; res->message = "busy"; return;
    }
    std::random_device rd; std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dist(100000, 999999);
    passkey_ = dist(gen);
    state_ = State::WAITING_PAIRING;
    publishState("CONFIRM_CODE:" + std::to_string(passkey_));
    res->success = true; res->message = "started";
  }

  void handleStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res){
    (void)req;
    stopServer();
    state_ = State::IDLE;
    publishState("IDLE");
    res->success = true; res->message = "stopped";
  }

  void handleConfirm(const std::shared_ptr<robofer::srv::SetBoolWithCode::Request> req,
                     std::shared_ptr<robofer::srv::SetBoolWithCode::Response> res){
    if(state_ != State::WAITING_PAIRING || req->code != passkey_){
      res->success = false; res->message = "invalid"; return;
    }
    if(req->accept){
      startServer();
      res->success = true; res->message = "accepted";
    } else {
      state_ = State::IDLE;
      publishState("IDLE");
      res->success = true; res->message = "rejected";
    }
  }

  void startServer(){
    if(running_) return;
    running_ = true;
    server_thread_ = std::thread(&BtProvisionNode::serverLoop, this);
    state_ = State::SPP_READY;
    publishState("SPP_READY");
  }

  void stopServer(){
    running_ = false;
    if(server_sock_ >= 0){
      ::shutdown(server_sock_, SHUT_RDWR);
      ::close(server_sock_);
      server_sock_ = -1;
    }
    if(server_thread_.joinable()) server_thread_.join();
  }

  void serverLoop(){
    int sock = ::socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if(sock < 0){
      RCLCPP_ERROR(get_logger(), "Cannot create Bluetooth socket");
      running_ = false; return;
    }
    server_sock_ = sock;
    sockaddr_rc loc{}; loc.rc_family = AF_BLUETOOTH; loc.rc_channel = (uint8_t)3; bdaddr_t any{}; loc.rc_bdaddr = any;
    if(::bind(server_sock_, (struct sockaddr*)&loc, sizeof(loc)) < 0){
      RCLCPP_ERROR(get_logger(), "Bind failed");
      ::close(server_sock_); server_sock_ = -1; running_ = false; return;
    }
    ::listen(server_sock_, 1);
    while(running_){
      sockaddr_rc rem{}; socklen_t opt = sizeof(rem);
      int client = ::accept(server_sock_, (struct sockaddr*)&rem, &opt);
      if(client < 0){ if(!running_) break; continue; }
      state_ = State::CONNECTED;
      publishState("CONNECTED");
      handleClient(client);
      ::close(client);
      state_ = State::SPP_READY;
      publishState("SPP_READY");
    }
    ::close(server_sock_); server_sock_ = -1;
    running_ = false;
  }

  void handleClient(int client){
    char buf[1024];
    while(true){
      int bytes = ::read(client, buf, sizeof(buf)-1);
      if(bytes <= 0) break;
      std::string msg(buf, bytes);
      if(msg.rfind("HELLO",0) == 0){
        std::string resp = std::string("SSID:") + currentSsid() + "\n";
        ::write(client, resp.c_str(), resp.size());
      } else if(msg.rfind("LIST",0) == 0){
        sendScanResults(client);
      } else if(msg.rfind("SET:",0) == 0){
        auto ssid_pos = msg.find("ssid=");
        auto pass_pos = msg.find(";pass=");
        if(ssid_pos != std::string::npos && pass_pos != std::string::npos){
          std::string ssid = msg.substr(ssid_pos+5, pass_pos - (ssid_pos+5));
          std::string pass = msg.substr(pass_pos+6);
          pass.erase(std::remove(pass.begin(), pass.end(), '\n'), pass.end());
          bool ok = sendCredentials(ssid, pass);
          if(ok){
            ::write(client, "OK\n", 3);
          } else {
            ::write(client, "ERROR:connect\n", 14);
          }
        }
      }
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Client<robofer::srv::WifiSetCredentials>::SharedPtr wifi_set_client_;
  rclcpp::Client<robofer::srv::WifiScan>::SharedPtr wifi_scan_client_;
  rclcpp::Client<robofer::srv::WifiGetStatus>::SharedPtr wifi_status_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<robofer::srv::SetBoolWithCode>::SharedPtr confirm_srv_;

  std::atomic<State> state_{State::IDLE};
  uint32_t passkey_{0};
  std::atomic<bool> running_{false};
  int server_sock_{-1};
  std::thread server_thread_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BtProvisionNode>());
  rclcpp::shutdown();
  return 0;
}

