#include <algorithm>
#include <atomic>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <memory>

#include "robofer/srv/wifi_get_status.hpp"
#include "robofer/srv/wifi_scan.hpp"
#include "robofer/srv/wifi_set_credentials.hpp"

using namespace std::chrono_literals;

class BtProvisionNode : public rclcpp::Node {
public:
  BtProvisionNode() : Node("bt_provision_server") {
    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "/wifi_prov/start",
        std::bind(&BtProvisionNode::handleStart, this, std::placeholders::_1,
                  std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        "/wifi_prov/stop",
        std::bind(&BtProvisionNode::handleStop, this, std::placeholders::_1,
                  std::placeholders::_2));
    state_pub_ =
        create_publisher<std_msgs::msg::String>("/wifi_prov/state", 10);

    wifi_set_client_ =
        create_client<robofer::srv::WifiSetCredentials>("/wifi/set_credentials");
    wifi_scan_client_ =
        create_client<robofer::srv::WifiScan>("/wifi/scan");
    wifi_status_client_ =
        create_client<robofer::srv::WifiGetStatus>("/wifi/get_status");

    publishState("IDLE");
  }

  ~BtProvisionNode() override { stopServer(); }

private:
  void publishState(const std::string &s) {
    std_msgs::msg::String msg;
    msg.data = s;
    state_pub_->publish(msg);
  }

  bool sendCredentials(const std::string &ssid, const std::string &pass) {
    auto req =
        std::make_shared<robofer::srv::WifiSetCredentials::Request>();
    req->ssid = ssid;
    req->password = pass;
    if (wifi_set_client_->wait_for_service(2s)) {
      auto fut = wifi_set_client_->async_send_request(req);
      if (rclcpp::spin_until_future_complete(shared_from_this(), fut) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        return fut.get()->success;
      }
    }
    return false;
  }

  std::string currentSsid() {
    auto req =
        std::make_shared<robofer::srv::WifiGetStatus::Request>();
    if (wifi_status_client_->wait_for_service(1s)) {
      auto fut = wifi_status_client_->async_send_request(req);
      if (rclcpp::spin_until_future_complete(shared_from_this(), fut) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        if (fut.get()->connected) {
          return fut.get()->ssid;
        }
      }
    }
    return std::string("ROBOFER");
  }

  void handleStart(const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
    if (running_) {
      res->success = false;
      res->message = "already running";
      return;
    }
    running_ = true;
    bt_thread_ = std::thread(&BtProvisionNode::serverLoop, this);
    res->success = true;
    res->message = "started";
  }

  void handleStop(const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr res) {
    bool was_running = running_;
    stopServer();
    res->success = was_running;
    res->message = "stopped";
  }

  void stopServer() {
    if (!running_)
      return;
    running_ = false;
    if (server_sock_ >= 0) {
      shutdown(server_sock_, SHUT_RDWR);
      close(server_sock_);
      server_sock_ = -1;
    }
    if (bt_thread_.joinable()) {
      bt_thread_.join();
    }
    publishState("IDLE");
  }

  void serverLoop() {
    int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (sock < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot create Bluetooth socket");
      running_ = false;
      return;
    }
    server_sock_ = sock;

    sockaddr_rc loc{};
    loc.rc_family = AF_BLUETOOTH;
    bdaddr_t any{};
    loc.rc_bdaddr = any;
    loc.rc_channel = static_cast<uint8_t>(3);
    if (bind(server_sock_, (sockaddr *)&loc, sizeof(loc)) < 0) {
      RCLCPP_ERROR(get_logger(), "Bind failed");
      close(server_sock_);
      server_sock_ = -1;
      running_ = false;
      return;
    }
    listen(server_sock_, 1);
    publishState("LISTEN");

    while (running_) {
      sockaddr_rc rem{};
      socklen_t opt = sizeof(rem);
      int client = accept(server_sock_, (sockaddr *)&rem, &opt);
      if (client < 0) {
        if (!running_)
          break;
        continue;
      }
      publishState("CONNECTED");
      handleClient(client);
      close(client);
      publishState("LISTEN");
    }
    if (server_sock_ >= 0) {
      close(server_sock_);
      server_sock_ = -1;
    }
  }

  void handleClient(int client) {
    std::string data;
    char buf[256];
    while (running_) {
      int n = read(client, buf, sizeof(buf));
      if (n <= 0)
        break;
      data.append(buf, n);
      size_t pos = 0;
      while ((pos = data.find('\n')) != std::string::npos) {
        std::string line = data.substr(0, pos);
        data.erase(0, pos + 1);
        processLine(line, client);
      }
    }
  }

  void processLine(const std::string &line, int client) {
    if (line.rfind("HELLO", 0) == 0) {
      std::string resp = std::string("SSID:") + currentSsid() + "\n";
      write(client, resp.c_str(), resp.size());
    } else if (line.rfind("LIST", 0) == 0) {
      auto req = std::make_shared<robofer::srv::WifiScan::Request>();
      if (wifi_scan_client_->wait_for_service(2s)) {
        auto fut = wifi_scan_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), fut) ==
            rclcpp::FutureReturnCode::SUCCESS) {
          for (const auto &net : fut.get()->networks) {
            std::string msg = "NET:ssid=" + net.ssid + ";rssi=" +
                              std::to_string(net.rssi) + ";sec=" +
                              net.security + "\n";
            write(client, msg.c_str(), msg.size());
          }
        }
      }
      write(client, "END\n", 4);
    } else if (line.rfind("SET:", 0) == 0) {
      auto ssid_pos = line.find("ssid=");
      auto pass_pos = line.find(";pass=");
      if (ssid_pos != std::string::npos && pass_pos != std::string::npos) {
        std::string ssid =
            line.substr(ssid_pos + 5, pass_pos - (ssid_pos + 5));
        std::string pass = line.substr(pass_pos + 6);
        bool ok = sendCredentials(ssid, pass);
        if (ok) {
          write(client, "OK\n", 3);
        } else {
          write(client, "ERROR\n", 6);
        }
      }
    }
  }

  // ROS entities
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::Client<robofer::srv::WifiSetCredentials>::SharedPtr
      wifi_set_client_;
  rclcpp::Client<robofer::srv::WifiScan>::SharedPtr wifi_scan_client_;
  rclcpp::Client<robofer::srv::WifiGetStatus>::SharedPtr
      wifi_status_client_;

  std::thread bt_thread_;
  std::atomic<bool> running_{false};
  int server_sock_{-1};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtProvisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

