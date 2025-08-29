#include "robofer/bluetoothctl_agent.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace {
  // Hace el fd no bloqueante (opcional, aquí lo dejamos bloqueante).
  void set_cloexec(int fd) {
    int flags = fcntl(fd, F_GETFD);
    if (flags >= 0) fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
  }
}

bool BluetoothctlAgent::start(LineHandler on_line) {
  static auto logger = rclcpp::get_logger("BluetoothctlAgent");
  if (running_) {
    RCLCPP_INFO(logger, "Agent already running");
    return true;
  }

  int to_child[2];     // padre escribe -> hijo lee (stdin)
  int from_child[2];   // hijo escribe -> padre lee (stdout/err)
  if (pipe(to_child) < 0 || pipe(from_child) < 0) {
    RCLCPP_INFO(logger, "pipe() failed");
    return false;
  }

  child_pid_ = fork();
  if (child_pid_ < 0) {
    RCLCPP_INFO(logger, "fork() failed");
    return false;
  }

  if (child_pid_ == 0) {
    // ---- PROCESO HIJO ----
    // Redirigir stdin/out/err
    dup2(to_child[0], STDIN_FILENO);
    dup2(from_child[1], STDOUT_FILENO);
    dup2(from_child[1], STDERR_FILENO);
    close(to_child[1]);
    close(from_child[0]);

    // Forzar salida en inglés para parsear mensajes de forma estable
    setenv("LANG", "C", 1);
    setenv("LC_ALL", "C", 1);

    // Ejecutar bluetoothctl
    execlp("bluetoothctl", "bluetoothctl", (char*)nullptr);
    _exit(1); // Si execlp falla
  }

  // ---- PROCESO PADRE ----
  close(to_child[0]);
  close(from_child[1]);
  in_fd_  = to_child[1];
  out_fd_ = from_child[0];
  set_cloexec(in_fd_);
  set_cloexec(out_fd_);

  on_line_ = std::move(on_line);
  running_ = true;

  reader_ = std::thread([this, logger]() {
    std::string buf; buf.reserve(4096);
    char tmp[256];
    while (running_) {
      ssize_t n = ::read(out_fd_, tmp, sizeof(tmp));
      if (n <= 0) break;
      buf.append(tmp, tmp + n);
      std::size_t pos;
      while ((pos = buf.find('\n')) != std::string::npos) {
        std::string line = buf.substr(0, pos);
        buf.erase(0, pos + 1);
        RCLCPP_INFO(logger, "[btctl] %s", line.c_str());
        if (on_line_) on_line_(line);
      }
    }
  });

  RCLCPP_INFO(logger, "bluetoothctl process started (pid %d)", child_pid_);
  return true;
}

void BluetoothctlAgent::send(const std::string& cmd) {
  static auto logger = rclcpp::get_logger("BluetoothctlAgent");
  if (in_fd_ < 0) {
    RCLCPP_INFO(logger, "send() ignored, no pipe");
    return;
  }
  std::string s = cmd + "\n";
  RCLCPP_INFO(logger, "send: %s", cmd.c_str());
  ::write(in_fd_, s.data(), s.size());
}

void BluetoothctlAgent::stop() {
  static auto logger = rclcpp::get_logger("BluetoothctlAgent");
  if (!running_) return;
  running_ = false;
  RCLCPP_INFO(logger, "Stopping agent");

  // Intenta salir de forma amable
  if (in_fd_ >= 0) {
    send("exit");
    ::close(in_fd_);
    in_fd_ = -1;
  }
  if (out_fd_ >= 0) {
    ::close(out_fd_);
    out_fd_ = -1;
  }
  if (reader_.joinable()) reader_.join();

  if (child_pid_ > 0) {
    int st = 0;
    waitpid(child_pid_, &st, 0);
    child_pid_ = -1;
  }
  RCLCPP_INFO(logger, "Agent stopped");
}

void BluetoothctlAgent::powerOn() {
  send("power on");
}

void BluetoothctlAgent::powerOff() {
  send("power off");
}

void BluetoothctlAgent::enableProvisionWindow(const std::string& alias, int discoverable_seconds) {
  static auto logger = rclcpp::get_logger("BluetoothctlAgent");
  RCLCPP_INFO(logger, "Enabling provision window alias='%s' timeout=%d", alias.c_str(), discoverable_seconds);
  if (!alias.empty()) {
    send("system-alias " + alias); // en algunas versiones es 'system-alias' o 'set-alias'
    // Si tu bluetoothctl no soporta system-alias, ignora esta línea.
  }
  send("agent DisplayYesNo");
  send("default-agent");
  send("pairable on");
  send("discoverable on");

  // algunos bluetoothctl soportan 'discoverable-timeout <s>'
  if (discoverable_seconds > 0) {
    send("discoverable-timeout " + std::to_string(discoverable_seconds));
  }
}

void BluetoothctlAgent::disableProvisionWindow() {
  static auto logger = rclcpp::get_logger("BluetoothctlAgent");
  RCLCPP_INFO(logger, "Disabling provision window");
  send("discoverable off");
  send("pairable off");
}

