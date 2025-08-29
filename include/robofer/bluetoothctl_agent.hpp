#pragma once
#include <functional>
#include <string>
#include <thread>
#include <atomic>
#include <regex>

// Agente mínimo para controlar una sesión persistente de `bluetoothctl`.
// - start(onLine): lanza bluetoothctl, redirige stdin/stdout y crea un hilo lector.
// - send(cmd): escribe "cmd\n" al stdin del proceso.
// - stop(): sale de bluetoothctl y limpia recursos.
// Sugerencia: en el callback onLine parseas "Confirm passkey 123456 (yes/no)" y "Pairing successful".

class BluetoothctlAgent {
public:
  using LineHandler = std::function<void(const std::string&)>;

  BluetoothctlAgent() = default;
  ~BluetoothctlAgent() { stop(); }

  // Arranca `bluetoothctl` y comienza a leer líneas en un hilo aparte.
  // Devuelve false si no se pudo lanzar el proceso.
  bool start(LineHandler on_line);

  // Envía un comando a bluetoothctl (se añade '\n' automáticamente).
  void send(const std::string& cmd);

  // Para el hilo lector y cierra el proceso bluetoothctl.
  void stop();

  // Atajos útiles
  void powerOn();
  void powerOff();
  void enableProvisionWindow(const std::string& alias = "", int discoverable_seconds = 180);
  void disableProvisionWindow();

private:
  int in_fd_  = -1;  // lo que ESCRIBES aquí va al stdin de bluetoothctl
  int out_fd_ = -1;  // lo que LEES aquí viene del stdout/stderr de bluetoothctl
  pid_t child_pid_ = -1;
  std::thread reader_;
  std::atomic<bool> running_{false};
  LineHandler on_line_;
};

