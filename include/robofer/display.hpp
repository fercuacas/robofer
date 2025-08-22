#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <memory>
#include <string>

namespace robo_eyes {

// Interfaz común de salida de vídeo
class Display {
public:
  virtual ~Display() = default;
  // Lee parámetros específicos del backend desde el mismo nodo y deja listo el dispositivo/ventana.
  virtual bool init(rclcpp::Node& node) = 0;
  // Ancho/alto del "lienzo" del display físico/ventana.
  virtual int width()  const = 0;
  virtual int height() const = 0;
  // Pinta un frame 8UC1 (0..255). Cada backend decide cómo centrar/llenar.
  virtual void pushMono8(const cv::Mat& mono8) = 0;
};

// Fábrica: backend="sim" | "st7735"
// Crea el backend de salida de vídeo correspondiente.
std::unique_ptr<Display> makeDisplay(const std::string& backend);

} // namespace robo_eyes
