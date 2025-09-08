#include "robofer/screen/Display.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Solo para el backend HW
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <gpiod.h>
#include <cerrno>
#include <cstring>
#include <vector>
#include <algorithm>

namespace robo_eyes {

// =======================
// IMPLEMENTACIÓN: SIM (OpenCV)
// =======================
class DisplaySim final : public Display {
public:
  bool init(rclcpp::Node& node) override {
    node_ = &node;
    title_ = node.declare_parameter<std::string>("sim_window_title", "RoboEyes SIM");
    scale_ = node.declare_parameter<int>("sim_scale", 6);
    // El sim usa como tamaño el del frame entrante; aquí dejamos 0 y lo fijamos al primer push
    w_ = node.declare_parameter<int>("sim_width",  0);
    h_ = node.declare_parameter<int>("sim_height", 0);
    return true;
  }

  int width()  const override { return w_>0 ? w_ : last_w_; }
  int height() const override { return h_>0 ? h_ : last_h_; }

  void pushMono8(const cv::Mat& mono8) override {
    if(mono8.empty()) return;
    last_w_ = mono8.cols; last_h_ = mono8.rows;

    if(!window_ready_) {
      cv::namedWindow(title_, cv::WINDOW_NORMAL);
      cv::resizeWindow(title_, mono8.cols * scale_, mono8.rows * scale_);
      window_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "SIM window '%s' ready (%dx%d x%dx scale)",
                  title_.c_str(), mono8.cols, mono8.rows, scale_);
    }
    cv::Mat view;
    cv::resize(mono8, view, cv::Size(mono8.cols*scale_, mono8.rows*scale_), 0, 0, cv::INTER_NEAREST);
    cv::imshow(title_, view);
    cv::waitKey(1);
  }

private:
  rclcpp::Node* node_{};
  std::string title_;
  int scale_{6};
  int w_{0}, h_{0};             // opcional, si el usuario fuerza
  int last_w_{0}, last_h_{0};   // recordado del último frame (0 = desconocido)
  bool window_ready_{false};
};


// =======================
// IMPLEMENTACIÓN: ST7735 (SPI + libgpiod)
// =======================
namespace {
constexpr uint8_t CMD_SWRESET = 0x01;
constexpr uint8_t CMD_SLPOUT  = 0x11;
constexpr uint8_t CMD_COLMOD  = 0x3A; // 16bpp = 0x05
constexpr uint8_t CMD_MADCTL  = 0x36;
constexpr uint8_t CMD_CASET   = 0x2A;
constexpr uint8_t CMD_RASET   = 0x2B;
constexpr uint8_t CMD_RAMWR   = 0x2C;
constexpr uint8_t CMD_INVOFF  = 0x20;
constexpr uint8_t CMD_INVON   = 0x21;
constexpr uint8_t CMD_NORON   = 0x13;
constexpr uint8_t CMD_DISPON  = 0x29;

inline void msleep(int ms){ usleep(ms*1000); }

struct GpioLine {
  gpiod_line* line = nullptr;
  gpiod_chip* chip = nullptr;
  ~GpioLine(){ if(line){ gpiod_line_release(line);} if(chip){ gpiod_chip_close(chip);} }
  bool request(const char* chip_name, unsigned offset, int initial=1, const char* name="st7735"){
    chip = gpiod_chip_open_by_name(chip_name);
    if(!chip){ return false; }
    line = gpiod_chip_get_line(chip, offset);
    if(!line){ return false; }
    return gpiod_line_request_output(line, name, initial) == 0;
  }
  void set(int v){ if(line) gpiod_line_set_value(line, v); }
};
} // anon

class DisplayST7735 final : public Display {
public:
  bool init(rclcpp::Node& node) override {
    node_ = &node;
    // ====== Parámetros hardware (defaults como los tuyos) ======
    spi_dev_      = node.declare_parameter<std::string>("spi_device", "/dev/spidev1.0");
    spi_hz_       = node.declare_parameter<int>("spi_hz", 24000000);   // <— 24 MHz por defecto
    lcd_w_        = node.declare_parameter<int>("lcd_width", 128);
    lcd_h_        = node.declare_parameter<int>("lcd_height", 160);
    x_off_        = node.declare_parameter<int>("x_offset", 0);
    y_off_        = node.declare_parameter<int>("y_offset", 0);
    madctl_       = uint8_t(node.declare_parameter<int>("madctl", 0xA0));
    invert_       = node.declare_parameter<bool>("invert", false);
    self_test_    = node.declare_parameter<bool>("self_test", true);
    spi_chunk_    = (size_t)node.declare_parameter<int>("spi_chunk", 2048);

    use_manual_cs_= node.declare_parameter<bool>("use_manual_cs", true);
    gpiochip_c_   = node.declare_parameter<std::string>("gpiochip_c", "gpiochip0");
    dc_offset_    = node.declare_parameter<int>("dc_offset", 75);
    rst_offset_   = node.declare_parameter<int>("rst_offset", 78);
    cs_offset_    = node.declare_parameter<int>("cs_offset", 233);

    // Si el bit MV está activo, la pantalla rota 90° y se intercambian ancho/alto
    if(madctl_ & 0x20) {
      std::swap(lcd_w_, lcd_h_);
      std::swap(x_off_, y_off_);
    }

    // ====== Inicialización HW ======
    if(!dc_.request(gpiochip_c_.c_str(), dc_offset_, 0) ||
       !rst_.request(gpiochip_c_.c_str(), rst_offset_, 1) ||
       (use_manual_cs_ && !cs_.request(gpiochip_c_.c_str(), cs_offset_, 1)) ){
      RCLCPP_FATAL(node.get_logger(), "GPIO reservation failed (chip=%s, DC=%d, RST=%d, CS=%d)",
                   gpiochip_c_.c_str(), dc_offset_, rst_offset_, cs_offset_);
      return false;
    }
    if(!openSpi()){
      RCLCPP_FATAL(node.get_logger(), "SPI open failed for %s", spi_dev_.c_str());
      return false;
    }
    if(!initLcd()){
      RCLCPP_FATAL(node.get_logger(), "ST7735 init failed");
      return false;
    }

    if(self_test_){
      fillColor(rgb565(255,0,0)); msleep(200);
      fillColor(rgb565(0,255,0)); msleep(200);
      fillColor(rgb565(0,0,255)); msleep(200);
      fillColor(rgb565(255,255,255)); msleep(200);
      fillColor(rgb565(0,0,0)); msleep(100);
      drawBorder(rgb565(255,255,255)); msleep(200);
      fillColor(rgb565(0,0,0));
    }
    return true;
  }

  int width()  const override { return lcd_w_; }
  int height() const override { return lcd_h_; }

  void pushMono8(const cv::Mat& mono8) override {
    if(mono8.empty()) return;

    const int eyes_w = mono8.cols;
    const int eyes_h = mono8.rows;
    const int ox = std::max(0, (lcd_w_ - eyes_w) / 2);
    const int oy = std::max(0, (lcd_h_ - eyes_h) / 2);

    std::vector<uint16_t> fr(lcd_w_ * lcd_h_, rgb565(0,0,0));
    if(mono8.type() == CV_8UC1){
      for(int y=0; y<eyes_h && (y+oy)<lcd_h_; ++y){
        const uint8_t* src = mono8.ptr<uint8_t>(y);
        uint16_t* dst = &fr[(y+oy)*lcd_w_ + ox];
        for(int x=0; x<eyes_w && (x+ox)<lcd_w_; ++x){
          dst[x] = (src[x] >= 128) ? 0xFFFF : 0x0000;
        }
      }
    } else if(mono8.type() == CV_8UC3){
      for(int y=0; y<eyes_h && (y+oy)<lcd_h_; ++y){
        const cv::Vec3b* src = mono8.ptr<cv::Vec3b>(y);
        uint16_t* dst = &fr[(y+oy)*lcd_w_ + ox];
        for(int x=0; x<eyes_w && (x+ox)<lcd_w_; ++x){
          const cv::Vec3b& p = src[x];
          dst[x] = rgb565(p[2], p[1], p[0]);
        }
      }
    } else {
      return;
    }
    pushFrameRGB565(fr);
  }

private:
  // ====== SPI/Panel helpers ======
  inline void cs_low()  { if(use_manual_cs_) cs_.set(0); }
  inline void cs_high() { if(use_manual_cs_) cs_.set(1); }

  bool openSpi(){
    fd_ = ::open(spi_dev_.c_str(), O_RDWR);
    if(fd_ < 0){
      RCLCPP_ERROR(node_->get_logger(), "open(%s) errno=%d:%s", spi_dev_.c_str(), errno, strerror(errno));
      return false;
    }
    uint8_t bits = 8;
    uint8_t mode_try = SPI_MODE_0 | (use_manual_cs_ ? SPI_NO_CS : 0);
    if(ioctl(fd_, SPI_IOC_WR_MODE, &mode_try) < 0){
      if(use_manual_cs_){
        RCLCPP_WARN(node_->get_logger(), "SPI_NO_CS not accepted. Falling back to MODE_0.");
        uint8_t mode_fb = SPI_MODE_0;
        if(ioctl(fd_, SPI_IOC_WR_MODE, &mode_fb) < 0){
          RCLCPP_ERROR(node_->get_logger(), "SPI_IOC_WR_MODE fallback errno=%d:%s", errno, strerror(errno));
          return false;
        }
      } else {
        RCLCPP_ERROR(node_->get_logger(), "SPI_IOC_WR_MODE errno=%d:%s", errno, strerror(errno));
        return false;
      }
    }
    if(ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0){
      RCLCPP_ERROR(node_->get_logger(), "SPI_IOC_WR_BITS_PER_WORD errno=%d:%s", errno, strerror(errno));
      return false;
    }
    uint32_t hz = spi_hz_;
    if(ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0){
      RCLCPP_ERROR(node_->get_logger(), "SPI_IOC_WR_MAX_SPEED_HZ errno=%d:%s", errno, strerror(errno));
      return false;
    }
    uint32_t rd_hz=0; uint8_t rd_mode=0, rd_bits=0;
    ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &rd_hz);
    ioctl(fd_, SPI_IOC_RD_MODE, &rd_mode);
    ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &rd_bits);
    RCLCPP_INFO(node_->get_logger(), "SPI OK: mode=0x%02X bits=%u hz=%u", rd_mode, rd_bits, rd_hz);
    return true;
  }

  bool writeCmd(uint8_t c){
    cs_low();
    dc_.set(0);
    ssize_t wr = ::write(fd_, &c, 1);
    cs_high();
    if(wr != 1){
      RCLCPP_ERROR(node_->get_logger(), "CMD 0x%02X write fail (%zd) errno=%d:%s", c, wr, errno, strerror(errno));
      return false;
    }
    return true;
  }

  bool writeDataChunked(const uint8_t* d, size_t n, bool keep_cs=false){
    const size_t CHUNK = spi_chunk_ ? spi_chunk_ : 1024;
    if(!keep_cs) cs_low();
    dc_.set(1);
    size_t sent = 0;
    while(sent < n){
      size_t todo = std::min(CHUNK, n - sent);
      ssize_t wr = ::write(fd_, d + sent, todo);
      if(wr < 0){
        if(!keep_cs) cs_high();
        RCLCPP_ERROR(node_->get_logger(), "DATA write fail (%zu bytes) errno=%d:%s",
                     todo, errno, strerror(errno));
        return false;
      }
      if((size_t)wr != todo){
        if(!keep_cs) cs_high();
        RCLCPP_WARN(node_->get_logger(), "DATA partial: %zd of %zu", wr, todo);
        return false;
      }
      sent += todo;
    }
    if(!keep_cs) cs_high();
    return true;
  }

  bool writeDataByte(uint8_t v){ return writeDataChunked(&v,1); }

  bool initLcd(){
    RCLCPP_INFO(node_->get_logger(),
      "Init ST7735: MADCTL=0x%02X invert=%s off(%d,%d) chunk=%zu CS=%s",
      madctl_, invert_?"ON":"OFF", x_off_, y_off_, spi_chunk_, use_manual_cs_?"MANUAL":"HW");

    rst_.set(0); msleep(20);
    rst_.set(1); msleep(120);

    if(!writeCmd(CMD_SWRESET)) return false; msleep(120);
    if(!writeCmd(CMD_SLPOUT))  return false; msleep(120);

    if(!writeCmd(CMD_COLMOD)) return false;
    if(!writeDataByte(0x05))  return false; msleep(10);

    if(!writeCmd(CMD_MADCTL)) return false;
    if(!writeDataByte(madctl_)) return false; msleep(10);

    if(invert_){ if(!writeCmd(CMD_INVON)) return false; }
    else       { if(!writeCmd(CMD_INVOFF)) return false; }
    msleep(10);

    if(!writeCmd(CMD_NORON)) return false; msleep(10);
    if(!writeCmd(CMD_DISPON)) return false; msleep(100);
    RCLCPP_INFO(node_->get_logger(), "ST7735 DISPON");
    return true;
  }

  void setAddrWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1){
    x0 += x_off_; x1 += x_off_;
    y0 += y_off_; y1 += y_off_;

    cs_low();

    dc_.set(0); uint8_t c = CMD_CASET; ::write(fd_, &c, 1);
    dc_.set(1); uint8_t ca[4] = { uint8_t(x0>>8), uint8_t(x0), uint8_t(x1>>8), uint8_t(x1) }; ::write(fd_, ca, 4);

    dc_.set(0); c = CMD_RASET; ::write(fd_, &c, 1);
    dc_.set(1); uint8_t ra[4] = { uint8_t(y0>>8), uint8_t(y0), uint8_t(y1>>8), uint8_t(y1) }; ::write(fd_, ra, 4);

    dc_.set(0); c = CMD_RAMWR; ::write(fd_, &c, 1);
  }

  static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b){
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  }

  bool fillColor(uint16_t color){
    setAddrWindow(0,0,lcd_w_-1,lcd_h_-1);
    std::vector<uint8_t> line(lcd_w_*2);
    for(int x=0;x<lcd_w_;++x){ line[2*x] = uint8_t(color>>8); line[2*x+1] = uint8_t(color); }
    dc_.set(1);
    for(int y=0;y<lcd_h_;++y){
      if(!writeDataChunked(line.data(), line.size(), /*keep_cs=*/true)){ cs_high(); return false; }
    }
    cs_high();
    return true;
  }

  void drawBorder(uint16_t color){
    auto putpix = [&](int x,int y){
      if(x<0||y<0||x>=lcd_w_||y>=lcd_h_) return;
      setAddrWindow(x,y,x,y);
      dc_.set(1);
      uint8_t p[2] = { uint8_t(color>>8), uint8_t(color) };
      if(!writeDataChunked(p,2, /*keep_cs=*/true)) { cs_high(); return; }
      cs_high();
    };
    for(int x=0;x<lcd_w_;++x){ putpix(x,0); putpix(x,lcd_h_-1); }
    for(int y=0;y<lcd_h_;++y){ putpix(0,y); putpix(lcd_w_-1,y); }
  }

  void pushFrameRGB565(const std::vector<uint16_t>& pix){
    setAddrWindow(0,0,lcd_w_-1,lcd_h_-1);
    std::vector<uint8_t> buf(lcd_w_ * lcd_h_ * 2);
    size_t k = 0;
    for(size_t i=0; i<pix.size(); ++i){
      buf[k++] = uint8_t(pix[i] >> 8);
      buf[k++] = uint8_t(pix[i] & 0xFF);
    }
    if(!writeDataChunked(buf.data(), buf.size())){
      RCLCPP_WARN(node_->get_logger(), "pushFrameRGB565 failed (%zu bytes)", buf.size());
    }
  }

private:
  rclcpp::Node* node_{};

  // Parámetros
  std::string spi_dev_;
  int         spi_hz_{24000000};
  int         lcd_w_{128}, lcd_h_{160};
  int         x_off_{0}, y_off_{0};
  uint8_t     madctl_{0xA0};
  bool        invert_{false};
  bool        self_test_{true};
  size_t      spi_chunk_{2048};

  bool        use_manual_cs_{true};
  std::string gpiochip_c_{"gpiochip0"};
  int         dc_offset_{75}, rst_offset_{78}, cs_offset_{233};

  // Estado HW
  int fd_{-1};
  GpioLine dc_, rst_, cs_;
};

std::unique_ptr<Display> make_display(const std::string& backend){
  if(backend == "sim")     return std::make_unique<DisplaySim>();
  if(backend == "st7735")  return std::make_unique<DisplayST7735>();
  // por defecto, sim para que funcione en PC
  return std::make_unique<DisplaySim>();
}

} // namespace robo_eyes
