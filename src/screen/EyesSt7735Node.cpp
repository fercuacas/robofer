#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <gpiod.h>
#include <cerrno>
#include <cstring>
#include <vector>
#include <algorithm>
#include "robofer/screen/Eyes.hpp"

using robo_eyes::RoboEyes;
using robo_eyes::Mood;

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

// ---------------- libgpiod helpers ----------------
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

// ---------------- Pantalla ST7735 ----------------
struct St7735 {
  rclcpp::Logger log = rclcpp::get_logger("St7735");
  int fd = -1;
  GpioLine dc, rst, cs;        // DC, RESET y (opcional) CS manual
  bool use_manual_cs = true;
  uint16_t w=128, h=160;
  uint8_t  madctl = 0xA0;      // orientación
  uint16_t xofs=0, yofs=0;     // offsets
  bool invert=false;
  size_t spi_chunk = 4096;     // tamaño máx. por write()

  // --- helpers CS ---
  inline void cs_low()  { if(use_manual_cs) cs.set(0); }
  inline void cs_high() { if(use_manual_cs) cs.set(1); }

bool openSpi(const char* dev, uint32_t hz){
  fd = ::open(dev, O_RDWR);
  if(fd<0){
    RCLCPP_ERROR(log, "open(%s) falló: errno=%d (%s)", dev, errno, strerror(errno));
    return false;
  }
  uint8_t bits = 8;

  // 1) intentamos con SPI_NO_CS (ideal), 2) si falla, reintentamos sin él (modo 0 puro)
  uint8_t mode_try = SPI_MODE_0 | (use_manual_cs ? SPI_NO_CS : 0);
  if(ioctl(fd, SPI_IOC_WR_MODE, &mode_try) < 0){
    if(use_manual_cs){
      RCLCPP_WARN(log, "El driver no acepta SPI_NO_CS (EINVAL). Reintentando con SPI_MODE_0.");
      uint8_t mode_fallback = SPI_MODE_0;
      if(ioctl(fd, SPI_IOC_WR_MODE, &mode_fallback) < 0){
        RCLCPP_ERROR(log, "SPI_IOC_WR_MODE (fallback) falló: errno=%d (%s)", errno, strerror(errno));
        return false;
      }
    } else {
      RCLCPP_ERROR(log, "SPI_IOC_WR_MODE falló: errno=%d (%s)", errno, strerror(errno));
      return false;
    }
  }

  if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)<0){
    RCLCPP_ERROR(log, "SPI_IOC_WR_BITS_PER_WORD falló: errno=%d (%s)", errno, strerror(errno));
    return false;
  }
  if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz)<0){
    RCLCPP_ERROR(log, "SPI_IOC_WR_MAX_SPEED_HZ falló: errno=%d (%s)", errno, strerror(errno));
    return false;
  }

  // log de configuración efectiva
  uint32_t rd_hz=0; uint8_t rd_mode=0, rd_bits=0;
  ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd_hz);
  ioctl(fd, SPI_IOC_RD_MODE, &rd_mode);
  ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &rd_bits);
  RCLCPP_INFO(log, "SPI abierto OK: mode=0x%02X, bits=%u, hz=%u", rd_mode, rd_bits, rd_hz);
  return true;
}

  bool writeCmd(uint8_t c){
    // CS bajo para el comando
    cs_low();
    dc.set(0);
    ssize_t wr = ::write(fd, &c, 1);
    cs_high();
    if(wr != 1){
      RCLCPP_ERROR(log, "CMD 0x%02X: write falló (%zd) errno=%d:%s", c, wr, errno, strerror(errno));
      return false;
    }
    return true;
  }

  bool writeDataChunked(const uint8_t* d, size_t n, bool keep_cs=false){
    const size_t CHUNK = spi_chunk ? spi_chunk : 1024;
    if(!keep_cs) cs_low();
    dc.set(1);
    size_t sent = 0;
    while(sent < n){
      size_t todo = std::min(CHUNK, n - sent);
      ssize_t wr = ::write(fd, d + sent, todo);
      if(wr < 0){
        if(!keep_cs) cs_high();
        RCLCPP_ERROR(log, "DATA chunk write falló (%zu bytes) errno=%d:%s", todo, errno, strerror(errno));
        return false;
      }
      if((size_t)wr != todo){
        if(!keep_cs) cs_high();
        RCLCPP_WARN(log, "DATA chunk parcial: %zd de %zu", wr, todo);
        return false;
      }
      sent += todo;
    }
    if(!keep_cs) cs_high();
    return true;
  }

  bool writeDataByte(uint8_t v){ return writeDataChunked(&v,1); }

  bool init(){
    RCLCPP_INFO(log, "Init ST7735: MADCTL=0x%02X, invert=%s, offsets x=%u y=%u, chunk=%zu, CS=%s",
                madctl, invert?"ON":"OFF", xofs, yofs, spi_chunk, use_manual_cs?"MANUAL":"HW");
    // Reset hardware
    rst.set(0); msleep(20);
    rst.set(1); msleep(120);

    if(!writeCmd(CMD_SWRESET)) return false; msleep(120);
    if(!writeCmd(CMD_SLPOUT))  return false; msleep(120);

    if(!writeCmd(CMD_COLMOD)) return false;
    if(!writeDataByte(0x05))  return false; msleep(10);

    if(!writeCmd(CMD_MADCTL)) return false;
    if(!writeDataByte(madctl)) return false; msleep(10);

    if(invert){ if(!writeCmd(CMD_INVON)) return false; }
    else      { if(!writeCmd(CMD_INVOFF)) return false; }
    msleep(10);

    if(!writeCmd(CMD_NORON)) return false; msleep(10);
    if(!writeCmd(CMD_DISPON)) return false; msleep(100);
    RCLCPP_INFO(log, "ST7735 DISPON");
    return true;
  }

  void setAddrWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1){
    x0 += xofs; x1 += xofs;
    y0 += yofs; y1 += yofs;

    // Caso óptimo: mantener CS bajo durante toda la secuencia CASET/RASET/RAMWR
    cs_low();

    dc.set(0); uint8_t c = CMD_CASET; ::write(fd, &c, 1);
    dc.set(1); uint8_t ca[4] = { uint8_t(x0>>8), uint8_t(x0), uint8_t(x1>>8), uint8_t(x1) }; ::write(fd, ca, 4);

    dc.set(0); c = CMD_RASET; ::write(fd, &c, 1);
    dc.set(1); uint8_t ra[4] = { uint8_t(y0>>8), uint8_t(y0), uint8_t(y1>>8), uint8_t(y1) }; ::write(fd, ra, 4);

    dc.set(0); c = CMD_RAMWR; ::write(fd, &c, 1);

    // ¡IMPORTANTE! NO subir CS aquí; lo subimos tras enviar los datos del frame/linea
    // (Quien llame decide cuándo cs_high()).
  }

  static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b){
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  }

  bool fillColor(uint16_t color){
    setAddrWindow(0,0,w-1,h-1);
    // enviamos el frame completo con CS mantenido BAJO
    std::vector<uint8_t> line(w*2);
    for(int x=0;x<w;++x){ line[2*x] = uint8_t(color>>8); line[2*x+1] = uint8_t(color); }
    dc.set(1);
    for(int y=0;y<h;++y){
      if(!writeDataChunked(line.data(), line.size(), /*keep_cs=*/true)){ cs_high(); return false; }
    }
    cs_high();
    return true;
  }

  void drawBorder(uint16_t color){
    auto putpix = [&](int x,int y){
      if(x<0||y<0||x>=w||y>=h) return;
      setAddrWindow(x,y,x,y);
      dc.set(1);
      uint8_t p[2] = { uint8_t(color>>8), uint8_t(color) };
      // mantener CS durante este pixel
      if(!writeDataChunked(p,2, /*keep_cs=*/true)) { cs_high(); return; }
      cs_high();
    };
    for(int x=0;x<w;++x){ putpix(x,0); putpix(x,h-1); }
    for(int y=0;y<h;++y){ putpix(0,y); putpix(w-1,y); }
  }

  void pushFrameRGB565(const std::vector<uint16_t>& pix){
    // Ventana completa
    setAddrWindow(0,0,w-1,h-1);

    // Convertimos TODO el frame a bytes grandes
    std::vector<uint8_t> buf(w * h * 2);
    size_t k = 0;
    for(size_t i=0; i<pix.size(); ++i){
      buf[k++] = uint8_t(pix[i] >> 8);
      buf[k++] = uint8_t(pix[i] & 0xFF);
    }

    // ¡Una sola transferencia “larga”!
    if(!writeDataChunked(buf.data(), buf.size())){
      RCLCPP_WARN(log, "pushFrameRGB565: fallo al enviar frame completo (%zu bytes)", buf.size());
    }
  }
};

inline uint16_t mono_to_rgb565(uint8_t v){
  return (v>=128) ? 0xFFFF : 0x0000;
}
} // anon

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robo_eyes_st7735");
  auto log = node->get_logger();

  // ---- Parámetros ----
  std::string spi_dev = node->declare_parameter<std::string>("spi_device", "/dev/spidev1.0");
  int spi_hz = node->declare_parameter<int>("spi_hz", 500000);
  int lcd_w = node->declare_parameter<int>("lcd_width", 128);
  int lcd_h = node->declare_parameter<int>("lcd_height", 160);
  int x_off = node->declare_parameter<int>("x_offset", 0);
  int y_off = node->declare_parameter<int>("y_offset", 0);
  int mad   = node->declare_parameter<int>("madctl", 0xA0);
  bool invert = node->declare_parameter<bool>("invert", false);
  bool self_test = node->declare_parameter<bool>("self_test", true);
  int spi_chunk = node->declare_parameter<int>("spi_chunk", 2048);

  bool use_manual_cs = node->declare_parameter<bool>("use_manual_cs", true);
  std::string gpiochip_c = node->declare_parameter<std::string>("gpiochip_c", "gpiochip0");
  int dc_offset  = node->declare_parameter<int>("dc_offset", 75);   // PC11 -> pin 12
  int rst_offset = node->declare_parameter<int>("rst_offset", 78);  // PC14 -> pin 18
  int cs_offset  = node->declare_parameter<int>("cs_offset", 233);  // PH9  -> pin 24 (CS manual)

  if(mad & 0x20) {
    std::swap(lcd_w, lcd_h);
    std::swap(x_off, y_off);
  }

  RCLCPP_INFO(log, "Params: spi=%s hz=%d lcd=%dx%d off(%d,%d) madctl=0x%02X invert=%s chunk=%d CS=%s dc=%d rst=%d cs=%d chip=%s",
              spi_dev.c_str(), spi_hz, lcd_w, lcd_h, x_off, y_off, mad, invert?"ON":"OFF",
              spi_chunk, use_manual_cs?"MANUAL":"HW", dc_offset, rst_offset, cs_offset, gpiochip_c.c_str());

  // ---- Inicializa LCD ----
  St7735 lcd;
  lcd.use_manual_cs = use_manual_cs;
  lcd.w = (uint16_t)lcd_w; lcd.h=(uint16_t)lcd_h;
  lcd.xofs = (uint16_t)x_off; lcd.yofs=(uint16_t)y_off;
  lcd.madctl = uint8_t(mad); lcd.invert = invert;
  lcd.spi_chunk = (spi_chunk>0)? (size_t)spi_chunk : 1024;

  if(!lcd.dc.request(gpiochip_c.c_str(), dc_offset, 0) ||
     !lcd.rst.request(gpiochip_c.c_str(), rst_offset, 1) ||
     (use_manual_cs && !lcd.cs.request(gpiochip_c.c_str(), cs_offset, 1)) ){
    RCLCPP_FATAL(log, "No pude reservar GPIO (chip=%s, DC=%d, RST=%d, CS=%d).",
                 gpiochip_c.c_str(), dc_offset, rst_offset, cs_offset);
    return 1;
  }
  if(!lcd.openSpi(spi_dev.c_str(), (uint32_t)spi_hz)){
    RCLCPP_FATAL(log, "No pude abrir %s", spi_dev.c_str());
    return 1;
  }
  if(!lcd.init()){
    RCLCPP_FATAL(log, "Falló init ST7735");
    return 1;
  }

  // ---- SELF TEST ----
  if(self_test){
    RCLCPP_INFO(log, "Self-test: ROJO");
    lcd.fillColor(St7735::rgb565(255,0,0)); msleep(400);
    RCLCPP_INFO(log, "Self-test: VERDE");
    lcd.fillColor(St7735::rgb565(0,255,0)); msleep(400);
    RCLCPP_INFO(log, "Self-test: AZUL");
    lcd.fillColor(St7735::rgb565(0,0,255)); msleep(400);
    RCLCPP_INFO(log, "Self-test: BLANCO");
    lcd.fillColor(St7735::rgb565(255,255,255)); msleep(400);
    RCLCPP_INFO(log, "Self-test: NEGRO + borde");
    lcd.fillColor(St7735::rgb565(0,0,0));
    lcd.drawBorder(St7735::rgb565(255,255,255)); msleep(400);
  }

  // ---- Ojos ----
  const int eyes_w = 128, eyes_h = 64, fps=30;
  RoboEyes eyes;
  eyes.begin(eyes_w, eyes_h, fps);
  eyes.setIdle(true);
  eyes.setAutoblinker(true);
  eyes.setCurious(false);
  eyes.setCyclops(false);
  eyes.setMood(Mood::DEFAULT);

  std::vector<uint16_t> framebuffer(lcd.w * lcd.h, 0x0000);
  rclcpp::Rate rate(fps);
  RCLCPP_INFO(log, "Render loop…");

  while(rclcpp::ok()){
    rclcpp::spin_some(node);
    eyes.update();

    const cv::Mat& m = eyes.frame(); // 8UC1
    int ox = (lcd.w - eyes_w)/2; if(ox<0) ox=0;
    int oy = (lcd.h - eyes_h)/2; if(oy<0) oy=0;

    std::fill(framebuffer.begin(), framebuffer.end(), 0x0000);
    for(int y=0; y<eyes_h && (y+oy)<lcd.h; ++y){
      const uint8_t* src = m.ptr<uint8_t>(y);
      uint16_t* dst = &framebuffer[(y+oy)*lcd.w + ox];
      for(int x=0; x<eyes_w && (x+ox)<lcd.w; ++x){
        dst[x] = (src[x] >= 128) ? 0xFFFF : 0x0000;
      }
    }
    lcd.pushFrameRGB565(framebuffer);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
