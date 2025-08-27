#include "robofer/ui_menu.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>

namespace robo_ui {

namespace {
std::pair<bool, std::string> query_wifi(){
  std::array<char,128> buf{};
  std::string ssid;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes' | cut -d: -f2", "r"), pclose);
  if(pipe && fgets(buf.data(), buf.size(), pipe.get())){
    ssid = buf.data();
    ssid.erase(std::remove(ssid.begin(), ssid.end(), '\n'), ssid.end());
  }
  bool connected = !ssid.empty();
  return {connected, ssid};
}
} // anonymous

MenuController::Item MenuController::build_default_tree(){
  Item root; root.label = "Menu"; root.is_submenu = true;

  Item modos; modos.label = "Modos"; modos.is_submenu = true;
  modos.children.push_back({"Angry", false, MenuAction::SET_ANGRY, {}});
  modos.children.push_back({"Sad",   false, MenuAction::SET_SAD,   {}});
  modos.children.push_back({"Happy", false, MenuAction::SET_HAPPY, {}});

  Item wifi; wifi.label = "Wi-Fi"; wifi.is_submenu = true;
  wifi.children.push_back({"Status: --", false, MenuAction::NONE, {}});
  wifi.children.push_back({"SSID: --",   false, MenuAction::NONE, {}});

  Item apagar; apagar.label = "Apagar"; apagar.is_submenu = false; apagar.action = MenuAction::POWEROFF;

  root.children = {modos, wifi, apagar};
  return root;
}

MenuController::MenuController(std::function<void(MenuAction)> on_action)
  : on_action_(std::move(on_action)) {
  root_ = build_default_tree();
  last_key_time_ = clock::now() - std::chrono::hours(1);
}

void MenuController::set_timeout_ms(int ms){ timeout_ms_ = std::max(0, ms); }
void MenuController::set_font_scale(double s){ font_scale_ = std::clamp(s, 0.1, 2.0); }

bool MenuController::is_active() const {
  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - last_key_time_).count();
  return dt < timeout_ms_;
}

void MenuController::on_key(UiKey key){
  last_key_time_ = clock::now();
  switch(key){
    case UiKey::UP:   up();   break;
    case UiKey::DOWN: down(); break;
    case UiKey::BACK: back(); break;
    case UiKey::OK:   enter(); break;
  }
}

void MenuController::draw(cv::Mat& canvas){
  if(!is_active()) return;
  draw_panel(canvas);
}

const MenuController::Item* MenuController::current_menu() const {
  const Item* cur = &root_;
  for(int idx : path_){
    if(idx < 0 || idx >= (int)cur->children.size()) return &root_;
    cur = &cur->children[idx];
  }
  return cur;
}

MenuController::Item* MenuController::current_menu(){
  Item* cur = &root_;
  for(int idx : path_){
    if(idx < 0 || idx >= (int)cur->children.size()) return &root_;
    cur = &cur->children[idx];
  }
  return cur;
}

void MenuController::up(){
  const Item* menu = current_menu();
  if(menu->children.empty()) return;
  sel_ = (sel_ - 1 + (int)menu->children.size()) % (int)menu->children.size();
}

void MenuController::down(){
  const Item* menu = current_menu();
  if(menu->children.empty()) return;
  sel_ = (sel_ + 1) % (int)menu->children.size();
}

void MenuController::back(){
  if(!path_.empty()){
    path_.pop_back();
    sel_ = 0;
  }
}

void MenuController::enter(){
  Item* menu = current_menu();
  if(menu->children.empty()) return;
  Item& it = menu->children[sel_];
  if(it.is_submenu){
    path_.push_back(sel_);
    sel_ = 0;
  } else {
    dispatch(it.action);
  }
}

void MenuController::dispatch(MenuAction a){
  if(on_action_) on_action_(a);
}

void MenuController::draw_panel(cv::Mat& canvas){
  const int W = canvas.cols, H = canvas.rows;
  Item* menu = current_menu();

  if(menu->label == "Wi-Fi" && menu->children.size() >= 2){
       if(!wifi_requested_){
      wifi_future_ = std::async(std::launch::async, query_wifi);
      wifi_requested_ = true;
    }

    if(wifi_future_.valid()){
      if(wifi_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
        auto [connected, ssid] = wifi_future_.get();
        menu->children[0].label = std::string("Status: ") + (connected ? "Connected" : "Disconnected");
        menu->children[0].color = connected ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
        menu->children[1].label = std::string("SSID: ") + (connected ? ssid : "-");
      } else {
        menu->children[0].label = "Status: ...";
        menu->children[1].label = "SSID: ...";
      }
    }
  } else {
    wifi_requested_ = false;
    wifi_future_ = std::future<std::pair<bool, std::string>>();
  }

  // Determine width based on longest label
  int baseline = 0;
  int max_w = cv::getTextSize(menu->label, cv::FONT_HERSHEY_SIMPLEX,
                              font_scale_, 1, &baseline).width;
  for(const auto& it : menu->children){
    std::string txt = it.label;
    if(it.is_submenu) txt += " >";
    int w = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, font_scale_,
                            1, &baseline).width;
    max_w = std::max(max_w, w);
  }

  const int outer_pad = std::max(4, W/64);
  const int panel_w = std::min(W - 2*outer_pad, std::max(max_w + 16, W*3/5));

  // Determine line height and padding from font size
  cv::Size sample_sz = cv::getTextSize("Ag", cv::FONT_HERSHEY_SIMPLEX,
                                      font_scale_, 1, &baseline);
  const int text_pad = std::max(2, static_cast<int>(std::round(4 * font_scale_)));
  const int line_h = std::max(8, sample_sz.height + baseline + text_pad*2);

  // Header height and total panel height
  cv::Size title_sz = cv::getTextSize(menu->label, cv::FONT_HERSHEY_SIMPLEX,
                                      font_scale_, 1, &baseline);
  const int header_h = title_sz.height + text_pad*2;
  const int panel_h = H;
  const int max_visible = std::max(1, (panel_h - header_h - text_pad) / line_h);  
  const int visible = std::min(max_visible, (int)menu->children.size());

  const int x = (W - panel_w) / 2;
  const int y = 0;
  cv::Mat roi = canvas(cv::Rect(x, y, panel_w, panel_h));
  roi.setTo(cv::Scalar(40,40,40));

  cv::rectangle(canvas, cv::Rect(x, y, panel_w, panel_h),
                cv::Scalar(200,200,200), 1, cv::LINE_8);

  draw_header(canvas, menu->label, x, y, panel_w, text_pad);

  // Update offset to keep selection visible
  if(sel_ < offset_) offset_ = sel_;
  if(sel_ >= offset_ + visible) offset_ = sel_ - visible + 1;
  offset_ = std::clamp(offset_, 0, std::max(0, (int)menu->children.size() - visible));

  draw_items(canvas, menu->children, x, y + header_h + text_pad, panel_w, line_h, offset_, visible, text_pad);
}

void MenuController::draw_header(cv::Mat& img, const std::string& title, int x, int y, int w, int pad){
  int baseline=0;
  cv::Size sz = cv::getTextSize(title, cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
  int tx = x + pad + 2;
  int ty = y + sz.height + pad;
  cv::rectangle(img, cv::Rect(x+1, y+1, w-2, sz.height + pad*2),
                cv::Scalar(200,200,200), cv::FILLED);
  cv::putText(img, title, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
              font_scale_, cv::Scalar(0,0,0), 1, cv::LINE_8);
}

void MenuController::draw_items(cv::Mat& img, const std::vector<Item>& items, int x, int y,
                                int w, int line_h, int start, int max_items, int pad){
  for(int row=0; row<max_items && start+row<(int)items.size(); ++row){
    int i = start + row;
    const auto& it = items[i];
    int row_y = y + row*line_h;
    bool sel = (i==sel_);
    if(sel){
      cv::rectangle(img, cv::Rect(x+2, row_y, w-4, line_h-2),
                    cv::Scalar(200,200,200), cv::FILLED);
    }
    std::string text = it.label;
    if(it.is_submenu) text += " >";
    int baseline=0;
    cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
    int tx = x + pad + 4;
    int ty = row_y + std::min(line_h - pad, sz.height + pad);
    cv::putText(img, text, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
                font_scale_, it.color, 1, cv::LINE_8);
  }
}

} // namespace robo_ui

