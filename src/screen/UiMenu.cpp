#include "robofer/screen/UiMenu.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace robo_ui {


MenuController::Item MenuController::buildDefaultTree(){
  Item root; root.label = "Menu"; root.is_submenu = true;

  Item modos; modos.label = "Modos"; modos.is_submenu = true;
  modos.children.push_back({"Angry", false, MenuAction::SET_ANGRY, {}});
  modos.children.push_back({"Sad",   false, MenuAction::SET_SAD,   {}});
  modos.children.push_back({"Happy", false, MenuAction::SET_HAPPY, {}});

  Item wifi; wifi.label = "Wi-Fi"; wifi.is_submenu = true;
  wifi.children.push_back({"Status: --", false, MenuAction::NONE, {}});
  wifi.children.push_back({"SSID: --",   false, MenuAction::NONE, {}});
  wifi.children.push_back({"Start provisioning", false, MenuAction::BT_CONNECT, {}});
  wifi.children.push_back({"Accept", false, MenuAction::BT_ACCEPT, {}});
  wifi.children.push_back({"Reject", false, MenuAction::BT_REJECT, {}});
  wifi.children.push_back({"Stop provisioning", false, MenuAction::BT_STOP, {}});

  Item bt; bt.label = "Bluetooth"; bt.is_submenu = true;
  bt.children.push_back({"Status: Off", false, MenuAction::NONE, {}});
  bt.children.push_back({"Enable Bluetooth", false, MenuAction::BT_ON, {}});
  bt.children.push_back({"", false, MenuAction::NONE, {}});
  bt.children.push_back({"", false, MenuAction::NONE, {}});
  bt.children.push_back({"", false, MenuAction::NONE, {}});

  Item music; music.label = "Music"; music.is_submenu = true;
  music.children.push_back({"None", false, MenuAction::NONE, {}});

  Item apagar; apagar.label = "Apagar"; apagar.is_submenu = false; apagar.action = MenuAction::POWEROFF;

  root.children = {modos, wifi, bt, music, apagar};
  return root;
}

MenuController::MenuController(std::function<void(MenuAction)> on_action)
  : on_action_(std::move(on_action)) {
  root_ = buildDefaultTree();
  last_key_time_ = clock::now() - std::chrono::hours(1);
}

void MenuController::setTimeoutMs(int ms){ timeout_ms_ = std::max(0, ms); }
void MenuController::setFontScale(double s){ font_scale_ = std::clamp(s, 0.1, 2.0); }

void MenuController::setWifiStatus(bool connected, const std::string& ssid){
  if(root_.children.size() > 1){
    Item& wifi = root_.children[1];
    if(wifi.label == "Wi-Fi" && wifi.children.size() >= 2){
      wifi.children[0].label = std::string("Status: ") + (connected ? "Connected" : "Disconnected");
      wifi.children[0].color = connected ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
      wifi.children[1].label = std::string("SSID: ") + (connected ? ssid : "-");
    }
  }
}

void MenuController::setBtState(const std::string& state, uint32_t code){
  if(root_.children.size() > 1){
    Item& wifi = root_.children[1];
    if(wifi.label == "Wi-Fi" && wifi.children.size() >= 6){
      // item2: start/state label
      if(state == "IDLE"){
        wifi.children[2].label = "Start provisioning";
        wifi.children[2].action = MenuAction::BT_CONNECT;
      } else {
        wifi.children[2].label = std::string("State: ") + state;
        wifi.children[2].action = MenuAction::NONE;
      }
      // items for confirm
      if(state.rfind("CONFIRM_CODE",0) == 0){
        wifi.children[3].label = std::string("Accept ") + std::to_string(code);
        wifi.children[3].action = MenuAction::BT_ACCEPT;
        wifi.children[4].label = "Reject";
        wifi.children[4].action = MenuAction::BT_REJECT;
      } else {
        wifi.children[3].label = "";
        wifi.children[3].action = MenuAction::NONE;
        wifi.children[4].label = "";
        wifi.children[4].action = MenuAction::NONE;
      }
      // item5: stop provisioning if not idle
      if(state == "IDLE"){
        wifi.children[5].label = "";
        wifi.children[5].action = MenuAction::NONE;
      } else {
        wifi.children[5].label = "Stop provisioning";
        wifi.children[5].action = MenuAction::BT_STOP;
      }
    }
  }
}

void MenuController::setBluetoothState(bool enabled, const std::string& device){
  if(root_.children.size() > 2){
    Item& bt = root_.children[2];
    if(bt.label == "Bluetooth" && bt.children.size() >= 5){
      bt.children[0].label = std::string("Status: ") + (enabled ? "On" : "Off");
      bt.children[0].color = enabled ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
      bt.children[1].label = enabled ? "Disable Bluetooth" : "Enable Bluetooth";
      bt.children[1].action = enabled ? MenuAction::BT_OFF : MenuAction::BT_ON;
      if(device.empty()){
        bt.children[2].label = "";
        bt.children[2].action = MenuAction::NONE;
        bt.children[3].label = "";
        bt.children[3].action = MenuAction::NONE;
        bt.children[4].label = "";
        bt.children[4].action = MenuAction::NONE;
      } else {
        bt.children[2].label = std::string("PIN: ") + device;
        bt.children[2].action = MenuAction::NONE;
        bt.children[3].label = "Accept";
        bt.children[3].action = MenuAction::BT_PAIR_ACCEPT;
        bt.children[4].label = "Reject";
        bt.children[4].action = MenuAction::BT_PAIR_REJECT;
      }
    }
  }
}

void MenuController::setMusicFiles(const std::vector<std::string>& files){
  for(auto& it : root_.children){
    if(it.label != "Music") continue;
    it.children.clear();
    if(files.empty()){
      it.children.push_back({"None", false, MenuAction::NONE, {}});
    } else {
      for(const auto& f : files){
        it.children.push_back({f, false, MenuAction::NONE, {}});
      }
    }
    break;
  }
}

bool MenuController::isActive() const {
  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - last_key_time_).count();
  return dt < timeout_ms_;
}

void MenuController::onKey(UiKey key){
  last_key_time_ = clock::now();
  switch(key){
    case UiKey::UP:   up();   break;
    case UiKey::DOWN: down(); break;
    case UiKey::BACK: back(); break;
    case UiKey::OK:   enter(); break;
  }
}

void MenuController::draw(cv::Mat& canvas){
  if(!isActive()) return;
  drawPanel(canvas);
}

const MenuController::Item* MenuController::currentMenu() const {
  const Item* cur = &root_;
  for(int idx : path_){
    if(idx < 0 || idx >= (int)cur->children.size()) return &root_;
    cur = &cur->children[idx];
  }
  return cur;
}

MenuController::Item* MenuController::currentMenu(){
  Item* cur = &root_;
  for(int idx : path_){
    if(idx < 0 || idx >= (int)cur->children.size()) return &root_;
    cur = &cur->children[idx];
  }
  return cur;
}

void MenuController::up(){
  const Item* menu = currentMenu();
  if(menu->children.empty()) return;
  sel_ = (sel_ - 1 + (int)menu->children.size()) % (int)menu->children.size();
}

void MenuController::down(){
  const Item* menu = currentMenu();
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
  Item* menu = currentMenu();
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

void MenuController::drawPanel(cv::Mat& canvas){
  const int W = canvas.cols, H = canvas.rows;
  Item* menu = currentMenu();


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

  drawHeader(canvas, menu->label, x, y, panel_w, text_pad);

  // Update offset to keep selection visible
  if(sel_ < offset_) offset_ = sel_;
  if(sel_ >= offset_ + visible) offset_ = sel_ - visible + 1;
  offset_ = std::clamp(offset_, 0, std::max(0, (int)menu->children.size() - visible));

  drawItems(canvas, menu->children, x, y + header_h + text_pad, panel_w, line_h, offset_, visible, text_pad);
}

void MenuController::drawHeader(cv::Mat& img, const std::string& title, int x, int y, int w, int pad){
  int baseline=0;
  cv::Size sz = cv::getTextSize(title, cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, &baseline);
  int tx = x + pad + 2;
  int ty = y + sz.height + pad;
  cv::rectangle(img, cv::Rect(x+1, y+1, w-2, sz.height + pad*2),
                cv::Scalar(200,200,200), cv::FILLED);
  cv::putText(img, title, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
              font_scale_, cv::Scalar(0,0,0), 1, cv::LINE_8);
}

void MenuController::drawItems(cv::Mat& img, const std::vector<Item>& items, int x, int y,
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

