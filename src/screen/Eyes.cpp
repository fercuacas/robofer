#include "robofer/screen/Eyes.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

using namespace robo_eyes;

static inline int clampi(int v, int lo, int hi){ return std::max(lo, std::min(v, hi)); }

RoboEyes::RoboEyes(){
  std::random_device rd; rng_ = std::mt19937(rd());
}

uint64_t RoboEyes::now_ms(){
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

int RoboEyes::rnd(int max_inclusive){
  if(max_inclusive <= 0) return 0;
  std::uniform_int_distribution<int> d(0, max_inclusive);
  return d(rng_);
}

void RoboEyes::begin(int w, int h, int fps){
  screen_w_ = w; screen_h_ = h; setFramerate(fps);
  canvas_.create(h, w, CV_8UC1);

  const double sx = w / 128.0, sy = h / 64.0;
  eyeL_w_def_ = eyeR_w_def_ = std::lround(36 * sx);
  eyeL_h_def_ = eyeR_h_def_ = std::lround(36 * sy);
  eyeL_w_cur_ = eyeR_w_cur_ = eyeL_w_def_;
  eyeL_h_cur_ = eyeR_h_cur_ = 1; // closed start
  eyeL_w_next_ = eyeR_w_next_ = eyeL_w_def_;
  eyeL_h_next_ = eyeR_h_next_ = eyeL_h_def_;
  eyeL_r_def_ = eyeR_r_def_ = eyeL_r_cur_ = eyeR_r_cur_ = eyeL_r_next_ = eyeR_r_next_ = std::lround(8 * ((sx+sy)*0.5));

  spaceBetweenDefault_ = spaceBetweenCurrent_ = spaceBetweenNext_ = std::lround(10 * sx);

  eyeLx_def_ = (screen_w_ - (eyeL_w_def_ + spaceBetweenDefault_ + eyeR_w_def_)) / 2;
  eyeLy_def_ = (screen_h_ - eyeL_h_def_) / 2;
  eyeLx_ = eyeLx_next_ = eyeLx_def_;
  eyeLy_ = eyeLy_next_ = eyeLy_def_;

  eyeRx_def_ = eyeLx_ + eyeL_w_def_ + spaceBetweenDefault_;
  eyeRy_def_ = eyeLy_;
  eyeRx_ = eyeRx_next_ = eyeRx_def_;
  eyeRy_ = eyeRy_next_ = eyeRy_def_;

  eyelidsHeightMax_ = eyeL_h_def_/2;
  eyelidsHappyBottomOffset_ = eyelidsHappyBottomOffset_next_ = 0;

  eyeL_open_ = eyeR_open_ = false;

  // Timers como en Arduino: programa el siguiente evento con variación
  const uint64_t now = now_ms();
  blinkTimer_ = now + blinkInterval_s_*1000 + rnd(std::max(0, blinkVar_s_))*1000;
  idleTimer_  = now + idleInterval_s_*1000  + rnd(std::max(0, idleVar_s_))*1000;

  fps_timer_ms_ = now;
}

void RoboEyes::setFramerate(int fps){ frame_interval_ms_ = std::max(1, 1000/std::max(1,fps)); }

void RoboEyes::setSize(int lw, int lh, int rw, int rh){
  eyeL_w_next_=lw; eyeL_h_next_=lh; eyeL_w_def_=lw; eyeL_h_def_=lh;
  eyeR_w_next_=rw; eyeR_h_next_=rh; eyeR_w_def_=rw; eyeR_h_def_=rh;
}

void RoboEyes::setBorderRadius(int l, int r){ eyeL_r_next_=eyeL_r_def_=l; eyeR_r_next_=eyeR_r_def_=r; }

void RoboEyes::setSpaceBetween(int px){ spaceBetweenNext_=spaceBetweenDefault_=px; }

void RoboEyes::setMood(Mood m){ tired_=angry_=happy_=frown_=false; if(m==TIRED) tired_=true; else if(m==ANGRY) angry_=true; else if(m==HAPPY) happy_=true; else if(m==FROWN) frown_=true;}

void RoboEyes::setPosition(Pos p){
  switch(p){
    case N:  eyeLx_next_=screenConstraintX()/2; eyeLy_next_=0; break;
    case NE: eyeLx_next_=screenConstraintX();   eyeLy_next_=0; break;
    case E:  eyeLx_next_=screenConstraintX();   eyeLy_next_=screenConstraintY()/2; break;
    case SE: eyeLx_next_=screenConstraintX();   eyeLy_next_=screenConstraintY(); break;
    case S:  eyeLx_next_=screenConstraintX()/2; eyeLy_next_=screenConstraintY(); break;
    case SW: eyeLx_next_=0;                     eyeLy_next_=screenConstraintY(); break;
    case W:  eyeLx_next_=0;                     eyeLy_next_=screenConstraintY()/2; break;
    case NW: eyeLx_next_=0;                     eyeLy_next_=0; break;
    case CENTER: default: eyeLx_next_=screenConstraintX()/2; eyeLy_next_=screenConstraintY()/2; break;
  }
}

void RoboEyes::setAutoblinker(bool on, int s, int var){ autoblinker_=on; blinkInterval_s_=s; blinkVar_s_=var; }
void RoboEyes::setIdle(bool on, int s, int var){ idle_=on; idleInterval_s_=s; idleVar_s_=var; }

void RoboEyes::closeBoth(){ eyeL_h_next_=1; eyeR_h_next_=1; eyeL_open_=false; eyeR_open_=false; }
void RoboEyes::openBoth(){ eyeL_open_=true; eyeR_open_=true; }
void RoboEyes::blinkBoth(){ closeBoth(); openBoth(); }

void RoboEyes::anim_confused(){ confused_=true; }
void RoboEyes::anim_laugh(){ laugh_=true; }

int RoboEyes::screenConstraintX() const { return screen_w_ - eyeL_w_cur_ - spaceBetweenCurrent_ - eyeR_w_cur_; }
int RoboEyes::screenConstraintY() const { return screen_h_ - std::max(eyeL_h_def_, eyeR_h_def_); }

void RoboEyes::clear(cv::Mat& img, int gray){ img.setTo(cv::Scalar(gray)); }

void RoboEyes::fillRect(cv::Mat& img, int x, int y, int w, int h, int gray){
  if(w<=0||h<=0) return; cv::rectangle(img, cv::Rect(x,y,w,h), cv::Scalar(gray), cv::FILLED, cv::LINE_8);
}

void RoboEyes::fillCircle(cv::Mat& img, int cx, int cy, int r, int gray){ if(r>0) cv::circle(img, {cx,cy}, r, cv::Scalar(gray), cv::FILLED, cv::LINE_8); }

void RoboEyes::fillRoundRect(cv::Mat& img, int x, int y, int w, int h, int r, int gray){
  if(w<=0||h<=0) return;
  r = clampi(r, 0, std::min(w,h)/2);
  if(r==0){ fillRect(img,x,y,w,h,gray); return; }
  // center + sides
  fillRect(img, x+r, y, w-2*r, h, gray);
  fillRect(img, x, y+r, r, h-2*r, gray);
  fillRect(img, x+w-r, y+r, r, h-2*r, gray);
  // corners
  fillCircle(img, x+r, y+r, r, gray);
  fillCircle(img, x+w-r-1, y+r, r, gray);
  fillCircle(img, x+r, y+h-r-1, r, gray);
  fillCircle(img, x+w-r-1, y+h-r-1, r, gray);
}

void RoboEyes::fillTriangle(cv::Mat& img, cv::Point a, cv::Point b, cv::Point c, int gray){
  std::vector<cv::Point> pts{a,b,c};
  cv::fillConvexPoly(img, pts, cv::Scalar(gray), cv::LINE_8);
}

// Línea sólida con “round caps”
static void draw_rounded_line(cv::Mat& img, cv::Point a, cv::Point b, int thick, int gray){
  cv::line(img, a, b, cv::Scalar(gray), thick, cv::LINE_8);
  int r = std::max(1, (thick+1)/2);
  cv::circle(img, a, r, cv::Scalar(gray), cv::FILLED, cv::LINE_8);
  cv::circle(img, b, r, cv::Scalar(gray), cv::FILLED, cv::LINE_8);
}

// Chevron ">" o "<" con vértice redondeado
// dir = +1 => ">" ; dir = -1 => "<"
static void draw_chevron(cv::Mat& img, cv::Rect roi, int dir, int thick, int gray){
  // MARGEN: sube/baja para variar “apertura” del chevron (menos margen = más pico)
  int m = std::max(1, std::min(roi.width, roi.height) / 5);

  int x0 = roi.x + m, x1 = roi.x + roi.width - m;
  int y0 = roi.y + m, y1 = roi.y + roi.height - m;
  int yc = roi.y + roi.height/2;

  if(dir > 0){
    // ">"
    cv::Point A(x0, y0), B(x1, yc), C(x0, y1);
    draw_rounded_line(img, A, B, thick, gray);
    draw_rounded_line(img, C, B, thick, gray);
    // vértice redondeado
    cv::circle(img, B, std::max(1, (thick+1)/2), cv::Scalar(gray), cv::FILLED, cv::LINE_8);
  } else {
    // "<"
    cv::Point A(x1, y0), B(x0, yc), C(x1, y1);
    draw_rounded_line(img, A, B, thick, gray);
    draw_rounded_line(img, C, B, thick, gray);
    cv::circle(img, B, std::max(1, (thick+1)/2), cv::Scalar(gray), cv::FILLED, cv::LINE_8);
  }
}

void RoboEyes::update(){
  const uint64_t t = now_ms();
  if(t - fps_timer_ms_ < (uint64_t)frame_interval_ms_) return; // limit FPS
  drawEyes();
  fps_timer_ms_ = t;
}

void RoboEyes::setWidth(int leftEye, int rightEye){ 
  setSize(leftEye, eyeL_h_def_, rightEye, eyeR_h_def_);
}
void RoboEyes::setHeight(int leftEye, int rightEye){
  setSize(eyeL_w_def_, leftEye, eyeR_w_def_, rightEye);
}
void RoboEyes::setBorderradius(int leftEye, int rightEye){
  setBorderRadius(leftEye, rightEye);
}
void RoboEyes::setSpacebetween(int space){
  setSpaceBetween(space);
}

// --- Overloads cómodos ---
void RoboEyes::setAutoblinker(bool active){
  setAutoblinker(active, blinkInterval_s_, blinkVar_s_);
}
void RoboEyes::setIdle(bool active){
  setIdle(active, idleInterval_s_, idleVar_s_);
}

// --- Control por ojo ---
void RoboEyes::close(){ closeBoth(); }
void RoboEyes::open(){ openBoth(); }
void RoboEyes::blink(){ blinkBoth(); }

void RoboEyes::close(bool left, bool right){
  if(left) { eyeL_h_next_ = 1; eyeL_open_ = false; }
  if(right){ eyeR_h_next_ = 1; eyeR_open_ = false; }
}
void RoboEyes::open(bool left, bool right){
  if(left)  eyeL_open_ = true;
  if(right) eyeR_open_ = true;
}
void RoboEyes::blink(bool left, bool right){
  close(left, right);
  open(left, right);
}

// --- Flickers directos ---
void RoboEyes::setHFlicker(bool flickerBit, int amplitude){
  hFlicker_ = flickerBit;
  if(amplitude >= 0) hFlickAmp_ = amplitude;
}
void RoboEyes::setHFlicker(bool flickerBit){
  hFlicker_ = flickerBit;
}

void RoboEyes::setVFlicker(bool flickerBit, int amplitude){
  vFlicker_ = flickerBit;
  if(amplitude >= 0) vFlickAmp_ = amplitude;
}
void RoboEyes::setVFlicker(bool flickerBit){
  vFlicker_ = flickerBit;
}

// --- Getters con nombres originales ---
int RoboEyes::getScreenConstraint_X() const { return screenConstraintX(); }
int RoboEyes::getScreenConstraint_Y() const { return screenConstraintY(); }

void RoboEyes::drawEyes(){
  // curious vertical offset/grow
  if(curious_){
    if(eyeLx_next_ <= 10){ eyeL_h_offset_ = std::lround(8 * (screen_h_/64.0)); } else if(eyeLx_next_ >= (screenConstraintX()-10) && cyclops_){ eyeL_h_offset_ = std::lround(8 * (screen_h_/64.0)); } else { eyeL_h_offset_=0; }
    if(eyeRx_next_ >= screen_w_ - eyeR_w_cur_ - 10){ eyeR_h_offset_ = std::lround(8 * (screen_h_/64.0)); } else { eyeR_h_offset_ = 0; }
  } else { eyeL_h_offset_=eyeR_h_offset_=0; }

  // heights with tweening + vertical centering
  eyeL_h_cur_ = (eyeL_h_cur_ + eyeL_h_next_ + eyeL_h_offset_) / 2;
  eyeLy_ += ((eyeL_h_def_ - eyeL_h_cur_)/2); eyeLy_ -= eyeL_h_offset_/2;
  eyeR_h_cur_ = (eyeR_h_cur_ + eyeR_h_next_ + eyeR_h_offset_) / 2;
  eyeRy_ += ((eyeR_h_def_ - eyeR_h_cur_)/2); eyeRy_ -= eyeR_h_offset_/2;

  if(eyeL_open_ && eyeL_h_cur_ <= 1 + eyeL_h_offset_) eyeL_h_next_ = eyeL_h_def_;
  if(eyeR_open_ && eyeR_h_cur_ <= 1 + eyeR_h_offset_) eyeR_h_next_ = eyeR_h_def_;

  // widths + space tweening
  eyeL_w_cur_ = (eyeL_w_cur_ + eyeL_w_next_) / 2;
  eyeR_w_cur_ = (eyeR_w_cur_ + eyeR_w_next_) / 2;
  spaceBetweenCurrent_ = (spaceBetweenCurrent_ + spaceBetweenNext_) / 2;

  // positions tweening
  eyeLx_ = (eyeLx_ + eyeLx_next_) / 2;
  eyeLy_ = (eyeLy_ + eyeLy_next_) / 2;

  eyeRx_next_ = eyeLx_next_ + eyeL_w_cur_ + spaceBetweenCurrent_;
  eyeRy_next_ = eyeLy_next_;
  eyeRx_ = (eyeRx_ + eyeRx_next_) / 2; eyeRy_ = (eyeRy_ + eyeRy_next_) / 2;

  // border radius tweening
  eyeL_r_cur_ = (eyeL_r_cur_ + eyeL_r_next_) / 2;
  eyeR_r_cur_ = (eyeR_r_cur_ + eyeR_r_next_) / 2;

  // macros
  const uint64_t t = now_ms();

  if(autoblinker_){
    if(t >= blinkTimer_){
      blinkBoth();
      blinkTimer_ = t + blinkInterval_s_*1000 + rnd(std::max(0, blinkVar_s_))*1000;
    }
  }

  if(laugh_){
    if(laughToggle_){ vFlicker_=true; vFlickAmp_=5; laughTimer_=t; laughToggle_=false; }
    else if(t >= laughTimer_ + laughDur_ms_){ vFlicker_=false; laughToggle_=true; laugh_=false; }
  }

  if(confused_){
    if(confusedToggle_){ hFlicker_=true; hFlickAmp_=20; confusedTimer_=t; confusedToggle_=false; }
    else if(t >= confusedTimer_ + confusedDur_ms_){ hFlicker_=false; confusedToggle_=true; confused_=false; }
  }

  if(idle_ && !frown_){
    if(t >= idleTimer_){
      eyeLx_next_ = rnd(std::max(0, screenConstraintX()));
      eyeLy_next_ = rnd(std::max(0, screenConstraintY()));
      idleTimer_ = t + idleInterval_s_*1000 + rnd(std::max(0, idleVar_s_))*1000;
    }
  }

  if(hFlicker_){ 
    int off = hFlickerAlt_? hFlickAmp_ : -hFlickAmp_; 
    eyeLx_ += off; eyeRx_ += off; 
    hFlickerAlt_ = !hFlickerAlt_; 
  }
  int draw_off_x = 0;
  if (frown_) {
    int amp = hFlickAmp_; // amplitud del temblor
    uint64_t now = now_ms();

    // Solo alternar si ha pasado el tiempo deseado
    if (now - frownFlickerLastChange_ >= (uint64_t)frownFlickerIntervalMs_) {
        hFlickerAlt_ = !hFlickerAlt_;
        frownFlickerLastChange_ = now;
    }

    draw_off_x = (hFlickerAlt_ ? amp : -amp);
  } else if (hFlicker_) {
    int amp = hFlickAmp_;
    draw_off_x = (hFlickerAlt_ ? amp : -amp);
    hFlickerAlt_ = !hFlickerAlt_;
  }
  if(vFlicker_){ int off = vFlickerAlt_? vFlickAmp_ : -vFlickAmp_; eyeLy_ += off; eyeRy_ += off; vFlickerAlt_ = !vFlickerAlt_; }

  if(cyclops_){ eyeR_w_cur_=0; eyeR_h_cur_=0; spaceBetweenCurrent_=0; }

  // Draw
  clear(canvas_, BGCOLOR);
  // if (frown_) {

  //   eyeLx_next_ = screenConstraintX() / 2;
  //   eyeLy_next_ = screenConstraintY() / 2;
  //   spaceBetweenNext_ = spaceBetweenDefault_;
  //   // compacta un poco para gesto de fuerza
  //   const int Lw = eyeL_w_cur_, Lh = eyeL_h_cur_;
  //   const int Rw = eyeR_w_cur_, Rh = eyeR_h_cur_;

  //   // grosor proporcional (ajusta si los quieres más finos/gruesos)
  //   int thickL = std::max(2, std::min(Lw, Lh) / 3);
  //   int thickR = std::max(2, std::min(Rw, Rh) / 3);

  //   // rectángulos de cada ojo
  //   cv::Rect roiL(eyeLx_, eyeLy_, Lw, Lh);
  //   draw_chevron(canvas_, roiL, +1, thickL, MAINCOLOR); // ">" a la derecha (apunta al centro)

  //   if(!cyclops_ && Rw>0 && Rh>0){
  //     cv::Rect roiR(eyeRx_, eyeRy_, Rw, Rh);
  //     draw_chevron(canvas_, roiR, -1, thickR, MAINCOLOR); // "<" a la izquierda
  //   }

  //   // En este modo no dibujamos párpados ni máscaras
  //   return;
  // }
  if (frown_) {
  const int Lw = eyeL_w_cur_, Lh = eyeL_h_cur_;
  const int Rw = eyeR_w_cur_, Rh = eyeR_h_cur_;

  int thickL = std::max(2, std::min(Lw, Lh) / 4);
  int thickR = std::max(2, std::min(Rw, Rh) / 4);

  // aplicar offset horizontal SOLO al dibujo
  cv::Rect roiL(eyeLx_ + draw_off_x, eyeLy_, Lw, Lh);
  draw_chevron(canvas_, roiL, +1, thickL, MAINCOLOR);

  if(!cyclops_ && Rw>0 && Rh>0){
    cv::Rect roiR(eyeRx_ + draw_off_x, eyeRy_, Rw, Rh);
    draw_chevron(canvas_, roiR, -1, thickR, MAINCOLOR);
  }
  return;
  }
  // base eyes
  fillRoundRect(canvas_, eyeLx_, eyeLy_, eyeL_w_cur_, eyeL_h_cur_, eyeL_r_cur_, MAINCOLOR);
  if(!cyclops_) fillRoundRect(canvas_, eyeRx_, eyeRy_, eyeR_w_cur_, eyeR_h_cur_, eyeR_r_cur_, MAINCOLOR);

  // mood transitions
  eyelidsTiredH_next_ = tired_ ? eyeL_h_cur_/2 : 0;
  eyelidsAngryH_next_ = angry_ ? eyeL_h_cur_/2 : 0;
  eyelidsHappyBottomOffset_next_ = happy_ ? eyeL_h_cur_/2 : 0;

  eyelidsTiredH_ = (eyelidsTiredH_ + eyelidsTiredH_next_) / 2;
  eyelidsAngryH_ = (eyelidsAngryH_ + eyelidsAngryH_next_) / 2;
  eyelidsHappyBottomOffset_ = (eyelidsHappyBottomOffset_ + eyelidsHappyBottomOffset_next_) / 2;

  // tired (top) — triangles eating eyes
  if(!cyclops_){
    fillTriangle(canvas_, {eyeLx_, eyeLy_-1}, {eyeLx_+eyeL_w_cur_, eyeLy_-1}, {eyeLx_, eyeLy_+eyelidsTiredH_-1}, BGCOLOR);
    fillTriangle(canvas_, {eyeRx_, eyeRy_-1}, {eyeRx_+eyeR_w_cur_, eyeRy_-1}, {eyeRx_+eyeR_w_cur_, eyeRy_+eyelidsTiredH_-1}, BGCOLOR);
  } else {
    fillTriangle(canvas_, {eyeLx_, eyeLy_-1}, {eyeLx_+eyeL_w_cur_/2, eyeLy_-1}, {eyeLx_, eyeLy_+eyelidsTiredH_-1}, BGCOLOR);
    fillTriangle(canvas_, {eyeLx_+eyeL_w_cur_/2, eyeLy_-1}, {eyeLx_+eyeL_w_cur_, eyeLy_-1}, {eyeLx_+eyeL_w_cur_, eyeLy_+eyelidsTiredH_-1}, BGCOLOR);
  }

  // angry (top) — mirrored triangles
  if(!cyclops_){
    fillTriangle(canvas_, {eyeLx_, eyeLy_-1}, {eyeLx_+eyeL_w_cur_, eyeLy_-1}, {eyeLx_+eyeL_w_cur_, eyeLy_+eyelidsAngryH_-1}, BGCOLOR);
    fillTriangle(canvas_, {eyeRx_, eyeRy_-1}, {eyeRx_+eyeR_w_cur_, eyeRy_-1}, {eyeRx_, eyeRy_+eyelidsAngryH_-1}, BGCOLOR);
  } else {
    fillTriangle(canvas_, {eyeLx_, eyeLy_-1}, {eyeLx_+eyeL_w_cur_/2, eyeLy_-1}, {eyeLx_+eyeL_w_cur_/2, eyeLy_+eyelidsAngryH_-1}, BGCOLOR);
    fillTriangle(canvas_, {eyeLx_+eyeL_w_cur_/2, eyeLy_-1}, {eyeLx_+eyeL_w_cur_, eyeLy_-1}, {eyeLx_+eyeL_w_cur_/2, eyeLy_+eyelidsAngryH_-1}, BGCOLOR);
  }

  // happy (bottom) — mask with rounded rect extension
  fillRoundRect(canvas_, eyeLx_-1, (eyeLy_+eyeL_h_cur_)-eyelidsHappyBottomOffset_+1,
                eyeL_w_cur_+2, eyeL_h_def_, eyeL_r_cur_, BGCOLOR);
  if(!cyclops_){
    fillRoundRect(canvas_, eyeRx_-1, (eyeRy_+eyeR_h_cur_)-eyelidsHappyBottomOffset_+1,
                  eyeR_w_cur_+2, eyeR_h_def_, eyeR_r_cur_, BGCOLOR);
  }
}