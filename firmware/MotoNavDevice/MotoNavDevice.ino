/*
 * MotoNav — прошивка устройства
 * Плата: Waveshare ESP32-S3-Touch-LCD-1.28 (круглый LCD GC9A01 240x240,
 *        сенсор CST816S, зарядка и замер батареи на борту)
 * BLE-протокол v1 — см. PROTOCOL.md в корне репозитория.
 *
 * Библиотека (Менеджер библиотек Arduino IDE): "GFX Library for Arduino"
 * BLE и Preferences входят в ядро esp32.
 *
 * Что умеет:
 *  - принимает навигацию со страницы и рисует манёвр, дистанцию, скорость;
 *  - отдаёт странице уровень батареи (процент + вольтаж), notify раз в 10 с;
 *  - настройки со страницы: спящий режим (N минут, 0 = никогда) и яркость,
 *    хранятся в энергонезависимой памяти;
 *  - без связи и без касаний экрана засыпает через N минут (deep sleep),
 *    просыпается от касания экрана.
 */

#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <esp_sleep.h>

// ---- пины Waveshare ESP32-S3-Touch-LCD-1.28 ----
#define PIN_LCD_DC   8
#define PIN_LCD_CS   9
#define PIN_LCD_SCK  10
#define PIN_LCD_MOSI 11
#define PIN_LCD_MISO 12
#define PIN_LCD_RST  14
#define PIN_LCD_BL   2   // подсветка, ШИМ
#define PIN_TP_INT   5   // прерывание сенсора — им же будим из сна
#define PIN_BAT_ADC  1   // делитель напряжения батареи 1:3

// если процент батареи заметно врёт — подстрой этот множитель (0.95…1.05)
#define BAT_CAL 1.0f

#define SVC_UUID   "6e6f7600-b5a3-f393-e0a9-e50e24dcca9e"
#define NAV_UUID   "6e6f7601-b5a3-f393-e0a9-e50e24dcca9e"
#define STATE_UUID "6e6f7602-b5a3-f393-e0a9-e50e24dcca9e"
#define CFG_UUID   "6e6f7603-b5a3-f393-e0a9-e50e24dcca9e"

// ---- цвета RGB565: тёмно-янтарный, как подсветка приборки BMW E39 ----
#define C_BG    0x0000
#define C_INK   0xFFFF
#define C_DIM   0x8C93
#define C_AMBER 0xFB60   // ~RGB(255,108,0)
#define C_AMB2  0x8A40   // тот же тон, приглушённый — кромка дороги, подписи
#define C_AMB3  0x38C0   // тёмно-янтарный — ореол/глубина, разметка
#define C_MINT  0x6E98
#define C_RED   0xE2C9

Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_MOSI, PIN_LCD_MISO);
Arduino_GC9A01 *panel = new Arduino_GC9A01(bus, PIN_LCD_RST, 0 /* поворот */, true /* IPS */);
// кадр собирается в буфере в памяти и выводится целиком — без мерцания
Arduino_Canvas *canvas = new Arduino_Canvas(240, 240, panel);
Arduino_GFX *gfx = canvas;
bool buffered = true;

Preferences prefs;
BLECharacteristic *stateChr, *cfgChr;

volatile bool connected = false;
volatile uint32_t lastTouchMs = 0;
uint32_t lastActivity = 0;

uint8_t sleepMin = 10, brightness = 80;

// последний принятый пакет навигации
volatile bool navFresh = false;
volatile uint8_t  navType = 0, navSpd = 0;
volatile uint16_t navDist = 0;
volatile uint32_t navAt = 0;
volatile uint8_t  navN = 0;      // точки дороги впереди (пакеты v2/v3)
volatile uint8_t  navProg = 255; // прогресс маршрута 0–100, 255 = неизвестен
int8_t navPX[16], navPY[16];     // пиксельные смещения от якоря

uint8_t battCache = 0;

void IRAM_ATTR touchISR() { lastTouchMs = millis(); }

void applyBrightness() {
  analogWrite(PIN_LCD_BL, map(brightness, 0, 100, 10, 255));
}

// ---- батарея ----
float readBattVolts() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_BAT_ADC);
  return mv / 8.0f * 3.0f / 1000.0f * BAT_CAL;
}

uint8_t battPct(float v) {
  const float   tv[] = {3.30, 3.50, 3.68, 3.74, 3.77, 3.79, 3.82, 3.87, 3.92, 3.98, 4.06, 4.20};
  const uint8_t tp[] = {0,    5,    10,   20,   30,   40,   50,   60,   70,   80,   90,   100};
  if (v <= tv[0]) return 0;
  if (v >= tv[11]) return 100;
  for (int i = 1; i < 12; i++)
    if (v < tv[i])
      return tp[i-1] + (uint8_t)((v - tv[i-1]) / (tv[i] - tv[i-1]) * (tp[i] - tp[i-1]));
  return 100;
}

void notifyState() {
  float v = readBattVolts();
  battCache = battPct(v);
  uint16_t mv = (uint16_t)(v * 1000);
  uint8_t buf[5] = {1, battCache, 0, (uint8_t)(mv & 0xFF), (uint8_t)(mv >> 8)};
  stateChr->setValue(buf, 5);
  if (connected) stateChr->notify();
}

// ---- BLE колбэки ----
class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    connected = true; lastActivity = millis(); navFresh = true;
  }
  void onDisconnect(BLEServer *s) override {
    connected = false; lastActivity = millis(); navAt = 0; navFresh = true;
    s->startAdvertising();
  }
};

class NavCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    uint8_t *d = c->getData();
    size_t n = c->getLength();
    if (n >= 7 && d[0] == 3) {                 // v3: + прогресс маршрута
      navType = d[1];
      navDist = d[2] | (d[3] << 8);
      navSpd  = d[4];
      navProg = d[5];
      uint8_t k = d[6]; if (k > 16) k = 16;
      if (n >= (size_t)(7 + k * 2)) {
        for (uint8_t i = 0; i < k; i++) {
          navPX[i] = (int8_t)d[7 + i * 2];
          navPY[i] = (int8_t)d[8 + i * 2];
        }
        navN = k;
      }
      navAt = millis(); navFresh = true; lastActivity = millis();
    } else if (n >= 6 && d[0] == 2) {          // v2: заголовок + форма дороги
      navType = d[1];
      navDist = d[2] | (d[3] << 8);
      navSpd  = d[4];
      navProg = 255;
      uint8_t k = d[5]; if (k > 16) k = 16;
      if (n >= (size_t)(6 + k * 2)) {
        for (uint8_t i = 0; i < k; i++) {
          navPX[i] = (int8_t)d[6 + i * 2];
          navPY[i] = (int8_t)d[7 + i * 2];
        }
        navN = k;
      }
      navAt = millis(); navFresh = true; lastActivity = millis();
    } else if (n >= 10 && d[0] == 1) {         // v1: без геометрии — просто стрелка
      navType = d[1];
      navDist = d[2] | (d[3] << 8);
      navSpd  = d[4];
      navN = 0; navProg = 255;
      navAt = millis(); navFresh = true; lastActivity = millis();
    }
  }
};

class CfgCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    uint8_t *d = c->getData();
    if (c->getLength() >= 3 && d[0] == 1) {
      sleepMin   = d[1];
      brightness = constrain(d[2], 10, 100);
      prefs.putUChar("sleep", sleepMin);
      prefs.putUChar("bri", brightness);
      applyBrightness();
      lastActivity = millis();
    }
  }
  void onRead(BLECharacteristic *c) override {
    uint8_t buf[3] = {1, sleepMin, brightness};
    c->setValue(buf, 3);
  }
};

// ---- рисование ----
void textAt(int cx, int y, const char *s, int size, uint16_t color) {
  int w = strlen(s) * 6 * size;
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(cx - w / 2, y);
  gfx->print(s);
}
void textCentered(const char *s, int y, int size, uint16_t color) { textAt(120, y, s, size, color); }

void drawBatteryBar(int x, int y, bool withText) {
  int w = 30, h = 12;
  uint16_t c = battCache <= 20 ? C_RED : C_MINT;
  gfx->drawRect(x, y, w, h, C_DIM);
  gfx->fillRect(x + w, y + 3, 3, h - 6, C_DIM);          // носик
  int fw = (w - 4) * battCache / 100;
  if (fw > 0) gfx->fillRect(x + 2, y + 2, fw, h - 4, c);
  if (withText) {
    char t[6]; snprintf(t, sizeof(t), "%d%%", battCache);
    gfx->setTextSize(1);
    gfx->setTextColor(C_DIM);
    gfx->setCursor(x + w + 8, y + 2);
    gfx->print(t);
  }
}

// толстая линия из двух треугольников + круглые стыки
void wideLine(float x0, float y0, float x1, float y1, float w, uint16_t c) {
  float dx = x1 - x0, dy = y1 - y0, len = sqrtf(dx * dx + dy * dy);
  if (len < 0.5f) return;
  float nx = -dy / len * w / 2, ny = dx / len * w / 2;
  gfx->fillTriangle(x0 + nx, y0 + ny, x0 - nx, y0 - ny, x1 + nx, y1 + ny, c);
  gfx->fillTriangle(x0 - nx, y0 - ny, x1 - nx, y1 - ny, x1 + nx, y1 + ny, c);
}

// наконечник стрелки; ang — направление в радианах (0 = вправо, -PI/2 = вверх)
void arrowHead(float x, float y, float ang, float s, uint16_t c) {
  gfx->fillTriangle(x + cosf(ang) * s,        y + sinf(ang) * s,
                    x + cosf(ang + 2.6f) * s, y + sinf(ang + 2.6f) * s,
                    x + cosf(ang - 2.6f) * s, y + sinf(ang - 2.6f) * s, c);
}

// маленькая иконка манёвра для нижней строки — со скруглёнными поворотами
void drawMini(uint8_t type, int cx, int cy) {
  uint16_t c = C_AMBER;
  switch (type) {
    case 0: // прямо
      segLine(cx, cy + 14, cx, cy - 4, 3, c);
      arrowHead(cx, cy - 8, -M_PI / 2, 10, c);
      break;
    case 1: // направо: стойка, плавная дуга, голова вправо
      segLine(cx - 9, cy + 14, cx - 9, cy - 1, 3, c);
      gfx->fillArc(cx, cy - 1, 12, 6, 180, 270, c);
      arrowHead(cx + 5, cy - 10, 0, 10, c);
      break;
    case 2: // налево
      segLine(cx + 9, cy + 14, cx + 9, cy - 1, 3, c);
      gfx->fillArc(cx, cy - 1, 12, 6, 270, 360, c);
      arrowHead(cx - 5, cy - 10, M_PI, 10, c);
      break;
    case 3: // круговое движение
      gfx->fillArc(cx, cy + 2, 10, 6, 0, 360, c);
      arrowHead(cx, cy - 11, -M_PI / 2, 9, c);
      break;
    case 4: // разворот: дуга сверху, голова вниз
      segLine(cx + 8, cy + 14, cx + 8, cy - 2, 3, c);
      gfx->fillArc(cx, cy - 2, 11, 5, 180, 360, c);
      segLine(cx - 8, cy - 2, cx - 8, cy + 2, 3, c);
      arrowHead(cx - 8, cy + 6, M_PI / 2, 9, c);
      break;
    default: { // финиш
      int s = 6;
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          gfx->fillRect(cx - 9 + i * s, cy - 8 + j * s, s, s, ((i + j) & 1) ? C_BG : c);
      gfx->drawRect(cx - 9, cy - 8, 3 * s, 3 * s, C_AMB2);
    }
  }
}

// сглаженная кривая Катмулла-Рома через точки маршрута:
// вдоль неё штампуются кружки, радиус плавно сужается вдаль — перспектива
void stampPath(const float *xs, const float *ys, int n, float r0, float r1, uint16_t c) {
  for (int i = 0; i < n - 1; i++) {
    int i0 = i > 0 ? i - 1 : 0, i3 = i + 2 < n ? i + 2 : n - 1;
    float p0x = xs[i0], p0y = ys[i0], p1x = xs[i], p1y = ys[i];
    float p2x = xs[i + 1], p2y = ys[i + 1], p3x = xs[i3], p3y = ys[i3];
    int st = (int)(hypotf(p2x - p1x, p2y - p1y) / 1.5f);
    if (st < 2) st = 2;
    for (int k = 0; k <= st; k++) {
      float t = (float)k / st, t2 = t * t, t3 = t2 * t;
      float x = 0.5f * (2*p1x + (-p0x + p2x)*t + (2*p0x - 5*p1x + 4*p2x - p3x)*t2 + (-p0x + 3*p1x - 3*p2x + p3x)*t3);
      float y = 0.5f * (2*p1y + (-p0y + p2y)*t + (2*p0y - 5*p1y + 4*p2y - p3y)*t2 + (-p0y + 3*p1y - 3*p2y + p3y)*t3);
      float u = ((float)i + t) / (float)(n - 1);     // 0 у шеврона → 1 вдали
      gfx->fillCircle((int16_t)x, (int16_t)y, (int16_t)(r0 + (r1 - r0) * u), c);
    }
  }
}

// разметка-пунктир по центру дороги (тёмные штрихи поверх сердцевины)
void stampDashes(const float *xs, const float *ys, int n) {
  int idx = 0;
  for (int i = 0; i < n - 1; i++) {
    int i0 = i > 0 ? i - 1 : 0, i3 = i + 2 < n ? i + 2 : n - 1;
    float p0x = xs[i0], p0y = ys[i0], p1x = xs[i], p1y = ys[i];
    float p2x = xs[i + 1], p2y = ys[i + 1], p3x = xs[i3], p3y = ys[i3];
    int st = (int)(hypotf(p2x - p1x, p2y - p1y) / 1.5f);
    if (st < 2) st = 2;
    for (int k = 0; k <= st; k++, idx++) {
      float t = (float)k / st, t2 = t * t, t3 = t2 * t;
      float x = 0.5f * (2*p1x + (-p0x + p2x)*t + (2*p0x - 5*p1x + 4*p2x - p3x)*t2 + (-p0x + 3*p1x - 3*p2x + p3x)*t3);
      float y = 0.5f * (2*p1y + (-p0y + p2y)*t + (2*p0y - 5*p1y + 4*p2y - p3y)*t2 + (-p0y + 3*p1y - 3*p2y + p3y)*t3);
      float u = ((float)i + t) / (float)(n - 1);
      if ((idx % 9) < 4)
        gfx->fillCircle((int16_t)x, (int16_t)y, (int16_t)(1.6f * (1 - u * 0.4f)) + 1, C_AMB3);
    }
  }
}

// двойной шеврон-указатель «моё положение»
void drawChevron2(int cx, int ty, float s) {
  for (int i = 0; i < 2; i++) {
    uint16_t c = i ? C_AMB2 : C_AMBER;
    float o = i * 15 * s;
    segLine(cx - 15 * s, ty + 18 * s + o, cx,          ty + 2 * s + o, 3.5f * s, c);
    segLine(cx,          ty + 2 * s + o, cx + 15 * s,  ty + 18 * s + o, 3.5f * s, c);
  }
}

// дорога впереди в стиле Beeline: объёмная полоса (ореол+кромка+сердцевина) с разметкой
void drawRoad() {
  const float ax = 120, ay = 182;                    // якорь у нижнего шеврона (по центру)
  float xs[17], ys[17];
  int n = 0;
  xs[n] = ax; ys[n] = ay; n++;
  for (uint8_t i = 0; i < navN && n < 17; i++) {
    xs[n] = ax + navPX[i]; ys[n] = ay - navPY[i]; n++;
  }
  if (n < 2) { drawChevron2(120, 196, 1.0f); return; }
  stampPath(xs, ys, n, 12, 7,    C_AMB3);            // ореол/глубина
  stampPath(xs, ys, n, 9,  5,    C_AMB2);            // кромка
  stampPath(xs, ys, n, 7,  3.2f, C_AMBER);          // сердцевина
  stampDashes(xs, ys, n);                            // разметка
  drawChevron2(120, 196, 1.0f);                      // двойной шеврон
}

// ---- крупные сегментные цифры (гладкие, из штампованных штрихов) ----
void segLine(float x0, float y0, float x1, float y1, float r, uint16_t c) {
  float dx = x1 - x0, dy = y1 - y0;
  int st = (int)(hypotf(dx, dy) / 2) + 1;
  for (int k = 0; k <= st; k++) {
    float t = (float)k / st;
    gfx->fillCircle((int16_t)(x0 + dx * t), (int16_t)(y0 + dy * t), (int16_t)r, c);
  }
}

void drawDigit(char ch, float x, float y, float w, float h, uint16_t c) {
  static const uint8_t M[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
  if (ch < '0' || ch > '9') return;
  uint8_t m = M[ch - '0'];
  float r = w * 0.14f, m2 = y + h / 2;
  if (r < 2.5f) r = 2.5f;
  if (m & 1)  segLine(x + r, y, x + w - r, y, r, c);            // верх
  if (m & 2)  segLine(x + w, y + r, x + w, m2 - r, r, c);       // правый верх
  if (m & 4)  segLine(x + w, m2 + r, x + w, y + h - r, r, c);   // правый низ
  if (m & 8)  segLine(x + r, y + h, x + w - r, y + h, r, c);    // низ
  if (m & 16) segLine(x, m2 + r, x, y + h - r, r, c);           // левый низ
  if (m & 32) segLine(x, y + r, x, m2 - r, r, c);               // левый верх
  if (m & 64) segLine(x + r, m2, x + w - r, m2, r, c);          // середина
}

float numberWidth(const char *s, float w) {
  const float gap = w * 0.45f, dw = w * 0.4f;
  int n = strlen(s);
  float total = 0;
  for (int i = 0; i < n; i++) total += (s[i] == '.' ? dw : w) + (i < n - 1 ? gap : 0);
  return total;
}

void drawNumber(const char *s, int cx, int y, float w, float h, uint16_t c) {
  const float gap = w * 0.45f, dw = w * 0.4f;
  int n = strlen(s);
  float x = cx - numberWidth(s, w) / 2;
  for (int i = 0; i < n; i++) {
    if (s[i] == '.') { gfx->fillCircle((int16_t)(x + dw / 2), (int16_t)(y + h), (int16_t)(w * 0.16f), c); x += dw + gap; }
    else             { drawDigit(s[i], x, y, w, h, c);                                                    x += w + gap; }
  }
}

// дуга прогресса маршрута по нижнему краю + засечки контрольных точек
void drawProgressArc() {
  if (navProg > 100) return;
  gfx->fillArc(120, 120, 118, 113, 45, 135, C_AMB2);   // дорожка ярче, чем фон
  if (navProg > 0)
    gfx->fillArc(120, 120, 118, 113, 135 - 0.9f * navProg, 135, C_AMBER);
  for (int i = 0; i <= 4; i++) {
    float d = (45 + 22.5f * i) * (float)M_PI / 180.0f;
    uint16_t c = ((float)i / 4 <= navProg / 100.0f) ? C_AMBER : C_AMB2;
    gfx->fillCircle((int16_t)(120 + cosf(d) * 115), (int16_t)(120 + sinf(d) * 115), 3, c);
  }
}

// ---- заставка: батарея справа, локатор с радар-волнами, буквы GPS ----
void drawBattRight() {
  int x = 214, y = 120, w = 13, h = 26;
  uint16_t c = battCache <= 20 ? C_RED : C_MINT;
  gfx->drawRect(x - w / 2, y - h / 2, w, h, C_DIM);
  gfx->fillRect(x - 3, y - h / 2 - 3, 6, 3, C_DIM);
  int fh = (h - 4) * battCache / 100;
  if (fh > 0) gfx->fillRect(x - w / 2 + 2, y + h / 2 - 2 - fh, w - 4, fh, c);
  char t[6]; snprintf(t, sizeof(t), "%d%%", battCache);
  gfx->setTextSize(1); gfx->setTextColor(C_DIM);
  gfx->setCursor(x - (int)strlen(t) * 3, y + h / 2 + 6); gfx->print(t);
}

void drawLocator(int cx, int cy, float s, uint16_t c) {
  gfx->fillTriangle(cx, cy - s, cx + s * 0.72f, cy + s * 0.8f, cx, cy + s * 0.4f, c);
  gfx->fillTriangle(cx, cy - s, cx, cy + s * 0.4f, cx - s * 0.72f, cy + s * 0.8f, c);
}

void drawWave(int cx, int cy, int r, uint16_t c) {
  for (float a = -0.15f * (float)M_PI; a <= 1.15f * (float)M_PI; a += 0.05f)
    gfx->fillCircle((int16_t)(cx + cosf(a) * r), (int16_t)(cy + sinf(a) * r), 1, c);
}

// буквы GPS — ломаные по долям рамки, штрихи с круглыми концами (гладко)
void glyphStroke(const float *fx, const float *fy, int n, float ox, float oy,
                 float w, float h, float r, uint16_t c) {
  for (int i = 0; i < n - 1; i++)
    segLine(ox + fx[i] * w, oy + fy[i] * h, ox + fx[i + 1] * w, oy + fy[i + 1] * h, r, c);
}
void drawGPS(int cx, int topY, float w, float h, float gap, uint16_t c) {
  static const float GX[] = {1.0,0.72,0.28,0.0,0.0,0.28,0.72,1.0,1.0,0.55};
  static const float GY[] = {0.16,0.0,0.0,0.25,0.75,1.0,1.0,0.75,0.52,0.52};
  static const float PX[] = {0.0,0.0,0.68,1.0,0.68,0.0};
  static const float PY[] = {1.0,0.0,0.0,0.22,0.46,0.46};
  static const float SX[] = {1.0,0.7,0.3,0.0,0.3,0.7,1.0,0.7,0.3,0.0};
  static const float SY[] = {0.16,0.0,0.0,0.24,0.48,0.56,0.8,1.0,1.0,0.84};
  float total = 3 * w + 2 * gap, x = cx - total / 2, r = w * 0.16f;
  if (r < 2) r = 2;
  for (int pass = 0; pass < 2; pass++) {                // тёмный ореол, потом яркая заливка
    uint16_t col = pass ? c : C_AMB3;
    float rr = pass ? r : r + 2;
    glyphStroke(GX, GY, 10, x,                 topY, w, h, rr, col);
    glyphStroke(PX, PY, 6,  x + w + gap,        topY, w, h, rr, col);
    glyphStroke(SX, SY, 10, x + 2 * (w + gap),  topY, w, h, rr, col);
  }
}

void drawSplash(const char *sub, uint16_t subCol) {
  drawWave(112, 96, 22, C_AMBER);
  drawWave(112, 96, 44, C_AMB2);
  drawWave(112, 96, 66, C_AMB3);
  drawLocator(112, 96, 20, C_AMBER);
  drawGPS(112, 170, 18, 24, 9, C_AMBER);
  textAt(120, 206, sub, 1, subCol);
  drawBattRight();
}

// стрелка манёвра, центр в (cx, cy)
void drawArrow(uint8_t type, int cx, int cy) {
  uint16_t c = C_AMBER;
  switch (type) {
    case 0: // прямо
      gfx->fillRect(cx - 8, cy - 8, 16, 44, c);
      gfx->fillTriangle(cx - 26, cy - 8, cx + 26, cy - 8, cx, cy - 44, c);
      break;
    case 1: // направо
      gfx->fillRect(cx - 36, cy - 8, 16, 44, c);              // подход снизу
      gfx->fillRect(cx - 36, cy - 8, 52, 16, c);              // колено вправо
      gfx->fillTriangle(cx + 14, cy - 28, cx + 14, cy + 28, cx + 46, cy, c);
      break;
    case 2: // налево
      gfx->fillRect(cx + 20, cy - 8, 16, 44, c);
      gfx->fillRect(cx - 16, cy - 8, 52, 16, c);
      gfx->fillTriangle(cx - 14, cy - 28, cx - 14, cy + 28, cx - 46, cy, c);
      break;
    case 3: // круговое движение
      for (int r = 24; r <= 30; r++) gfx->drawCircle(cx, cy + 4, r, c);
      gfx->fillTriangle(cx - 14, cy - 22, cx + 14, cy - 22, cx, cy - 48, c);
      break;
    case 4: // разворот
      gfx->fillRect(cx - 28, cy - 16, 14, 44, c);
      gfx->fillRect(cx + 14, cy - 16, 14, 44, c);
      gfx->fillRect(cx - 28, cy - 24, 56, 14, c);             // перемычка сверху
      gfx->fillTriangle(cx - 42, cy + 24, cx - 0, cy + 24, cx - 21, cy + 50, c);
      break;
    default: { // финиш — клетчатый флаг
      int s = 12;
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
          gfx->fillRect(cx - 20 + i * s, cy - 30 + j * s, s, s,
                        ((i + j) & 1) ? C_INK : C_BG);
      gfx->drawRect(cx - 20, cy - 30, 4 * s, 3 * s, C_DIM);
      gfx->fillRect(cx - 24, cy - 30, 4, 70, C_DIM);          // древко
    }
  }
}

void drawScreen() {
  gfx->fillScreen(C_BG);

  // заставка (вариант 2): локатор с радар-волнами + GPS
  if (!connected) { drawSplash("Ready for the trip", C_DIM); return; }
  if (navAt == 0) { drawSplash("connected", C_MINT);          return; }

  // экран навигации (вариант Л): дорога снизу, крупные метры сверху
  if (navN) drawRoad();
  else      drawArrow(navType, 120, 120);

  // маска под цифрами и крупная дистанция сверху, число+единицы по центру
  gfx->fillRect(0, 0, 240, 66, C_BG);
  char d[8]; const char *u;
  if (navDist >= 1000) { snprintf(d, sizeof(d), "%u.%u", (unsigned)(navDist / 1000), (unsigned)((navDist % 1000) / 100)); u = "km"; }
  else                 { snprintf(d, sizeof(d), "%u", (unsigned)navDist); u = "m"; }
  const float dW = 22, uSize = 2, uGap = 10;
  float nw = numberWidth(d, dW), uw = strlen(u) * 6 * uSize;
  float startX = 120 - (nw + uGap + uw) / 2;
  drawNumber(d, (int)(startX + nw / 2), 20, dW, 36, C_AMBER);
  textAt((int)(startX + nw + uGap + uw / 2), 42, u, uSize, C_AMB2);

  drawBattRight();
  drawProgressArc();
}

// ---- сон ----
void goToSleep() {
  gfx->fillScreen(C_BG);
  textCentered("sleep...", 110, 3, C_DIM);
  if (buffered) canvas->flush();
  delay(600);
  analogWrite(PIN_LCD_BL, 0);
  panel->displayOff();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_TP_INT, 0); // касание экрана будит
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);

  prefs.begin("motonav");
  sleepMin   = prefs.getUChar("sleep", 10);
  brightness = prefs.getUChar("bri", 80);

  pinMode(PIN_TP_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TP_INT), touchISR, FALLING);

  if (!canvas->begin()) {           // не хватило памяти под буфер (PSRAM выключен?)
    buffered = false; gfx = panel;  // — рисуем напрямую, без сглаживания кадра
    panel->begin();
  }
  applyBrightness();

  BLEDevice::init("MotoNav");
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new SrvCB());
  BLEService *svc = srv->createService(SVC_UUID);

  BLECharacteristic *navChr = svc->createCharacteristic(
      NAV_UUID, BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_WRITE);
  navChr->setCallbacks(new NavCB());

  stateChr = svc->createCharacteristic(
      STATE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  stateChr->addDescriptor(new BLE2902());

  cfgChr = svc->createCharacteristic(
      CFG_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  cfgChr->setCallbacks(new CfgCB());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  notifyState();   // чтобы страница сразу увидела батарею при подключении
  lastActivity = millis();
  navFresh = true; // первая отрисовка
}

void loop() {
  static uint32_t lastDraw = 0, lastBatt = 0;
  uint32_t now = millis();

  if (lastTouchMs > lastActivity) lastActivity = lastTouchMs;

  // связь есть, но данные перестали приходить — вернуться на экран ожидания
  if (connected && navAt != 0 && now - navAt > 20000) { navAt = 0; navFresh = true; }

  if (navFresh && now - lastDraw > 250) {
    navFresh = false; lastDraw = now;
    drawScreen();
    if (buffered) canvas->flush();
  }

  if (now - lastBatt > 10000) {
    lastBatt = now;
    uint8_t prev = battCache;
    notifyState();
    if (battCache != prev && (!connected || navAt == 0)) navFresh = true;
  }

  // сон: только без связи, 0 = никогда
  if (!connected && sleepMin > 0 && now - lastActivity > (uint32_t)sleepMin * 60000UL)
    goToSleep();

  delay(20);
}
