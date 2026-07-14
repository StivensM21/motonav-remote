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
#define C_AMB2  0x79A0   // тот же тон, приглушённый — для подписей
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
volatile uint8_t  navN = 0;      // точки дороги впереди (пакет v2)
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
    if (n >= 6 && d[0] == 2) {                 // v2: заголовок + форма дороги
      navType = d[1];
      navDist = d[2] | (d[3] << 8);
      navSpd  = d[4];
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
      navN = 0;
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

// маленькая иконка манёвра для нижней строки
void drawMini(uint8_t type, int cx, int cy) {
  uint16_t c = C_AMBER;
  switch (type) {
    case 0: // прямо
      wideLine(cx, cy + 14, cx, cy - 4, 5, c);
      arrowHead(cx, cy - 8, -M_PI / 2, 10, c);
      break;
    case 1: // направо
      wideLine(cx - 8, cy + 14, cx - 8, cy - 4, 5, c);
      wideLine(cx - 8, cy - 4, cx + 4, cy - 4, 5, c);
      arrowHead(cx + 8, cy - 4, 0, 10, c);
      break;
    case 2: // налево
      wideLine(cx + 8, cy + 14, cx + 8, cy - 4, 5, c);
      wideLine(cx + 8, cy - 4, cx - 4, cy - 4, 5, c);
      arrowHead(cx - 8, cy - 4, M_PI, 10, c);
      break;
    case 3: // круговое движение
      gfx->drawCircle(cx, cy + 2, 7, c);
      gfx->drawCircle(cx, cy + 2, 8, c);
      arrowHead(cx, cy - 9, -M_PI / 2, 9, c);
      break;
    case 4: // разворот
      wideLine(cx + 7, cy + 14, cx + 7, cy - 4, 5, c);
      wideLine(cx + 7, cy - 4, cx - 7, cy - 4, 5, c);
      wideLine(cx - 7, cy - 4, cx - 7, cy + 4, 5, c);
      arrowHead(cx - 7, cy + 8, M_PI / 2, 9, c);
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
// вдоль неё штампуются кружки — дорога выходит ровной, без изломов на стыках
void stampPath(const float *xs, const float *ys, int n, int r, uint16_t c) {
  for (int i = 0; i < n - 1; i++) {
    int i0 = i > 0 ? i - 1 : 0, i3 = i + 2 < n ? i + 2 : n - 1;
    float p0x = xs[i0], p0y = ys[i0], p1x = xs[i], p1y = ys[i];
    float p2x = xs[i + 1], p2y = ys[i + 1], p3x = xs[i3], p3y = ys[i3];
    int st = (int)(hypotf(p2x - p1x, p2y - p1y) / 2);
    if (st < 2) st = 2;
    for (int k = 0; k <= st; k++) {
      float t = (float)k / st, t2 = t * t, t3 = t2 * t;
      float x = 0.5f * (2*p1x + (-p0x + p2x)*t + (2*p0x - 5*p1x + 4*p2x - p3x)*t2 + (-p0x + 3*p1x - 3*p2x + p3x)*t3);
      float y = 0.5f * (2*p1y + (-p0y + p2y)*t + (2*p0y - 5*p1y + 4*p2y - p3y)*t2 + (-p0y + 3*p1y - 3*p2y + p3y)*t3);
      gfx->fillCircle((int16_t)x, (int16_t)y, r, c);
    }
  }
}

// дорога впереди в стиле Beeline: гладкая полоса от шеврона вверх по форме маршрута
void drawRoad() {
  const float ax = 120, ay = 168;
  float xs[17], ys[17];
  int n = 0;
  xs[n] = ax; ys[n] = ay; n++;                       // старт — у шеврона
  for (uint8_t i = 0; i < navN && n < 17; i++) {
    xs[n] = ax + navPX[i]; ys[n] = ay - navPY[i]; n++;
  }
  if (n < 2) return;
  stampPath(xs, ys, n, 8, C_AMB2);                   // тёмная кромка
  stampPath(xs, ys, n, 6, C_AMBER);                  // сердцевина
  // шеврон-указатель в начале дороги
  gfx->fillTriangle(102, 184, 138, 184, 120, 148, C_AMBER);
  gfx->drawTriangle(102, 184, 138, 184, 120, 148, C_BG);
  gfx->fillTriangle(108, 181, 132, 181, 120, 157, C_BG);
  gfx->fillTriangle(111, 180, 129, 180, 120, 162, C_AMBER);
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

  if (!connected) {
    textCentered("GPS", 78, 5, C_AMBER);
    textCentered("Ready for the trip", 132, 2, C_DIM);
    drawBatteryBar(92, 208, true);
    return;
  }
  if (navAt == 0) {
    textCentered("GPS", 68, 5, C_AMBER);
    textCentered("connected", 122, 2, C_MINT);
    textCentered("no route yet", 147, 2, C_DIM);
    drawBatteryBar(92, 208, true);
    return;
  }

  // верх: батарея и скорость, ненавязчиво
  drawBatteryBar(104, 8, false);
  char sp[12]; snprintf(sp, sizeof(sp), "%u km/h", (unsigned)navSpd);
  textAt(120, 26, sp, 1, C_DIM);

  // середина: дорога по форме маршрута; без геометрии (пакет v1) — большая стрелка
  if (navN) drawRoad();
  else      drawArrow(navType, 120, 84);

  // низ: иконка манёвра и дистанция, как на Beeline
  drawMini(navType, 68, 200);
  char d[8]; const char *u;
  if (navDist >= 1000) { snprintf(d, sizeof(d), "%u.%u", (unsigned)(navDist / 1000), (unsigned)((navDist % 1000) / 100)); u = "km"; }
  else                 { snprintf(d, sizeof(d), "%u", (unsigned)navDist); u = "m"; }
  textAt(138, 188, d, 4, C_AMBER);
  textAt(138, 220, u, 2, C_AMB2);
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
