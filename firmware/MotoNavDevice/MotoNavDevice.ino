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

// ---- цвета RGB565 (та же палитра, что на странице) ----
#define C_BG    0x0000
#define C_INK   0xFFFF
#define C_DIM   0x8C93
#define C_AMBER 0xFDA4
#define C_MINT  0x6E98
#define C_RED   0xE2C9

Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_MOSI, PIN_LCD_MISO);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, PIN_LCD_RST, 0 /* поворот */, true /* IPS */);

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
    if (c->getLength() >= 10 && d[0] == 1) {
      navType = d[1];
      navDist = d[2] | (d[3] << 8);
      navSpd  = d[4];
      navAt = millis(); navFresh = true;
      lastActivity = millis();
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
void textCentered(const char *s, int y, int size, uint16_t color) {
  int w = strlen(s) * 6 * size;
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(120 - w / 2, y);
  gfx->print(s);
}

void drawBatteryBar() {
  // маленький индикатор внизу круглого экрана
  int x = 92, y = 208, w = 30, h = 12;
  uint16_t c = battCache <= 20 ? C_RED : C_MINT;
  gfx->drawRect(x, y, w, h, C_DIM);
  gfx->fillRect(x + w, y + 3, 3, h - 6, C_DIM);          // носик
  int fw = (w - 4) * battCache / 100;
  if (fw > 0) gfx->fillRect(x + 2, y + 2, fw, h - 4, c);
  char t[6]; snprintf(t, sizeof(t), "%d%%", battCache);
  gfx->setTextSize(1);
  gfx->setTextColor(C_DIM);
  gfx->setCursor(x + w + 8, y + 2);
  gfx->print(t);
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
    textCentered("MotoNav", 84, 4, C_INK);
    textCentered("zhdu telefon...", 130, 2, C_DIM);
    drawBatteryBar();
    return;
  }
  if (navAt == 0) {
    textCentered("MotoNav", 74, 4, C_INK);
    textCentered("na svyazi", 120, 2, C_MINT);
    textCentered("marshruta net", 145, 2, C_DIM);
    drawBatteryBar();
    return;
  }

  drawArrow(navType, 120, 78);

  char d[12];
  if (navDist >= 1000) snprintf(d, sizeof(d), "%d.%d km", navDist / 1000, (navDist % 1000) / 100);
  else                 snprintf(d, sizeof(d), "%d m", navDist);
  textCentered(d, 140, 4, C_INK);

  char s[16]; snprintf(s, sizeof(s), "%d km/h", navSpd);
  textCentered(s, 180, 2, C_DIM);

  drawBatteryBar();
}

// ---- сон ----
void goToSleep() {
  gfx->fillScreen(C_BG);
  textCentered("sleep...", 110, 3, C_DIM);
  delay(600);
  analogWrite(PIN_LCD_BL, 0);
  gfx->displayOff();
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

  gfx->begin();
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
