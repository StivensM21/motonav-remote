# MotoNav — BLE-протокол (v1)

Сервис: `6e6f7600-b5a3-f393-e0a9-e50e24dcca9e`

| Характеристика | UUID | Доступ | Назначение |
|---|---|---|---|
| NAV | `6e6f7601-…` | Write Without Response | навигация: страница → устройство (10 байт, как раньше) |
| STATE | `6e6f7602-…` | Read + Notify | статус устройства: устройство → страница |
| CFG | `6e6f7603-…` | Read + Write | настройки: сон, яркость |

(полные UUID отличаются только третьим байтом: `…7601`, `…7602`, `…7603`)

## STATE — статус устройства (5 байт)

Устройство шлёт notify раз в ~10 секунд или при изменении.

| Байт | Значение |
|---|---|
| 0 | версия протокола = 1 |
| 1 | батарея, % (0–100) |
| 2 | флаги: бит 0 = заряжается |
| 3–4 | напряжение батареи, мВ, uint16 little-endian |

## CFG — настройки (3 байта)

Страница читает при подключении (устройство отдаёт текущие значения)
и пишет при нажатии «Применить».

| Байт | Значение |
|---|---|
| 0 | версия протокола = 1 |
| 1 | спящий режим: уснуть через N минут без движения/данных; 0 = никогда |
| 2 | яркость экрана, % (10–100) |

Устройство должно сохранять настройки в энергонезависимую память (NVS/EEPROM).

## Пример для прошивки (ESP32, Arduino BLE)

```cpp
#include <BLEDevice.h>
#include <BLE2902.h>
#include <Preferences.h>

#define SVC_UUID   "6e6f7600-b5a3-f393-e0a9-e50e24dcca9e"
#define NAV_UUID   "6e6f7601-b5a3-f393-e0a9-e50e24dcca9e"
#define STATE_UUID "6e6f7602-b5a3-f393-e0a9-e50e24dcca9e"
#define CFG_UUID   "6e6f7603-b5a3-f393-e0a9-e50e24dcca9e"

Preferences prefs;
BLECharacteristic *navChr, *stateChr, *cfgChr;
uint8_t sleepMin = 10, brightness = 80;

class CfgCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.size() >= 3 && (uint8_t)v[0] == 1) {
      sleepMin   = (uint8_t)v[1];
      brightness = (uint8_t)v[2];
      prefs.putUChar("sleep", sleepMin);
      prefs.putUChar("bri", brightness);
      // тут же применить яркость:
      // ledcWrite(BACKLIGHT_CH, map(brightness, 0, 100, 0, 255));
    }
  }
  void onRead(BLECharacteristic *c) override {
    uint8_t buf[3] = {1, sleepMin, brightness};
    c->setValue(buf, 3);
  }
};

void notifyState(uint8_t battPct, bool charging, uint16_t battMv) {
  uint8_t buf[5] = {1, battPct, (uint8_t)(charging ? 1 : 0),
                    (uint8_t)(battMv & 0xFF), (uint8_t)(battMv >> 8)};
  stateChr->setValue(buf, 5);
  stateChr->notify();
}

void setupBle() {
  prefs.begin("motonav");
  sleepMin   = prefs.getUChar("sleep", 10);
  brightness = prefs.getUChar("bri", 80);

  BLEDevice::init("MotoNav");
  BLEServer *srv = BLEDevice::createServer();
  BLEService *svc = srv->createService(SVC_UUID);

  navChr = svc->createCharacteristic(NAV_UUID,
             BLECharacteristic::PROPERTY_WRITE_NR);
  stateChr = svc->createCharacteristic(STATE_UUID,
             BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  stateChr->addDescriptor(new BLE2902());
  cfgChr = svc->createCharacteristic(CFG_UUID,
             BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  cfgChr->setCallbacks(new CfgCB());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->start();
}

// в loop(): раз в 10 секунд измерить батарею и вызвать notifyState(...);
// счётчик простоя: если нет движения/записей NAV дольше sleepMin минут
// и sleepMin != 0 — гасить экран / уходить в deep sleep.
```

Страница также понимает стандартный Battery Service (`0x180F` / `0x2A19`) —
если прошивке проще отдавать батарею через него, панель это подхватит
(но без вольтажа и флага зарядки).
