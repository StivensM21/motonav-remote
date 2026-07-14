---
name: run-motonav-remote
description: Run, test and screenshot the MotoNav web BLE remote page headless (Chromium + Web Bluetooth mock). Use for "run the page", "test motonav", "screenshot the remote", "verify the BLE flow". Firmware cannot run in a container — hardware only.
---

# Run: MotoNav remote

Все пути — относительно корня репозитория.

Проект — две части:

1. **`index.html`** — статичная страница (Web Bluetooth пульт). Запускается
   headless-Chromium'ом через `driver.mjs`; Bluetooth в контейнере мокается.
2. **`firmware/MotoNavDevice/`** — прошивка для Waveshare
   ESP32-S3-Touch-LCD-1.28. **В контейнере не запускается** — нужно железо,
   а тулчейн ESP32 не скачать (сетевая политика отдаёт 403 на
   downloads.arduino.cc и release-ассеты GitHub; проверено). Прошивку
   проверяют заливкой на плату по `firmware/MotoNavDevice/README.md`.

## Prerequisites

Node 22 и Chromium уже в контейнере (`/opt/pw-browsers/chromium`,
`playwright install` запускать НЕ надо). Единственная установка:

```bash
npm i --prefix .claude/skills/run-motonav-remote --no-save playwright-core
```

## Build

Нет сборки — страница статичная.

## Run (agent path) — драйвер

```bash
node .claude/skills/run-motonav-remote/driver.mjs
```

Что он делает: открывает `index.html` через `file://`, подменяет
`navigator.bluetooth` моком устройства (характеристики NAV/STATE/CFG из
`PROTOCOL.md`), жмёт «Подключить устройство», проверяет что панель
устройства появилась и батарея/вольтаж/настройки отрисованы из мока,
меняет сон и яркость, жмёт «Применить» и сверяет байты, ушедшие в CFG
(`[1, сон, яркость]`). Печатает `PASS`/`FAIL` по строке на проверку,
код выхода 0 = всё прошло.

Скриншот: `motonav-page.png` в текущей директории
(переопределить: `MOTONAV_SS=/path/shot.png node …`).

Всё, что страница «записала в устройство», доступно в браузерном контексте
как `window.__ble` (`.cfg` — последняя запись настроек, `.nav` — массив
навигационных пакетов) — удобно для новых проверок.

## Direct invocation

Функции страницы глобальные — чистую логику можно дёргать без BLE вообще
(так драйвер проверяет `bearingTo`; тем же способом доступны `setBattery`,
`mapType`, `haversine` через `page.evaluate`).

## Run (human path)

Продакшен: https://stivensm21.github.io/motonav-remote/ — публикуется
GitHub Pages из ветки `main` автоматически (~1 мин после пуша).
Открывать в Chrome на Android: Web Bluetooth не работает в iOS Safari
и требует HTTPS + жеста пользователя. Headless бесполезно — реального
Bluetooth в контейнере нет.

## Test

Драйвер и есть тест-сьют. Другого нет.

## Gotchas

- **Сеть из контейнера почти вся закрыта** (agent-proxy отдаёт 403 даже на
  `*.github.io`). Nominatim/OSRM со страницы недоступны → поток «построить
  маршрут» в контейнере не проверить, только BLE-часть через мок.
- **Не запускай `playwright install`** — Chromium уже лежит в
  `/opt/pw-browsers/chromium`, драйвер передаёт `executablePath`.
- **GitHub Pages кеширует `index.html` ~10 минут.** На телефоне старая
  версия? Открой с `?v=N` в конце URL. Симптом устаревшей страницы:
  устройство рисует большую стрелку вместо дороги (страница шлёт пакет v1).
- `npm i` именно с `--prefix .claude/skills/run-motonav-remote` — node
  резолвит `playwright-core` от файла драйвера, а не от cwd.
- Репозиторий `StivensM21/gps` — устаревшая копия этой же страницы,
  не деплоится и не обновляется. Рабочий юнит — только этот репозиторий.

## Troubleshooting

- `Cannot find package 'playwright-core'` → не выполнен `npm i` из
  Prerequisites (или выполнен без `--prefix`).
- `curl: (22) … 403` на произвольный хост → сетевая политика контейнера,
  это не поломка; работать через моки.
