#!/usr/bin/env node
// Драйвер страницы MotoNav: headless Chromium + мок Web Bluetooth.
// Проходит поток «Подключить устройство → панель устройства → применить
// настройки», проверяет отрисовку и байты, ушедшие в характеристику CFG,
// затем снимает скриншот. Код выхода 0 = все проверки прошли.
import { chromium } from 'playwright-core';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '../../..');
const shot = process.env.MOTONAV_SS || path.join(process.cwd(), 'motonav-page.png');

const fails = [];
const ok = (name, cond) => { console.log((cond ? 'PASS ' : 'FAIL ') + name); if (!cond) fails.push(name); };

const browser = await chromium.launch({ executablePath: '/opt/pw-browsers/chromium' });
const page = await browser.newPage({ viewport: { width: 420, height: 900 } });
page.on('pageerror', e => { console.log('PAGEERROR ' + e.message); fails.push('pageerror'); });

// Мок Web Bluetooth: «устройство» с характеристиками NAV/STATE/CFG из PROTOCOL.md.
// Всё, что страница пишет в устройство, складывается в window.__ble.
await page.addInitScript(() => {
  const written = { cfg: null, nav: [] };
  window.__ble = written;
  const dv = a => new DataView(new Uint8Array(a).buffer);
  const bytes = b => b instanceof ArrayBuffer
    ? [...new Uint8Array(b)]
    : [...new Uint8Array(b.buffer, b.byteOffset, b.byteLength)];
  const chr = uuid => ({
    uuid,
    readValue: async () =>
      String(uuid).startsWith('6e6f7602') ? dv([1, 77, 1, 0x7c, 0x0f]) // STATE: 77%, заряжается, 3964 мВ
                                          : dv([1, 5, 60]),            // CFG: сон 5 мин, яркость 60
    startNotifications: async () => {},
    addEventListener: () => {},
    writeValue: async b => { written.cfg = bytes(b); },
    writeValueWithoutResponse: async b => { written.nav.push(bytes(b)); },
  });
  const service = { getCharacteristic: async u => chr(u) };
  navigator.bluetooth = {
    requestDevice: async () => ({
      name: 'MotoNav',
      addEventListener: () => {},
      gatt: { connect: async () => ({ getPrimaryService: async () => service }) },
    }),
  };
});

await page.goto('file://' + path.join(repo, 'index.html'));
ok('страница загрузилась', (await page.title()) === 'MotoNav — пульт');

await page.click('#connect');
await page.waitForSelector('#devCard', { state: 'visible', timeout: 3000 }).catch(() => {});
ok('панель устройства появилась',
   await page.$eval('#devCard', el => getComputedStyle(el).display !== 'none').catch(() => false));
ok('батарея из STATE отрисована', (await page.$eval('#dBatt', el => el.textContent)) === '77%');
ok('статус зарядки виден', (await page.$eval('#dBattSub', el => el.textContent)).includes('заряжается'));
ok('вольтаж отрисован', (await page.$eval('#dVolt', el => el.textContent)) === '3.96 В');
ok('настройки прочитаны из CFG',
   (await page.$eval('#cfgSleep', el => el.value)) === '5' &&
   (await page.$eval('#cfgBri', el => el.value)) === '60');

// меняем настройки и жмём «Применить» — страница должна записать [1, сон, яркость]
await page.selectOption('#cfgSleep', '2');
await page.$eval('#cfgBri', el => { el.value = 40; el.oninput(); });
await page.click('#cfgSave');
await page.waitForFunction(() => window.__ble.cfg !== null, { timeout: 3000 }).catch(() => {});
const cfg = await page.evaluate(() => window.__ble.cfg);
ok('в CFG ушло [1,2,40], получили [' + cfg + ']', JSON.stringify(cfg) === '[1,2,40]');

// чистая математика страницы
const bN = await page.evaluate(() => bearingTo(52.0, 4.0, 53.0, 4.0));
const bE = await page.evaluate(() => bearingTo(52.0, 4.0, 52.0, 5.0));
ok('bearingTo: север ~0°, восток ~90°', Math.abs(bN) < 1 && Math.abs(bE - 90) < 2);

await page.screenshot({ path: shot, fullPage: true });
console.log('screenshot: ' + shot);
await browser.close();
process.exit(fails.length ? 1 : 0);
