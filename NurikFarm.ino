#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <U8g2lib.h>
#include "secrets.h"

// ===================== НАСТРОЙКИ =====================
const char* WIFI_SSID = WIFI_SSID_SEC;
const char* WIFI_PASS = WIFI_PASS_SEC;
const String VK_TOKEN = VK_TOKEN_SEC;
const String GROUP_ID = GROUP_ID_SEC;
const String API_VERSION = "5.199";

// Пины
const int RELAY_PINS[6] = { 19, 18, 5, 17, 16, 4 };
const int SENSOR_PINS[6] = { 36, 39, 34, 35, 32, 33 };

// Тайминги полива
const uint32_t WATER_PULSE_MS = 4000;
const uint32_t SETTLE_MS = 120000;
const int MAX_WATER_CYCLES = 6;
const uint32_t SENSOR_SAMPLES = 7;

// Шаги настройки
const int THRESHOLD_STEP_SMALL = 10;
const int THRESHOLD_STEP_BIG = 100;
const long INTERVAL_STEP_SMALL = 600;
const long INTERVAL_STEP_BIG = 3600;

// Сеть / отказоустойчивость
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
const uint32_t WIFI_RETRY_MS = 30000;        // пауза между попытками
const uint32_t INTERNET_CHECK_MS = 3600000;  // раз в час
const uint32_t LONGPOLL_TIMEOUT_MS = 15000;
const uint32_t LP_MAX_ERRORS = 3;  // после N ошибок — полный реинит
const uint32_t WDT_TIMEOUT_S = 60;

#define SENT_HISTORY 12

// ===================== OLED =====================
#define I2C_SDA 22
#define I2C_SCL 21
U8G2_SH1106_128X32_VISIONOX_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

String logLines[4];  // кольцевой буфер лога (4 строки по 21 символу)
int logIdx = 0;
uint32_t logScreenUntilMs = 0;  // сколько показывать экран лога

// ===================== NVS =====================
Preferences prefs;
const char* NVS_NS = "poliv";

// ===================== СОСТОЯНИЕ ДАТЧИКОВ =====================
struct SensorState {
  int threshold;
  bool disabled;
  bool wateredToday;
  int lastValue;
};
SensorState sensors[6];

uint32_t pollIntervalSec = 3600;
int lastDayChecked = -1;

String lastPeerId;

// Экраны бота
String currentScreen = "main";
int currentFlower = -1;

// История отправленных (анти-эхо)
String sentHistory[SENT_HISTORY];
int sentIdx = 0;

// ===================== LONG POLL =====================
String longPollServer, longPollKey, longPollTs;
bool lpReady = false;
uint8_t lpErrorCount = 0;
uint32_t lastLpInitTryMs = 0;

// ===================== ТАЙМЕРЫ =====================
uint32_t lastPollMs = 0;
uint32_t lastNetCheckMs = 0;
uint32_t lastWifiTryMs = 0;
uint32_t lastDisplayMs = 0;

// ===================== АВТОМАТ ПОЛИВА (неблокирующий) =====================
enum WaterPhase : uint8_t {
  WP_IDLE,
  WP_PULSE_ON,
  WP_SETTLE,
  WP_FINISH_OK,
  WP_FINISH_FAIL
};
struct WaterTask {
  WaterPhase phase = WP_IDLE;
  int sensor = -1;
  int cycle = 0;
  uint32_t phaseStart = 0;
  bool manual = false;  // ручной полив из бота — не ставит авто-флаги
};
WaterTask waterTask;

// Последовательный "дневной" полив (по одному датчику за раз)
int dayResetCursor = -1;  // -1 = не активен, 0..5 = индекс датчика
bool dayResetActive = false;

// ===================== ПРОТОТИПЫ =====================
void logAdd(const String& s);
void displayStatus();
void displayRedraw();
void saveState();
void loadState();
bool connectWiFiBlocking();
void ensureNetwork();
void checkInternetHourly();
bool initLongPoll();
void doLongPoll();
void handleVkUpdate(JsonVariant v);
void processCommand(String cmd, const String& peerId);
void startWatering(int idx, bool manual);
void waterFsmTick();
void beginDayReset();
void dayResetTick();
void pollSensorsTick();
int readSensor(int idx);
int getCurrentDay();
void sendMessage(const String& peerId, const String& text, const String& keyboard = "");
void sendNotification(const String& text);
void sendMainMenu(const String& peerId);
void sendFlowerMenu(const String& peerId, int idx);
void sendSettingsMenu(const String& peerId);
String vkRequest(const String& method, const String& params);
String urlEncode(const String& str);
String makeKeyboardMain();
String makeKeyboardFlower();
String makeKeyboardSettings();
String normalizeText(const String& s);
void trackSent(const String& text);
bool wasSentByBot(const String& text);

// ===================== ЛОГ -> SERIAL =====================
void logAdd(const String& s) {
  Serial.println(s);
  logLines[logIdx] = s.substring(0, 21);
  logIdx = (logIdx + 1) % 4;
  logScreenUntilMs = millis() + 8000;  // показать лог 8 секунд
}

// ===================== ДИСПЛЕЙ =====================
bool initDisplay() {
  Wire.begin(I2C_SDA, I2C_SCL);  // SDA=22, SCL=21 — под ваш модуль
  Wire.setClock(100000);         // 100 кГц — стабильнее при длинных проводах
  display.begin();
  display.setPowerSave(0);
  display.setContrast(255);
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.setCursor(0, 10);
  display.print("Nurik Farm");
  display.setCursor(0, 25);
  display.print("Loading...");
  display.sendBuffer();
  Serial.println("OLED: SH1106 128x32 ok");
  return true;
}

void displayStatus() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_t_cyrillic);  // 5×7, кириллица
  char buf[24];

  // ===== Строка 1 (baseline y=7): Wi-Fi, время, мин с последнего опроса =====
  display.setCursor(0, 7);
  display.print(WiFi.status() == WL_CONNECTED ? "W+" : "W-");

  time_t now = time(nullptr);
  if (now > 100000) {
    struct tm* t = localtime(&now);
    snprintf(buf, sizeof(buf), " %02d:%02d", t->tm_hour, t->tm_min);
    display.print(buf);
  } else {
    display.print(" --:--");
  }

  // Время с последнего опроса датчиков (в минутах)
  uint32_t sinceMin = (millis() - lastPollMs) / 60000UL;
  if (sinceMin > 999) sinceMin = 999;  // чтобы не вылезти за ширину
  snprintf(buf, sizeof(buf), " t%03lum", (unsigned long)sinceMin);
  display.print(buf);

  // ===== Строка 2 (baseline y=17): датчики 1–3 =====
  display.setCursor(0, 17);
  for (int i = 0; i < 3; i++) {
    if (i > 0) display.print(' ');
    if (sensors[i].disabled) {
      snprintf(buf, sizeof(buf), "%d:OFF-", i + 1);
    } else {
      char mark = sensors[i].wateredToday ? '+' : '-';
      snprintf(buf, sizeof(buf), "%d:%04d%c", i + 1, sensors[i].lastValue, mark);
    }
    display.print(buf);
  }

  // ===== Строка 3 (baseline y=27): датчики 4–6 или POLIV =====
  display.setCursor(0, 27);
  if (waterTask.phase != WP_IDLE) {
    snprintf(buf, sizeof(buf), "POLIV #%d  %d/%d",
             waterTask.sensor + 1, waterTask.cycle, MAX_WATER_CYCLES);
    display.print(buf);
  } else {
    for (int i = 3; i < 6; i++) {
      if (i > 3) display.print(' ');
      if (sensors[i].disabled) {
        snprintf(buf, sizeof(buf), "%d:OFF-", i + 1);
      } else {
        char mark = sensors[i].wateredToday ? '+' : '-';
        snprintf(buf, sizeof(buf), "%d:%04d%c", i + 1, sensors[i].lastValue, mark);
      }
      display.print(buf);
    }
  }

  display.sendBuffer();
}

void displayRedraw() {
  if (millis() - lastDisplayMs < 1000) return;
  lastDisplayMs = millis();
  displayStatus();
}

// ===================== NVS: ПОЛНОЕ СОСТОЯНИЕ =====================
void loadState() {
  prefs.begin(NVS_NS, true);
  pollIntervalSec = prefs.getULong("interval", 3600);
  lastDayChecked = prefs.getInt("day", -1);
  uint8_t wf = prefs.getUChar("wflags", 0);
  uint8_t df = prefs.getUChar("dflags", 0);
  for (int i = 0; i < 6; i++) {
    String k = "thr" + String(i);
    sensors[i].threshold = prefs.getInt(k.c_str(), 3000);
    sensors[i].wateredToday = (wf >> i) & 1;
    sensors[i].disabled = (df >> i) & 1;
    sensors[i].lastValue = 0;
  }
  prefs.end();
  Serial.printf("NVS: interval=%lu day=%d wflags=%02x dflags=%02x\n",
                pollIntervalSec, lastDayChecked, wf, df);
}

void saveState() {
  prefs.begin(NVS_NS, false);
  prefs.putULong("interval", pollIntervalSec);
  prefs.putInt("day", lastDayChecked);
  uint8_t wf = 0, df = 0;
  for (int i = 0; i < 6; i++) {
    if (sensors[i].wateredToday) wf |= (1 << i);
    if (sensors[i].disabled) df |= (1 << i);
    String k = "thr" + String(i);
    prefs.putInt(k.c_str(), sensors[i].threshold);
  }
  prefs.putUChar("wflags", wf);
  prefs.putUChar("dflags", df);
  prefs.end();
}

// ===================== СЕТЬ =====================
bool connectWiFiBlocking() {
  if (WiFi.status() == WL_CONNECTED) return true;
  lastWifiTryMs = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    esp_task_wdt_reset();
  }
  bool ok = (WiFi.status() == WL_CONNECTED);
  logAdd(ok ? "WiFi подключён" : "WiFi таймаут");
  return ok;
}

void ensureNetwork() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiTryMs < WIFI_RETRY_MS) return;
  connectWiFiBlocking();
}

// Проверка реального интернета (не только Wi-Fi): DNS + TCP к API VK
bool internetAvailable() {
  esp_task_wdt_reset();
  WiFiClientSecure probe;
  probe.setInsecure();
  probe.setTimeout(5000);
  bool ok = probe.connect("api.vk.com", 443);
  probe.stop();
  esp_task_wdt_reset();
  return ok;
}

void checkInternetHourly() {
  if (millis() - lastNetCheckMs < INTERNET_CHECK_MS) return;
  lastNetCheckMs = millis();
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFiBlocking();
    return;
  }
  if (internetAvailable()) {
    Serial.println("Net check: OK");
    return;
  }
  logAdd("Интернет потерян! Переподключение");
  WiFi.disconnect(true);
  delay(500);
  if (connectWiFiBlocking()) {
    configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    initLongPoll();  // ключ мог устареть
  }
}

void syncNtpBlocking() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  time_t now = time(nullptr);
  int tries = 0;
  while (now < 100000 && tries < 40) {
    delay(500);
    now = time(nullptr);
    tries++;
    esp_task_wdt_reset();
  }
  logAdd(String("NTP: ") + ctime(&now));
}

// ===================== LONG POLL =====================
bool initLongPoll() {
  if (WiFi.status() != WL_CONNECTED) return false;
  lastLpInitTryMs = millis();
  String resp = vkRequest("messages.getLongPollServer",
                          "group_id=" + GROUP_ID + "&v=" + API_VERSION + "&lp_version=3");
  esp_task_wdt_reset();
  DynamicJsonDocument doc(2048);
  if (deserializeJson(doc, resp)) {
    lpReady = false;
    return false;
  }
  if (!doc["response"].is<JsonObject>()) {
    lpReady = false;
    return false;
  }
  longPollServer = doc["response"]["server"].as<String>();
  longPollKey = doc["response"]["key"].as<String>();
  longPollTs = doc["response"]["ts"].as<String>();
  lpReady = true;
  lpErrorCount = 0;
  Serial.println("Long Poll готов");
  return true;
}

void lpFail() {
  lpErrorCount++;
  if (lpErrorCount >= LP_MAX_ERRORS) {
    lpReady = false;
    logAdd("LP: слишком много ошибок, реинит");
  }
}

void doLongPoll() {
  if (!lpReady) {
    if (millis() - lastLpInitTryMs > 10000) initLongPoll();
    return;
  }
  if (WiFi.status() != WL_CONNECTED) return;
  esp_task_wdt_reset();
  String url = "https://" + longPollServer + "?act=a_check&key=" + longPollKey + "&ts=" + longPollTs + "&wait=3&mode=234";
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(LONGPOLL_TIMEOUT_MS);
  http.setConnectTimeout(10000);
  http.begin(client, url);
  int code = http.GET();
  esp_task_wdt_reset();
  if (code != 200) {
    if (code == -11) Serial.println("LP: timeout (-11)");  // нормально, просто пустое ожидание
    else {
      Serial.printf("LP HTTP %d\n", code);
      lpFail();
    }
    http.end();
    return;
  }
  lpErrorCount = 0;

  String payload = http.getString();
  http.end();
  esp_task_wdt_reset();
  DynamicJsonDocument doc(16384);
  if (deserializeJson(doc, payload)) {
    lpFail();
    return;
  }

  if (doc.containsKey("ts")) longPollTs = doc["ts"].as<String>();

  if (doc.containsKey("failed")) {
    Serial.printf("LP failed=%d\n", doc["failed"].as<int>());
    lpReady = false;  // ключ/тс протухли — полный реинит
    return;
  }
  if (!doc.containsKey("updates")) return;

  JsonArray updates = doc["updates"].as<JsonArray>();
  for (JsonVariant v : updates) handleVkUpdate(v);
}

// ===================== АНТИ-ЭХО =====================
String normalizeText(const String& s) {
  String out = s;
  out.trim();
  out.replace("\r", " ");
  out.replace("\n", " ");
  out.replace("<br>", " ");
  while (out.indexOf("  ") >= 0) out.replace("  ", " ");
  return out;
}
void trackSent(const String& text) {
  sentHistory[sentIdx] = normalizeText(text);
  sentIdx = (sentIdx + 1) % SENT_HISTORY;
}
bool wasSentByBot(const String& text) {
  String t = normalizeText(text);
  if (!t.length()) return false;
  for (int i = 0; i < SENT_HISTORY; i++)
    if (sentHistory[i] == t) return true;
  return false;
}

// ===================== VK API =====================
String vkRequest(const String& method, const String& params) {
  if (WiFi.status() != WL_CONNECTED) return "";
  esp_task_wdt_reset();
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(10000);
  String url = "https://api.vk.com/method/" + method + "?access_token=" + VK_TOKEN + "&" + params;
  http.begin(client, url);
  int code = http.GET();
  esp_task_wdt_reset();
  String payload = (code == 200) ? http.getString() : "";
  esp_task_wdt_reset();
  http.end();
  return payload;
}

String urlEncode(const String& str) {
  String out;
  for (size_t i = 0; i < str.length(); i++) {
    char c = str[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') out += c;
    else {
      char buf[5];
      sprintf(buf, "%%%02X", (unsigned char)c);
      out += buf;
    }
  }
  return out;
}

void sendMessage(const String& peerId, const String& text, const String& keyboard) {
  String params = "peer_id=" + peerId + "&message=" + urlEncode(text) + "&random_id=" + String(random(1, 999999)) + "&v=" + API_VERSION;
  if (keyboard.length()) params += "&keyboard=" + urlEncode(keyboard);
  vkRequest("messages.send", params);
  trackSent(text);
}

void sendNotification(const String& text) {
  logAdd(text);
  if (lastPeerId.length()) sendMessage(lastPeerId, "🔔 " + text);
}

// ===================== КЛАВИАТУРЫ =====================
String makeKeyboardMain() {
  return R"({"one_time":false,"buttons":[
    [{"action":{"type":"text","label":"Цветок #1"},"color":"primary"},{"action":{"type":"text","label":"Цветок #2"},"color":"primary"}],
    [{"action":{"type":"text","label":"Цветок #3"},"color":"primary"},{"action":{"type":"text","label":"Цветок #4"},"color":"primary"}],
    [{"action":{"type":"text","label":"Цветок #5"},"color":"primary"},{"action":{"type":"text","label":"Цветок #6"},"color":"primary"}],
    [{"action":{"type":"text","label":"Статус"},"color":"secondary"},{"action":{"type":"text","label":"Настройки"},"color":"secondary"}]
  ]})";
}
String makeKeyboardFlower() {
  return R"({"one_time":false,"buttons":[
    [{"action":{"type":"text","label":"Полить"},"color":"positive"}],
    [{"action":{"type":"text","label":"-100"},"color":"negative"},{"action":{"type":"text","label":"-10"},"color":"negative"},{"action":{"type":"text","label":"+10"},"color":"positive"},{"action":{"type":"text","label":"+100"},"color":"positive"}],
    [{"action":{"type":"text","label":"Сбросить флаг полива"},"color":"secondary"},{"action":{"type":"text","label":"Включить датчик"},"color":"positive"},{"action":{"type":"text","label":"Выключить датчик"},"color":"negative"}],
    [{"action":{"type":"text","label":"Назад"},"color":"secondary"}]
  ]})";
}
String makeKeyboardSettings() {
  return R"({"one_time":false,"buttons":[
    [{"action":{"type":"text","label":"-60 мин"},"color":"negative"},{"action":{"type":"text","label":"-10 мин"},"color":"negative"},{"action":{"type":"text","label":"+10 мин"},"color":"positive"},{"action":{"type":"text","label":"+60 мин"},"color":"positive"}],
    [{"action":{"type":"text","label":"Сбросить все флаги"},"color":"secondary"}],
    [{"action":{"type":"text","label":"Назад"},"color":"secondary"}]
  ]})";
}

// ===================== ЭКРАНЫ БОТА =====================
void sendMainMenu(const String& peerId) {
  currentScreen = "main";
  currentFlower = -1;
  sendMessage(peerId, "Управление поливом\nВыберите действие:", makeKeyboardMain());
}

void sendFlowerMenu(const String& peerId, int idx) {
  currentScreen = "flower";
  currentFlower = idx;
  String s = "Цветок #" + String(idx + 1) + "\n";
  s += "Показание: " + String(sensors[idx].lastValue) + "\n";
  s += "Порог: " + String(sensors[idx].threshold) + "\n";
  s += "Полив сегодня: " + String(sensors[idx].wateredToday ? "да" : "нет");
  if (sensors[idx].disabled) s += "\nОтключён";
  sendMessage(peerId, s, makeKeyboardFlower());
}

void sendSettingsMenu(const String& peerId) {
  currentScreen = "settings";
  currentFlower = -1;
  String s = "Настройки\n";
  s += "Интервал опроса: " + String(pollIntervalSec / 60) + " мин\n";
  sendMessage(peerId, s, makeKeyboardSettings());
}

// ===================== ДАТЧИКИ =====================
int readSensor(int idx) {
  long sum = 0;
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    sum += analogRead(SENSOR_PINS[idx]);
    delay(5);
  }
  return sum / SENSOR_SAMPLES;
}

// ===================== АВТОМАТ ПОЛИВА =====================
void startWatering(int idx, bool manual) {
  if (waterTask.phase != WP_IDLE) return;  // уже поливаем — не перебиваем
  waterTask = { WP_PULSE_ON, idx, 1, millis(), manual };
  digitalWrite(RELAY_PINS[idx], LOW);  // active-low → вкл
  logAdd("Полив #" + String(idx + 1) + " цикл 1/" + String(MAX_WATER_CYCLES));
}

void waterFsmTick() {
  switch (waterTask.phase) {
    case WP_PULSE_ON:
      if (millis() - waterTask.phaseStart >= WATER_PULSE_MS) {
        digitalWrite(RELAY_PINS[waterTask.sensor], HIGH);  // выкл
        waterTask.phase = WP_SETTLE;
        waterTask.phaseStart = millis();
      }
      break;

    case WP_SETTLE:
      if (millis() - waterTask.phaseStart >= SETTLE_MS) {
        int idx = waterTask.sensor;
        sensors[idx].lastValue = readSensor(idx);
        if (sensors[idx].lastValue <= sensors[idx].threshold) {
          waterTask.phase = WP_FINISH_OK;
        } else if (waterTask.cycle >= MAX_WATER_CYCLES) {
          waterTask.phase = WP_FINISH_FAIL;
        } else {
          waterTask.cycle++;
          digitalWrite(RELAY_PINS[idx], LOW);
          waterTask.phase = WP_PULSE_ON;
          waterTask.phaseStart = millis();
          logAdd("Полив #" + String(idx + 1) + " цикл " + String(waterTask.cycle) + "/" + String(MAX_WATER_CYCLES));
        }
      }
      break;

    case WP_FINISH_OK:
      {
        int idx = waterTask.sensor;
        digitalWrite(RELAY_PINS[idx], HIGH);
        sensors[idx].wateredToday = true;
        saveState();
        sendNotification("Цветок #" + String(idx + 1) + " полит");
        waterTask.phase = WP_IDLE;
        break;
      }

    case WP_FINISH_FAIL:
      {
        int idx = waterTask.sensor;
        digitalWrite(RELAY_PINS[idx], HIGH);
        if (!waterTask.manual) {
          sendNotification("Полив #" + String(idx + 1) + " порог не достигнут");
        } else {
          sendNotification("Ручной полив #" + String(idx + 1) + " порог не достигнут");
        }
        waterTask.phase = WP_IDLE;
        break;
      }

    default: break;
  }
}

// ===================== ОПРОС ДАТЧИКОВ =====================
void pollSensorsTick() {
  // Пока идёт полив — не вмешиваемся
  if (waterTask.phase != WP_IDLE) return;

  // Интервал ещё не истёк
  if (millis() - lastPollMs < pollIntervalSec * 1000UL) return;

  // Проходим по всем датчикам
  for (int i = 0; i < 6; i++) {
    if (sensors[i].disabled) continue;

    int val = readSensor(i);
    sensors[i].lastValue = val;
    Serial.printf("Датчик #%d: %d (порог %d)\n", i + 1, val, sensors[i].threshold);

    if (val > sensors[i].threshold) {
      startWatering(i, false);
      return;  // ← не сбрасываем lastPollMs!
               //    После полива снова зайдём сюда
               //    и проверим остальные датчики
    }
  }

  // Все шесть проверены и никому не нужен полив — вот теперь сбрасываем таймер
  lastPollMs = millis();
}

// ===================== НОВЫЙ ДЕНЬ =====================
int getCurrentDay() {
  time_t now = time(nullptr);
  return localtime(&now)->tm_yday;
}

void beginDayReset() {
  dayResetActive = true;
  dayResetCursor = 0;
  logAdd("Смена дня");
}

void dayResetTick() {
  if (!dayResetActive) return;
  if (waterTask.phase != WP_IDLE) return;  // ждём освобождения автомата
  if (dayResetCursor >= 6) {               // все обработаны
    for (int i = 0; i < 6; i++) {
      sensors[i].wateredToday = false;
    }
    lastDayChecked = getCurrentDay();
    dayResetActive = false;
    saveState();
    logAdd("Новый день: флаги полива сброшены");
    return;
  }
  int i = dayResetCursor++;
}

void checkDayTick() {
  if (dayResetActive) return;
  time_t now = time(nullptr);
  if (now < 100000) return;  // время ещё не синхронизировано
  int today = getCurrentDay();
  if (lastDayChecked == -1) {  // первый запуск: ставим день без полива
    lastDayChecked = today;
    saveState();
    return;
  }
  if (today != lastDayChecked) beginDayReset();
}

// ===================== ОБРАБОТКА СОБЫТИЙ VK =====================
void handleVkUpdate(JsonVariant v) {
  long groupIdLong = GROUP_ID.toInt();

  if (v.is<JsonObject>()) {  // новый формат
    JsonObject ev = v.as<JsonObject>();
    if (ev["type"].as<String>() != "message_new") return;
    if (!ev["object"].containsKey("message")) return;
    JsonObject msg = ev["object"]["message"];
    long fromId = msg["from_id"].as<long>();
    long peerIdL = msg["peer_id"].as<long>();
    String cmd = msg["text"].as<String>();

    if (fromId == groupIdLong || fromId == -groupIdLong) return;
    if (wasSentByBot(cmd)) return;
    if (!cmd.length()) return;

    lastPeerId = String(peerIdL);
    Serial.printf("<< %ld: %s\n", peerIdL, cmd.c_str());
    processCommand(cmd, String(peerIdL));
    return;
  }

  if (v.is<JsonArray>()) {  // старый формат
    JsonArray u = v.as<JsonArray>();
    if (u.size() < 6 || u[0].as<int>() != 4) return;
    long peerIdL = u[3].as<long>();
    String text5 = u[5].as<String>();
    String text6 = (u.size() > 6) ? u[6].as<String>() : "";
    if (wasSentByBot(text5) || wasSentByBot(text6)) return;
    String cmd = (text6.length() > 0) ? text6 : text5;
    if (!cmd.length()) return;
    lastPeerId = String(peerIdL);
    Serial.printf("<< %ld: '%s'\n", peerIdL, cmd.c_str());
    processCommand(cmd, String(peerIdL));
  }
}

// ===================== КОМАНДЫ =====================
void processCommand(String cmd, const String& peerId) {
  cmd.trim();
  if (currentScreen == "main") {
    for (int i = 0; i < 6; i++) {
      if (cmd == "Цветок #" + String(i + 1)) {
        sendFlowerMenu(peerId, i);
        return;
      }
    }
    if (cmd == "Статус") {
      String s = "Состояние:\nИнтервал: " + String(pollIntervalSec / 60) + " мин\n\n";
      for (int i = 0; i < 6; i++) {
        sensors[i].lastValue = readSensor(i);
        s += "#" + String(i + 1) + ": raw=" + String(sensors[i].lastValue) + ", порог=" + String(sensors[i].threshold);
        if (sensors[i].disabled) s += " отключён";
        if (sensors[i].wateredToday) s += " полит";
        s += "\n";
      }
      sendMessage(peerId, s, makeKeyboardMain());
      return;
    }
    if (cmd == "Настройки") {
      sendSettingsMenu(peerId);
      return;
    }
    sendMainMenu(peerId);
    return;
  }

  if (currentScreen == "flower" && currentFlower >= 0 && currentFlower < 6) {
    if (cmd == "Назад") {
      sendMainMenu(peerId);
      return;
    }
    if (cmd == "Полить") {
      if (waterTask.phase == WP_IDLE) startWatering(currentFlower, true);
      else sendNotification("Полив уже выполняется");
    }
    if (cmd == "Сбросить флаг полива") {
      sensors[currentFlower].wateredToday = false;
      saveState();
      sendNotification("Флаг полива#" + String(currentFlower + 1) + " сброшен");
    }
    if (cmd == "Включить датчик") {
      sensors[currentFlower].disabled = false;
      saveState();
      sendNotification("Датчик#" + String(currentFlower + 1) + " включен");
    }
    if (cmd == "Выключить датчик") {
      sensors[currentFlower].disabled = true;
      saveState();
      sendNotification("Датчик#" + String(currentFlower + 1) + " выключен");
    }
    if (cmd == "-100") {
      if (sensors[currentFlower].threshold - THRESHOLD_STEP_BIG > 0) {
        sensors[currentFlower].threshold = sensors[currentFlower].threshold - THRESHOLD_STEP_BIG;
        saveState();
      }
    }
    if (cmd == "-10") {
      if (sensors[currentFlower].threshold - THRESHOLD_STEP_SMALL > 0) {
        sensors[currentFlower].threshold = sensors[currentFlower].threshold - THRESHOLD_STEP_BIG;
        saveState();
      }
    }
    if (cmd == "+10") {
      if (sensors[currentFlower].threshold + THRESHOLD_STEP_SMALL > 0) {
        sensors[currentFlower].threshold = sensors[currentFlower].threshold - THRESHOLD_STEP_BIG;
        saveState();
      }
      if (cmd == "+100") {
        if (sensors[currentFlower].threshold + THRESHOLD_STEP_BIG > 0) {
          sensors[currentFlower].threshold = sensors[currentFlower].threshold - THRESHOLD_STEP_BIG;
          saveState();
        }
      }
      sendFlowerMenu(peerId, currentFlower);
      return;
    }
    }
    if (currentScreen == "settings") {
      if (cmd == "Назад") {
        sendMainMenu(peerId);
        return;
      }
      if (cmd == "Сбросить все флаги") {
        for (int i = 0; i < 6; i++) {
          sensors[i].disabled = false;
          sensors[i].wateredToday = false;
        }
        pollIntervalSec = 3600;
        saveState();
        sendNotification("Все флаги сброшены");
        sendSettingsMenu(peerId);
        return;
      }
      int delta = 0;
      if (cmd == "-60 мин") delta = -INTERVAL_STEP_BIG;
      else if (cmd == "-10 мин") delta = -INTERVAL_STEP_SMALL;
      else if (cmd == "+10 мин") delta = INTERVAL_STEP_SMALL;
      else if (cmd == "+60 мин") delta = INTERVAL_STEP_BIG;
      if (delta) {
        long v = (long)pollIntervalSec + delta;
        if (v > 0) {
          pollIntervalSec = v;
          saveState();
        }
        sendSettingsMenu(peerId);
        return;
      }
      sendSettingsMenu(peerId);
      return;
    }
    sendMainMenu(peerId);
}

  // ===================== SETUP =====================
  void setup() {
    Serial.begin(115200);
    delay(200);

    esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT_S * 1000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true
    };

    esp_err_t wdt_ret = esp_task_wdt_init(&wdt_config);
    if (wdt_ret == ESP_ERR_INVALID_STATE) {
      // Arduino Core уже инициализировал TWDT — перенастраиваем таймаут
      esp_task_wdt_reconfigure(&wdt_config);
      Serial.println("WDT: reconfigured");
    } else {
      Serial.println("WDT: initialized");
    }
    esp_task_wdt_add(NULL);

    for (int i = 0; i < 6; i++) {
      pinMode(RELAY_PINS[i], OUTPUT);
      digitalWrite(RELAY_PINS[i], HIGH);  // active-low → выкл
    }
    for (int i = 0; i < 6; i++) {
      sensors[i] = { 3000, false, false, 0 };
      pinMode(SENSOR_PINS[i], INPUT);
    }
    for (int i = 0; i < SENT_HISTORY; i++) sentHistory[i] = "";
    for (int i = 0; i < 4; i++) logLines[i] = "";

    initDisplay();
    loadState();

    connectWiFiBlocking();
    syncNtpBlocking();
    for (int i = 0; i < 6; i++) { sensors[i].lastValue = readSensor(i); }

    // Восстановление после перезагрузки: если день сменился, а аварийный полив
    // не выполнен (сбой питания) — начинаем его заново. Флаги уже в NVS.
    checkDayTick();
    initLongPoll();
    lastNetCheckMs = millis();  // первый чек через час после старта
  }

  // ===================== LOOP =====================
  void loop() {
    esp_task_wdt_reset();
    ensureNetwork();        // реконнект Wi-Fi с паузой 30 c
    checkInternetHourly();  // раз в час: реальный интернет, NTP, Long Poll
    checkDayTick();         // смена дня + аварийный полив (неблокирующе)
    dayResetTick();         // по одному датчику за раз
    pollSensorsTick();      // регулярный опрос (один полив за раз)
    waterFsmTick();         // конечный автомат полива
    doLongPoll();           // VK Long Poll (сам восстанавливается)
    displayRedraw();        // OLED: статус или лог
    delay(10);
  }