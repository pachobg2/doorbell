/*
 * doorbell — ESP32-C3-Zero MQTT doorbell
 * Raw Arduino/C++, espMqttClient, MQTT QoS 1, HA auto-discovery
 *
 * Mirrors smart_switch v2.0.0's architecture: espMqttClient, retained
 * discovery configs, LWT availability, diagnostic entities, runtime
 * WiFiManager setup portal (NVS-persisted settings, no compiled-in
 * credentials), ArduinoOTA. Not battery-powered for now, so there's no
 * deep sleep — it stays connected continuously.
 *
 * The doorbell switch (GPIO0, one leg to GND) doubles as the setup
 * control, same button-reuse pattern smart_switch/TH_2_v4 use:
 *   - Debounced press: fires the doorbell event immediately (on press,
 *     not release — see handleButton() for why this differs from
 *     smart_switch's release-triggered relay toggle).
 *   - Held BUTTON_SETUP_HOLD_MS (10s): opens the setup portal immediately
 *     (doesn't wait for release).
 *   - Held FACTORY_RESET_HOLD_MS (5s) again while the portal is open:
 *     wipes all saved settings and restarts unconfigured. A quick press
 *     instead cancels the portal.
 *   - A never-configured device goes straight to the portal on boot.
 * No button-hold OTA gesture is needed (unlike TH_2_v4, which is
 * normally asleep) — ArduinoOTA already runs continuously in loop() on
 * this always-on device.
 *
 * The doorbell press itself is published as a Home Assistant MQTT
 * `event` entity (device_class "doorbell") rather than a momentary
 * binary_sensor — semantically correct for a stateless trigger, and a
 * new pattern for this fleet (see publishDiscovery()).
 *
 * Diagnostics: WiFi signal, reset reason, boot count, connect-fail count
 * (resets on next successful connect), total-fail count (lifetime,
 * NVS-persisted), and firmware version — same set as smart_switch v2.0.0,
 * minus anything battery-specific.
 *
 * Hardware:
 *   GPIO0  - doorbell switch, one leg to GND, INPUT_PULLUP (active low).
 *            NOTE: on the *original* ESP32, GPIO0 is a boot-mode
 *            strapping pin and wiring a switch there is risky. On the
 *            ESP32-C3 the strapping pins are GPIO2/GPIO8/GPIO9 instead --
 *            GPIO0 is a normal GPIO on this chip, so this wiring is fine
 *            here even though it wouldn't be on a classic ESP32 board.
 *   GPIO10 - single WS2812 status LED (same pin as smart_switch)
 *
 * Libraries needed (Library Manager / PlatformIO):
 *   espMqttClient   (bertmelis/espMqttClient)
 *   Adafruit NeoPixel
 *   WiFiManager     (tzapu/WiFiManager)
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <espMqttClient.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <Preferences.h>
#include <vector>
// OTA password, setup-portal AP password/timeout, device identity/firmware
// version, and button-hold thresholds. WiFi/MQTT credentials are NOT here
// -- they're runtime settings, see Settings below.
#include "config.h"

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
static const uint8_t BUTTON_PIN = 0;  // doorbell switch, also doubles as the setup control
static const uint8_t LED_PIN    = 10;
static const uint8_t LED_COUNT  = 1;

// Default LED brightness as a percentage (0-100), used until a value is
// loaded from NVS or set via MQTT/HA.
static const uint8_t DEFAULT_LED_BRIGHTNESS_PCT = 50;

// ---------------------------------------------------------------------------
// Runtime settings (WiFi/MQTT/identity, via the setup portal)
// ---------------------------------------------------------------------------
// Nothing here is compiled in -- read from NVS at boot (defaults to
// unconfigured) and only ever written by runMaintenanceMode() after a
// portal save, or wiped by a button-hold factory reset. Declared before
// buildTopics() below since that reads settings.deviceId.
struct Settings {
  String wifiSsid;
  String wifiPassword;
  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser;
  String mqttPassword;
  String deviceId;   // used in MQTT topics/unique_ids -- keep stable once this device exists in HA
  String deviceName; // friendly name shown in Home Assistant
  bool configured = false;
};
Settings settings;
Preferences settingsPrefs;

String getShortChipId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[7];
  snprintf(buf, sizeof(buf), "%06X", (uint32_t)(mac & 0xFFFFFFULL));
  return String(buf);
}

void loadSettings() {
  String chipId = getShortChipId();
  settingsPrefs.begin("settings", true); // read-only
  settings.configured   = settingsPrefs.getBool("configured", false);
  settings.wifiSsid     = settingsPrefs.getString("wifiSsid", "");
  settings.wifiPassword = settingsPrefs.getString("wifiPass", "");
  settings.mqttHost     = settingsPrefs.getString("mqttHost", "");
  settings.mqttPort     = settingsPrefs.getUShort("mqttPort", 1883);
  settings.mqttUser     = settingsPrefs.getString("mqttUser", "");
  settings.mqttPassword = settingsPrefs.getString("mqttPass", "");
  settings.deviceId     = settingsPrefs.getString("deviceId", "doorbell_" + chipId);
  settings.deviceName   = settingsPrefs.getString("deviceName", "Doorbell " + chipId);
  settingsPrefs.end();
}

void saveSettings() {
  settingsPrefs.begin("settings", false);
  settingsPrefs.putBool("configured", settings.configured);
  settingsPrefs.putString("wifiSsid", settings.wifiSsid);
  settingsPrefs.putString("wifiPass", settings.wifiPassword);
  settingsPrefs.putString("mqttHost", settings.mqttHost);
  settingsPrefs.putUShort("mqttPort", settings.mqttPort);
  settingsPrefs.putString("mqttUser", settings.mqttUser);
  settingsPrefs.putString("mqttPass", settings.mqttPassword);
  settingsPrefs.putString("deviceId", settings.deviceId);
  settingsPrefs.putString("deviceName", settings.deviceName);
  settingsPrefs.end();
}

// Boot count / lifetime fail count: NVS-persisted, not RTC memory -- this
// device doesn't deep-sleep, so a "boot" is a rare, meaningful event
// (actual power cycle, crash, or OTA restart), and there's no flash-wear
// concern from writing on every one of those.
uint32_t bootCount = 0;
uint32_t totalFailCount = 0; // lifetime MQTT disconnects, never resets

void loadCounters() {
  settingsPrefs.begin("settings", true);
  bootCount = settingsPrefs.getUInt("bootCount", 0);
  totalFailCount = settingsPrefs.getUInt("totalFail", 0);
  settingsPrefs.end();
}

void incrementBootCount() {
  bootCount++;
  settingsPrefs.begin("settings", false);
  settingsPrefs.putUInt("bootCount", bootCount);
  settingsPrefs.end();
}

void incrementTotalFailCount() {
  totalFailCount++;
  settingsPrefs.begin("settings", false);
  settingsPrefs.putUInt("totalFail", totalFailCount);
  settingsPrefs.end();
}

// ---------------------------------------------------------------------------
// MQTT topics -- built at runtime from settings.deviceId, not compiled in
// ---------------------------------------------------------------------------
String baseTopic, doorbellEventTopic, availabilityTopic,
       wifiSignalTopic, resetReasonTopic,
       connectFailCountTopic, totalFailCountTopic, bootCountTopic,
       firmwareVersionTopic, uptimeTopic, ledBrightnessStateTopic, ledBrightnessCommandTopic,
       otaRestartCommandTopic;

String discoveryDoorbellTopic, discoveryWifiSignalTopic,
       discoveryResetReasonTopic, discoveryConnectFailCountTopic,
       discoveryTotalFailCountTopic, discoveryBootCountTopic,
       discoveryFirmwareVersionTopic, discoveryUptimeTopic, discoveryLedBrightnessTopic,
       discoveryOtaRestartTopic;

void buildTopics() {
  baseTopic = String("doorbell/") + settings.deviceId;
  doorbellEventTopic = baseTopic + "/event";
  availabilityTopic  = baseTopic + "/availability";
  wifiSignalTopic    = baseTopic + "/wifi_signal/state";
  resetReasonTopic   = baseTopic + "/reset_reason/state";
  connectFailCountTopic = baseTopic + "/connect_fail_count/state";
  totalFailCountTopic   = baseTopic + "/total_fail_count/state";
  bootCountTopic        = baseTopic + "/boot_count/state";
  firmwareVersionTopic  = baseTopic + "/firmware_version/state";
  uptimeTopic           = baseTopic + "/uptime/state";
  ledBrightnessStateTopic   = baseTopic + "/led_brightness/state";
  ledBrightnessCommandTopic = baseTopic + "/led_brightness/set";
  otaRestartCommandTopic    = baseTopic + "/ota_restart/set";

  discoveryDoorbellTopic     = String("homeassistant/event/") + settings.deviceId + "/doorbell/config";
  discoveryWifiSignalTopic   = String("homeassistant/sensor/") + settings.deviceId + "/wifi_signal/config";
  discoveryResetReasonTopic  = String("homeassistant/sensor/") + settings.deviceId + "/reset_reason/config";
  discoveryConnectFailCountTopic = String("homeassistant/sensor/") + settings.deviceId + "/connect_fail_count/config";
  discoveryTotalFailCountTopic   = String("homeassistant/sensor/") + settings.deviceId + "/total_fail_count/config";
  discoveryBootCountTopic        = String("homeassistant/sensor/") + settings.deviceId + "/boot_count/config";
  discoveryFirmwareVersionTopic  = String("homeassistant/sensor/") + settings.deviceId + "/firmware_version/config";
  discoveryUptimeTopic           = String("homeassistant/sensor/") + settings.deviceId + "/uptime/config";
  discoveryLedBrightnessTopic = String("homeassistant/number/") + settings.deviceId + "/led_brightness/config";
  discoveryOtaRestartTopic    = String("homeassistant/button/") + settings.deviceId + "/ota_restart/config";
}

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
espMqttClient mqttClient;
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Preferences prefs;

uint8_t ledBrightnessPct = DEFAULT_LED_BRIGHTNESS_PCT; // 0-100, persisted in NVS

// button debounce + hold tracking
bool lastButtonReading = HIGH;
bool buttonStable = HIGH;
unsigned long lastButtonChangeMs = 0;
static const unsigned long DEBOUNCE_MS = 20;
unsigned long pressStartMs = 0;  // 0 == not currently tracking a press
bool setupTriggered = false;     // true once the current press has already opened the portal

// mqtt reconnect / diagnostics
unsigned long lastMqttAttemptMs = 0;
unsigned long mqttBackoffMs = 1000;
static const unsigned long MQTT_BACKOFF_MAX_MS = 30000;
uint32_t connectFailCount = 0; // resets to 0 on next successful connect
bool everConnected = false;

unsigned long lastWifiSignalPublishMs = 0;
static const unsigned long WIFI_SIGNAL_INTERVAL_MS = 120000; // 2 min

bool bootAnimationDone = false;

// wifi reconnect state (non-blocking)
bool wifiConnectInProgress = false;
unsigned long wifiConnectStartMs = 0;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;

// setup-portal LED pulse (used during a hold and while the portal's open)
static const unsigned long SETUP_LED_BLINK_PERIOD_MS = 1000;
static const unsigned long SETUP_LED_PULSE_MS = 150;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void connectWiFi();
void pollWiFi();
bool checkedPublish(const String &topic, uint8_t qos, bool retain, const String &payload);
void connectMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason);
void onMqttMessage(const espMqttClientTypes::MessageProperties& properties,
                    const char* topic, const uint8_t* payload, size_t len,
                    size_t index, size_t total);
void publishDiscovery();
void fireDoorbellEvent();
void handleButton();
void updateStatusLed();
void runBootAnimation();
void publishDiagnostics(bool force);
String resetReasonString();
void setLedBrightness(uint8_t pct, bool save, bool publish);
void publishLedBrightness();
void setLedColor(uint8_t r, uint8_t g, uint8_t b);
void runMaintenanceMode(bool viaButton);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  prefs.begin("doorbell", false);
  ledBrightnessPct = prefs.getUChar("led_bright", DEFAULT_LED_BRIGHTNESS_PCT);
  if (ledBrightnessPct > 100) ledBrightnessPct = DEFAULT_LED_BRIGHTNESS_PCT;

  led.begin();
  led.setBrightness(map(ledBrightnessPct, 0, 100, 0, 255));
  led.clear();
  led.show();

  loadSettings();
  loadCounters();
  buildTopics();

  // A never-configured device goes straight to the portal -- no button
  // hold needed, same as door_sensor/TH_2_v4/smart_switch.
  if (!settings.configured) {
    runMaintenanceMode(false);
    // Only reached if the portal timed out / failed -- nothing saved yet,
    // so just keep retrying instead of falling through to a normal cycle
    // with empty WiFi/MQTT settings.
    Serial.println("[setup] Still unconfigured after portal timeout -- retrying.");
    delay(2000);
    ESP.restart();
  }

  incrementBootCount();

  connectWiFi();

  // One-time bounded wait at boot only — gives ArduinoOTA's mDNS responder
  // and the first MQTT attempt a real chance at a live link. This never
  // recurs after setup(), so it never blocks the button during normal
  // operation the way a loop()-level blocking wait would.
  {
    unsigned long waitStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - waitStart < 5000) {
      delay(100);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnectInProgress = false;
    Serial.printf("[wifi] connected, ip=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[wifi] not yet connected at boot, will keep retrying in loop()");
  }

  mqttClient.setServer(settings.mqttHost.c_str(), settings.mqttPort);
  mqttClient.setCredentials(settings.mqttUser.c_str(), settings.mqttPassword.c_str());
  mqttClient.setClientId(settings.deviceId.c_str());
  mqttClient.setWill(availabilityTopic.c_str(), 1, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  connectMqtt();

  ArduinoOTA.setHostname(settings.deviceId.c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
  pollWiFi();

  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttAttemptMs >= mqttBackoffMs) {
      connectMqtt();
    }
  }

  mqttClient.loop();
  ArduinoOTA.handle();

  handleButton();

  if (!bootAnimationDone && mqttClient.connected()) {
    runBootAnimation();
    bootAnimationDone = true;
  }

  unsigned long now = millis();
  if (now - lastWifiSignalPublishMs >= WIFI_SIGNAL_INTERVAL_MS) {
    lastWifiSignalPublishMs = now;
    publishDiagnostics(false);
  }
}

// ---------------------------------------------------------------------------
// MQTT publish helper — logs a failure instead of silently dropping it.
// mqttClient.publish() returns 0 on failure (e.g. espMqttClient's internal
// low-memory guard, or not connected).
// ---------------------------------------------------------------------------
bool checkedPublish(const String &topic, uint8_t qos, bool retain, const String &payload) {
  uint16_t packetId = mqttClient.publish(topic.c_str(), qos, retain, payload.c_str());
  if (packetId == 0) {
    Serial.print("[mqtt] publish FAILED topic=");
    Serial.print(topic);
    Serial.print(" free_heap=");
    Serial.println(ESP.getFreeHeap());
  }
  return packetId != 0;
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED || wifiConnectInProgress) return;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(settings.wifiSsid.c_str(), settings.wifiPassword.c_str());
  wifiConnectInProgress = true;
  wifiConnectStartMs = millis();
  Serial.println("[wifi] connecting...");
}

// Non-blocking — call every loop() iteration.
void pollWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiConnectInProgress) {
      wifiConnectInProgress = false;
      Serial.printf("[wifi] connected, ip=%s\n", WiFi.localIP().toString().c_str());
    }
    return;
  }

  if (!wifiConnectInProgress) {
    connectWiFi();
  } else if (millis() - wifiConnectStartMs > WIFI_CONNECT_TIMEOUT_MS) {
    Serial.println("[wifi] connect attempt timed out, will retry");
    wifiConnectInProgress = false;
  }
}

// ---------------------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------------------
void connectMqtt() {
  lastMqttAttemptMs = millis();
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("MQTT connected");
  mqttBackoffMs = 1000;
  everConnected = true;
  connectFailCount = 0;

  checkedPublish(availabilityTopic, 1, true, "online");

  mqttClient.subscribe(ledBrightnessCommandTopic.c_str(), 1);
  mqttClient.subscribe(otaRestartCommandTopic.c_str(), 1);

  publishDiscovery();
  publishLedBrightness();
  updateStatusLed();
  publishDiagnostics(true);
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  Serial.printf("MQTT disconnected, reason: %u\n", static_cast<uint8_t>(reason));
  if (everConnected) {
    connectFailCount++;
    incrementTotalFailCount();
  }
  mqttBackoffMs = min(mqttBackoffMs * 2, MQTT_BACKOFF_MAX_MS);
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties,
                    const char* topic, const uint8_t* payload, size_t len,
                    size_t index, size_t total) {
  String topicStr(topic);
  String payloadStr;
  payloadStr.reserve(len);
  for (size_t i = 0; i < len; i++) payloadStr += (char)payload[i];

  if (topicStr == ledBrightnessCommandTopic) {
    int pct = payloadStr.toInt();
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    setLedBrightness((uint8_t)pct, true, true);
  } else if (topicStr == otaRestartCommandTopic) {
    Serial.println("OTA restart requested via MQTT, rebooting...");
    checkedPublish(availabilityTopic, 1, true, "offline");
    delay(200);
    ESP.restart();
  }
}

// ---------------------------------------------------------------------------
// Home Assistant discovery
// ---------------------------------------------------------------------------
void publishDiscovery() {
  String deviceJson = String("{") +
      "\"identifiers\":[\"" + settings.deviceId + "\"]," +
      "\"name\":\"" + settings.deviceName + "\"," +
      "\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\"," +
      "\"model\":\"" + DEVICE_MODEL + "\"," +
      "\"hw_version\":\"" + DEVICE_HW_VERSION + "\"," +
      "\"sw_version\":\"" + FIRMWARE_VERSION + "\"" +
      "}";

  // Doorbell press (event entity -- stateless trigger, not a binary_sensor)
  {
    String payload = String("{") +
        "\"name\":\"Doorbell\"," +
        "\"unique_id\":\"" + settings.deviceId + "_doorbell\"," +
        "\"device_class\":\"doorbell\"," +
        "\"event_types\":[\"press\"]," +
        "\"state_topic\":\"" + doorbellEventTopic + "\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryDoorbellTopic, 1, true, payload);
  }

  // WiFi signal (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"WiFi Signal\"," +
        "\"unique_id\":\"" + settings.deviceId + "_wifi_signal\"," +
        "\"state_topic\":\"" + wifiSignalTopic + "\"," +
        "\"unit_of_measurement\":\"dBm\"," +
        "\"device_class\":\"signal_strength\"," +
        "\"state_class\":\"measurement\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryWifiSignalTopic, 1, true, payload);
  }

  // Reset reason (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"Reset Reason\"," +
        "\"unique_id\":\"" + settings.deviceId + "_reset_reason\"," +
        "\"state_topic\":\"" + resetReasonTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryResetReasonTopic, 1, true, payload);
  }

  // Boot count (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"Boot Count\"," +
        "\"unique_id\":\"" + settings.deviceId + "_boot_count\"," +
        "\"state_topic\":\"" + bootCountTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"state_class\":\"total_increasing\"," +
        "\"icon\":\"mdi:counter\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryBootCountTopic, 1, true, payload);
  }

  // Connect fail count (diagnostic) -- resets to 0 on next successful
  // connect, so "measurement" not "total_increasing".
  {
    String payload = String("{") +
        "\"name\":\"Connect Fail Count\"," +
        "\"unique_id\":\"" + settings.deviceId + "_connect_fail_count\"," +
        "\"state_topic\":\"" + connectFailCountTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"state_class\":\"measurement\"," +
        "\"icon\":\"mdi:wifi-alert\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryConnectFailCountTopic, 1, true, payload);
  }

  // Total fail count (diagnostic) -- lifetime, never resets
  {
    String payload = String("{") +
        "\"name\":\"Total Fail Count\"," +
        "\"unique_id\":\"" + settings.deviceId + "_total_fail_count\"," +
        "\"state_topic\":\"" + totalFailCountTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"state_class\":\"total_increasing\"," +
        "\"icon\":\"mdi:counter\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryTotalFailCountTopic, 1, true, payload);
  }

  // Firmware version (diagnostic)
  {
    String payload = String("{") +
        "\"name\":\"Firmware Version\"," +
        "\"unique_id\":\"" + settings.deviceId + "_firmware_version\"," +
        "\"state_topic\":\"" + firmwareVersionTopic + "\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryFirmwareVersionTopic, 1, true, payload);
  }

  // Uptime (diagnostic) -- seconds since this boot; zeroes on any reset or
  // power loss (see uptimeSeconds())
  {
    String payload = String("{") +
        "\"name\":\"Uptime\"," +
        "\"unique_id\":\"" + settings.deviceId + "_uptime\"," +
        "\"state_topic\":\"" + uptimeTopic + "\"," +
        "\"unit_of_measurement\":\"s\"," +
        "\"device_class\":\"duration\"," +
        "\"state_class\":\"measurement\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryUptimeTopic, 1, true, payload);
  }

  // LED brightness (number entity, global brightness control)
  {
    String payload = String("{") +
        "\"name\":\"LED Brightness\"," +
        "\"unique_id\":\"" + settings.deviceId + "_led_brightness\"," +
        "\"state_topic\":\"" + ledBrightnessStateTopic + "\"," +
        "\"command_topic\":\"" + ledBrightnessCommandTopic + "\"," +
        "\"min\":0," +
        "\"max\":100," +
        "\"step\":1," +
        "\"unit_of_measurement\":\"%\"," +
        "\"icon\":\"mdi:brightness-percent\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryLedBrightnessTopic, 1, true, payload);
  }

  // OTA restart button (diagnostic) — reboots cleanly before an OTA push
  {
    String payload = String("{") +
        "\"name\":\"OTA Restart\"," +
        "\"unique_id\":\"" + settings.deviceId + "_ota_restart\"," +
        "\"command_topic\":\"" + otaRestartCommandTopic + "\"," +
        "\"payload_press\":\"PRESS\"," +
        "\"device_class\":\"restart\"," +
        "\"entity_category\":\"diagnostic\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryOtaRestartTopic, 1, true, payload);
  }
}

// ---------------------------------------------------------------------------
// Doorbell event -- QoS 1, NOT retained (a stateless event shouldn't replay
// a stale "someone rang" on every HA restart the way a retained topic would).
// ---------------------------------------------------------------------------
void fireDoorbellEvent() {
  if (mqttClient.connected()) {
    checkedPublish(doorbellEventTopic, 1, false, "{\"event_type\":\"press\"}");
  }
  // Brief white flash acknowledges the press regardless of MQTT state --
  // blocking delay is fine here, same tolerance as runBootAnimation().
  setLedColor(255, 255, 255);
  delay(150);
  updateStatusLed();
}

// ---------------------------------------------------------------------------
// Button (GPIO0, active low, debounced) -- doubles as the setup control.
//
// Fires on a debounced PRESS, not release -- unlike smart_switch's relay
// toggle, a doorbell should ring the instant it's pressed. This means a
// press that turns into a long setup-mode hold will still have rung once
// first; harmless for a doorbell (no lasting side effect), unlike a relay
// toggle would be.
//
// Held past BUTTON_SETUP_HOLD_MS: commits immediately (without waiting
// for release) to opening the setup portal instead. See
// runMaintenanceMode() for the in-portal factory-reset gesture.
// ---------------------------------------------------------------------------
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastButtonChangeMs = millis();
  }

  if (millis() - lastButtonChangeMs > DEBOUNCE_MS && reading != buttonStable) {
    buttonStable = reading;

    // active low: pressed == LOW
    bool pressed = (buttonStable == LOW);

    if (pressed) {
      pressStartMs = millis();
      setupTriggered = false;
      fireDoorbellEvent();
    } else {
      pressStartMs = 0;
    }
  }

  // Long-hold detection, independent of the debounce-driven edges above --
  // has to fire the instant the threshold is crossed, not wait for release.
  if (pressStartMs != 0 && !setupTriggered && buttonStable == LOW &&
      millis() - pressStartMs >= BUTTON_SETUP_HOLD_MS) {
    setupTriggered = true;
    Serial.println("[button] held past setup threshold -- entering Setup Mode.");
    runMaintenanceMode(true);
    // runMaintenanceMode() only returns on portal timeout/cancel; resync
    // debounce state in case the button is still held, so it doesn't
    // immediately retrigger.
    lastButtonReading = digitalRead(BUTTON_PIN);
    buttonStable = lastButtonReading;
    pressStartMs = 0;
    setupTriggered = false;
  }

  lastButtonReading = reading;
}

// ---------------------------------------------------------------------------
// LED status
// ---------------------------------------------------------------------------
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void updateStatusLed() {
  if (WiFi.status() == WL_CONNECTED) {
    setLedColor(0, 255, 0); // green, idle, WiFi connected
  } else {
    setLedColor(0, 0, 0); // off, no WiFi
  }
}

void setLedBrightness(uint8_t pct, bool save, bool publish) {
  ledBrightnessPct = pct;
  led.setBrightness(map(ledBrightnessPct, 0, 100, 0, 255));
  led.show();

  if (save) {
    prefs.putUChar("led_bright", ledBrightnessPct);
  }
  if (publish) {
    publishLedBrightness();
  }
}

void publishLedBrightness() {
  if (!mqttClient.connected()) return;
  checkedPublish(ledBrightnessStateTopic, 1, true, String(ledBrightnessPct));
}

void runBootAnimation() {
  for (int i = 0; i < 3; i++) {
    setLedColor(0, 0, 255); // blue
    delay(700);
    setLedColor(0, 0, 0);
    delay(700);
  }
  updateStatusLed();
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
String resetReasonString() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON:   return "Power on";
    case ESP_RST_EXT:       return "External pin";
    case ESP_RST_SW:        return "Software reset";
    case ESP_RST_PANIC:     return "Panic/exception";
    case ESP_RST_INT_WDT:   return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "Task watchdog";
    case ESP_RST_WDT:       return "Other watchdog";
    case ESP_RST_BROWNOUT:  return "Brownout";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "Unknown";
  }
}

// Seconds since this boot. esp_timer_get_time() is 64-bit microseconds since
// boot, so unlike millis() it doesn't wrap back to zero at ~49.7 days --
// uptime should only ever zero on a real reset or power loss.
uint32_t uptimeSeconds() {
  return (uint32_t)(esp_timer_get_time() / 1000000LL);
}

void publishDiagnostics(bool force) {
  if (!mqttClient.connected()) return;

  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();
    checkedPublish(wifiSignalTopic, 1, true, String(rssi));
  }

  static bool resetReasonSent = false;
  if (force || !resetReasonSent) {
    checkedPublish(resetReasonTopic, 1, true, resetReasonString());
    resetReasonSent = true;
  }

  checkedPublish(bootCountTopic, 1, true, String(bootCount));
  checkedPublish(connectFailCountTopic, 1, true, String(connectFailCount));
  checkedPublish(totalFailCountTopic, 1, true, String(totalFailCount));
  checkedPublish(firmwareVersionTopic, 1, true, String(FIRMWARE_VERSION));
  checkedPublish(uptimeTopic, 1, true, String(uptimeSeconds()));
}

// ---------------------------------------------------------------------------
// Setup portal
//
// Entered when the button is held past BUTTON_SETUP_HOLD_MS, or when this
// device has never been configured yet (settings.configured == false).
// Broadcasts "<DEVICE_MANUFACTURER> <DEVICE_MODEL> XXXX" (last 4 hex chars
// of the chip MAC), and serves a page (WiFiManager) with a WiFi picker plus
// custom fields for MQTT and device identity -- one form, one Save. Holding
// the button again for FACTORY_RESET_HOLD_MS while this page is open wipes
// the device back to a fully unconfigured state instead. If the portal
// succeeds, settings are saved and the device restarts. If it times out or
// is cancelled, this returns and the caller resumes with whatever settings
// already existed (unchanged).
// ---------------------------------------------------------------------------
void runMaintenanceMode(bool viaButton) {
  Serial.println(viaButton
    ? "Button held >=10s -- entering Setup Mode."
    : "No saved WiFi config yet -- entering first-time setup.");

  // Clean disconnect before handing the radio to WiFiManager, in case this
  // was reached mid-operation (viaButton) on an already-connected device.
  if (mqttClient.connected()) {
    checkedPublish(availabilityTopic, 1, true, "offline");
    mqttClient.disconnect();
    delay(200);
  }
  WiFi.disconnect();

  String wifiStatusStr = settings.configured
    ? (settings.wifiSsid.length() ? ("last connected: " + settings.wifiSsid) : String("no WiFi saved yet"))
    : String("not yet configured");
  String mqttStatusStr = (settings.configured && settings.mqttHost.length())
    ? (settings.mqttHost + ":" + String(settings.mqttPort))
    : String("not yet configured");
  String statusHtml = String("<div style='background:#f4f4f4;border-radius:6px;padding:10px;margin:10px 0;font-size:0.9em;'>")
    + "<strong>Device status</strong><br>"
    + "WiFi: " + wifiStatusStr + "<br>"
    + "MQTT broker: " + mqttStatusStr + "<br>"
    + "Boot count: " + String(bootCount) + " &middot; connect fails: " + String(connectFailCount)
    + " this run / " + String(totalFailCount) + " total"
    + "</div>";

  char mqttPortStr[6];
  snprintf(mqttPortStr, sizeof(mqttPortStr), "%u", settings.mqttPort);

  WiFiManagerParameter p_mqtt_heading(
    "<p style='margin-bottom:0;'><strong>MQTT &amp; device settings</strong><br>"
    "(same form as the WiFi network above -- fill in both, then Save once)</p>");
  WiFiManagerParameter p_mqtt_host("mqtt_host", "MQTT broker host or IP", settings.mqttHost.c_str(), 64, "required");
  WiFiManagerParameter p_mqtt_port("mqtt_port", "MQTT broker port", mqttPortStr, 6);
  WiFiManagerParameter p_mqtt_user("mqtt_user", "MQTT username", settings.mqttUser.c_str(), 32);
  WiFiManagerParameter p_mqtt_pass("mqtt_pass", "MQTT password", settings.mqttPassword.c_str(), 32, "type='password'");
  WiFiManagerParameter p_device_name("device_name", "Device name (shown in Home Assistant)", settings.deviceName.c_str(), 40);
  WiFiManagerParameter p_device_id("device_id", "Device ID (MQTT topics, no spaces)", settings.deviceId.c_str(), 32);

  WiFiManager wm;
  String versionHeader = "<style>body::before{content:'" + String(DEVICE_MANUFACTURER) + " " + String(DEVICE_MODEL)
                          + "\\A Firmware v" + String(FIRMWARE_VERSION)
                          + "';white-space:pre-line;display:block;text-align:center;color:#888;margin:4px 0;}"
                          + "form[action='/wifi'] button{font-size:0;}"
                          + "form[action='/wifi'] button::after{content:'Configure';font-size:1rem;}"
                          + "h1,h3{display:none;}</style>";
  wm.setCustomHeadElement(versionHeader.c_str());
  wm.addParameter(&p_mqtt_heading);
  wm.addParameter(&p_mqtt_host);
  wm.addParameter(&p_mqtt_port);
  wm.addParameter(&p_mqtt_user);
  wm.addParameter(&p_mqtt_pass);
  wm.addParameter(&p_device_name);
  wm.addParameter(&p_device_id);
  wm.setConfigPortalTimeout(PORTAL_TIMEOUT_SEC);
  std::vector<const char*> menu = {"custom", "wifi", "sep", "restart", "exit"};
  wm.setMenu(menu);
  wm.setCustomMenuHTML(statusHtml.c_str());
  wm.setCaptivePortalEnable(true);

  const char* apPassword = AP_PASSWORD;
  size_t apPasswordLen = strlen(apPassword);
  if (apPasswordLen > 0 && apPasswordLen < 8) {
    Serial.printf("[setup] AP_PASSWORD is %u characters -- WPA2 needs at least 8, "
                  "falling back to an OPEN setup network instead of failing silently.\n",
                  (unsigned)apPasswordLen);
    apPassword = nullptr;
  }

  String apName = String(DEVICE_MANUFACTURER) + " " + String(DEVICE_MODEL) + " " + getShortChipId().substring(2);
  wm.setConfigPortalBlocking(false);
  wm.startConfigPortal(apName.c_str(), apPassword);

  unsigned long resetHoldStart = 0;
  bool sawIdleSinceEntry = false;
  bool ledOn = false;
  while (wm.getConfigPortalActive() && WiFi.status() != WL_CONNECTED) {
    wm.process();

    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    if (!pressed && resetHoldStart == 0) {
      sawIdleSinceEntry = true;
    }

    if (pressed) {
      if (resetHoldStart == 0) resetHoldStart = millis();
      if (millis() - resetHoldStart >= FACTORY_RESET_HOLD_MS) {
        Serial.println("Button held during portal -- factory reset requested.");
        settingsPrefs.begin("settings", false);
        settingsPrefs.clear();
        settingsPrefs.end();
        WiFi.disconnect(true, true);
        setLedColor(0, 0, 0);
        Serial.println("Restarting into unconfigured state...");
        Serial.flush();
        delay(200);
        ESP.restart();
      }
    } else if (resetHoldStart != 0) {
      resetHoldStart = 0;
      if (sawIdleSinceEntry) {
        Serial.println("Button pressed -- canceling setup portal early.");
        wm.stopConfigPortal();
        break;
      }
    }

    bool shouldBeOn = (millis() % SETUP_LED_BLINK_PERIOD_MS) < SETUP_LED_PULSE_MS;
    if (shouldBeOn != ledOn) {
      ledOn = shouldBeOn;
      setLedColor(0, 0, ledOn ? 255 : 0); // pulsing blue while the portal is open
    }
    delay(10);
  }
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (!connected) {
    Serial.println("Setup portal timed out / no connection -- resuming with existing settings.");
    setLedColor(0, 0, 0);
    updateStatusLed();
    return;
  }

  settings.mqttHost     = p_mqtt_host.getValue();
  int parsedPort        = atoi(p_mqtt_port.getValue());
  settings.mqttPort     = (parsedPort > 0 && parsedPort <= 65535) ? (uint16_t)parsedPort : 1883;
  settings.mqttUser     = p_mqtt_user.getValue();
  settings.mqttPassword = p_mqtt_pass.getValue();
  settings.deviceName   = p_device_name.getValue();
  settings.deviceId     = p_device_id.getValue();
  settings.deviceId.replace(" ", "_");
  settings.wifiSsid     = WiFi.SSID();
  settings.wifiPassword = WiFi.psk();

  if (settings.mqttHost.length() == 0) {
    saveSettings();
    Serial.println("Setup portal closed with an empty MQTT broker host -- not marking as configured.");
    setLedColor(0, 0, 0);
    updateStatusLed();
    return;
  }

  settings.configured = true;
  saveSettings();
  Serial.printf("Setup saved: device_id=%s mqtt=%s:%u\n",
                settings.deviceId.c_str(), settings.mqttHost.c_str(), settings.mqttPort);

  setLedColor(0, 0, 0);
  Serial.println("Restarting into normal operation...");
  Serial.flush();
  delay(200);
  ESP.restart();
}
