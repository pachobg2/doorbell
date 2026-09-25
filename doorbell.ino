/*
 * doorbell — ESP32-C3-Zero battery-powered MQTT doorbell (deep sleep)
 * Raw Arduino/C++, espMqttClient, MQTT QoS 1, HA auto-discovery
 *
 * As of v2.0.0 this is a battery-powered, deep-sleeping device, built on the
 * same machinery as door_sensor (deep sleep, RTC-kept counters, awake
 * watchdog, blocking connect + PUBACK-confirmed publishes) with the runtime
 * WiFiManager setup portal used across the fleet.
 *
 * How it works:
 *   - It sleeps in deep sleep almost all the time. Pressing the doorbell
 *     switch (GPIO0, one leg to GND) wakes it; a heartbeat timer wakes it
 *     every HEARTBEAT_INTERVAL_US (12h) to report diagnostics.
 *   - On a press wake the LED lights up immediately (before WiFi is even
 *     up), then it connects, publishes the doorbell event FIRST (before
 *     discovery/diagnostics, to keep ring latency as low as possible), then
 *     the diagnostics, then stays awake while the LED is lit (still
 *     connected, so further presses during that window are reported too),
 *     then goes back to sleep.
 *   - LED: off except after a press, when it lights in the color chosen in
 *     HA (LED Color select) for the time chosen in HA (LED On Time number,
 *     default 5s). Both persist in NVS. Because the device sleeps, a change
 *     made in HA is picked up on the next wake (retained MQTT command) and
 *     applies immediately if the LED is currently lit.
 *   - Setup: a never-configured device goes straight to the WiFiManager
 *     portal. On a configured device, hold the switch >= BUTTON_SETUP_HOLD_MS
 *     (10s) to open it (the press still rings once first, harmlessly);
 *     while it's open, holding again >= FACTORY_RESET_HOLD_MS (5s) wipes all
 *     settings and restarts unconfigured, and a quick press cancels it.
 *   - OTA: an "OTA Update" retained MQTT switch in HA. Flip it, then press the
 *     doorbell (or wait for the next heartbeat): the device sees the retained
 *     command on its next wake and opens a ~5 minute OTA window. There is no
 *     button-hold OTA gesture (unlike TH_2_v4).
 *
 * The doorbell press is a Home Assistant MQTT `event` entity (device_class
 * "doorbell"), QoS 1, not retained.
 *
 * Diagnostics: WiFi signal, reset reason, boot count (wakes since the last
 * power loss, like door_sensor), connect-fail count, total-fail count, firmware
 * version, and uptime (RTC-counter based: continues across deep sleep, zeroes
 * on any real reset or power loss). No battery-voltage monitoring yet -- no
 * divider is wired on this board.
 *
 * Hardware:
 *   GPIO0  - doorbell switch, one leg to GND, INPUT_PULLUP while awake.
 *            Deep-sleep GPIO wake needs a reliably-held HIGH while idle, and
 *            the internal pull-up isn't dependable across deep sleep -- add an
 *            EXTERNAL ~10k pull-up from GPIO0 to 3.3V (same advice as
 *            TH_2_v4/door_sensor). NOTE: on the *original* ESP32, GPIO0 is a
 *            boot-mode strapping pin; on the ESP32-C3 the strapping pins are
 *            GPIO2/GPIO8/GPIO9, so GPIO0 is a normal (and wake-capable) GPIO.
 *   GPIO10 - single WS2812 LED. Note it (and the board's regulator) draw a
 *            small quiescent current even when "off" -- that, not the ESP32
 *            in deep sleep, will likely dominate battery life.
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
#include <esp_sleep.h>
#include <esp_timer.h>
#include "esp_private/esp_clk.h" // esp_clk_rtc_time(), for uptime across deep sleep
#include <Preferences.h>
#include <vector>
// OTA password, setup-portal AP password/timeout, device identity/firmware
// version, button-hold thresholds and timing. WiFi/MQTT credentials are NOT
// here -- they're runtime settings, see Settings below.
#include "config.h"

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
static const uint8_t BUTTON_PIN = 0;  // doorbell switch, also the setup control and deep-sleep wake source
static const uint8_t LED_PIN    = 10;
static const uint8_t LED_COUNT  = 1;

static const uint8_t DEFAULT_LED_BRIGHTNESS_PCT = 50;

// ---------------------------------------------------------------------------
// Runtime settings (WiFi/MQTT/identity, via the setup portal)
// ---------------------------------------------------------------------------
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
Preferences prefs; // LED brightness/color/on-time

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

// ---------------------------------------------------------------------------
// Persisted state (survives deep sleep, in RTC memory)
// ---------------------------------------------------------------------------
RTC_DATA_ATTR bool discoverySent = false;
RTC_DATA_ATTR uint32_t bootCount = 0;         // wakes since the last power loss
RTC_DATA_ATTR uint32_t connectFailCount = 0;  // increments on any wake that fails to publish, resets on success
RTC_DATA_ATTR uint32_t totalFailCount = 0;    // lifetime total failed wakes -- never resets
RTC_DATA_ATTR uint8_t cachedWifiChannel = 0;  // 0 = unknown yet, let WiFi.begin() auto-select

// ---------------------------------------------------------------------------
// Uptime
// ---------------------------------------------------------------------------
// Time since the last real reset or power loss -- NOT since the last wake.
// A deep-sleep timer/GPIO wake is a continuation of the same "up" period, so
// it counts; any other reset reason (power-on, manual reset, brownout,
// watchdog, software restart, a battery that died and was replaced...) zeroes
// it. Uses the RTC counter, which keeps counting through deep sleep;
// millis()/esp_timer don't. Called first thing in setup().
RTC_DATA_ATTR uint64_t g_uptimeStartUs = 0;

void initUptime() {
  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
    g_uptimeStartUs = esp_clk_rtc_time();
    // Any real reset (including the restart at the end of an OTA/USB flash,
    // which leaves RTC memory intact) re-announces discovery once, so a
    // newly added entity actually shows up in HA without a power cycle.
    discoverySent = false;
  }
}

uint32_t uptimeSeconds() {
  return (uint32_t)((esp_clk_rtc_time() - g_uptimeStartUs) / 1000000ULL);
}

// ---------------------------------------------------------------------------
// MQTT topics -- built at runtime from settings.deviceId, not compiled in
// ---------------------------------------------------------------------------
String baseTopic, doorbellEventTopic, availabilityTopic,
       wifiSignalTopic, resetReasonTopic,
       connectFailCountTopic, totalFailCountTopic, bootCountTopic,
       firmwareVersionTopic, uptimeTopic,
       ledBrightnessStateTopic, ledBrightnessCommandTopic,
       ledColorStateTopic, ledColorCommandTopic, ledOnTimeStateTopic, ledOnTimeCommandTopic,
       otaCommandTopic, otaStateTopic;

String discoveryDoorbellTopic, discoveryWifiSignalTopic,
       discoveryResetReasonTopic, discoveryConnectFailCountTopic,
       discoveryTotalFailCountTopic, discoveryBootCountTopic,
       discoveryFirmwareVersionTopic, discoveryUptimeTopic, discoveryLedBrightnessTopic,
       discoveryLedColorTopic, discoveryLedOnTimeTopic, discoveryOtaTopic;

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
  ledColorStateTopic        = baseTopic + "/led_color/state";
  ledColorCommandTopic      = baseTopic + "/led_color/set";
  ledOnTimeStateTopic       = baseTopic + "/led_on_time/state";
  ledOnTimeCommandTopic     = baseTopic + "/led_on_time/set";
  otaCommandTopic           = baseTopic + "/ota/set";
  otaStateTopic             = baseTopic + "/ota/state";

  String sbase = String("homeassistant/sensor/") + settings.deviceId;
  discoveryDoorbellTopic     = String("homeassistant/event/") + settings.deviceId + "/doorbell/config";
  discoveryWifiSignalTopic   = sbase + "/wifi_signal/config";
  discoveryResetReasonTopic  = sbase + "/reset_reason/config";
  discoveryConnectFailCountTopic = sbase + "/connect_fail_count/config";
  discoveryTotalFailCountTopic   = sbase + "/total_fail_count/config";
  discoveryBootCountTopic        = sbase + "/boot_count/config";
  discoveryFirmwareVersionTopic  = sbase + "/firmware_version/config";
  discoveryUptimeTopic           = sbase + "/uptime/config";
  discoveryLedBrightnessTopic = String("homeassistant/number/") + settings.deviceId + "/led_brightness/config";
  discoveryLedColorTopic      = String("homeassistant/select/") + settings.deviceId + "/led_color/config";
  discoveryLedOnTimeTopic     = String("homeassistant/number/") + settings.deviceId + "/led_on_time/config";
  discoveryOtaTopic           = String("homeassistant/switch/") + settings.deviceId + "/ota/config";
}

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
espMqttClient mqttClient;
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint8_t ledBrightnessPct = DEFAULT_LED_BRIGHTNESS_PCT; // 0-100, persisted in NVS

// The LED is off except right after a doorbell press, when it lights up in a
// color chosen from HA for a duration chosen from HA (both persisted in NVS).
// The option names below are exactly what the HA select entity shows and
// sends back, so they double as the MQTT payloads.
struct LedColorOption { const char* name; uint8_t r, g, b; };
static const LedColorOption LED_COLORS[] = {
  {"White",  255, 255, 255},
  {"Red",    255,   0,   0},
  {"Green",    0, 255,   0},
  {"Blue",     0,   0, 255},
  {"Yellow", 255, 180,   0},
  {"Orange", 255,  80,   0},
  {"Purple", 160,   0, 255},
  {"Cyan",     0, 255, 255},
  {"Pink",   255,  60, 120},
};
static const uint8_t LED_COLOR_COUNT = sizeof(LED_COLORS) / sizeof(LED_COLORS[0]);
static const uint8_t DEFAULT_LED_COLOR_INDEX = 0; // White
static const uint8_t DEFAULT_LED_ON_SECS = 5;
static const uint8_t MIN_LED_ON_SECS = 1;
static const uint8_t MAX_LED_ON_SECS = 60;
uint8_t ledColorIndex = DEFAULT_LED_COLOR_INDEX; // persisted in NVS
uint8_t ledOnSecs = DEFAULT_LED_ON_SECS;         // persisted in NVS
bool ledLit = false;              // true while a press-triggered light-up is active
unsigned long ledLitStartMs = 0;  // when the current light-up began
unsigned long ledLitUntilMs = 0;  // millis() deadline for the light-up

// button debounce + hold tracking (used while awake -- see handleButton())
bool lastButtonReading = HIGH;
bool buttonStable = HIGH;
unsigned long lastButtonChangeMs = 0;
static const unsigned long DEBOUNCE_MS = 20;
unsigned long pressStartMs = 0;  // 0 == not currently tracking a press
bool setupTriggered = false;     // true once the current press has already opened the portal
bool g_portalExited = false;     // portal was opened mid-cycle and closed without saving

// setup-portal LED pulse (used while the portal's open)
static const unsigned long SETUP_LED_BLINK_PERIOD_MS = 1000;
static const unsigned long SETUP_LED_PULSE_MS = 150;

// Set from the MQTT library's callbacks (its own background task), read
// from the main flow.
volatile bool mqttConnectedFlag = false;
volatile uint16_t lastAckedPacketId = 0;

// Remote commands arrive as retained messages right after subscribe.
volatile bool g_otaCmdReceived = false;
char g_otaCmdPayload[8] = {0};
volatile bool g_brightnessCmdReceived = false;
char g_brightnessCmdPayload[8] = {0};
volatile bool g_colorCmdReceived = false;
char g_colorCmdPayload[16] = {0};
volatile bool g_onTimeCmdReceived = false;
char g_onTimeCmdPayload[8] = {0};

// ---------------------------------------------------------------------------
// Function declarations
// ---------------------------------------------------------------------------
void connectWiFi();
bool attemptWifiConnect(uint8_t channel);
bool connectMQTT();
bool publishWithAck(const String& topic, const String& payload, bool retain);
void sendDiscoveryConfig();
int publishState(const String& resetReasonStr);
void applyRemoteCommands();
void publishLedSettings();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic,
                   const uint8_t* payload, size_t len, size_t index, size_t total);
void startAwakeWatchdog(uint32_t timeoutMs);
void stopAwakeWatchdog();
void fireDoorbellEvent();
void handleButton();
void setLedColor(uint8_t r, uint8_t g, uint8_t b);
void updateStatusLed();
void serviceLed();
void lightLedForPress();
void setLedBrightness(uint8_t pct, bool save);
void setLedColorIndex(uint8_t index, bool save);
void setLedOnSecs(uint8_t secs, bool save);
String resetReasonToString(esp_reset_reason_t reason);
void enterOtaMode();
void runMaintenanceMode(bool viaButton);
void goToSleep();

// ---------------------------------------------------------------------------
// MQTT callbacks
// ---------------------------------------------------------------------------
void onMqttConnect(bool sessionPresent) {
  mqttConnectedFlag = true;
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  mqttConnectedFlag = false;
  Serial.printf("MQTT disconnected, reason: %u\n", static_cast<uint8_t>(reason));
}

void onMqttPublish(uint16_t packetId) {
  lastAckedPacketId = packetId;
}

static void captureCmd(char* dst, size_t dstSize, const uint8_t* payload, size_t len) {
  size_t copyLen = len < dstSize - 1 ? len : dstSize - 1;
  memcpy(dst, payload, copyLen);
  dst[copyLen] = '\0';
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic,
                   const uint8_t* payload, size_t len, size_t index, size_t total) {
  if (otaCommandTopic.equals(topic)) {
    captureCmd(g_otaCmdPayload, sizeof(g_otaCmdPayload), payload, len);
    g_otaCmdReceived = true;
  } else if (ledBrightnessCommandTopic.equals(topic)) {
    captureCmd(g_brightnessCmdPayload, sizeof(g_brightnessCmdPayload), payload, len);
    g_brightnessCmdReceived = true;
  } else if (ledColorCommandTopic.equals(topic)) {
    captureCmd(g_colorCmdPayload, sizeof(g_colorCmdPayload), payload, len);
    g_colorCmdReceived = true;
  } else if (ledOnTimeCommandTopic.equals(topic)) {
    captureCmd(g_onTimeCmdPayload, sizeof(g_onTimeCmdPayload), payload, len);
    g_onTimeCmdReceived = true;
  }
}

// ---------------------------------------------------------------------------
// Awake watchdog
// A hardware timer that force-restarts the device if it's ever awake too
// long -- a hang, a stuck library call -- independent of everything else, so
// it can't drain the battery awake. Re-armed with a longer deadline while a
// light-up, OTA window or portal legitimately needs to stay awake.
// ---------------------------------------------------------------------------
esp_timer_handle_t g_watchdogTimer = nullptr;

void watchdogTimeoutHandler(void* arg) {
  esp_restart();
}

void startAwakeWatchdog(uint32_t timeoutMs) {
  if (g_watchdogTimer != nullptr) {
    esp_timer_stop(g_watchdogTimer);
    esp_timer_delete(g_watchdogTimer);
    g_watchdogTimer = nullptr;
  }
  esp_timer_create_args_t args = {};
  args.callback = &watchdogTimeoutHandler;
  args.arg = nullptr;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "awake_wdt";
  esp_timer_create(&args, &g_watchdogTimer);
  esp_timer_start_once(g_watchdogTimer, (uint64_t)timeoutMs * 1000ULL);
}

void stopAwakeWatchdog() {
  if (g_watchdogTimer != nullptr) {
    esp_timer_stop(g_watchdogTimer);
    esp_timer_delete(g_watchdogTimer);
    g_watchdogTimer = nullptr;
  }
}

// ---------------------------------------------------------------------------
// Setup / main flow
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  unsigned long bootMs = millis();
  startAwakeWatchdog(AWAKE_WATCHDOG_TIMEOUT_MS); // re-armed with a longer deadline for a light-up/OTA/portal

  initUptime();
  bootCount++;

  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  bool pressWake = (wakeupCause == ESP_SLEEP_WAKEUP_GPIO); // even a tap that's already released by now
  esp_reset_reason_t resetReason = esp_reset_reason();
  String resetReasonStr = resetReasonToString(resetReason);
  if (resetReason == ESP_RST_BROWNOUT) {
    connectFailCount++; // a brownout mid-cycle means that wake never got to publish either
    totalFailCount++;
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  prefs.begin("doorbell", false);
  ledBrightnessPct = prefs.getUChar("led_bright", DEFAULT_LED_BRIGHTNESS_PCT);
  if (ledBrightnessPct > 100) ledBrightnessPct = DEFAULT_LED_BRIGHTNESS_PCT;
  ledColorIndex = prefs.getUChar("led_color", DEFAULT_LED_COLOR_INDEX);
  if (ledColorIndex >= LED_COLOR_COUNT) ledColorIndex = DEFAULT_LED_COLOR_INDEX;
  ledOnSecs = prefs.getUChar("led_secs", DEFAULT_LED_ON_SECS);
  if (ledOnSecs < MIN_LED_ON_SECS || ledOnSecs > MAX_LED_ON_SECS) ledOnSecs = DEFAULT_LED_ON_SECS;

  led.begin();
  led.setBrightness(map(ledBrightnessPct, 0, 100, 0, 255));
  led.clear();
  led.show();

  // Instant feedback: light the LED right away on a press wake, before WiFi
  // or anything slow -- the visitor sees the press registered immediately.
  if (pressWake) lightLedForPress();

  loadSettings();
  buildTopics();

  Serial.printf("Boot #%lu, wakeup cause: %d (%s), reset: %s\n[settings] device_id=%s wifi_ssid=%s mqtt=%s:%u configured=%s\n",
                (unsigned long)bootCount, (int)wakeupCause, pressWake ? "button press" : "timer/reset",
                resetReasonStr.c_str(), settings.deviceId.c_str(), settings.wifiSsid.c_str(),
                settings.mqttHost.c_str(), settings.mqttPort, settings.configured ? "yes" : "no");

  // A never-configured device goes straight to the portal.
  if (!settings.configured) {
    runMaintenanceMode(false);
    // Only reached if the portal timed out -- a successful save restarts
    // the device itself and never returns here.
    goToSleep();
    return;
  }

  // Seed the button state for the linger loop below: if the press that woke
  // us is still down, its hold time counts from boot, and it must not fire
  // a second event (that one is published below).
  buttonStable = lastButtonReading = digitalRead(BUTTON_PIN);
  if (pressWake && buttonStable == LOW) {
    pressStartMs = bootMs > 0 ? bootMs : 1;
  }

  // Connect (one whole retry on a press wake -- a lost ring is worse than a
  // slightly slower failure on a heartbeat wake).
  bool connected = false;
  for (int attempt = 1; attempt <= (pressWake ? 2 : 1) && !connected; attempt++) {
    if (attempt > 1) {
      Serial.println("Retrying connection for the press...");
      mqttClient.disconnect(true);
      WiFi.disconnect(true);
      delay(200);
    }
    connectWiFi();
    connected = (WiFi.status() == WL_CONNECTED) && connectMQTT();
  }

  if (connected) {
    bool ringOk = true;
    if (pressWake) {
      // Event first, ahead of discovery/diagnostics, to keep ring latency down.
      ringOk = publishWithAck(doorbellEventTopic, "{\"event_type\":\"press\"}", false);
    }

    if (!discoverySent) {
      sendDiscoveryConfig();
      discoverySent = true;
    }

    int failedTopics = publishState(resetReasonStr);
    if (failedTopics == 0 && ringOk) {
      connectFailCount = 0; // every topic confirmed by the broker, counter clears
    } else {
      Serial.printf("%d topic(s) never got a PUBACK this cycle (ring published: %s).\n",
                    failedTopics, ringOk ? "yes" : "NO");
      connectFailCount++;
      totalFailCount++;
    }

    applyRemoteCommands();

    if (g_otaCmdReceived && strcmp(g_otaCmdPayload, "ON") == 0) {
      Serial.println("OTA requested via MQTT switch -- entering OTA window.");
      // Clear the retained command immediately so we don't re-trigger on
      // every subsequent wake, and reflect the reset back to the HA UI.
      publishWithAck(otaCommandTopic, "OFF", true);
      publishWithAck(otaStateTopic, "OFF", true);
      enterOtaMode();
    }
  } else {
    Serial.println("WiFi/MQTT connect failed, skipping publish this cycle.");
    connectFailCount++;
    totalFailCount++;
  }

  // Stay awake, still connected, while the LED is lit (a further press
  // during that window is reported live and restarts the timer) or the
  // button is still held (a hold this long opens the setup portal). A held
  // button is only waited on for a bounded time, so a stuck switch can't keep
  // the device awake.
  unsigned long lingerStart = millis();
  const unsigned long maxHeldLingerMs = BUTTON_SETUP_HOLD_MS + 3000UL;
  while (!g_portalExited) {
    serviceLed();
    handleButton();
    bool held = (buttonStable == LOW) && (millis() - lingerStart < maxHeldLingerMs);
    if (!ledLit && !held) break;
    delay(5);
  }

  if (g_portalExited) {
    // Portal opened via a hold and closed without saving -- WiFi/MQTT are
    // already torn down, just go back to sleep.
    goToSleep();
    return;
  }

  mqttClient.disconnect();
  delay(MQTT_DISCONNECT_DELAY_MS);
  WiFi.disconnect(true);
  goToSleep();
}

void loop() {
  // Never reached in normal operation -- the device deep-sleeps at the end of setup().
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
// Single connection attempt on the given channel (0 = let the radio
// auto-scan/pick). Returns true if connected within WIFI_CONNECT_TIMEOUT_MS.
bool attemptWifiConnect(uint8_t channel) {
  WiFi.begin(settings.wifiSsid.c_str(), settings.wifiPassword.c_str(), channel);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(20);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected in %lums, IP: %s, channel: %u\n",
                  millis() - start, WiFi.localIP().toString().c_str(), WiFi.channel());
    return true;
  }
  Serial.printf("[debug] Final WiFi status: %d\n", WiFi.status());
  return false;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // no modem power-save during the connect -- faster/more reliable out of deep sleep

  // Reusing the channel from the last successful connect skips most of the
  // scan; fall back to a full scan if the cached channel attempt fails.
  bool connected = attemptWifiConnect(cachedWifiChannel);
  if (!connected && cachedWifiChannel != 0) {
    Serial.println("[debug] Cached-channel connect failed, retrying with auto channel scan...");
    WiFi.disconnect();
    delay(100);
    connected = attemptWifiConnect(0);
  }
  cachedWifiChannel = connected ? WiFi.channel() : 0;
}

// ---------------------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------------------
bool connectMQTT() {
  mqttClient.setServer(settings.mqttHost.c_str(), settings.mqttPort);
  mqttClient.setCredentials(settings.mqttUser.c_str(), settings.mqttPassword.c_str());
  mqttClient.setClientId(settings.deviceId.c_str());
  // LWT: broker marks the device "offline" if it drops without a clean disconnect
  mqttClient.setWill(availabilityTopic.c_str(), 1, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);

  for (uint32_t attempt = 1; attempt <= MQTT_CONNECT_ATTEMPTS; attempt++) {
    mqttConnectedFlag = false;
    mqttClient.connect();

    unsigned long start = millis();
    while (!mqttConnectedFlag && millis() - start < MQTT_CONNECT_TIMEOUT_MS) {
      delay(10);
    }

    if (mqttConnectedFlag) {
      Serial.printf("MQTT connected in %lums (attempt %lu)\n", millis() - start, (unsigned long)attempt);
      publishWithAck(availabilityTopic, "online", true);

      // Subscribe now: retained commands are delivered right away, and by
      // the time the publishes below finish they've been received.
      g_otaCmdReceived = g_brightnessCmdReceived = g_colorCmdReceived = g_onTimeCmdReceived = false;
      mqttClient.subscribe(ledBrightnessCommandTopic.c_str(), 1);
      mqttClient.subscribe(ledColorCommandTopic.c_str(), 1);
      mqttClient.subscribe(ledOnTimeCommandTopic.c_str(), 1);
      mqttClient.subscribe(otaCommandTopic.c_str(), 1);
      return true;
    }

    Serial.printf("MQTT connect attempt %lu timed out.\n", (unsigned long)attempt);
    mqttClient.disconnect(true); // force-clear state before retrying
    delay(200);
  }
  return false;
}

// Publishes at QoS 1 and waits for the broker's PUBACK before returning,
// retrying as a brand-new publish up to MQTT_PUBLISH_ATTEMPTS times.
bool publishWithAck(const String& topic, const String& payload, bool retain) {
  for (uint32_t attempt = 1; attempt <= MQTT_PUBLISH_ATTEMPTS; attempt++) {
    lastAckedPacketId = 0;
    uint16_t packetId = mqttClient.publish(topic.c_str(), 1, retain, payload.c_str());
    if (packetId == 0) {
      Serial.printf("[mqtt] queue failed for %s (attempt %lu/%lu)\n",
                    topic.c_str(), (unsigned long)attempt, (unsigned long)MQTT_PUBLISH_ATTEMPTS);
      delay(100);
      continue;
    }

    unsigned long start = millis();
    while (lastAckedPacketId != packetId && millis() - start < MQTT_ACK_TIMEOUT_MS) {
      delay(5);
    }
    if (lastAckedPacketId == packetId) return true;

    Serial.printf("[mqtt] no PUBACK for %s (attempt %lu/%lu)\n",
                  topic.c_str(), (unsigned long)attempt, (unsigned long)MQTT_PUBLISH_ATTEMPTS);
  }
  Serial.printf("[mqtt] giving up on %s\n", topic.c_str());
  return false;
}

// ---------------------------------------------------------------------------
// Home Assistant discovery (sent once per power-life / real reset)
// ---------------------------------------------------------------------------
static String deviceBlock() {
  return String("\"device\":{\"identifiers\":[\"") + settings.deviceId
      + "\"],\"name\":\"" + settings.deviceName
      + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER
      + "\",\"model\":\"" + DEVICE_MODEL
      + "\",\"sw_version\":\"" + FIRMWARE_VERSION
      + "\",\"hw_version\":\"" + DEVICE_HW_VERSION + "\"}";
}

// A diagnostic sensor entry. expire_after = 3x the heartbeat, so a dead
// device eventually shows "unavailable" instead of frozen values.
static void sendSensorDiscovery(const String& topic, const char* name, const char* uid,
                                const String& stateTopic, const String& extra) {
  String payload = String("{")
    + "\"name\":\"" + name + "\","
    + "\"unique_id\":\"" + settings.deviceId + "_" + uid + "\","
    + "\"state_topic\":\"" + stateTopic + "\","
    + extra
    + "\"entity_category\":\"diagnostic\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"availability_topic\":\"" + availabilityTopic + "\","
    + deviceBlock() + "}";
  publishWithAck(topic, payload, true);
}

void sendDiscoveryConfig() {
  // Doorbell press (event entity -- stateless trigger, not a binary_sensor)
  {
    String payload = String("{")
      + "\"name\":\"Doorbell\","
      + "\"unique_id\":\"" + settings.deviceId + "_doorbell\","
      + "\"device_class\":\"doorbell\","
      + "\"event_types\":[\"press\"],"
      + "\"state_topic\":\"" + doorbellEventTopic + "\","
      + "\"availability_topic\":\"" + availabilityTopic + "\","
      + deviceBlock() + "}";
    publishWithAck(discoveryDoorbellTopic, payload, true);
  }

  sendSensorDiscovery(discoveryWifiSignalTopic, "WiFi Signal", "wifi_signal", wifiSignalTopic,
    "\"unit_of_measurement\":\"dBm\",\"device_class\":\"signal_strength\",\"state_class\":\"measurement\",");
  sendSensorDiscovery(discoveryResetReasonTopic, "Reset Reason", "reset_reason", resetReasonTopic, "");
  sendSensorDiscovery(discoveryBootCountTopic, "Boot Count", "boot_count", bootCountTopic,
    "\"state_class\":\"total_increasing\",\"icon\":\"mdi:counter\",");
  sendSensorDiscovery(discoveryConnectFailCountTopic, "Connect Fail Count", "connect_fail_count", connectFailCountTopic,
    "\"state_class\":\"measurement\",\"icon\":\"mdi:wifi-alert\",");
  sendSensorDiscovery(discoveryTotalFailCountTopic, "Total Fail Count", "total_fail_count", totalFailCountTopic,
    "\"state_class\":\"total_increasing\",\"icon\":\"mdi:counter\",");
  sendSensorDiscovery(discoveryFirmwareVersionTopic, "Firmware Version", "firmware_version", firmwareVersionTopic, "");
  // Uptime -- seconds since the last real reset/power loss (deep-sleep wakes
  // don't count as a reset, see initUptime())
  sendSensorDiscovery(discoveryUptimeTopic, "Uptime", "uptime", uptimeTopic,
    "\"unit_of_measurement\":\"s\",\"device_class\":\"duration\",\"state_class\":\"measurement\",");

  // Controls: "retain":true makes HA publish commands retained, so a change
  // made while this device sleeps is still on the broker when it next wakes.
  // No expire_after -- these are controls, not readings.
  {
    String payload = String("{")
      + "\"name\":\"LED Brightness\","
      + "\"unique_id\":\"" + settings.deviceId + "_led_brightness\","
      + "\"state_topic\":\"" + ledBrightnessStateTopic + "\","
      + "\"command_topic\":\"" + ledBrightnessCommandTopic + "\","
      + "\"min\":0,\"max\":100,\"step\":1,\"unit_of_measurement\":\"%\","
      + "\"icon\":\"mdi:brightness-percent\",\"retain\":true,"
      + "\"availability_topic\":\"" + availabilityTopic + "\","
      + deviceBlock() + "}";
    publishWithAck(discoveryLedBrightnessTopic, payload, true);
  }
  {
    String options = "[";
    for (uint8_t i = 0; i < LED_COLOR_COUNT; i++) {
      if (i) options += ",";
      options += String("\"") + LED_COLORS[i].name + "\"";
    }
    options += "]";
    String payload = String("{")
      + "\"name\":\"LED Color\","
      + "\"unique_id\":\"" + settings.deviceId + "_led_color\","
      + "\"state_topic\":\"" + ledColorStateTopic + "\","
      + "\"command_topic\":\"" + ledColorCommandTopic + "\","
      + "\"options\":" + options + ","
      + "\"icon\":\"mdi:palette\",\"retain\":true,"
      + "\"availability_topic\":\"" + availabilityTopic + "\","
      + deviceBlock() + "}";
    publishWithAck(discoveryLedColorTopic, payload, true);
  }
  {
    String payload = String("{")
      + "\"name\":\"LED On Time\","
      + "\"unique_id\":\"" + settings.deviceId + "_led_on_time\","
      + "\"state_topic\":\"" + ledOnTimeStateTopic + "\","
      + "\"command_topic\":\"" + ledOnTimeCommandTopic + "\","
      + "\"min\":" + String(MIN_LED_ON_SECS) + ",\"max\":" + String(MAX_LED_ON_SECS) + ",\"step\":1,"
      + "\"unit_of_measurement\":\"s\",\"icon\":\"mdi:timer-outline\",\"retain\":true,"
      + "\"availability_topic\":\"" + availabilityTopic + "\","
      + deviceBlock() + "}";
    publishWithAck(discoveryLedOnTimeTopic, payload, true);
  }
  // OTA Update: a retained switch (a plain "button" entity's press is a
  // one-shot, non-retained message a sleeping device would simply miss).
  {
    String payload = String("{")
      + "\"name\":\"OTA Update\","
      + "\"unique_id\":\"" + settings.deviceId + "_ota\","
      + "\"command_topic\":\"" + otaCommandTopic + "\","
      + "\"state_topic\":\"" + otaStateTopic + "\","
      + "\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"retain\":true,"
      + "\"entity_category\":\"config\","
      + "\"availability_topic\":\"" + availabilityTopic + "\","
      + deviceBlock() + "}";
    publishWithAck(discoveryOtaTopic, payload, true);
    publishWithAck(otaStateTopic, "OFF", true);
  }
}

// ---------------------------------------------------------------------------
// Per-wake state publish. Returns how many topics never got a PUBACK.
// ---------------------------------------------------------------------------
int publishState(const String& resetReasonStr) {
  int failed = 0;
  if (WiFi.status() == WL_CONNECTED) {
    if (!publishWithAck(wifiSignalTopic, String(WiFi.RSSI()), true)) failed++;
  }
  if (!publishWithAck(resetReasonTopic, resetReasonStr, true)) failed++;
  if (!publishWithAck(bootCountTopic, String((unsigned long)bootCount), true)) failed++;
  if (!publishWithAck(connectFailCountTopic, String((unsigned long)connectFailCount), true)) failed++;
  if (!publishWithAck(totalFailCountTopic, String((unsigned long)totalFailCount), true)) failed++;
  if (!publishWithAck(firmwareVersionTopic, String(FIRMWARE_VERSION), true)) failed++;
  if (!publishWithAck(uptimeTopic, String((unsigned long)uptimeSeconds()), true)) failed++;
  return failed;
}

// LED brightness/color/on-time: apply any command received this wake (if it
// changed), then always echo the current values back as state.
void applyRemoteCommands() {
  if (g_brightnessCmdReceived) {
    int pct = atoi(g_brightnessCmdPayload);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    if ((uint8_t)pct != ledBrightnessPct) setLedBrightness((uint8_t)pct, true);
  }
  if (g_colorCmdReceived) {
    for (uint8_t i = 0; i < LED_COLOR_COUNT; i++) {
      if (strcasecmp(g_colorCmdPayload, LED_COLORS[i].name) == 0) {
        if (i != ledColorIndex) setLedColorIndex(i, true);
        break;
      }
    }
  }
  if (g_onTimeCmdReceived) {
    int secs = atoi(g_onTimeCmdPayload);
    if (secs < MIN_LED_ON_SECS) secs = MIN_LED_ON_SECS;
    if (secs > MAX_LED_ON_SECS) secs = MAX_LED_ON_SECS;
    if ((uint8_t)secs != ledOnSecs) setLedOnSecs((uint8_t)secs, true);
  }
  publishLedSettings();
}

void publishLedSettings() {
  publishWithAck(ledBrightnessStateTopic, String(ledBrightnessPct), true);
  publishWithAck(ledColorStateTopic, LED_COLORS[ledColorIndex].name, true);
  publishWithAck(ledOnTimeStateTopic, String(ledOnSecs), true);
}

// ---------------------------------------------------------------------------
// Doorbell event -- QoS 1, NOT retained (a stateless event shouldn't replay
// a stale "someone rang" on every HA restart the way a retained topic would).
// Used for presses that happen while already awake (the wake press itself is
// published in setup()).
// ---------------------------------------------------------------------------
void fireDoorbellEvent() {
  lightLedForPress(); // light first so the LED reacts instantly, even if MQTT is down
  if (mqttClient.connected()) {
    publishWithAck(doorbellEventTopic, "{\"event_type\":\"press\"}", false);
  }
}

// ---------------------------------------------------------------------------
// Button (GPIO0, active low, debounced) -- used while awake, during the
// linger window. The press that woke the device is handled in setup().
//
// A new debounced press rings again and restarts the LED timer. Held past
// BUTTON_SETUP_HOLD_MS: commits immediately (without waiting for release) to
// opening the setup portal. See runMaintenanceMode() for the in-portal
// factory-reset gesture.
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
    // Only returns on portal timeout/cancel (a save restarts the device).
    g_portalExited = true;
    pressStartMs = 0;
  }

  lastButtonReading = reading;
}

// ---------------------------------------------------------------------------
// LED
// ---------------------------------------------------------------------------
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

// The LED is off unless a press-triggered light-up is currently active.
void updateStatusLed() {
  if (ledLit) {
    const LedColorOption& c = LED_COLORS[ledColorIndex];
    setLedColor(c.r, c.g, c.b);
  } else {
    setLedColor(0, 0, 0);
  }
}

// Non-blocking: lights the LED in the selected color and records the
// deadline; serviceLed() turns it off again. A press while already lit
// restarts the timer. Also re-arms the awake watchdog so it can't fire in
// the middle of a (up to 60s) light-up.
void lightLedForPress() {
  ledLit = true;
  ledLitStartMs = millis();
  ledLitUntilMs = ledLitStartMs + (unsigned long)ledOnSecs * 1000UL;
  uint32_t wdMs = (uint32_t)ledOnSecs * 1000UL + 45000UL;
  startAwakeWatchdog(wdMs > AWAKE_WATCHDOG_TIMEOUT_MS ? wdMs : AWAKE_WATCHDOG_TIMEOUT_MS);
  updateStatusLed();
}

void serviceLed() {
  if (ledLit && (long)(millis() - ledLitUntilMs) >= 0) {
    ledLit = false;
    updateStatusLed();
  }
}

void setLedBrightness(uint8_t pct, bool save) {
  ledBrightnessPct = pct;
  led.setBrightness(map(ledBrightnessPct, 0, 100, 0, 255));
  led.show(); // redraw current color at the new brightness
  if (save) prefs.putUChar("led_bright", ledBrightnessPct);
}

void setLedColorIndex(uint8_t index, bool save) {
  ledColorIndex = index;
  if (ledLit) updateStatusLed(); // recolor immediately if currently lit
  if (save) prefs.putUChar("led_color", ledColorIndex);
}

void setLedOnSecs(uint8_t secs, bool save) {
  ledOnSecs = secs;
  if (ledLit) ledLitUntilMs = ledLitStartMs + (unsigned long)ledOnSecs * 1000UL; // applies to the light-up in progress
  if (save) prefs.putUChar("led_secs", ledOnSecs);
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
// Human-readable reset reason. Note that after a deep-sleep wake this is
// always "deep_sleep_wake"; the interesting ones are brownout, watchdog,
// panic and power_on.
String resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "power_on";
    case ESP_RST_EXT:       return "external_pin";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt_watchdog";
    case ESP_RST_TASK_WDT:  return "task_watchdog";
    case ESP_RST_WDT:       return "other_watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep_wake";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}

// ---------------------------------------------------------------------------
// OTA window (remote-triggered via the "OTA Update" switch). Assumes WiFi is
// connected. LED solid blue = OTA window open; a doorbell press cancels it.
// ---------------------------------------------------------------------------
void enterOtaMode() {
  startAwakeWatchdog(OTA_WINDOW_MS + 30000); // OTA legitimately needs to stay awake this long
  ArduinoOTA.setHostname(settings.deviceId.c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  setLedColor(0, 0, 255);
  Serial.printf("OTA ready, staying awake for up to %lu ms...\n", (unsigned long)OTA_WINDOW_MS);

  // Wait for the press that started this cycle to be released first, so it
  // doesn't immediately cancel the window it just caused.
  while (digitalRead(BUTTON_PIN) == LOW) delay(10);
  delay(50);

  unsigned long start = millis();
  while (millis() - start < OTA_WINDOW_MS) {
    ArduinoOTA.handle();
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("Button pressed -- canceling OTA window early.");
      break;
    }
    delay(10);
  }
  ledLit = false;
  updateStatusLed();
  startAwakeWatchdog(AWAKE_WATCHDOG_TIMEOUT_MS);
  Serial.println("OTA window over, resuming normal cycle.");
}

// ---------------------------------------------------------------------------
// Setup portal
//
// Entered when the device has never been configured, or when the switch is
// held past BUTTON_SETUP_HOLD_MS. Broadcasts "<DEVICE_MANUFACTURER>
// <DEVICE_MODEL> XXXX" (last 4 hex chars of the chip MAC), and serves a page
// (WiFiManager) with a WiFi picker plus custom fields for MQTT and device
// identity -- one form, one Save. Holding the switch again for
// FACTORY_RESET_HOLD_MS while the page is open wipes the device back to a
// fully unconfigured state; a quick press cancels. If the portal succeeds,
// settings are saved and the device restarts; if it times out or is
// cancelled, this returns and the caller goes back to sleep with whatever
// settings already existed.
// ---------------------------------------------------------------------------
void runMaintenanceMode(bool viaButton) {
  Serial.println(viaButton
    ? "Button held >=10s -- entering Setup Mode."
    : "No saved WiFi config yet -- entering first-time setup.");
  startAwakeWatchdog((PORTAL_TIMEOUT_SEC + 60) * 1000UL);

  // Clean teardown before handing the radio to WiFiManager, in case this
  // was reached mid-cycle on an already-connected device.
  if (mqttClient.connected()) {
    mqttClient.disconnect();
    delay(MQTT_DISCONNECT_DELAY_MS);
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
    + "Wake count: " + String((unsigned long)bootCount) + " &middot; connect fails: " + String((unsigned long)connectFailCount)
    + " (" + String((unsigned long)totalFailCount) + " total)"
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
        WiFi.disconnect(true, true); // also erase the radio's own persisted WiFi credentials
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
    ledLit = false;
    setLedColor(0, 0, 0);
    stopAwakeWatchdog();
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
    ledLit = false;
    setLedColor(0, 0, 0);
    stopAwakeWatchdog();
    return;
  }

  settings.configured = true;
  saveSettings();
  Serial.printf("Setup saved: device_id=%s mqtt=%s:%u\n",
                settings.deviceId.c_str(), settings.mqttHost.c_str(), settings.mqttPort);

  setLedColor(0, 0, 0);
  stopAwakeWatchdog();
  Serial.println("Restarting into normal operation...");
  Serial.flush();
  delay(200);
  ESP.restart();
}

// ---------------------------------------------------------------------------
// Sleep
// ---------------------------------------------------------------------------
void goToSleep() {
  stopAwakeWatchdog(); // about to sleep on our own terms, no need for the failsafe to fire mid-sleep
  ledLit = false;
  setLedColor(0, 0, 0);

  esp_sleep_enable_timer_wakeup(HEARTBEAT_INTERVAL_US);

  // Wait for the switch to be released (and stop bouncing) before arming
  // the level-triggered wake, or the tail of the press that got us here
  // would wake the device again immediately. If it's stuck down, skip the
  // GPIO wake for this sleep (timer only) rather than looping awake.
  unsigned long waitStart = millis();
  unsigned long highSince = 0;
  while (millis() - waitStart < 30000UL) {
    if (digitalRead(BUTTON_PIN) == HIGH) {
      if (highSince == 0) highSince = millis();
      if (millis() - highSince >= 50) break;
    } else {
      highSince = 0;
    }
    delay(5);
  }
  if (highSince != 0 && millis() - highSince >= 50) {
    // Idle level is HIGH (pull-up), pressed pulls LOW -> wake on LOW.
    esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  } else {
    Serial.println("Switch appears stuck down -- sleeping on the timer only this time.");
  }

  Serial.println("Going to sleep...");
  Serial.flush();
  esp_deep_sleep_start();
}
