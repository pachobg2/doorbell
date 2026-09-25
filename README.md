# doorbell — ESP32-C3-Zero MQTT doorbell

Raw Arduino C++, `espMqttClient` with QoS 1 + PUBACK, non-blocking WiFi/MQTT
reconnect with exponential backoff, LWT availability, retained HA MQTT
discovery, diagnostic entities, NVS-persisted status-LED brightness,
`ArduinoOTA`, manufacturer `P@cho`. Not battery-powered (for now) — stays
connected continuously, no deep sleep, same architecture as `smart_switch`
v2.0.0.

WiFi, MQTT, and device identity are configured at runtime via a built-in
`WiFiManager` web setup portal and persisted in NVS — same system as
`door_sensor`/`TH_2_v4`/`smart_switch`. The doorbell switch doubles as the
setup control (button-hold gestures), same convention `smart_switch`/
`TH_2_v4` use.

## Files

- `doorbell.ino` — the sketch.
- `config.h.example` — copy to `config.h` and fill in: OTA password,
  setup-portal AP password/timeout, device identity/firmware version,
  button-hold thresholds. WiFi/MQTT credentials are *not* here — see
  "Setup Mode". Keep `config.h` out of git (already covered by
  `.gitignore`).

## Hardware

ESP32-C3-Zero:

- **GPIO0** — doorbell switch, one leg to GND, internal pull-up (active
  low). Debounced press fires the doorbell event immediately; held ≥10s
  opens the setup portal; held ≥5s again while the portal is open triggers
  a factory reset (see "Setup Mode"). **Note**: on the *original* ESP32,
  GPIO0 is a boot-mode strapping pin and wiring a switch there is risky.
  On the ESP32-C3 the strapping pins are GPIO2/GPIO8/GPIO9 instead —
  GPIO0 is a normal GPIO on this chip, so this wiring is fine here.
- **GPIO10** — single WS2812 status LED (same pin as `smart_switch`).

## Before building

1. **Arduino IDE board package**: "esp32 by Espressif Systems" (Boards
   Manager).
2. **Libraries** (Library Manager):
   - `espMqttClient` by bertmelis
   - `Adafruit NeoPixel`
   - `WiFiManager` by tzapu
3. Select board **"ESP32C3 Dev Module"**.
4. Copy `config.h.example` to `config.h` and fill in your OTA password and
   (optionally) the setup-portal AP password. WiFi/MQTT are set up after
   flashing, through the portal — see "Setup Mode" below.

## First flash vs. later updates

First flash needs a USB cable. A never-configured device boots straight
into the setup portal (see below). After that, `ArduinoOTA` exposes the
board as a network port in `Tools > Port` — subsequent updates can go out
over WiFi, protected by `OTA_PASSWORD` from `config.h`.

## Setup Mode

On first boot (or after a factory reset), the device broadcasts its own
temporary WiFi network — `<manufacturer> <model> XXXX` (last 4 hex chars
of the chip MAC), password from `AP_PASSWORD` in `config.h` — and serves a
setup page: the normal WiFiManager network picker, plus MQTT broker
host/port/user/password and device name/ID fields on the same page, saved
together in one submission. The portal times out after `PORTAL_TIMEOUT_SEC`
(default 10 min) and resumes with whatever settings already existed.

To reopen the portal on an already-configured device, **hold the doorbell
switch for 10 seconds** (`BUTTON_SETUP_HOLD_MS`) — it fires the instant the
hold crosses that threshold, without needing to release first. The LED
pulses blue for the duration of the hold and while the portal is open.

While the portal is open, **holding the switch again for 5 seconds**
(`FACTORY_RESET_HOLD_MS`) wipes all saved settings — WiFi, MQTT, device
identity — and the radio's own saved WiFi credentials, then restarts fully
unconfigured. A quick press instead cancels the portal early.

## MQTT / Home Assistant

Base topic: `doorbell/<device_id>/...` (device ID is set during Setup
Mode, defaults to `doorbell_<chip-id>` if never configured).

| Purpose | Topic | Payload |
|---|---|---|
| Doorbell press (event) | `.../event` | `{"event_type":"press"}` (not retained) |
| Availability / LWT | `.../availability` | `online` / `offline` |
| LED brightness | `.../led_brightness/state`, `.../set` | 0-100 |
| WiFi signal | `.../wifi_signal/state` | dBm |
| Reset reason | `.../reset_reason/state` | string |
| Boot count | `.../boot_count/state` | integer |
| Connect fail count (resets on success) | `.../connect_fail_count/state` | integer |
| Total fail count (lifetime) | `.../total_fail_count/state` | integer |
| Firmware version | `.../firmware_version/state` | string |
| Uptime (zeroes on any reset/power loss) | `.../uptime/state` | seconds |
| OTA request/restart | `.../ota_restart/set` | any payload |

The doorbell press is modeled as Home Assistant's MQTT **`event` entity**
(`device_class: "doorbell"`, `homeassistant/event/<device_id>/doorbell/config`)
rather than a momentary binary_sensor — semantically correct for a
stateless trigger, and the first project in this fleet to use that
platform. Trigger automations off it the same way you would any other HA
event entity (state changes to the event's timestamp/attributes on every
firing, no ON/OFF to track).

On every MQTT connect the firmware publishes retained HA discovery configs
for the doorbell event entity, the LED-brightness number entity, an
OTA-restart button, and diagnostic sensors (WiFi signal, reset reason,
boot count, connect/total fail counts, firmware version) — all bundled
under one device in Home Assistant automatically, no `configuration.yaml`
edits needed.

LED brightness persists in NVS (Preferences namespace `doorbell`), so it
survives a reboot.

## Status LED

| Color | Meaning |
|---|---|
| 3 blue blinks | Boot animation, once MQTT first connects |
| Green | Idle, WiFi connected |
| Off | No WiFi |
| Brief white flash | Doorbell press acknowledged |
| Pulsing blue | Setup-mode button hold in progress, or portal open |

## Diagnostics

WiFi signal, boot count, and connect/total fail counts republish every 2
minutes. Reset reason is sent once per boot (or on every MQTT reconnect via
`publishDiagnostics(true)` in `onMqttConnect`). Connect fail count resets
to 0 on the next successful MQTT connect; total fail count is
NVS-persisted and never resets — both increment together on every MQTT
disconnect. Boot count is also NVS-persisted, incremented once per actual
device boot. MQTT reconnects use exponential backoff (1s doubling to a 30s
cap); WiFi has a single bounded 5s wait at boot only, then non-blocking
retry via `pollWiFi()` every `loop()` iteration.

## Config file

`config.h` (gitignored) holds the OTA password, setup-portal AP
password/timeout, device identity constants (`DEVICE_MANUFACTURER`,
`DEVICE_MODEL`, `DEVICE_HW_VERSION`, `FIRMWARE_VERSION`), and button-hold
thresholds — copy `config.h.example` to `config.h` and fill in real
values. WiFi credentials, MQTT broker settings, and this device's own
name/ID are **not** here — they're runtime settings, configured through
the setup portal (see "Setup Mode") and persisted in NVS.

## Version History

| Version | Date | Changes |
|---|---|---|
| v1.0.0 | 2026-09-22 | Initial release: WiFiManager setup portal, doorbell switch (GPIO0) doubling as the setup control (button-hold gestures, same convention as `smart_switch`), doorbell press modeled as an MQTT `event` entity, full diagnostic set (WiFi signal, reset reason, boot count, connect/total fail counts, firmware version), NVS-persisted LED brightness. |
| v1.0.1 | 2026-09-25 | Added an `Uptime` diagnostic sensor (seconds since boot, `device_class: duration`). Uses `esp_timer_get_time()` (64-bit) rather than `millis()`, so it zeroes on any reboot or power loss but never wraps back to zero on its own at ~49.7 days. |
