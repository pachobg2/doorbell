# doorbell — ESP32-C3-Zero battery-powered MQTT doorbell

Raw Arduino C++, `espMqttClient` with QoS 1 + PUBACK, retained HA MQTT
discovery, diagnostic entities, NVS-persisted LED brightness/color/on-time,
`ArduinoOTA`, manufacturer `P@cho`. **Battery-powered and deep-sleeping** (as
of v2.0.0): it sleeps almost all the time and wakes when the doorbell switch
is pressed, plus a 12-hour heartbeat wake to report diagnostics — same
deep-sleep machinery as `door_sensor`, with the runtime `WiFiManager` setup
portal used across the fleet.

## Files

- `doorbell.ino` — the sketch.
- `config.h.example` — copy to `config.h` and fill in: OTA password,
  setup-portal AP password/timeout, device identity/firmware version,
  button-hold thresholds, deep-sleep/MQTT timing. WiFi/MQTT credentials are
  *not* here — see "Setup Mode". Keep `config.h` out of git (already covered
  by `.gitignore`).

## Hardware

ESP32-C3-Zero:

- **GPIO0** — doorbell switch, one leg to GND (active low), wakes the device
  from deep sleep. **Add an external ~10k pull-up from GPIO0 to 3.3V**:
  deep-sleep GPIO wake needs a reliably-held HIGH while idle, and the
  internal pull-up isn't dependable across sleep (same advice as
  `TH_2_v4`/`door_sensor`). On the *original* ESP32, GPIO0 is a boot-mode
  strapping pin and wiring a switch there is risky; on the ESP32-C3 the
  strapping pins are GPIO2/GPIO8/GPIO9, so GPIO0 is a normal, wake-capable
  GPIO here.
- **GPIO10** — single WS2812 LED (same pin as `smart_switch`). Off except
  after a press. Note the LED and the board's regulator draw a small
  quiescent current even when "off" — that, not the ESP32 in deep sleep,
  will likely dominate battery life.
- No battery-voltage monitoring yet (no divider is wired on this board), so
  there are no battery entities.

## Behavior

- **Press**: wakes the device and the LED lights up **immediately** (before
  WiFi is even up) in the color chosen in HA for the time chosen in HA
  (default 5 s). It then connects (with one full retry if that fails),
  publishes the doorbell event **first** — ahead of discovery/diagnostics — to
  keep ring latency low, then the diagnostics, and stays awake, still
  connected, while the LED is lit: a further press in that window rings
  again and restarts the timer. Then it goes back to sleep.
- **Heartbeat** (every 12 h, `HEARTBEAT_INTERVAL_US`): wakes, publishes
  diagnostics/uptime, applies any pending HA commands, sleeps. No LED.
- **Awake watchdog**: a hardware timer force-restarts the device if it's
  ever awake unexpectedly long, so a hang can't drain the battery.
- If WiFi/MQTT can't connect on a press, the LED still lights (so the press
  is acknowledged) but the ring is lost; the connect-fail counters go up.

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

First flash needs a USB cable. A never-configured device boots straight into
the setup portal (see below). After that, updates go over WiFi via the
**OTA Update** switch in Home Assistant: flip it on, then press the doorbell
(or wait for the next heartbeat). The device sees the retained command on
its next wake, clears it, and opens a ~5 minute OTA window (LED solid blue;
pressing the doorbell cancels it) — it then shows up in `Tools > Port` as a
network port, protected by `OTA_PASSWORD` from `config.h`. There is no
button-hold OTA gesture.

## Setup Mode

On first boot (or after a factory reset), the device broadcasts its own
temporary WiFi network — `<manufacturer> <model> XXXX` (last 4 hex chars of
the chip MAC), password from `AP_PASSWORD` in `config.h` — and serves a setup
page: the normal WiFiManager network picker, plus MQTT broker
host/port/user/password and device name/ID fields on the same page, saved
together in one submission. The portal times out after `PORTAL_TIMEOUT_SEC`
(default 10 min) and the device goes back to sleep with whatever settings it
already had.

To reopen the portal on an already-configured device, **press and hold the
doorbell switch for 10 seconds** (`BUTTON_SETUP_HOLD_MS`) — it fires the
instant the hold crosses that threshold, without needing to release first.
The press still rings once at the start (harmless). The LED pulses blue while
the portal is open.

While the portal is open, **holding the switch again for 5 seconds**
(`FACTORY_RESET_HOLD_MS`) wipes all saved settings — WiFi, MQTT, device
identity — and the radio's own saved WiFi credentials, then restarts fully
unconfigured. A quick press instead cancels the portal early.

## MQTT / Home Assistant

Base topic: `doorbell/<device_id>/...` (device ID is set during Setup Mode,
defaults to `doorbell_<chip-id>` if never configured).

| Purpose | Topic | Payload |
|---|---|---|
| Doorbell press (event) | `.../event` | `{"event_type":"press"}` (not retained) |
| Availability / LWT | `.../availability` | `online` / `offline` |
| LED brightness (retained cmd) | `.../led_brightness/state`, `.../set` | 0-100 |
| LED color (HA select, retained cmd) | `.../led_color/state`, `.../set` | `White` / `Red` / `Green` / `Blue` / `Yellow` / `Orange` / `Purple` / `Cyan` / `Pink` |
| LED on-time (HA number, retained cmd) | `.../led_on_time/state`, `.../set` | seconds, 1-60 (default 5) |
| OTA Update (retained switch) | `.../ota/state`, `.../ota/set` | `ON` / `OFF` |
| WiFi signal | `.../wifi_signal/state` | dBm |
| Reset reason | `.../reset_reason/state` | string |
| Boot count (wakes since last power loss) | `.../boot_count/state` | integer |
| Connect fail count (resets on success) | `.../connect_fail_count/state` | integer |
| Total fail count (lifetime) | `.../total_fail_count/state` | integer |
| Firmware version | `.../firmware_version/state` | string |
| Uptime (zeroes on reset/power loss, keeps counting through deep sleep) | `.../uptime/state` | seconds |

The doorbell press is Home Assistant's MQTT **`event` entity**
(`device_class: "doorbell"`, `homeassistant/event/<device_id>/doorbell/config`),
not a momentary binary_sensor — semantically correct for a stateless trigger.
Trigger automations off it like any other HA event entity.

Because the device sleeps, the controls (LED brightness/color/on-time, OTA
Update) are **retained** MQTT commands that the device picks up on its next
wake (a press or the heartbeat) and echoes back as state. A change to the
LED color/on-time made in HA therefore takes effect on the next press; if
that press is what wakes it, the LED recolors as soon as the retained
command arrives (about a second in). Diagnostic sensors carry an
`expire_after` of 3× the heartbeat (36 h), so a dead battery shows as
"unavailable" rather than frozen values.

Discovery configs are sent once per power-life, and again after any real
(non-deep-sleep) reset so new entities appear after a flash without a power
cycle.

## Status LED

| LED | Meaning |
|---|---|
| Off | Normal idle state |
| Lit in the selected **LED Color** for **LED On Time** seconds (default 5s) | A doorbell press — lights immediately on wake, even if WiFi/MQTT are down; a press while lit restarts the timer |
| Solid blue | OTA window open |
| Pulsing blue | Setup portal open |

The color (a Home Assistant select: White, Red, Green, Blue, Yellow, Orange,
Purple, Cyan, Pink; default White) and on-time (a number, 1-60 s, default 5)
are chosen from HA, and the overall **LED Brightness** applies to all of the
above. All three persist in NVS.

## Diagnostics

Published on every wake (press or heartbeat). Connect fail count resets to 0
on the next fully-acknowledged cycle; total fail count is lifetime (kept in
RTC memory, so it clears if the battery is fully disconnected). Boot count
increments once per wake, not per power-up. Uptime uses the RTC counter, so
it continues across deep-sleep wakes and zeroes on power-on, manual reset,
brownout, watchdog, software restart, or a dead-and-replaced battery.

## Config file

`config.h` (gitignored) holds the OTA password, setup-portal AP
password/timeout, device identity constants (`DEVICE_MANUFACTURER`,
`DEVICE_MODEL`, `DEVICE_HW_VERSION`, `FIRMWARE_VERSION`), button-hold
thresholds, and the deep-sleep/MQTT timing constants — copy
`config.h.example` to `config.h` and fill in real values. WiFi credentials,
MQTT broker settings, and this device's own name/ID are **not** here —
they're runtime settings, configured through the setup portal and persisted
in NVS.

## Version History

| Version | Date | Changes |
|---|---|---|
| v1.0.0 | 2026-09-22 | Initial release: WiFiManager setup portal, doorbell switch (GPIO0) doubling as the setup control (button-hold gestures, same convention as `smart_switch`), doorbell press modeled as an MQTT `event` entity, full diagnostic set (WiFi signal, reset reason, boot count, connect/total fail counts, firmware version), NVS-persisted LED brightness. |
| v1.0.1 | 2026-09-25 | Added an `Uptime` diagnostic sensor (seconds since boot, `device_class: duration`). Uses `esp_timer_get_time()` (64-bit) rather than `millis()`, so it zeroes on any reboot or power loss but never wraps back to zero on its own at ~49.7 days. |
| v1.1.0 | 2026-09-25 | LED is now off when idle (previously solid green while WiFi was connected) and lights up on a doorbell press instead of a brief white flash. New HA `select` entity **LED Color** and **LED On Time** number (1-60 s, default 5), both NVS-persisted and applied immediately; light-up is non-blocking and a press while lit restarts the timer. |
| v2.0.0 | 2026-09-25 | **Battery-powered, deep-sleep rewrite.** Sleeps until the doorbell switch is pressed (GPIO0 level wake) or a 12 h heartbeat fires; LED lights instantly on wake, the ring is published first, and the device stays awake while the LED is lit. Built on `door_sensor`'s deep-sleep machinery (RTC-kept counters, awake watchdog, blocking connect with PUBACK-confirmed publishes, one full connect retry on a press). Uptime now uses the RTC counter so it continues across deep sleep and zeroes only on a real reset/power loss; boot count is now wakes since power loss; discovery re-sent after any real reset. Control entities (LED brightness/color/on-time) are retained commands picked up on the next wake. New retained **OTA Update** switch replaces the always-on `ArduinoOTA` and the OTA Restart button (removed). Setup portal is unchanged (10 s hold; 5 s in-portal factory reset). Adds deep-sleep/MQTT timing constants to `config.h`. Breaking: needs an external ~10k pull-up on GPIO0; no battery monitoring yet. |
