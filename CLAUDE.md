# PET Bottle Collector Robot

## Project Overview

Autonomous PET bottle collection robot using dual processors:
- **ESP32**: Motor control, sensors, WiFi AP, LCD — test firmware in `esp32_test/esp32_test.ino`
- **Raspberry Pi 5 + Hailo-8 NPU**: YOLO vision + navigation brain — `camera.py`, `navigator.py`

Communication: Pi <-> ESP32 is **WiFi HTTP**. Both devices connect to a mobile hotspot (`petbottle_hotspot` / `petbottle123`) as STA clients on the same local network. ESP32 uses static IP `192.168.43.100`. The Pi sends commands via `GET /cmd?c=<CMD>` and polls sensor data via `GET /sensor` every 200ms.

## Build & Flash

### Compile (arduino-cli)
```
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --output-dir build_test esp32_test/esp32_test.ino
```

### Flash via USB
```
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --input-dir build_test
```
Hold BOOT button on ESP32 during upload if it fails to connect.

### Flash via OTA
Connect to the same mobile hotspot, open `http://192.168.43.100/ota`, upload `.bin` file.

### ESP32 Arduino core
- arduino-cli has v2.0.17 installed — code uses v2 API: `ledcSetup()` + `ledcAttachPin()` + `ledcWrite(channel, duty)`
- Arduino IDE has v3.3.7 installed — if editing for IDE, use v3 API: `ledcAttach(pin, freq, res)` + `ledcWrite(pin, duty)`
- Do NOT mix v2 and v3 LEDC APIs in the same sketch

## Key Files

| File | Purpose |
|------|---------|
| `esp32_test/esp32_test.ino` | **Active ESP32 firmware** — combined test with WiFi STA (hotspot), web UI, OTA, camera feed |
| `camera.py` | YOLO v5/v7/v8 model support, detection, postprocessing (runs on Pi) |
| `navigator.py` | Autonomous brain + Flask MJPEG video stream on port 5000 (runs on Pi) |
| `pi_admin.py` | Admin dashboard on port 8080 — proxies navigator APIs, data recording, model switching (runs on Pi) |
| `server.py` | OTA update server for remote deployment (runs on Pi) |
| `flash_ota.bat` | One-click compile + OTA flash from Windows |
| `petbottle/petbottle.ino` | **LEGACY — do not use.** Sensor-only sketch, pins conflict |

## ESP32 Pin Map (New Schematic)

### Drive Wheels (BTS7960B x2)
| Function | GPIO | LEDC Channel |
|----------|------|-------------|
| Left RPWM (fwd) | 4 | 0 |
| Left LPWM (rev) | 5 | 1 |
| Left EN | 26 | — |
| Right RPWM (fwd) | 16 | 2 |
| Right LPWM (rev) | 17 | 3 |
| Right EN | 27 | — |

### Arm Lift (L298N #1)
| Function | GPIO | LEDC Channel |
|----------|------|-------------|
| ARM EN | 14 | 4 |
| ARM IN1 | 15 | — |
| ARM IN2 | 19 | — |

### Swing Drive (L298N #2) — rotates Pi+Camera platform 180°
| Function | GPIO | LEDC Channel |
|----------|------|-------------|
| SWING EN | 2 | 5 |
| SWING IN1 | 25 | — |
| SWING IN2 | 23 | — |

### Other
| Function | GPIO |
|----------|------|
| Servo Left / Right | 13 (ch6) / 18 (ch7) |
| LCD SDA / SCL | 21 / 22 |
| Buzzer | 3 |
| Ultrasonic TRIG (shared) | 12 |
| Ultrasonic S1-Front/S2-Right/S3-Back/S4-Left | 33 / 35 / 32 / 34 |
| Limit Switch 1 / 2 | 36 / 39 |
| E18-D80NK IR proximity (bin-full sensor) | 15 |

**Note:** GPIO 36 and 39 are input-only with no internal pull-up. External 10kΩ pull-up resistors to 3.3V are required for the limit switches.

## LEDC Channel Pairing Rules

ESP32 LEDC channels share hardware timers in pairs: ch0/1, ch2/3, ch4/5, ch6/7. Both channels in a pair MUST use the same frequency and resolution. Current layout:
- Timer 0: ch0 + ch1 → Left wheel (1 kHz, 8-bit)
- Timer 1: ch2 + ch3 → Right wheel (1 kHz, 8-bit)
- Timer 2: ch4 + ch5 → Arm lift (ch4) + Swing drive (ch5) via L298N (1 kHz, 8-bit)
- Timer 3: ch6 + ch7 → Servos (50 Hz, 12-bit)

Never assign channels out-of-order or across timer pairs for different motors.

## ESP32 HTTP API (used by both navigator and web debug UI)

Base URL: `http://192.168.43.100` (ESP32 static IP on hotspot)

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Web control dashboard (tabbed test UI) |
| `/cmd?c=<CMD>` | GET | Execute command (Pi navigator + web UI) |
| `/sensor` | GET | JSON: ultrasonic, limits, speeds (polled every 200ms by Pi) |
| `/cmdlog` | GET | JSON: recent Pi-ESP32 command log |
| `/ota` | GET/POST | Firmware upload page |

### Pi Direct Commands (bypass test-UI mode system)

The navigator sends these `PI`-prefixed commands so it always has motor control regardless of the ESP32's current web-UI mode:

| Command | Action |
|---------|--------|
| `PIFW<spd>` | Wheels forward at speed (0-80) |
| `PIBW<spd>` | Wheels backward |
| `PITL<spd>` | Turn left (spin in place) |
| `PITR<spd>` | Turn right (spin in place) |
| `PIDW<l>,<r>` | Differential wheel (left_spd, right_spd, range -80..80) |
| `PIX` | Stop wheels |
| `PISTOP` | Emergency stop all motors/servos/buzzer |
| `P` | Start pickup sequence (also works from any mode) |
| `PA` | Abort pickup sequence |

## Navigator State Machine

`WAITING` → user starts → `SCANNING` → bottle detected → `VERIFYING` → confirmed → `APPROACHING` → centered & close → `ALIGNING` → aligned → `PICKING_UP` → done → check bin

After each successful `PICKING_UP`: navigator waits ~1.5 s for the dropped bottle to settle, then samples the E18-D80NK IR proximity reading (`irProx` field in the JSON sensor stream). If `irProx=true` for N consecutive reads → `MISSION_COMPLETE` (terminal). Else → back to `SCANNING`.

`MISSION_COMPLETE` is sticky — operator must press STOP (→ `WAITING`) and START to begin a new mission. This forces a deliberate "bin emptied" confirmation.

At any point: ultrasonic < 60 cm → `AVOIDING` (back up + turn) → `SCANNING`

## Navigator Video Stream (Pi)

navigator.py serves a Flask MJPEG stream on port 5000:
- `http://<pi-ip>:5000/video_feed` — live MJPEG stream with YOLO detections
- `http://<pi-ip>:5000/stats` — JSON detection stats (fps, bottles, model, state)

Pi IP is assigned by the phone hotspot DHCP (check `hostname -I` on the Pi).

Auto-starts on boot via systemd service `petbottle-navigator`.

## Changes from Original Schematic

- LCD moved from GPIO 14/15 to **21/22**
- Buzzer moved from GPIO 23 to **3**
- Stepper motor (NEMA 17 + L298N) **removed**, replaced by geared motor
- Arm lift changed from BTS7960B to **L298N #1** (EN=14, IN1=15, IN2=19)
- **Swing drive** added via **L298N #2** (EN=2, IN1=25, IN2=23) — rotates Pi+Camera platform 180° to scan for bottles
- Right wheel motor has **inverted polarity** — corrected in software via `setRight()` (no physical wire change)
- Drive wheel speed **capped at 80** (was 255) to prevent BTS7960 burnout
- Ultrasonic sensors increased from **2 to 4** (ECHO: 33, 35, 32, 34)
- **Limit switches** added on GPIO 36, 39 (need external 10kΩ pull-up)
- Ensemble mode **removed** — single YOLO model only (yolov8)
- Navigator now includes **Flask MJPEG video stream** on port 5000

## Known Issues

- **GPIO 0 must NOT be used for motor drivers** — it is a boot strapping pin (LOW = download mode). Previously ARM_IN1 was accidentally set to GPIO 0 in code, which prevented flashing when the L298N was connected. Fixed: ARM_IN1 = GPIO 15.
- GPIO 2 is a boot strapping pin with pull-down — used for L298N #2 EN which is safe since it starts LOW
- GPIO 36/39 have no internal pull-up — need external 10kΩ resistors for limit switches, otherwise they read as always pressed
- HEF models compiled for Hailo8L show warnings on Hailo8 (lower performance)
- `petbottle/petbottle.ino` has conflicting GPIO assignments — never flash it on the robot's ESP32
- **Motor isolation issue (WIP)**: Arm/swing/servo commands were also moving wheels — fix applied (safe boot sequence + per-command EN management) but needs testing after recompile
- **No OTA/API authentication** — anyone on the hotspot network can upload firmware or send commands. Add token auth before deploying in public
- Pickup sequence has a **30-second timeout** — if limit switch doesn't trigger, arm motor stops automatically to prevent burnout
