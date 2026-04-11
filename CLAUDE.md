# PET Bottle Collector Robot

## Project Overview

Autonomous PET bottle collection robot using dual processors:
- **ESP32**: Motor control, sensors, WiFi AP, LCD — firmware in `robotic_arm/robotic_arm.ino`
- **Raspberry Pi 5 + Hailo-8 NPU**: YOLO vision + navigation brain — `camera.py`, `navigator.py`

Communication: Pi connects to ESP32 WiFi AP (`PetBottle_Robot` / `petbottle123` / `192.168.4.1`) and sends HTTP commands.

## Build & Flash

### Compile (arduino-cli)
```
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --output-dir build_robotic robotic_arm/robotic_arm.ino
```

### Flash via OTA (preferred)
Double-click `flash_ota.bat` — auto compiles, switches WiFi, uploads, reconnects.

### Flash via USB (fallback)
```
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs robotic_arm/robotic_arm.ino
```
Hold BOOT button on ESP32 during upload if it fails to connect.

### ESP32 Arduino core
- arduino-cli has v2.0.17 installed — code uses v2 API: `ledcSetup()` + `ledcAttachPin()` + `ledcWrite(channel, duty)`
- Arduino IDE has v3.3.7 installed — if editing for IDE, use v3 API: `ledcAttach(pin, freq, res)` + `ledcWrite(pin, duty)`
- Do NOT mix v2 and v3 LEDC APIs in the same sketch

## Key Files

| File | Purpose |
|------|---------|
| `robotic_arm/robotic_arm.ino` | **Main ESP32 firmware** — motors, servos, sensors, WiFi, OTA (the only sketch to flash) |
| `camera.py` | YOLO v5/v7/v8 model stitching, detection, postprocessing (runs on Pi) |
| `navigator.py` | Autonomous brain: roam, avoid, approach, align, pickup (runs on Pi) |
| `server.py` | OTA update server for remote deployment (runs on Pi) |
| `flash_ota.bat` | One-click compile + OTA flash from Windows |
| `petbottle/petbottle.ino` | **LEGACY — do not use.** Sensor-only sketch, pins conflict with robotic_arm.ino |

## ESP32 Pin Map (21 GPIOs)

### Drive Wheels (BTS7960B x2)
| Function | GPIO | LEDC Channel |
|----------|------|-------------|
| Left RPWM (fwd) | 4 | 0 |
| Left LPWM (rev) | 5 | 1 |
| Left EN | 26 | — |
| Right RPWM (fwd) | 16 | 2 |
| Right LPWM (rev) | 17 | 3 |
| Right EN | 27 | — |

### Arm Lift (BTS7960B #3)
| Function | GPIO | LEDC Channel |
|----------|------|-------------|
| Lift RPWM (up) | 2 | 4 |
| Lift LPWM (down) | 25 | 5 |
| Lift EN | 19 | — |

### Other
| Function | GPIO |
|----------|------|
| Stepper IN1/IN2/IN3/IN4 | 32, 33, 22, 21 |
| Servo Left / Right | 13 (ch6) / 18 (ch7) |
| LCD SDA / SCL | 14 / 15 |
| Buzzer | 23 |
| Ultrasonic TRIG (shared) | 12 |
| Ultrasonic ECHO L / R | 34 / 35 |

**Note:** Right Wheel and Arm Lift pins were swapped from the original schematic to diagnose a BTS7960B hardware issue. The schematic PDF still shows the original (2/25/19 = Right, 16/17/27 = Lift).

## LEDC Channel Pairing Rules

ESP32 LEDC channels share hardware timers in pairs: ch0/1, ch2/3, ch4/5, ch6/7. Both channels in a pair MUST use the same frequency and resolution. Current layout:
- Timer 0: ch0 + ch1 → Left wheel (1 kHz, 8-bit)
- Timer 1: ch2 + ch3 → Right wheel (1 kHz, 8-bit)
- Timer 2: ch4 + ch5 → Arm lift (1 kHz, 8-bit)
- Timer 3: ch6 + ch7 → Servos (50 Hz, 12-bit)

Never assign channels out-of-order or across timer pairs for different motors.

## ESP32 HTTP API

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Web control dashboard |
| `/cmd?c=<CMD>` | GET | Execute command (F80, J80, TL60, TR60, X, P, H, E, etc.) |
| `/sensor` | GET | JSON: `{left, right, wheels:{left,right}, base, lift, motors}` |
| `/ota` | GET/POST | Firmware upload page |

## Command Reference

| Command | Action |
|---------|--------|
| `F<speed>` | Forward (0-255) |
| `J<speed>` | Backward |
| `TL<speed>` / `TR<speed>` | Turn left / right |
| `W<l>,<r>` | Raw wheel (-255 to 255) |
| `X` | Stop wheels |
| `B<angle>` / `BH` | Rotate base / home base |
| `U<speed>` / `D<speed>` / `S` | Lift up / down / stop |
| `L<angle>` / `R<angle>` | Left / right servo |
| `O` / `C` | Both servos open / close |
| `P` | Full pickup sequence (~15s) |
| `H` | Home all motors |
| `E` | Emergency stop |

## Navigator State Machine

`ROAMING` → bottle detected → `APPROACHING` → centered & close → `ALIGNING` → aligned → `PICKING_UP` → done → `ROAMING`

At any point: ultrasonic < 15 cm → `AVOIDING` (back up + turn) → `ROAMING`

## Known Issues

- Right wheel reverse was failing due to LEDC channel-pair timer conflict (fixed: channels now in natural pair order 0/1, 2/3, 4/5)
- GPIO 2 is a boot strapping pin with pull-down — avoid using it for motor PWM (currently used for Arm Lift RPWM which is safe since EN starts LOW)
- `petbottle/petbottle.ino` has conflicting GPIO assignments (5, 18, 19, 32, 33 overlap with robotic_arm.ino) — never flash it on the robot's ESP32
