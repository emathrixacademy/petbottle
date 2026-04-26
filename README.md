# PET Bottle Collector Robot

An autonomous robot that detects, approaches, and collects PET bottles using computer vision, AI verification, and a robotic arm.

## Architecture

```
Mobile Phone Hotspot (2.4GHz)
    petbottle_hotspot / petbottle123
         |                    |
    WiFi STA              WiFi STA
         |                    |
Raspberry Pi 5                          ESP32
+----------------------------+  WiFi   +---------------------------+
| CSI Camera (IMX708)       |  HTTP   | DHCP IP (auto-discovered) |
| Hailo-8 NPU (YOLO)       |         |                           |
|                            | GET    | HC-SR04 x4 (ultrasonic)   |
| camera.py (detection)    <--/sensor--> 4x distance sensors (cm)  |
|                            |         |                           |
| navigator.py (brain)     | GET    | BTS7960B x2 (drive wheels)|
|   + AI vision verification-/cmd?c=--> L298N #1 (arm lift)       |
|   Bottle found? ----------command--> L298N #2 (swing drive)     |
|   Scan area! -------------command--> Servos (scooper L/R)       |
|   Obstacle? --------------command--> LCD 16x2 + Buzzer          |
|   Pickup! ----------------cmd?c=P--> Limit switches x2          |
|                            |         | E18-D80NK IR (bin full)   |
| vision_ai.py (AI verify) |         |                           |
| pi_admin.py (dashboard)  |         | Web UI (test/debug)       |
|   :5000 MJPEG stream      |         |                           |
|   :8080 control dashboard  |         |                           |
+----------------------------+         +---------------------------+
```

## Features

- **Multi-model YOLO detection** — YOLOv5, YOLOv7, YOLOv8 on Hailo-8 NPU for real-time bottle detection at 30+ FPS
- **AI-powered verification** — secondary vision AI confirms detected objects are actually PET bottles before pickup, reducing false positives
- **Scene analysis** — AI periodically scans the environment to suggest navigation direction and spot bottles YOLO may have missed
- **Obstacle intelligence** — AI identifies obstacles and suggests optimal avoidance direction, combined with 4-sensor ultrasonic array
- **Auto-discovery** — ESP32 found automatically on any phone hotspot (no static IP needed)
- **Web dashboard** — phone-accessible control panel with 3 tabs: Autonomous mode, Manual Control, Data/Recording
- **Manual controls** — touch-friendly D-pad, arm, swing, servo, buzzer controls all routed through Pi
- **Bin-full detection** — IR proximity sensor inside collection bin triggers MISSION_COMPLETE when full
- **OTA updates** — both ESP32 firmware and Pi software updatable wirelessly

## Hardware

| Component | Qty | Purpose |
|-----------|-----|---------|
| ESP32 Dev Board | 1 | Motor/sensor controller, WiFi STA |
| Raspberry Pi 5 | 1 | Vision + navigation brain |
| Hailo-8 NPU (M.2) | 1 | YOLO AI accelerator |
| Pi Camera Module 3 (Wide) | 1 | Bottle/obstacle detection (IMX708) |
| BTS7960B Motor Driver | 2 | Drive wheels (left/right) |
| L298N Motor Driver | 2 | #1: Arm lift, #2: Swing drive |
| HC-SR04 Ultrasonic | 4 | Front/Right/Back/Left obstacle sensing |
| DC Motors | 2 | Drive wheels |
| DC Geared Motor | 1 | Arm lift (via L298N #1) |
| DC Geared Motor | 1 | Swing drive — rotates Pi+Camera platform 180° |
| MG996R Servo | 2 | Left/right scooper |
| Limit Switches | 2 | Arm end-stops (top/bottom) |
| E18-D80NK IR Proximity | 1 | Bin-full detection |
| LCD I2C 16x2 | 1 | Status display |
| Buzzer | 1 | Audio feedback |
| 12V Battery | 1 | Motor power |
| 5V Supply | 1 | Logic/servo power |

## ESP32 Pin Map

### Drive Wheels (BTS7960B x2)
| Function | GPIO | LEDC Channel |
|----------|------|-------------|
| Left RPWM (fwd) | 4 | 0 |
| Left LPWM (rev) | 5 | 1 |
| Left EN | 26 | -- |
| Right RPWM (fwd) | 16 | 2 |
| Right LPWM (rev) | 17 | 3 |
| Right EN | 27 | -- |

### Arm Lift (L298N #1)
| Function | GPIO | LEDC Channel |
|----------|------|-------------|
| ARM EN | 14 | 4 |
| ARM IN1 | 15 | -- |
| ARM IN2 | 19 | -- |

### Swing Drive (L298N #2) — rotates Pi+Camera platform 180°
| Function | GPIO | LEDC Channel |
|----------|------|-------------|
| SWING EN | 2 | 5 |
| SWING IN1 | 25 | -- |
| SWING IN2 | 23 | -- |

### Other
| Function | GPIO |
|----------|------|
| Servo Left / Right | 13 (ch6) / 18 (ch7) |
| LCD SDA / SCL | 21 / 22 |
| Buzzer | 3 |
| Ultrasonic TRIG (shared) | 12 |
| Ultrasonic ECHO S1/S2/S3/S4 | 33 / 35 / 32 / 34 |
| Limit Switch 1 (bottom) / 2 (top) | 36 / 39 |
| E18-D80NK IR proximity (bin-full) | 15 |

**Notes:**
- GPIO 36 and 39 are input-only with no internal pull-up. External 10kΩ pull-up resistors to 3.3V are required for the limit switches.
- Right wheel motor has inverted polarity — corrected in software via `setRight()` function.
- Swing drive rotates the Raspberry Pi + Camera platform up to 180° to scan the area for bottles.
- **Do NOT use GPIO 0 for motor drivers** — it is a strapping pin (LOW = download mode).

## Software

### ESP32 Firmware

| Sketch | Purpose |
|--------|---------|
| `esp32_test/esp32_test.ino` | **Active firmware** — WiFi STA (hotspot), web UI, OTA, all motor/sensor controls |
| `petbottle/petbottle.ino` | **LEGACY — do not use.** Pins conflict |

### Raspberry Pi Software

| File | Purpose |
|------|---------|
| `raspi/camera.py` | YOLO v5/v7/v8 model support with Hailo-8 NPU, real-time bottle detection |
| `raspi/navigator.py` | Autonomous state machine + Flask MJPEG video stream on port 5000 |
| `raspi/vision_ai.py` | AI vision verification — confirms YOLO detections, scene analysis, obstacle ID |
| `raspi/pi_admin.py` | Web dashboard on port 8080 — manual controls, autonomous mode, data recording |
| `raspi/server.py` | OTA update server for remote code deployment |

### Pi Dashboard (port 8080)

Accessible from any phone on the hotspot at `http://<pi-ip>:8080`

| Tab | Features |
|-----|----------|
| Autonomous | Start/stop autonomous mode, live camera feed, YOLO detection stats, ultrasonic display, YOLO model switching |
| Manual Control | D-pad wheel control with speed slider, arm up/down, swing left/right, servo open/close, pickup sequence, buzzer, live sensor readout |
| Data | Data recording, Pi→ESP32 command log, navigation CSV download, Pi OTA upload |

### Pi Direct Commands (PI-prefixed, bypass ESP32 mode system)

| Command | Action |
|---------|--------|
| `PIFW<spd>` | Wheels forward at speed (0-80) |
| `PIBW<spd>` | Wheels backward |
| `PITL<spd>` | Turn left (spin in place) |
| `PITR<spd>` | Turn right (spin in place) |
| `PIDW<l>,<r>` | Differential wheel (left_spd, right_spd, range -80..80) |
| `PIX` | Stop wheels |
| `PISTOP` | Emergency stop all motors/servos/buzzer |
| `PIAU` | Arm up (full speed, until top limit) |
| `PIAD` | Arm down (pulsed, until bottom limit) |
| `PIAS` | Arm stop |
| `PISWL` | Swing platform left |
| `PISWR` | Swing platform right |
| `PISWS` | Swing stop |
| `PISO` | Servos open (scoopers spread) |
| `PISC` | Servos close (scoopers scoop) |
| `PIBZ` | Buzzer short beep |
| `PIBZL` | Buzzer long beep |
| `P` | Start pickup sequence |
| `PA` | Abort pickup sequence |

## How It Works

### Detection Pipeline

```
Camera Frame (30 FPS)
       |
   YOLO on Hailo-8 NPU (local, fast)
       |
   Bottle detected?
       |
   AI Vision Verification (cloud, smart)
       |--- "Yes, it's a PET bottle" → APPROACH
       |--- "No, that's a cup/can/shoe" → keep SCANNING
       |--- API unavailable → trust YOLO, APPROACH
```

### Autonomous State Machine

```
WAITING → START → SCANNING → bottle detected → VERIFYING
                     ↑              ↓ AI confirmed
                     |         APPROACHING → centered → ALIGNING
                     |                                      ↓
                     +←── bin not full ←── PICKING_UP ←─ aligned
                                                ↓
                                          bin full?
                                                ↓
                                        MISSION_COMPLETE
```

- **SCANNING**: Slow rotation, YOLO detection + periodic AI scene analysis
- **VERIFYING**: YOLO confirms N frames, then AI verifies it's a real PET bottle
- **APPROACHING**: Drive toward bottle using visual centering
- **ALIGNING**: Fine-tune position before pickup
- **PICKING_UP**: ESP32 runs arm down → scoop → lift → drop sequence
- **AVOIDING**: Ultrasonic < 60cm triggers backup + turn
- **MISSION_COMPLETE**: Bin full (IR sensor), requires manual reset

### Obstacle Avoidance

| Ultrasonic Distance | Action |
|---------------------|--------|
| < 60 cm | Emergency stop, back up, turn away |
| 60-100 cm | Slow down |
| 100-150 cm | Steer away from closer side |
| > 150 cm | Full roaming speed |

AI obstacle assessment identifies the obstacle type and suggests optimal avoidance direction.

## Network Setup

Both Pi and ESP32 connect to a **mobile phone hotspot** as WiFi STA clients:

- **SSID**: `petbottle_hotspot`
- **Password**: `petbottle123`
- **Band**: 2.4 GHz (required for ESP32)
- **IPs**: Assigned by phone DHCP (no static IPs)
- **Discovery**: Pi auto-scans the network for ESP32 at startup

This means the robot works on **any phone** — just name the hotspot `petbottle_hotspot` with password `petbottle123`.

### Endpoints

| Endpoint | Port | Description |
|----------|------|-------------|
| `http://<esp32-ip>/` | 80 | ESP32 web control dashboard |
| `http://<esp32-ip>/cmd?c=<CMD>` | 80 | Send command to ESP32 |
| `http://<esp32-ip>/sensor` | 80 | Sensor data (JSON) |
| `http://<esp32-ip>/ota` | 80 | ESP32 firmware update |
| `http://<pi-ip>:5000/video_feed` | 5000 | Live MJPEG stream with YOLO overlay |
| `http://<pi-ip>:5000/stats` | 5000 | Detection stats (JSON) |
| `http://<pi-ip>:8080/` | 8080 | Pi control dashboard |

## Setup

### ESP32

1. Install [arduino-cli](https://arduino.github.io/arduino-cli/) or Arduino IDE
2. Install ESP32 board package (`esp32:esp32`)
3. Install library: `LiquidCrystal I2C`
4. Compile and flash:
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --output-dir build_test esp32_test/esp32_test.ino
   arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --input-dir build_test
   ```
5. Hold BOOT button on ESP32 during USB upload if it fails to connect

### Raspberry Pi

1. Install Hailo runtime and HailoRT Python bindings
2. Install dependencies: `pip3 install numpy opencv-python picamera2 flask --break-system-packages`
3. Place `.hef` model files in `~/testing/`
4. Connect to hotspot: `sudo nmcli dev wifi connect petbottle_hotspot password petbottle123`
5. Run:
   ```bash
   cd ~/petbottle/raspi
   python3 navigator.py &
   python3 pi_admin.py
   ```

### Auto-Start on Boot

```bash
sudo cp petbottle-navigator.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable petbottle-navigator
```

## Firmware Update (OTA)

After the first USB flash, updates can be done wirelessly:
1. Connect to `petbottle_hotspot`
2. Open `http://<esp32-ip>/ota` (check ESP32 LCD for IP)
3. Upload the compiled `.bin` file

Or use `flash_ota.bat` (Windows) for one-click compile + OTA flash.

Pi software can be updated via the dashboard at `http://<pi-ip>:8080/upload`.

## YOLO Models (Hailo HEF format)

| Model | File | Purpose |
|-------|------|---------|
| YOLOv8 | `yolov8s-coco.hef` | Default model for bottle detection |
| YOLOv5 | `yolov5s.hef` | Alternative model |
| YOLOv7 | `yolov7.hef` | Alternative model |

Models can be switched live from the dashboard without restarting.

## LEDC Channel Pairing

ESP32 LEDC channels share hardware timers in pairs:
- Timer 0: ch0 + ch1 — Left wheel (1 kHz, 8-bit)
- Timer 1: ch2 + ch3 — Right wheel (1 kHz, 8-bit)
- Timer 2: ch4 + ch5 — Arm lift + Swing drive (1 kHz, 8-bit)
- Timer 3: ch6 + ch7 — Servos (50 Hz, 12-bit)

## License

This project was built for educational purposes by [Emathrix Academy](https://github.com/emathrixacademy).
