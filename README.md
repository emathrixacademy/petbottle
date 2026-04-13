# PET Bottle Collector Robot

An autonomous robot that detects, approaches, and collects PET bottles using computer vision and a robotic arm.

## Architecture

```
Raspberry Pi 5                          ESP32 (WiFi AP)
+------------------------+    WiFi     +---------------------------+
| CSI Camera             |             | 192.168.4.1               |
| Hailo-8 NPU (YOLOv8)  |             |                           |
|                        | GET /sensor | HC-SR04 x4 (ultrasonic)   |
| camera.py (detection) <----JSON------> 4x distance sensors (cm)  |
|                        |             |                           |
| navigator.py (brain)   | GET /cmd   | BTS7960B x2 (drive wheels)|
|   Bottle found? -------command------>   L298N #1 (arm lift)      |
|   Scan area! ----------command------>   L298N #2 (swing drive)   |
|   Obstacle? -----------command------>   Servos (gripper L/R)     |
|   Pickup! -------------cmd?c=P----->   LCD 16x2 + Buzzer        |
|                        |             |   Limit switches x2       |
| Flask MJPEG stream     |             |                           |
|   :5000/video_feed     |             |                           |
+------------------------+             +---------------------------+
```

## Hardware

| Component | Qty | Purpose |
|-----------|-----|---------|
| ESP32 Dev Board | 1 | Motor/sensor controller, WiFi AP |
| Raspberry Pi 5 | 1 | Vision + navigation brain |
| Hailo-8 NPU (M.2) | 1 | YOLO AI accelerator |
| Pi Camera V2/V3 | 1 | Bottle/obstacle detection |
| BTS7960B Motor Driver | 2 | Drive wheels (left/right) |
| L298N Motor Driver | 2 | #1: Arm lift, #2: Swing drive (Pi+Camera platform rotation) |
| HC-SR04 Ultrasonic | 4 | Obstacle distance sensing |
| 24V DC Motors | 2 | Drive wheels |
| DC Geared Motor | 1 | Arm lift (via L298N #1) |
| DC Geared Motor | 1 | Swing drive — rotates Pi+Camera platform 180° (via L298N #2) |
| MG996R Servo | 2 | Left/right arm gripper |
| Limit Switches | 2 | Arm end-stops (GPIO 36, 39) |
| LCD I2C 16x2 | 1 | Status display |
| 24V 30Ah Battery | 1 | Main power |
| DC-DC 24V to 5V 5A | 1 | Logic/servo power |

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
| Ultrasonic ECHO 1/2/3/4 | 33 / 35 / 32 / 34 |
| Limit Switch 1 / 2 | 36 / 39 |

**Notes:**
- GPIO 36 and 39 are input-only with no internal pull-up. External 10k pull-up resistors to 3.3V are required for the limit switches.
- Right wheel motor has inverted polarity — corrected in software via `setRight()` function (forward/reverse logic swapped, no physical wire change).
- Swing drive rotates the Raspberry Pi + Camera platform up to 180° to scan the area for bottles.
- **Do NOT use GPIO 0 for motor drivers** — it is a strapping pin (LOW = download mode). ARM IN1 must stay on GPIO 15.

## Software

### ESP32 Firmware

| Sketch | Purpose |
|--------|---------|
| `esp32_test/esp32_test.ino` | **Active firmware** -- combined component test with WiFi AP, web UI, OTA, live camera feed from Pi |
| `esp32_main/esp32_main.ino` | **Old firmware** -- full autonomous motor control (uses old pin map, needs updating) |

#### Test Sketch Web UI (`test_all.ino`)

Accessible at `http://192.168.4.1` when connected to `PetBottle_Robot` WiFi.

| Tab | Features |
|-----|----------|
| Camera | Live MJPEG video feed from Pi with YOLO detections, detection stats (FPS, bottles, model, state), Pi-ESP32 command log |
| Ultrasonic | Live scrolling line graph of 4 sensors, color-coded distance (red < 15cm, yellow < 50cm, green > 50cm) |
| Wheels | Forward, backward, turn left/right, stop, speed slider |
| Arm | Arm lift up/down, speed slider, limit switch status |
| Swing | Swing drive left/right (rotates Pi+Camera platform), speed slider |
| Servos | Open/close both, sweep left/right, angle sliders for each servo |
| Buzzer | Short beep, long beep, beep patterns |
| LCD | Test pattern, clear, backlight toggle |

#### Individual Test Sketches (for isolated testing)
| Folder | What it tests |
|--------|---------------|
Individual test sketches have been merged into `esp32_test/esp32_test.ino`.

### Raspberry Pi Software

| File | Purpose |
|------|---------|
| `raspi/camera.py` | YOLO v5/v7/v8 model support with Hailo-8 NPU, real-time bottle detection, orientation analysis, person filtering |
| `raspi/navigator.py` | Autonomous state machine + Flask MJPEG video stream on port 5000 |
| `raspi/web_detect.py` | Standalone web detection app (for testing without navigation) |
| `raspi/server.py` | Git-based OTA update server for remote code deployment |

### Navigator Auto-Start

The navigator runs automatically on Pi boot via systemd:

```bash
# Service: petbottle-navigator.service
# Runs: navigator.py --no-show --model yolov8
# Video stream: http://<pi-ip>:5000/video_feed
# Stats: http://<pi-ip>:5000/stats

# Manual control:
sudo systemctl start petbottle-navigator
sudo systemctl stop petbottle-navigator
sudo systemctl status petbottle-navigator
```

### YOLO Models (Hailo HEF format)

| Model | File | Purpose |
|-------|------|---------|
| YOLOv8 | `yolov8s-coco.hef` | Default model for bottle detection |
| YOLOv5 | `yolov5s.hef` | Alternative model |
| YOLOv7 | `yolov7.hef` | Alternative model |

## How It Works

### Autonomous Loop

```
+--------+  bottle   +----------+  centered  +----------+
| SCANNING| detected | VERIFYING|  confirmed | APPROACH |
| slow    +--------->| move     +----------->| drive    |
| rotate  |          | closer   |            | towards  |
+---------+          +----+-----+            +----+-----+
    |                  lost                       |
    | obstacle         bottle                  aligned
    | detected           v                        v
+---+----+         +---------+           +-------+----+
|AVOIDING|         | SCANNING|           | PICKING UP |
| back up|         |         |           | (ESP32 'P' |
| & turn |         +---------+           |  command)  |
+--------+                               +-------+----+
                                                  |
                                                  v
                                            +---------+
                                            | SCANNING|
                                            +---------+
```

### Obstacle Avoidance

| Ultrasonic Distance | Action |
|---------------------|--------|
| < 60 cm | Emergency stop, back up, turn away |
| 60-100 cm | Slow down |
| 100-150 cm | Steer away from closer side |
| > 150 cm | Full roaming speed |

## Setup

### ESP32

1. Install [arduino-cli](https://arduino.github.io/arduino-cli/) or Arduino IDE
2. Install ESP32 board package (`esp32:esp32`)
3. Install library: `LiquidCrystal I2C`
4. Compile and flash `test_all/test_all.ino`:
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --output-dir build_test esp32_test/esp32_test.ino
   arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --input-dir build_test
   ```
5. Hold BOOT button on ESP32 during USB upload if it fails to connect

### Raspberry Pi

1. Install Hailo runtime and HailoRT Python bindings
2. Install dependencies: `pip3 install numpy opencv-python picamera2 requests flask --break-system-packages`
3. Place `.hef` model files in `~/testing/`
4. Enable auto-start:
   ```bash
   sudo cp petbottle-navigator.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable petbottle-navigator
   ```

### Running

```bash
# Automatic: navigator starts on Pi boot, connects to ESP32 WiFi
# Video stream available at http://<pi-ip>:5000

# Manual:
python3 navigator.py --no-show --model yolov8
```

## WiFi & API

- **SSID:** `PetBottle_Robot`
- **Password:** `petbottle123`
- **ESP32 IP:** `192.168.4.1`
- **Pi IP:** `192.168.4.4` (on ESP32 network)

### ESP32 Endpoints
| Endpoint | Description |
|----------|-------------|
| `http://192.168.4.1/` | Web control dashboard (tabbed test UI) |
| `http://192.168.4.1/cmd?c=F80` | Send command |
| `http://192.168.4.1/sensor` | Sensor data (JSON) |
| `http://192.168.4.1/cmdlog` | Command log (JSON) |
| `http://192.168.4.1/ota` | Firmware update |

### Pi Endpoints
| Endpoint | Description |
|----------|-------------|
| `http://192.168.4.4:5000/` | Navigator stream page |
| `http://192.168.4.4:5000/video_feed` | MJPEG video stream |
| `http://192.168.4.4:5000/stats` | Detection stats (JSON) |

## Firmware Update (OTA)

After the first USB flash, updates can be done wirelessly:
1. Connect to `PetBottle_Robot` WiFi
2. Open `http://192.168.4.1/ota`
3. Upload the compiled `.bin` file

Or use `flash_ota.bat` (Windows) for one-click compile + OTA flash.

## LEDC Channel Pairing

ESP32 LEDC channels share hardware timers in pairs. Current layout:
- Timer 0: ch0 + ch1 -- Left wheel (1 kHz, 8-bit)
- Timer 1: ch2 + ch3 -- Right wheel (1 kHz, 8-bit)
- Timer 2: ch4 + ch5 -- Arm lift L298N (1 kHz, 8-bit)
- Timer 3: ch6 + ch7 -- Servos (50 Hz, 12-bit)

## License

This project was built for educational purposes by [Emathrix Academy](https://github.com/emathrixacademy).
