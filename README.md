# PET Bottle Collector Robot

An autonomous robot that detects, approaches, and collects PET bottles using computer vision and a robotic arm.

## Architecture

```
Raspberry Pi 5                          ESP32 (WiFi AP)
+------------------------+    WiFi     +---------------------------+
| CSI Camera             |             | 192.168.4.1               |
| Hailo-8 NPU (YOLOv5/7/8)            |                           |
|                        | GET /sensor | HC-SR04 x2 (ultrasonic)   |
| camera.py (detection) <----JSON------> Left/Right distance (cm)  |
|                        |             |                           |
| navigator.py (brain)   | GET /cmd   | BTS7960B x3 (motors)      |
|   Bottle found? -------command------>   Drive wheels L/R         |
|   Obstacle? -----------command------>   Arm lift up/down         |
|   Pickup! -------------cmd?c=P----->   Servos open/close        |
|                        |             |   Stepper base rotate     |
+------------------------+             |   LCD 16x2 + Buzzer       |
                                       +---------------------------+
```

## Hardware

| Component | Qty | Purpose |
|-----------|-----|---------|
| ESP32 Dev Board | 1 | Motor/sensor controller, WiFi AP |
| Raspberry Pi 5 | 1 | Vision + navigation brain |
| Hailo-8 NPU (M.2) | 1 | YOLO AI accelerator |
| Pi Camera V2/V3 | 1 | Bottle/obstacle detection |
| BTS7960B Motor Driver | 3 | Drive wheels (x2) + arm lift |
| L298N Motor Driver | 1 | NEMA 17 stepper (base rotation) |
| HC-SR04 Ultrasonic | 2 | Obstacle distance sensing |
| 24V DC Motors | 2 | Drive wheels |
| 58SW31ZY DC Geared Motor | 1 | Arm lift (16 RPM) |
| NEMA 17 Stepper | 1 | Base platform rotation |
| MG996R Servo | 2 | Left/right arm gripper |
| LCD I2C 16x2 | 1 | Status display |
| 24V 30Ah Battery | 1 | Main power |
| DC-DC 24V to 5V 5A | 1 | Logic/servo power |

## Software

### ESP32 Firmware (`robotic_arm/robotic_arm.ino`)

Complete motor controller with:
- 3x BTS7960B PWM motor control (drive wheels + arm lift)
- L298N stepper control via AccelStepper (base rotation)
- 2x MG996R servo control (gripper open/close)
- 2x HC-SR04 ultrasonic sensors
- LCD I2C status display
- WiFi Access Point + WebServer
- Web control dashboard with touch-friendly UI
- HTTP API (`/cmd`, `/sensor`)
- OTA firmware update (`/ota`)
- Full pickup sequence (lower arm, grab, lift, rotate, deposit, home)
- Emergency stop

### Raspberry Pi Software

| File | Purpose |
|------|---------|
| `camera.py` | YOLO v5/v7/v8 model stitching with Hailo-8 NPU, real-time bottle detection, orientation analysis (upright/laydown/tilted), person filtering |
| `navigator.py` | Autonomous state machine: roaming (S-curve), obstacle avoidance, bottle approach, alignment, pickup triggering |
| `server.py` | Git-based OTA update server for remote code deployment |

### YOLO Models (Hailo HEF format)

| Model | Size | Specialization |
|-------|------|----------------|
| `yolov5s.hef` | 16 MB | FAR range (bottle < 10% of frame) |
| `yolov7.hef` | 55 MB | MID range (10-40% of frame) |
| `yolov8s-coco.hef` | 35 MB | CLOSE range (> 25% of frame) |

All models detect COCO class 39 (bottle). Range-specialized ensemble mode runs all three and merges results via NMS for robust detection at any distance.

## How It Works

### Autonomous Loop

```
+--------+  bottle   +------------+  centered  +----------+
| ROAMING| detected  | APPROACHING|  & close   | ALIGNING |
| S-curve+---------->| drive to   +----------->| fine     |
| explore|           +------+-----+            | adjust   |
+---+----+                  |                  +----+-----+
    |                  lost bottle                  |
    | obstacle         (15 frames)              aligned
    | detected              v                       v
+---+----+           +-----+-----+         +-------+----+
|AVOIDING|           |  ROAMING  |         | PICKING UP |
| back up|           |           |         | (ESP32 'P' |
| & turn |           +-----------+         |  command)  |
+--------+                                 +-------+----+
                                                   |
                                                   v
                                             +-----+-----+
                                             |  ROAMING  |
                                             +-----------+
```

### Obstacle Avoidance

| Ultrasonic Distance | Action |
|---------------------|--------|
| < 15 cm | Emergency stop, back up, turn away |
| 15-40 cm | Slow down to 50/255 |
| 40-60 cm | Steer away from closer side |
| > 60 cm | Full roaming speed (80/255) |

### Pickup Sequence (Command: `P`, ~15 seconds)

1. Stop wheels, open both servos
2. Lower arm (speed 150, 3 seconds)
3. Close both servos (grab bottle)
4. Lift arm (speed 200, 3 seconds)
5. Rotate base 180 degrees (deposit side)
6. Lower slightly, open servos (release into bin)
7. Lift arm, return base to 0, servos open

## Setup

### ESP32

1. Install [Arduino IDE](https://www.arduino.cc/en/software) or [arduino-cli](https://arduino.github.io/arduino-cli/)
2. Install ESP32 board package (`esp32:esp32`)
3. Install libraries: `LiquidCrystal I2C`, `AccelStepper`
4. Open `robotic_arm/robotic_arm.ino`
5. Set partition scheme: **Minimal SPIFFS (1.9MB APP with OTA)**
6. Flash via USB (first time), then use OTA for subsequent updates

### Raspberry Pi

1. Install Hailo runtime and HailoRT Python bindings
2. Install dependencies: `pip install numpy opencv-python picamera2 requests`
3. Place `.hef` model files in the project root
4. Connect Pi to ESP32 WiFi: `PetBottle_Robot` / `petbottle123`

### Running

```bash
# On Raspberry Pi — start autonomous mode
python3 navigator.py

# Or camera-only mode (detection without driving)
python3 camera.py
```

### Keyboard Controls (during camera/navigator view)

| Key | Action |
|-----|--------|
| `5` | Switch to YOLOv5 model |
| `7` | Switch to YOLOv7 model |
| `8` | Switch to YOLOv8 model |
| `A` | Ensemble mode (all models) |
| `S` | Stop/resume navigator |
| `Q` / `ESC` | Quit |

## WiFi & API

- **SSID:** `PetBottle_Robot`
- **Password:** `petbottle123`
- **IP:** `192.168.4.1`

| Endpoint | Description |
|----------|-------------|
| `http://192.168.4.1/` | Web control dashboard |
| `http://192.168.4.1/cmd?c=F80` | Send command (forward at speed 80) |
| `http://192.168.4.1/sensor` | Get sensor data (JSON) |
| `http://192.168.4.1/ota` | Upload new firmware (.bin) |

## Firmware Update (OTA)

After the first USB flash, all subsequent updates can be done wirelessly:

1. Edit `robotic_arm/robotic_arm.ino`
2. Double-click `flash_ota.bat` (Windows) — compiles, switches WiFi, uploads, reconnects
3. Or manually: compile to `.bin`, connect to `PetBottle_Robot` WiFi, upload at `http://192.168.4.1/ota`

## Wiring Schematic

See `PetBottle_Robotic_Arm_Schematic.pdf` for complete wiring diagrams including:
- ESP32 GPIO pin assignments (21 pins)
- BTS7960B motor driver connections (3 boards)
- L298N stepper driver wiring
- HC-SR04 ultrasonic sensor wiring (with voltage divider)
- Servo, LCD, buzzer connections
- Power distribution (24V battery + DC-DC converter)

## License

This project was built for educational purposes by [Emathrix Academy](https://github.com/emathrixacademy).
