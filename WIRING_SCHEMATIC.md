# PET Bottle Collector Robot — Wiring Schematic

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    POWER DISTRIBUTION                           │
│                                                                 │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────────┐       │
│  │ 12V Motor│    │ 5V ESP32 │    │ 5V Raspberry Pi 5    │       │
│  │ Battery  │    │ Supply   │    │ USB-C Power Supply   │       │
│  └────┬─────┘    └────┬─────┘    └──────────┬───────────┘       │
│       │               │                     │                   │
│       ▼               ▼                     ▼                   │
│  BTS7960 x2      ESP32 DevKit         Raspberry Pi 5            │
│  L298N x1        (WiFi AP)            + Hailo-8 NPU             │
│  Hybrid Drive                         + Pi Camera Module 3      │
└─────────────────────────────────────────────────────────────────┘
```

## ESP32 Pin Assignment

```
                    ┌──────────────────┐
                    │     ESP32        │
                    │    DevKit V1     │
                    │                  │
          3.3V ─────┤ 3V3         VIN ├───── 5V Input
           GND ─────┤ GND         GND ├───── GND
                    │                  │
   Buzzer ──────────┤ GPIO 3    GPIO 36├───── Limit SW1 (BOTTOM) ←─┐
   L_RPWM ──────────┤ GPIO 4    GPIO 39├───── Limit SW2 (TOP)   ←─┤
   L_LPWM ──────────┤ GPIO 5    GPIO 35├───── US Echo S2 (RIGHT)   │
                    │                  │                            │
   US TRIG ─────────┤ GPIO 12   GPIO 34├───── US Echo S4 (LEFT)    │
   Servo L ─────────┤ GPIO 13   GPIO 33├───── US Echo S1 (FRONT)   │
   ARM PWM ─────────┤ GPIO 14   GPIO 32├───── US Echo S3 (BACK)    │
   IR Prox ─────────┤ GPIO 15   GPIO 27├───── R_EN                 │
   R_RPWM ──────────┤ GPIO 16   GPIO 26├───── L_EN                 │
   R_LPWM ──────────┤ GPIO 17   GPIO 25├───── SWING IN1            │
   Servo R ─────────┤ GPIO 18   GPIO 23├───── SWING IN2            │
   ARM DIR ─────────┤ GPIO 19   GPIO 22├───── LCD SCL              │
                    │                  │                            │
   LCD SDA ─────────┤ GPIO 21    GPIO 2├───── SWING EN             │
                    │                  │                            │
                    └──────────────────┘                            │
                                                                    │
                                               10kΩ pull-up ───────┘
                                               to 3.3V (required
                                               for GPIO 36 & 39)
```

## Drive Wheels — BTS7960B x2

```
                    12V Motor Supply
                         │
            ┌────────────┼────────────┐
            │            │            │
     ┌──────┴──────┐           ┌──────┴──────┐
     │  BTS7960B   │           │  BTS7960B   │
     │  LEFT WHEEL │           │  RIGHT WHEEL│
     │             │           │             │
     │ RPWM ← GPIO 4 (ch0)    │ RPWM ← GPIO 16 (ch2)
     │ LPWM ← GPIO 5 (ch1)    │ LPWM ← GPIO 17 (ch3)
     │ R_EN ← GPIO 26         │ R_EN ← GPIO 27
     │ L_EN ← GPIO 26         │ L_EN ← GPIO 27
     │ VCC  ← 5V              │ VCC  ← 5V
     │ GND  → GND             │ GND  → GND
     │             │           │             │
     │   ┌─────┐   │           │   ┌─────┐   │
     │   │MOTOR│   │           │   │MOTOR│   │
     │   │ LEFT│   │           │   │RIGHT│   │
     │   │     │   │           │   │(INV)│   │  ← Inverted polarity
     │   └─────┘   │           │   └─────┘   │    corrected in SW
     └─────────────┘           └─────────────┘

     LEDC: Timer 0 (ch0+ch1)   Timer 1 (ch2+ch3)
            1 kHz, 8-bit        1 kHz, 8-bit
     Speed: Fixed 60/255 fwd/bwd, Turn default 50 adjustable
     Max PWM cap: 80/255
```

## Arm Lift — e-Gizmo Hybrid Drive 2R0

```
     12V Motor Supply
          │
    ┌─────┴──────────────┐
    │  Hybrid Drive 2R0  │
    │  (~6A rated)       │
    │                    │
    │  PWM+ ← GPIO 14 (ch4, Timer 2)
    │  DIR+ ← GPIO 19
    │  PWM- → GND        │
    │  DIR- → GND        │
    │                    │
    │  ┌──────────────┐  │
    │  │  ARM MOTOR   │  │
    │  │  (geared DC) │  │
    │  └──────────────┘  │
    └────────────────────┘

    ⚠ IMPORTANT: Never flip DIR while PWM > 0
      Sequence: PWM=0 → wait 20ms → flip DIR → wait 10ms → ramp PWM

    Speeds: Default 20, Manual Down 80, Manual Up 180, Pickup Lift 255
    Pulsed lowering: 200ms on / 300ms off (prevents motor damage)

    Limit Switches:
    ┌────────────────────────────────────────────┐
    │  SW1 (BOTTOM) ─── GPIO 36 ──┬── 10kΩ ── 3.3V
    │  SW2 (TOP)    ─── GPIO 39 ──┴── 10kΩ ── 3.3V
    │  (GPIO 36/39 are input-only, NO internal pull-up)
    └────────────────────────────────────────────┘
```

## Swing Drive — L298N #2

```
     12V Motor Supply
          │
    ┌─────┴─────────────┐
    │      L298N #2     │
    │                   │
    │  ENA ← GPIO 2 (ch5, Timer 2)
    │  IN1 ← GPIO 25   │
    │  IN2 ← GPIO 23   │
    │  5V  → (onboard)  │
    │  GND → GND        │
    │                   │
    │  ┌─────────────┐  │
    │  │ SWING MOTOR │  │
    │  │ (rotates Pi │  │
    │  │  platform   │  │
    │  │  180°)      │  │
    │  └─────────────┘  │
    └───────────────────┘

    LEDC: Timer 2 (ch5, paired with ARM ch4)
           1 kHz, 8-bit
    ⚠ GPIO 2 is boot strapping pin (pull-down) — safe for EN (starts LOW)
```

## Servos — Scoopers

```
    5V Supply
      │
    ┌─┴────────────────────────────────────┐
    │                                      │
    │  ┌──────────┐      ┌──────────┐      │
    │  │ SERVO L  │      │ SERVO R  │      │
    │  │ (left    │      │ (right   │      │
    │  │ scooper) │      │ scooper) │      │
    │  │          │      │          │      │
    │  │ SIG ← GPIO 13   SIG ← GPIO 18    │
    │  │ VCC ← 5V        VCC ← 5V         │
    │  │ GND → GND       GND → GND        │
    │  └──────────┘      └──────────┘      │
    └──────────────────────────────────────┘

    LEDC: Timer 3 (ch6 + ch7)
           50 Hz, 12-bit
    Duty: 102 (0°) to 512 (180°)

    Pickup positions:
      OPEN:  Left=0°   Right=180°  (spread apart)
      CLOSE: Left=180° Right=0°    (sweep inward to scoop)

    Timing:
      Scoop close wait: 1000ms (1 sec)
      Drop open wait:   10000ms (10 sec)
```

## Ultrasonic Sensors — 4x HC-SR04

```
                        5V
                         │
    ┌────────────────────┼────────────────────┐
    │                    │                    │
    │  ┌───────┐  ┌───────┐  ┌───────┐  ┌───────┐
    │  │  S1   │  │  S2   │  │  S3   │  │  S4   │
    │  │ FRONT │  │ RIGHT │  │ BACK  │  │ LEFT  │
    │  │       │  │       │  │       │  │       │
    │  │TRIG←──┼──┼──GPIO 12 (shared)──┼──┤      │
    │  │ECHO→GPIO33 ECHO→GPIO35 ECHO→GPIO32 ECHO→GPIO34
    │  │VCC←5V │  │VCC←5V │  │VCC←5V │  │VCC←5V │
    │  │GND→GND│  │GND→GND│  │GND→GND│  │GND→GND│
    │  └───────┘  └───────┘  └───────┘  └───────┘
    │
    │  Read interval: 60ms per sensor (round-robin)
    │  Thresholds: <60cm STOP, <100cm SLOW, <150cm WARN
    └─────────────────────────────────────────────┘
```

## IR Proximity Sensor — E18-D80NK (Bin Full Detection)

```
    5V
     │
    ┌┴──────────────┐
    │  E18-D80NK    │
    │  IR Proximity │
    │               │
    │  OUT → GPIO 15│  (LOW = object detected = bin full)
    │  VCC ← 5V    │
    │  GND → GND   │
    └───────────────┘

    Mounted inside collection bin
    irProx=true for N consecutive reads → MISSION_COMPLETE
```

## LCD Display — I2C 16x2

```
    5V
     │
    ┌┴───────────┐
    │  LCD 16x2  │
    │  I2C       │
    │            │
    │  SDA ← GPIO 21
    │  SCL ← GPIO 22
    │  VCC ← 5V │
    │  GND → GND│
    └────────────┘
```

## Buzzer

```
    ┌────────────┐
    │   Buzzer   │
    │            │
    │  + ← GPIO 3
    │  - → GND  │
    └────────────┘

    ⚠ GPIO 3 is ESP32's default UART RX — buzzer blocks serial RX
```

## Raspberry Pi 5 + Hailo-8 NPU

```
    USB-C Power
         │
    ┌────┴──────────────────────────────────┐
    │         Raspberry Pi 5                │
    │                                       │
    │  ┌──────────┐    ┌─────────────┐      │
    │  │ Hailo-8  │    │ Pi Camera   │      │
    │  │ NPU      │    │ Module 3    │      │
    │  │ (M.2)    │    │ (IMX708)    │      │
    │  │          │    │ Wide-angle  │      │
    │  │ YOLO v5  │    │ via CSI     │      │
    │  │ YOLO v7  │    │             │      │
    │  │ YOLO v8  │    │             │      │
    │  └──────────┘    └─────────────┘      │
    │                                       │
    │  WiFi ──► ESP32 AP (PetBottle_Robot)  │
    │           IP: 192.168.4.4             │
    │                                       │
    │  Communication: WiFi HTTP             │
    │    Commands: GET 192.168.4.1/cmd?c=X  │
    │    Sensors:  GET 192.168.4.1/sensor   │
    │                                       │
    │  Services:                            │
    │    navigator.py  → port 5000 (MJPEG)  │
    │    pi_admin.py   → port 8080 (dashboard)│
    └───────────────────────────────────────┘
```

## LEDC Timer/Channel Assignment

```
    ┌─────────┬──────────┬───────────────────────────────┐
    │ Timer   │ Channels │ Assignment                    │
    ├─────────┼──────────┼───────────────────────────────┤
    │ Timer 0 │ ch0 + ch1│ Left wheel (1kHz, 8-bit)     │
    │ Timer 1 │ ch2 + ch3│ Right wheel (1kHz, 8-bit)    │
    │ Timer 2 │ ch4 + ch5│ Arm lift + Swing (1kHz, 8-bit)│
    │ Timer 3 │ ch6 + ch7│ Servos L + R (50Hz, 12-bit)  │
    └─────────┴──────────┴───────────────────────────────┘

    ⚠ Paired channels MUST share same frequency & resolution
```

## Pickup Sequence Timing

```
    START
      │
      ▼
    LOWERING ──── Arm pulses down (200ms on / 300ms off)
      │            until SW1 (bottom limit) triggers
      ▼
    SCOOPING ──── Servos close to grab bottle
      │            Wait: 1000ms (1 sec)
      ▼
    LIFTING ───── Arm up at full speed (255)
      │            until SW2 (top limit) triggers
      ▼
    DROPPING ──── Servos open to release into bin
      │            Wait: 10000ms (10 sec)
      ▼
    DONE ───────── Check IR proximity (bin full?)
      │
      ├── Bin NOT full → back to SCANNING
      └── Bin full → MISSION_COMPLETE (sticky)

    Timeout: 30 seconds max per pickup attempt
```
