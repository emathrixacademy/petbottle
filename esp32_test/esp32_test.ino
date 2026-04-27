/*
  PET Bottle Robot — Combined Component Test
  ============================================
  WiFi STA: connects to mobile hotspot (SSID/pass below)
  Web UI:  http://<dhcp-ip>
  OTA:     http://<dhcp-ip>/ota
  Sensor:  http://<dhcp-ip>/sensor
  Serial:  115200 baud

  Select a test mode from the menu (Serial or Web), then use that mode's commands.
  Send 'M' at any time to return to the main menu.

  Pin Map (new schematic):
    LCD:         SDA=21, SCL=22
    Ultrasonic:  TRIG=12, ECHO=33,35,32,34
    BTS7960 L:   RPWM=4(ch0), LPWM=5(ch1), EN=26
    BTS7960 R:   RPWM=16(ch2), LPWM=17(ch3), EN=27
    Hybrid 2R0:  PWM=14(ch4), DIR=19  (arm lift — replaces L298N #1)
    L298N #2:    EN=2(ch5), IN1=25, IN2=23  (swing drive)
    Limit SW:    36, 15
    Servo L:     13(ch6)
    Servo R:     18(ch7)
    Buzzer:      3
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==================== WiFi STA (mobile hotspot) ====================

const char* WIFI_SSID = "petbottle_hotspot";
const char* WIFI_PASS = "petbottle123";

WebServer server(80);

// ==================== PIN DEFINITIONS ====================

// --- LCD ---
#define LCD_SDA  21
#define LCD_SCL  22

// --- Ultrasonic (shared trigger) ---
#define TRIG_PIN  12
#define ECHO_1    33
#define ECHO_2    35
#define ECHO_3    32
#define ECHO_4    34
const int echoPins[] = {ECHO_1, ECHO_2, ECHO_3, ECHO_4};
const char* echoLabels[] = {"S1(33)", "S2(35)", "S3(32)", "S4(34)"};
const int NUM_SENSORS = 4;

// --- Left Wheel (BTS7960) ---
#define L_RPWM  4
#define L_LPWM  5
#define L_EN    26
#define L_FWD_CH  0   // Timer 0
#define L_REV_CH  1   // Timer 0

// --- Right Wheel (BTS7960) ---
#define R_RPWM  16
#define R_LPWM  17
#define R_EN    27
#define R_FWD_CH  2   // Timer 1
#define R_REV_CH  3   // Timer 1

// --- Arm Lift (e-Gizmo DC Motor Hybrid Drive 2R0, ~6A) ---
// Active-HIGH wiring: PWM+ → GPIO14, DIR+ → GPIO19, PWM-/DIR- → GND
// DIR moved off GPIO 15 (boot strapping pin — was unreliable for relay drive).
// GPIO 15 (old ARM_IN1) is now FREE.
#define ARM_PWM  14
#define ARM_DIR  19
#define ARM_CH   4     // Timer 2

// --- Swing Drive (L298N #2) — rotates Pi+Camera platform 180° ---
#define SWING_EN   2
#define SWING_IN1  25
#define SWING_IN2  23
#define SWING_CH   5   // Timer 2

// --- Limit Switches ---
#define LIMIT_1  36
#define LIMIT_2  15

// --- IR Proximity (E18-D80NK, NPN open-collector, idle HIGH, LOW on detect) ---
#define IR_PROX  39

// --- Servos ---
#define SERVO_L     13
#define SERVO_R     18
#define SERVO_L_CH  6   // Timer 3
#define SERVO_R_CH  7   // Timer 3

// --- Buzzer ---
#define BUZZER  3

void executeCmd(String cmd);
volatile unsigned long lastPiCmdMs = 0;

// ==================== CONSTANTS ====================

#define WHEEL_PWM_FREQ  1000
#define WHEEL_PWM_RES   8
#define ARM_PWM_FREQ    1000
#define ARM_PWM_RES     8
#define SERVO_FREQ      50
#define SERVO_RES       12
#define DUTY_MIN        102
#define DUTY_MAX        512

// ==================== CACHED SENSOR DATA ====================
// Reads ONE sensor per tick (round-robin) to avoid ultrasonic crosstalk.
// HC-SR04 needs ~60ms between triggers on a shared TRIG line — reading
// one sensor every 60ms gives each sensor a full echo-decay window.
long cachedDist[4] = {999, 999, 999, 999};
unsigned long lastSensorRead = 0;
int nextSensorIdx = 0;              // which sensor to read next (0-3)
#define SENSOR_READ_INTERVAL 60     // one sensor every 60ms (full cycle = 240ms)

// ==================== STATE ====================

enum TestMode {
  MODE_MENU,
  MODE_ULTRASONIC,
  MODE_WHEELS,
  MODE_ARM,
  MODE_SWING,
  MODE_SERVOS,
  MODE_BUZZER,
  MODE_LCD
};

TestMode currentMode = MODE_MENU;
const int wheelSpeed = 60; // fixed drive speed — not adjustable
int turnSpeed  = 50;      // default turn speed, adjustable via +/-
int armSpeed   = 20;     // default arm PWM
int swingSpeed = 0;      // default swing PWM — 0 for safety

// --- Manual arm motion safety profile ---
// DOWN: pulsed (stop-and-go) at 50% PWM until BOTTOM limit (SW2) trips.
//       Bottom limit must be physically positioned ~5° above 0 to act as
//       the "lowest allowable" stop.
// UP:   continuous full force (180 PWM) until TOP limit (SW1) trips.
#define MANUAL_DOWN_SPEED    60
#define MANUAL_DOWN_ON_MS    200       // motor on per pulse
#define MANUAL_DOWN_OFF_MS   300       // motor off between pulses
#define MANUAL_UP_SPEED      180
bool armManualPulseActive = false;     // true while manual arm motion is active
bool armManualPulseOn     = false;     // current phase of the on/off pulse
int  armManualPulseDir    = 0;         // +1 = up, -1 = down
unsigned long armManualPulseTimer = 0; // millis() of last phase change

// ==================== PICKUP STATE MACHINE ====================
// Sequence: Pi sends "P" → lower arm slowly → limit switch → scoop →
//           lift full speed → limit switch → drop → done

enum PickupState {
  PU_IDLE,        // not running
  PU_LOWERING,    // pulsing arm down slowly (on/off)
  PU_SCOOPING,    // arm at bottom, closing scoopers
  PU_LIFTING,     // arm going up full speed, scoopers closed
  PU_DROPPING,    // arm at top, opening scoopers to drop bottle
  PU_DONE         // finished, resetting
};

PickupState puState = PU_IDLE;
unsigned long puTimer = 0;         // general purpose timer for pickup steps
unsigned long puStartTime = 0;     // when pickup sequence began (for timeout)
bool puArmOn = false;              // tracks arm pulse on/off during lowering
bool puLiftOn = false;             // tracks arm pulse on/off during lifting
#define PU_LOWER_SPEED    80
#define PU_LOWER_ON_MS    150      // arm on duration during pulsed lowering
#define PU_LOWER_OFF_MS   400      // arm off duration during pulsed lowering
#define PU_SCOOP_CLOSE_MS 1500     // time to wait after closing scoopers
#define PU_LIFT_SPEED     255
#define PU_LIFT_ON_MS     200      // arm on duration during pulsed lifting
#define PU_LIFT_OFF_MS    400      // arm off duration during pulsed lifting
#define PU_DROP_OPEN_MS   12000    // time to wait after opening scoopers
// Servos are MIRRORED — sweep inward to scoop, outward to release.
// If scooper direction is backwards, swap OPEN/CLOSE for both.
#define PU_SERVO_OPEN_L   0        // scooper open (spread apart)
#define PU_SERVO_OPEN_R   180
#define PU_SERVO_CLOSE_L  180      // scooper closed (swept inward to scoop)
#define PU_SERVO_CLOSE_R  0
#define PU_TIMEOUT_MS     45000    // abort pickup if stuck for 45 seconds (slower sequence)

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==================== COMMAND LOG BUFFER ====================

#define CMD_LOG_SIZE 30
String cmdLog[CMD_LOG_SIZE];
int cmdLogHead = 0;
int cmdLogCount = 0;

void logCmd(String source, String cmd, String result) {
  unsigned long t = millis() / 1000;
  String entry = String(t) + "s [" + source + "] " + cmd + " -> " + result;
  cmdLog[cmdLogHead] = entry;
  cmdLogHead = (cmdLogHead + 1) % CMD_LOG_SIZE;
  if (cmdLogCount < CMD_LOG_SIZE) cmdLogCount++;
}

// ==================== FORWARD DECLARATIONS ====================

void handleWebRoot();
void handleWebCmd();
void handleWebSensor();
void handleWebCmdLog();
void handleOtaPage();
void handleOtaUpload();
// ==================== HELPER FUNCTIONS ====================

long readDistance(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return duration / 58;
}

// Hard PWM cap on wheels — BTS7960 + the chosen geared motors burn out
// above ~80/255 duty under load. Every wheel-control path is funnelled
// through setLeft/setRight, so clamping here is the single chokepoint
// that keeps the cap honest no matter who calls it (PI* commands from
// the Pi, web-UI MODE_WHEELS buttons, future code, etc.).
#define WHEEL_PWM_MAX 80

// Left wheel: normal polarity
void setLeft(int spd) {
  digitalWrite(L_EN, HIGH);
  if      (spd > 0) { ledcWrite(L_FWD_CH, 0);                              ledcWrite(L_REV_CH, constrain(spd,  0, WHEEL_PWM_MAX)); }
  else if (spd < 0) { ledcWrite(L_FWD_CH, constrain(-spd, 0, WHEEL_PWM_MAX)); ledcWrite(L_REV_CH, 0);                              }
  else              { ledcWrite(L_FWD_CH, 0);                              ledcWrite(L_REV_CH, 0);                                  }
}

// Right wheel: inverted polarity (motor wired backwards)
void setRight(int spd) {
  digitalWrite(R_EN, HIGH);
  if      (spd > 0) { ledcWrite(R_FWD_CH, constrain(spd,  0, WHEEL_PWM_MAX)); ledcWrite(R_REV_CH, 0);                              }
  else if (spd < 0) { ledcWrite(R_FWD_CH, 0);                              ledcWrite(R_REV_CH, constrain(-spd, 0, WHEEL_PWM_MAX)); }
  else              { ledcWrite(R_FWD_CH, 0);                              ledcWrite(R_REV_CH, 0);                                  }
}

void stopWheels() {
  ledcWrite(L_FWD_CH, 0); ledcWrite(L_REV_CH, 0);
  ledcWrite(R_FWD_CH, 0); ledcWrite(R_REV_CH, 0);
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN, LOW);
}

// Arm lift motor (Hybrid Drive 2R0) — single DIR pin + PWM.
// IMPORTANT: direction is switched by an internal relay. Never flip DIR
// while PWM is non-zero — that hot-switches the relay contacts and burns
// them out. Always: PWM=0 → wait → flip DIR → settle → ramp PWM up.
// We track the last commanded direction so the relay-protect dance only
// runs on actual direction changes (repeated same-direction calls are
// no-ops, which is what the pickup pulse-lowering needs).
int lastArmDir = 0;
bool limitsReady = false;   // becomes true once a limit pin reads HIGH (pull-up installed)

void setArm(int dir) {
  if (dir == 0) {
    ledcWrite(ARM_CH, 0);
    // Keep lastArmDir — the relay didn't physically move, so the next
    // same-direction call should NOT trigger the relay-flip dance.
    // This is critical for pulsed lowering: relays only survive ~10^5
    // cycles, and resetting would throw the relay every ~500ms pulse.
    return;
  }
  // Direction-aware limit guard: refuse to drive INTO a pressed limit,
  // but allow motion AWAY from it.
  // Physical wiring: SW1 (GPIO 36) = BOTTOM/DOWN limit, SW2 (GPIO 39) = TOP/UP limit.
  if (limitsReady) {
    if (dir > 0 && digitalRead(LIMIT_2) == LOW) {
      Serial.println("ARM: TOP limit (SW2) pressed — UP blocked (DOWN still allowed)");
      return;
    }
    if (dir < 0 && digitalRead(LIMIT_1) == LOW) {
      Serial.println("ARM: BOTTOM limit (SW1) pressed — DOWN blocked (UP still allowed)");
      return;
    }
  }
  if (dir != lastArmDir) {
    ledcWrite(ARM_CH, 0);
    delay(20);  // let motor coast & PWM rail settle
    digitalWrite(ARM_DIR, dir > 0 ? HIGH : LOW);
    delay(10);  // relay throw time
    lastArmDir = dir;
    Serial.printf("ARM: DIR pin (GPIO19) = %s  | armSpeed=%d  | physical pin readback=%d\n",
                  dir > 0 ? "HIGH (UP)" : "LOW (DOWN)", armSpeed, digitalRead(ARM_DIR));
  }
  // Soft ramp-up when going UP to avoid current spike tripping the PSU
  if (dir > 0 && armSpeed > 20) {
    for (int s = 20; s < armSpeed; s += 5) {
      ledcWrite(ARM_CH, s);
      delay(30);
    }
  }
  ledcWrite(ARM_CH, armSpeed);
}

void stopArm() { setArm(0); }

// Swing drive motor (L298N #2) — rotates Pi+Camera platform.
// PWM clamped here so the web UI manual mode (which exposes a 0-255
// slider) can't burn the gear-train. 150/255 ≈ 60% — plenty for the
// camera platform's mass; full 255 was overkill and risked stripping.
#define SWING_PWM_MAX 150
void setSwing(int dir) {
  int spd = constrain(swingSpeed, 0, SWING_PWM_MAX);
  if (dir > 0) {
    digitalWrite(SWING_IN1, HIGH); digitalWrite(SWING_IN2, LOW);
    ledcWrite(SWING_CH, spd);
  } else if (dir < 0) {
    digitalWrite(SWING_IN1, LOW); digitalWrite(SWING_IN2, HIGH);
    ledcWrite(SWING_CH, spd);
  } else {
    digitalWrite(SWING_IN1, LOW); digitalWrite(SWING_IN2, LOW);
    ledcWrite(SWING_CH, 0);
  }
}

void stopSwing() { setSwing(0); }

int angleToDuty(int angle) {
  return map(constrain(angle, 0, 180), 0, 180, DUTY_MIN, DUTY_MAX);
}

void setServo(int ch, int angle) {
  ledcWrite(ch, angleToDuty(angle));
}

// Non-blocking buzzer — call buzzerUpdate() from loop()
unsigned long buzzerOffAt = 0;

void beep(int ms) {
  digitalWrite(BUZZER, HIGH);
  buzzerOffAt = millis() + ms;
}

void buzzerUpdate() {
  if (buzzerOffAt > 0 && millis() >= buzzerOffAt) {
    digitalWrite(BUZZER, LOW);
    buzzerOffAt = 0;
  }
}

void stopAll() {
  stopWheels();
  stopArm();
  stopSwing();
  ledcWrite(SERVO_L_CH, 0);
  ledcWrite(SERVO_R_CH, 0);
  digitalWrite(BUZZER, LOW);
}

// ==================== PICKUP SEQUENCE ====================

void pickupStart() {
  if (puState != PU_IDLE) {
    Serial.println("Pickup already running!");
    return;
  }
  Serial.println("=== PICKUP SEQUENCE START ===");
  puStartTime = millis();
  stopWheels();
  // Open scoopers first (ready to receive bottle)
  setServo(SERVO_L_CH, PU_SERVO_OPEN_L);
  setServo(SERVO_R_CH, PU_SERVO_OPEN_R);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("PICKUP: Lower");
  beep(100);
  puState = PU_LOWERING;
  puTimer = millis();
  armSpeed = PU_LOWER_SPEED;
  setArm(-1);  // down
  Serial.println("Lowering arm...");
}

void pickupAbort() {
  Serial.println("=== PICKUP ABORTED ===");
  stopArm();
  setServo(SERVO_L_CH, PU_SERVO_OPEN_L);
  setServo(SERVO_R_CH, PU_SERVO_OPEN_R);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("PICKUP ABORTED");
  puState = PU_IDLE;
  beep(300);
}

// Called every loop() iteration — non-blocking state machine
void pickupUpdate() {
  if (puState == PU_IDLE) return;

  unsigned long now = millis();

  // Safety timeout — abort if sequence takes too long (limit switch broken?)
  if (now - puStartTime >= PU_TIMEOUT_MS) {
    Serial.println("=== PICKUP TIMEOUT — ABORTING ===");
    pickupAbort();
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("PICKUP TIMEOUT!");
    return;
  }
  bool limBottom = (digitalRead(LIMIT_1) == LOW);  // GPIO 36 = SW1 = BOTTOM/DOWN
  bool limTop    = (digitalRead(LIMIT_2) == LOW);  // GPIO 39 = SW2 = TOP/UP

  switch (puState) {

    case PU_LOWERING:
      if (limBottom) {
        stopArm();
        Serial.println("Bottom limit hit — scooping!");
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("PICKUP: Scoop");
        setServo(SERVO_L_CH, PU_SERVO_CLOSE_L);
        setServo(SERVO_R_CH, PU_SERVO_CLOSE_R);
        puTimer = now;
        puState = PU_SCOOPING;
        beep(50);
      }
      break;

    case PU_SCOOPING:
      if (now - puTimer >= PU_SCOOP_CLOSE_MS) {
        Serial.println("Scooped! Lifting...");
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("PICKUP: Lift");
        armSpeed = PU_LIFT_SPEED;
        setArm(1);  // up
        puState = PU_LIFTING;
      }
      break;

    case PU_LIFTING:
      if (limTop) {
        stopArm();
        Serial.println("Top limit hit — dropping bottle!");
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("PICKUP: Drop");
        setServo(SERVO_L_CH, PU_SERVO_OPEN_L);
        setServo(SERVO_R_CH, PU_SERVO_OPEN_R);
        puTimer = now;
        puState = PU_DROPPING;
        beep(50);
      }
      break;

    case PU_DROPPING:
      // Wait for bottle to fall into bin
      if (now - puTimer >= PU_DROP_OPEN_MS) {
        Serial.println("=== PICKUP COMPLETE ===");
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("PICKUP: Done!");
        puState = PU_DONE;
        puTimer = now;
        beep(50);  // single beep = success (non-blocking)
        break;
      }
      break;

    case PU_DONE:
      // Short pause then return to idle
      if (now - puTimer >= 500) {
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Ready");
        armSpeed = 20;
        puState = PU_IDLE;
        Serial.println("Pickup idle — ready for next bottle");
      }
      break;

    default:
      break;
  }
}

// ==================== COMMAND EXECUTOR ====================

void executeCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // ── Direct Pi commands (bypass mode system) ──────────────
  // These work from ANY mode so the navigator never needs to
  // know or switch the ESP32's current test-UI mode.
  //   PIFW <spd>   = wheels forward
  //   PIBW <spd>   = wheels backward
  //   PITL <spd>   = turn left (spin in place)
  //   PITR <spd>   = turn right (spin in place)
  //   PIDW <l>,<r> = differential wheel (left_spd, right_spd)
  //   PIX          = stop wheels
  //   PISTOP       = emergency stop all
  //   P            = pickup sequence
  //   PA           = pickup abort
  if (cmd.startsWith("PI") || cmd.startsWith("pi")) {
    String pi = cmd.substring(2);
    pi.toUpperCase();
    pi.trim();

    if (pi.startsWith("FW")) {
      int spd = constrain(pi.substring(2).toInt(), 0, 80);
      setLeft(spd); setRight(spd);
      Serial.printf("PI: FORWARD %d\n", spd);
    } else if (pi.startsWith("BW")) {
      int spd = constrain(pi.substring(2).toInt(), 0, 80);
      setLeft(-spd); setRight(-spd);
      Serial.printf("PI: BACKWARD %d\n", spd);
    } else if (pi.startsWith("TL")) {
      int spd = constrain(pi.substring(2).toInt(), 0, 80);
      setLeft(-spd); setRight(spd);
      Serial.printf("PI: TURN LEFT %d\n", spd);
    } else if (pi.startsWith("TR")) {
      int spd = constrain(pi.substring(2).toInt(), 0, 80);
      setLeft(spd); setRight(-spd);
      Serial.printf("PI: TURN RIGHT %d\n", spd);
    } else if (pi.startsWith("DW")) {
      // Differential wheel: PIDW<left>,<right>
      String params = pi.substring(2);
      int comma = params.indexOf(',');
      if (comma > 0) {
        int lspd = constrain(params.substring(0, comma).toInt(), -80, 80);
        int rspd = constrain(params.substring(comma + 1).toInt(), -80, 80);
        setLeft(lspd); setRight(rspd);
        Serial.printf("PI: DIFF L=%d R=%d\n", lspd, rspd);
      }
    } else if (pi == "X") {
      stopWheels();
      Serial.println("PI: WHEELS STOPPED");
    } else if (pi == "STOP") {
      stopAll();
      Serial.println("PI: EMERGENCY STOP ALL");
    } else {
      Serial.printf("PI: unknown command '%s'\n", pi.c_str());
    }
    return;
  }

  cmd.toUpperCase();

  char c0 = cmd.charAt(0);
  int val = 0;
  if (cmd.length() > 1) val = cmd.substring(1).toInt();

  // Pickup command — works from ANY mode (Pi sends this)
  if (cmd == "P") {
    pickupStart();
    return;
  }
  if (cmd == "PA") {  // Pickup Abort
    pickupAbort();
    return;
  }

  // Block other commands while pickup is running
  if (puState != PU_IDLE) {
    Serial.println("Pickup in progress — send PA to abort");
    return;
  }

  // Mode switching
  if (cmd == "M") {
    stopAll();
    currentMode = MODE_MENU;
    Serial.println("Back to MENU");
    return;
  }

  // Mode selection from menu
  if (currentMode == MODE_MENU) {
    // Stop all motors when switching modes
    stopAll();

    switch (c0) {
      case '1': currentMode = MODE_ULTRASONIC; Serial.println("Mode: ULTRASONIC"); break;
      case '2':
        currentMode = MODE_WHEELS;
        Serial.println("Mode: WHEELS");
        break;
      case '3': currentMode = MODE_ARM; Serial.println("Mode: ARM LIFT"); break;
      case '7': currentMode = MODE_SWING; Serial.println("Mode: SWING DRIVE"); break;
      case '4':
        currentMode = MODE_SERVOS;
        setServo(SERVO_L_CH, PU_SERVO_OPEN_L); setServo(SERVO_R_CH, PU_SERVO_OPEN_R);
        Serial.println("Mode: SERVOS (mirrored, starting OPEN)");
        break;
      case '5': currentMode = MODE_BUZZER; Serial.println("Mode: BUZZER"); break;
      case '6':
        currentMode = MODE_LCD;
        lcd.init(); lcd.backlight();
        lcd.setCursor(0, 0); lcd.print("LCD Test Ready");
        Serial.println("Mode: LCD");
        break;
    }
    return;
  }

  // Wheels mode
  if (currentMode == MODE_WHEELS) {
    switch (c0) {
      case 'F':
        setLeft(wheelSpeed);
        setRight(wheelSpeed);
        Serial.printf("FORWARD at %d (fixed)\n", wheelSpeed); break;
      case 'J': case 'B':
        setLeft(-wheelSpeed);
        setRight(-wheelSpeed);
        Serial.printf("BACKWARD at %d (fixed)\n", wheelSpeed); break;
      case 'L':
        setLeft(-turnSpeed);
        setRight(turnSpeed);
        Serial.printf("TURN LEFT at %d\n", turnSpeed); break;
      case 'R':
        setLeft(turnSpeed);
        setRight(-turnSpeed);
        Serial.printf("TURN RIGHT at %d\n", turnSpeed); break;
      case 'X': case 'S':
        stopWheels(); Serial.println("STOPPED"); break;
      case '+':
        turnSpeed = constrain(turnSpeed + 10, 0, 80);
        Serial.printf("Turn speed: %d (drive fixed at %d)\n", turnSpeed, wheelSpeed); break;
      case '-':
        turnSpeed = constrain(turnSpeed - 10, 0, 80);
        Serial.printf("Turn speed: %d (drive fixed at %d)\n", turnSpeed, wheelSpeed); break;
    }
    return;
  }

  // Arm mode
  if (currentMode == MODE_ARM) {
    switch (c0) {
      case 'U':
        armManualPulseActive = false;
        armSpeed = MANUAL_UP_SPEED;
        setArm(1);
        Serial.printf("ARM UP at %d (until TOP limit)\n", armSpeed);
        break;
      case 'D':
        armManualPulseActive = false;
        armSpeed = MANUAL_DOWN_SPEED;
        setArm(-1);
        Serial.printf("ARM DOWN at %d (until BOTTOM limit)\n", armSpeed);
        break;
      case 'S':
        armManualPulseActive = false;
        stopArm();
        Serial.println("STOPPED");
        break;
      case '+':
        armSpeed = constrain(armSpeed + 10, 0, 255);
        Serial.printf("Speed: %d\n", armSpeed); break;
      case '-':
        armSpeed = constrain(armSpeed - 10, 0, 255);
        Serial.printf("Speed: %d\n", armSpeed); break;
    }
    return;
  }

  // Swing drive mode
  if (currentMode == MODE_SWING) {
    switch (c0) {
      case 'L':
        if (val > 0) swingSpeed = constrain(val, 0, 255);
        setSwing(-1);
        Serial.printf("SWING LEFT at %d\n", swingSpeed); break;
      case 'R':
        if (val > 0) swingSpeed = constrain(val, 0, 255);
        setSwing(1);
        Serial.printf("SWING RIGHT at %d\n", swingSpeed); break;
      case 'S': stopSwing(); Serial.println("STOPPED"); break;
      case '+':
        swingSpeed = constrain(swingSpeed + 10, 0, 255);
        Serial.printf("Speed: %d\n", swingSpeed); break;
      case '-':
        swingSpeed = constrain(swingSpeed - 10, 0, 255);
        Serial.printf("Speed: %d\n", swingSpeed); break;
    }
    return;
  }

  // Servos mode — MIRRORED: right servo = 180 - left servo
  if (currentMode == MODE_SERVOS) {
    switch (c0) {
      case 'O':
        setServo(SERVO_L_CH, PU_SERVO_OPEN_L); setServo(SERVO_R_CH, PU_SERVO_OPEN_R);
        Serial.printf("OPEN (L=%d, R=%d)\n", PU_SERVO_OPEN_L, PU_SERVO_OPEN_R); break;
      case 'C':
        setServo(SERVO_L_CH, PU_SERVO_CLOSE_L); setServo(SERVO_R_CH, PU_SERVO_CLOSE_R);
        Serial.printf("SCOOP (L=%d, R=%d)\n", PU_SERVO_CLOSE_L, PU_SERVO_CLOSE_R); break;
      case 'H':
        setServo(SERVO_L_CH, 90); setServo(SERVO_R_CH, 90);
        Serial.println("HALF (L=90, R=90)"); break;
      case 'L':
        if (val >= 0 && val <= 180) {
          setServo(SERVO_L_CH, val); setServo(SERVO_R_CH, 180 - val);
          Serial.printf("Mirror: L=%d R=%d\n", val, 180 - val);
        } else {
          Serial.println("Mirrored sweep...");
          for (int a = 0; a <= 180; a += 5) { setServo(SERVO_L_CH, a); setServo(SERVO_R_CH, 180 - a); delay(30); }
          for (int a = 180; a >= 0; a -= 5) { setServo(SERVO_L_CH, a); setServo(SERVO_R_CH, 180 - a); delay(30); }
        }
        break;
      case 'R':
        if (val >= 0 && val <= 180) {
          setServo(SERVO_R_CH, val); setServo(SERVO_L_CH, 180 - val);
          Serial.printf("Mirror: R=%d L=%d\n", val, 180 - val);
        } else {
          Serial.println("Mirrored sweep...");
          for (int a = 0; a <= 180; a += 5) { setServo(SERVO_R_CH, a); setServo(SERVO_L_CH, 180 - a); delay(30); }
          for (int a = 180; a >= 0; a -= 5) { setServo(SERVO_R_CH, a); setServo(SERVO_L_CH, 180 - a); delay(30); }
        }
        break;
      default:
        if (c0 >= '1' && c0 <= '9') {
          int angle = map(c0 - '0', 1, 9, 20, 160);
          setServo(SERVO_L_CH, angle);
          setServo(SERVO_R_CH, 180 - angle);
          Serial.printf("Angle: L=%d R=%d\n", angle, 180 - angle);
        }
        break;
    }
    return;
  }

  // Buzzer mode
  if (currentMode == MODE_BUZZER) {
    switch (c0) {
      case 'B': Serial.println("Short beep"); beep(100); break;
      case 'L': Serial.println("Long beep"); beep(500); break;
      default:
        if (c0 >= '1' && c0 <= '9') {
          int count = c0 - '0';
          Serial.printf("Beep x%d\n", count);
          for (int i = 0; i < count; i++) { beep(80); delay(120); }
        }
        break;
    }
    return;
  }

  // LCD mode
  if (currentMode == MODE_LCD) {
    switch (c0) {
      case 'T':
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Line 1 Test OK!");
        lcd.setCursor(0, 1); lcd.print("Line 2 Test OK!");
        Serial.println("Test pattern"); break;
      case 'C': lcd.clear(); Serial.println("LCD cleared"); break;
      case 'B': {
        static bool blOn = true;
        blOn = !blOn;
        if (blOn) lcd.backlight(); else lcd.noBacklight();
        Serial.printf("Backlight: %s\n", blOn ? "ON" : "OFF");
        break;
      }
    }
    return;
  }
}

// ==================== SERIAL MENU PRINTS ====================

void printMenu() {
  Serial.println();
  Serial.println("=========================================");
  Serial.println("  PET Bottle Robot - Component Test Menu");
  Serial.printf("  WiFi: %s\n", WIFI_SSID);
  Serial.printf("  Web:  http://%s\n", WiFi.localIP().toString().c_str());
  Serial.println("=========================================");
  Serial.println("  1 = Ultrasonic Sensors (x4)");
  Serial.println("  2 = Drive Wheels (BTS7960 x2)");
  Serial.println("  3 = Arm Lift (Hybrid Drive 2R0 + Limits)");
  Serial.println("  4 = Servos (Gripper L+R)");
  Serial.println("  5 = Buzzer");
  Serial.println("  6 = LCD Display");
  Serial.println("  M = Return to this menu (from any mode)");
  Serial.println("=========================================");
}

// ==================== WEB HANDLERS ====================

// While the Pi is actively driving over USB serial, lock the web /cmd
// endpoint so a browser tab on the dashboard can't override the
// navigator's commands. PISTOP and PIX are always allowed through as
// emergency overrides — operator must always have a kill switch.
#define AUTO_LOCK_MS 5000

void handleWebCmd() {
  if (server.hasArg("c")) {
    String cmd = server.arg("c");
    String source = server.client().remoteIP().toString();
    String srcLabel = "Web(" + source + ")";

    bool piActive = (lastPiCmdMs > 0) &&
                    (millis() - lastPiCmdMs < AUTO_LOCK_MS);
    bool isEmergency = (cmd == "PISTOP" || cmd == "PIX" || cmd == "PA");
    if (piActive && !isEmergency) {
      Serial.printf("Web CMD [%s] LOCKED (Pi active): %s\n",
                    srcLabel.c_str(), cmd.c_str());
      logCmd(srcLabel, cmd, "LOCKED");
      server.send(423, "text/plain",
                  "LOCKED: Pi navigator is driving. Use /stop on the "
                  "navigator UI (port 5000), or send PISTOP/PIX/PA.");
      return;
    }

    Serial.printf("Web CMD [%s]: %s\n", srcLabel.c_str(), cmd.c_str());
    executeCmd(cmd);
    String result = "OK: " + cmd;
    logCmd(srcLabel, cmd, "OK");
    server.send(200, "text/plain", result);
  } else {
    server.send(400, "text/plain", "Missing ?c=<command>");
  }
}

void handleWebCmdLog() {
  String json = "[";
  for (int i = 0; i < cmdLogCount; i++) {
    int idx = (cmdLogHead - cmdLogCount + i + CMD_LOG_SIZE) % CMD_LOG_SIZE;
    if (i > 0) json += ",";
    // Escape quotes in log entries
    String entry = cmdLog[idx];
    entry.replace("\"", "\\\"");
    json += "\"" + entry + "\"";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleWebSensor() {
  String json = "{";
  json += "\"mode\":\"" + String(
    currentMode == MODE_MENU ? "menu" :
    currentMode == MODE_ULTRASONIC ? "ultrasonic" :
    currentMode == MODE_WHEELS ? "wheels" :
    currentMode == MODE_ARM ? "arm" :
    currentMode == MODE_SWING ? "swing" :
    currentMode == MODE_SERVOS ? "servos" :
    currentMode == MODE_BUZZER ? "buzzer" : "lcd") + "\",";

  json += "\"ultrasonic\":{";
  for (int i = 0; i < NUM_SENSORS; i++) {
    json += "\"s" + String(i+1) + "\":" + String(cachedDist[i]);
    if (i < NUM_SENSORS - 1) json += ",";
  }
  json += "},";

  json += "\"limits\":{";
  json += "\"sw1\":" + String(digitalRead(LIMIT_1) == LOW ? "true" : "false") + ",";
  json += "\"sw2\":" + String(digitalRead(LIMIT_2) == LOW ? "true" : "false");
  json += "},";

  json += "\"irProx\":" + String(digitalRead(IR_PROX) == LOW ? "true" : "false") + ",";

  json += "\"wheelSpeed\":" + String(wheelSpeed) + ",";
  json += "\"armSpeed\":" + String(armSpeed) + ",";
  json += "\"swingSpeed\":" + String(swingSpeed) + ",";

  json += "\"pickup\":\"" + String(
    puState == PU_IDLE ? "idle" :
    puState == PU_LOWERING ? "lowering" :
    puState == PU_SCOOPING ? "scooping" :
    puState == PU_LIFTING ? "lifting" :
    puState == PU_DROPPING ? "dropping" : "done") + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleWebRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Robot Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#f0f4f8;color:#1a2332;padding:12px;
  -webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
h1{text-align:center;font-size:1.3rem;margin-bottom:2px;color:#1565c0;font-weight:800}
h2{text-align:center;margin:4px 0;font-size:.9rem;color:#78909c;font-weight:400}
.tabs{display:flex;gap:4px;margin:10px 0;flex-wrap:wrap;justify-content:center}
.tab{font-size:.82rem;font-weight:600;border:2px solid #e0e8f0;border-radius:10px;
  padding:9px 13px;cursor:pointer;background:#fff;color:#78909c;transition:all .15s}
.tab.active{background:#1976d2;color:#fff;border-color:#1976d2;box-shadow:0 2px 8px rgba(25,118,210,.3)}
.panel{display:none;background:#fff;border-radius:14px;padding:14px;margin:8px 0;
  box-shadow:0 2px 12px rgba(0,0,0,.06);border:1px solid #e8eef3}
.panel.active{display:block}
.grid{display:grid;gap:8px}
.g3{grid-template-columns:1fr 1fr 1fr}
.g2{grid-template-columns:1fr 1fr}
button{font-size:.95rem;font-weight:700;border:none;border-radius:10px;
  padding:12px 8px;cursor:pointer;transition:transform .08s,box-shadow .15s;
  touch-action:manipulation;box-shadow:0 2px 6px rgba(0,0,0,.08)}
button:active{transform:scale(.94)}
.bf{background:#1976d2;color:#fff}.bb{background:#42a5f5;color:#fff}
.bl{background:#90caf9;color:#1a2332}.br{background:#90caf9;color:#1a2332}
.bx{background:#e53935;color:#fff}
.bu{background:#1e88e5;color:#fff}.bd{background:#42a5f5;color:#fff}
.bs{background:#cfd8dc;color:#37474f}
.bo{background:#43a047;color:#fff}.bc{background:#e53935;color:#fff}
.bg{background:#0288d1;color:#fff}
.bp{background:#7b1fa2;color:#fff}
.be{background:#d32f2f;color:#fff;font-size:1.1rem}
.bz{background:#546e7a;color:#fff}
.slider-row{display:flex;align-items:center;gap:8px;margin:8px 0}
.slider-row label{width:70px;font-size:.82rem;color:#78909c;font-weight:600}
.slider-row input[type=range]{flex:1;height:26px;accent-color:#1976d2}
.slider-row span{width:36px;text-align:right;font-family:monospace;color:#1565c0;font-weight:700}
#log{background:#f5f7fa;border:1px solid #e0e8f0;border-radius:8px;padding:8px;margin-top:10px;
  font-family:monospace;font-size:.72rem;color:#1565c0;height:70px;
  overflow-y:auto;white-space:pre-wrap}
#sensors{background:#e3f2fd;border:1px solid #bbdefb;border-radius:8px;padding:10px;margin:8px 0;
  font-family:monospace;font-size:.82rem;color:#1565c0;text-align:center;font-weight:600}
.lim{font-size:.8rem;color:#e65100;text-align:center;margin:6px 0;font-weight:600}
.card{background:#f5f9ff;border:1px solid #e0e8f0;border-radius:8px;padding:10px;text-align:center}
.card-val{font-weight:800}.card-lbl{font-size:.7rem;color:#90a4ae;margin-top:2px}
.ip-box{display:flex;align-items:center;gap:6px;justify-content:center;
  background:#f5f7fa;border-radius:10px;padding:6px 10px;margin:6px auto;max-width:400px}
.ip-box span{font-size:.75rem;color:#78909c}
.ip-box code{font-family:monospace;font-size:.85rem;color:#1565c0;font-weight:700}
</style>
</head>
<body>
<h1>PET Bottle Robot</h1>
<h2>Component Test Dashboard</h2>

<div class="ip-box">
  <span>ESP32</span><code>192.168.43.100</code>
  <span style="margin-left:8px">Pi</span><code>192.168.43.101</code>
</div>

<div id="sensors">Sensors: loading...</div>
<div id="limStatus" class="lim"></div>

<div class="tabs">
  <button class="tab" onclick="setTab('camera',this)">Camera</button>
  <button class="tab" onclick="setTab('ultrasonic',this)">Ultrasonic</button>
  <button class="tab active" onclick="setTab('wheels',this)">Wheels</button>
  <button class="tab" onclick="setTab('arm',this)">Arm</button>
  <button class="tab" onclick="setTab('swing',this)">Swing</button>
  <button class="tab" onclick="setTab('servos',this)">Servos</button>
  <button class="tab" onclick="setTab('buzzer',this)">Buzzer</button>
  <button class="tab" onclick="setTab('lcd',this)">LCD</button>
</div>

<!-- CAMERA & AUTONOMOUS MONITOR -->
<div id="p-camera" class="panel">
<div style="text-align:center;margin-bottom:8px">
  <div class="slider-row" style="justify-content:center">
    <label style="width:auto">Pi IP:</label>
    <input type="text" id="piIp" value="192.168.43.101" style="background:#f5f7fa;color:#1565c0;
      border:1px solid #bbdefb;border-radius:8px;padding:8px 12px;font-family:monospace;
      font-size:.9rem;width:140px;text-align:center">
    <button class="bg" onclick="connectCam()" id="camBtn" style="padding:8px 16px;font-size:.85rem">Connect</button>
  </div>
  <div id="camStatus" style="font-size:.75rem;color:#90a4ae;margin:4px 0">Not connected</div>
</div>

<div id="camPlaceholder" style="background:#f5f9ff;border:1px solid #e0e8f0;border-radius:10px;
    padding:16px;color:#90a4ae;font-size:.85rem;text-align:center">
    Click Connect to poll Pi detection stats.
  </div>
<div id="piStats" style="display:none">
  <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin:8px 0">
    <div class="card">
      <div id="psBotCount" class="card-val" style="font-size:1.6rem;color:#2e7d32">0</div>
      <div class="card-lbl">Bottles</div>
    </div>
    <div class="card">
      <div id="psState" class="card-val" style="font-size:1rem;color:#e65100">--</div>
      <div class="card-lbl">State</div>
    </div>
    <div class="card">
      <div id="psFps" class="card-val" style="font-size:1.6rem;color:#1565c0">0</div>
      <div class="card-lbl">FPS</div>
    </div>
  </div>
  <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin:4px 0">
    <div class="card">
      <div id="psModel" class="card-val" style="font-size:.85rem;color:#7b1fa2">--</div>
      <div class="card-lbl">Model</div>
    </div>
    <div class="card">
      <div id="psInfer" class="card-val" style="font-size:.85rem;color:#e65100">--</div>
      <div class="card-lbl">Infer ms</div>
    </div>
    <div class="card">
      <div id="psPersons" class="card-val" style="font-size:1rem;color:#c62828">0</div>
      <div class="card-lbl">Persons</div>
    </div>
  </div>
  <div style="text-align:center;margin:10px 0">
    <a id="piAdminLink" href="#" target="_blank"
      style="color:#1976d2;font-size:.85rem;font-weight:600">Open Pi Admin (live video + model switch)</a>
  </div>
</div>

<h2 style="margin-top:10px">Pi &harr; ESP32 Command Log</h2>
<div id="cmdlog" style="background:#f5f7fa;border:1px solid #e0e8f0;border-radius:8px;padding:8px;
  margin-top:6px;font-family:monospace;font-size:.7rem;color:#1565c0;height:140px;
  overflow-y:auto;white-space:pre-wrap">Waiting for commands...</div>
</div>

<!-- ULTRASONIC -->
<div id="p-ultrasonic" class="panel">
<canvas id="ucanvas" width="360" height="220" style="width:100%;background:#f5f9ff;border:1px solid #e0e8f0;border-radius:10px"></canvas>
<p style="text-align:center;color:#90a4ae;font-size:.75rem;margin-top:6px">Live distance graph &mdash; red=close, green=far</p>
</div>

<!-- WHEELS -->
<div id="p-wheels" class="panel active">
<div class="slider-row">
  <label>Speed</label>
  <input type="range" id="wspd" min="30" max="80" value="30">
  <span id="wspdV">30</span>
</div>
<div class="grid g3">
  <div></div>
  <button class="bf" onpointerdown="wcmd('F')">Forward</button>
  <div></div>
  <button class="bl" onpointerdown="wcmd('L')">Left</button>
  <button class="bx" onpointerdown="wcmd('X')">STOP</button>
  <button class="br" onpointerdown="wcmd('R')">Right</button>
  <div></div>
  <button class="bb" onpointerdown="wcmd('B')">Back</button>
  <div></div>
</div>
</div>

<!-- ARM LIFT -->
<div id="p-arm" class="panel">
<div style="margin-bottom:10px">
  <button class="bf" style="width:100%;padding:14px;font-size:1rem;border-radius:12px;
    box-shadow:0 3px 10px rgba(25,118,210,.25)" onpointerdown="cmd('P')">
    Start Pickup Sequence
  </button>
  <div id="puStatus" style="text-align:center;font-size:.85rem;font-weight:600;
    color:#78909c;margin-top:6px">Idle</div>
  <button class="bs" style="width:100%;margin-top:6px;padding:8px" onpointerdown="cmd('PA')">Abort Pickup</button>
</div>
<div class="slider-row">
  <label>Speed</label>
  <input type="range" id="aspd" min="20" max="180" value="20">
  <span id="aspdV">20</span>
</div>
<div class="grid g3">
  <button class="bu" onpointerdown="acmd('U')">Arm Up</button>
  <button class="bs" onpointerdown="acmd('S')">Stop</button>
  <button class="bd" onpointerdown="acmd('D')">Arm Down</button>
</div>
</div>

<!-- SWING DRIVE -->
<div id="p-swing" class="panel">
<div class="slider-row">
  <label>Speed</label>
  <input type="range" id="sspd" min="30" max="80" value="30">
  <span id="sspdV">30</span>
</div>
<div class="grid g3">
  <button class="bl" onpointerdown="swcmd('L')">Swing Left</button>
  <button class="bs" onpointerdown="swcmd('S')">Stop</button>
  <button class="br" onpointerdown="swcmd('R')">Swing Right</button>
</div>
</div>

<!-- SERVOS -->
<div id="p-servos" class="panel">
<div class="grid g2">
  <button class="bo" onpointerdown="scmd('O')">Both Close</button>
  <button class="bc" onpointerdown="scmd('C')">Both Open</button>
</div>
<div class="grid g3" style="margin-top:8px">
  <button class="bg" onpointerdown="scmd('H')">Half (90)</button>
  <button class="bg" onpointerdown="scmd('L')">Sweep L</button>
  <button class="bg" onpointerdown="scmd('R')">Sweep R</button>
</div>
<div class="slider-row">
  <label>Left</label>
  <input type="range" id="sl" min="0" max="180" value="90"
    oninput="document.getElementById('slV').textContent=this.value;scmd('L'+this.value)">
  <span id="slV">90</span>
</div>
<div class="slider-row">
  <label>Right</label>
  <input type="range" id="sr" min="0" max="180" value="90"
    oninput="document.getElementById('srV').textContent=this.value;scmd('R'+this.value)">
  <span id="srV">90</span>
</div>
</div>

<!-- BUZZER -->
<div id="p-buzzer" class="panel">
<div class="grid g3">
  <button class="bz" onpointerdown="zcmd('B')">Short Beep</button>
  <button class="bz" onpointerdown="zcmd('L')">Long Beep</button>
  <button class="bz" onpointerdown="zcmd('3')">Beep x3</button>
</div>
</div>

<!-- LCD -->
<div id="p-lcd" class="panel">
<div class="grid g3">
  <button class="bg" onpointerdown="lcmd('T')">Test Pattern</button>
  <button class="bs" onpointerdown="lcmd('C')">Clear</button>
  <button class="bz" onpointerdown="lcmd('B')">Backlight</button>
</div>
</div>

<!-- EMERGENCY + OTA -->
<div style="margin:12px 0;text-align:center">
  <button class="be" style="width:100%;padding:16px;border-radius:12px;
    box-shadow:0 3px 10px rgba(211,47,47,.25)" onpointerdown="emergency()">EMERGENCY STOP ALL</button>
</div>
<div style="text-align:center;margin:8px 0">
  <a href="/ota" style="color:#1976d2;font-size:.85rem;font-weight:600">Firmware Update (OTA)</a>
</div>

<div id="log">Ready.\n</div>

<script>
const log=document.getElementById('log');
const wspd=document.getElementById('wspd'),wspdV=document.getElementById('wspdV');
const aspd=document.getElementById('aspd'),aspdV=document.getElementById('aspdV');
const sspd=document.getElementById('sspd'),sspdV=document.getElementById('sspdV');
wspd.oninput=()=>wspdV.textContent=wspd.value;
aspd.oninput=()=>aspdV.textContent=aspd.value;
sspd.oninput=()=>sspdV.textContent=sspd.value;

// Camera + Command Log
let camOn=false,camTimer=null,logTimer=null;
var camFails=0;
var stateColors={WAITING:'#e65100',SCANNING:'#f57f17',ROAMING:'#2e7d32',
  VERIFYING:'#ef6c00',APPROACHING:'#1565c0',ALIGNING:'#f57f17',
  PICKING_UP:'#7b1fa2',AVOIDING:'#c62828',STOPPED:'#78909c'};
function connectCam(){
  let ip=document.getElementById('piIp').value.trim();
  let ph=document.getElementById('camPlaceholder');
  let ps=document.getElementById('piStats');
  let st=document.getElementById('camStatus');
  let btn=document.getElementById('camBtn');
  if(camOn){
    ph.style.display='block';ps.style.display='none';
    st.textContent='Disconnected';st.style.color='#90a4ae';
    btn.textContent='Connect';btn.className='bg';
    camOn=false;camFails=0;
    if(camTimer)clearInterval(camTimer);
    return;
  }
  st.textContent='Connecting to '+ip+'...';st.style.color='#e65100';
  camFails=0;camOn=true;
  ph.style.display='none';ps.style.display='block';
  btn.textContent='Disconnect';btn.className='bc';
  document.getElementById('piAdminLink').href='http://'+ip+':8080';
  camTimer=setInterval(()=>{
    fetch('http://'+ip+':5000/stats',{signal:AbortSignal.timeout(3000)}).then(r=>r.json()).then(d=>{
      camFails=0;
      st.textContent='Connected — '+ip;st.style.color='#2e7d32';
      document.getElementById('psBotCount').textContent=d.bottles||0;
      document.getElementById('psPersons').textContent=d.persons||0;
      document.getElementById('psFps').textContent=d.fps||0;
      document.getElementById('psInfer').textContent=(d.inference_ms||'--')+'ms';
      document.getElementById('psModel').textContent=d.model||'--';
      var se=document.getElementById('psState');
      se.textContent=d.state||'--';
      se.style.color=stateColors[d.state]||'#78909c';
    }).catch(()=>{
      camFails++;
      if(camFails>=3){
        st.textContent='Pi not responding (retry '+camFails+')';st.style.color='#c62828';
      }
    });
  },1500);
}
// Poll command log from ESP32
function pollCmdLog(){
  fetch('/cmdlog').then(r=>r.json()).then(logs=>{
    let el=document.getElementById('cmdlog');
    if(logs.length===0){el.textContent='Waiting for commands...';return;}
    el.innerHTML='';
    logs.forEach(l=>{
      let span=document.createElement('span');
      span.textContent=l+'\n';
      span.style.color=l.indexOf('[Pi')>=0?'#1565c0':'#2e7d32';
      el.appendChild(span);
    });
    el.scrollTop=el.scrollHeight;
  }).catch(()=>{});
}
logTimer=setInterval(pollCmdLog,800);
pollCmdLog();

let curTab='wheels';
function setTab(t,el){
  // Switch mode on ESP
  let modeMap={camera:'M',ultrasonic:'1',wheels:'2',arm:'3',swing:'7',servos:'4',buzzer:'5',lcd:'6'};
  cmd('M');
  setTimeout(()=>cmd(modeMap[t]),200);
  curTab=t;
  document.querySelectorAll('.tab').forEach(b=>b.classList.remove('active'));
  document.querySelectorAll('.panel').forEach(p=>p.classList.remove('active'));
  el.classList.add('active');
  document.getElementById('p-'+t).classList.add('active');
}

function cmd(c){
  log.textContent+='> '+c+'\n';
  log.scrollTop=log.scrollHeight;
  fetch('/cmd?c='+encodeURIComponent(c))
    .then(r=>r.text()).then(t=>{log.textContent+=t+'\n';log.scrollTop=log.scrollHeight})
    .catch(e=>{log.textContent+='ERR: '+e+'\n'});
}

function wcmd(c){ cmd(c+(c=='X'||c=='S'?'':wspd.value)); }
function acmd(c){ cmd(c+(/[UD]/.test(c)?aspd.value:'')); }
function swcmd(c){ cmd(c+(/[LR]/.test(c)?sspd.value:'')); }
function scmd(c){ cmd(c); }
function zcmd(c){ cmd(c); }
function lcmd(c){ cmd(c); }

function emergency(){
  cmd('M');
  log.textContent+='*** EMERGENCY STOP ***\n';
}

// Ultrasonic graph
const uc=document.getElementById('ucanvas');
const ctx=uc.getContext('2d');
const maxPts=60;
const sHist=[[],[],[],[]];
const sColors=['#1976d2','#2e7d32','#e65100','#7b1fa2'];
const sNames=['S1(33)','S2(35)','S3(32)','S4(34)'];
const maxDist=300;

function drawGraph(){
  const W=uc.width,H=uc.height;
  const top=30,bot=20,left=36,right=10;
  const gW=W-left-right,gH=H-top-bot;
  ctx.clearRect(0,0,W,H);

  // Grid
  ctx.strokeStyle='#e0e8f0';ctx.lineWidth=1;
  for(let d=0;d<=maxDist;d+=50){
    let y=top+gH-(d/maxDist)*gH;
    ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(left+gW,y);ctx.stroke();
    ctx.fillStyle='#90a4ae';ctx.font='9px monospace';ctx.textAlign='right';
    ctx.fillText(d+'cm',left-4,y+3);
  }
  // Danger zone (0-15cm)
  let dangerY=top+gH-(15/maxDist)*gH;
  ctx.fillStyle='rgba(229,57,53,0.08)';
  ctx.fillRect(left,dangerY,gW,top+gH-dangerY);

  // Lines
  for(let s=0;s<4;s++){
    let h=sHist[s];if(h.length<2)continue;
    ctx.strokeStyle=sColors[s];ctx.lineWidth=2;
    ctx.beginPath();
    for(let i=0;i<h.length;i++){
      let x=left+gW-(h.length-1-i)*(gW/(maxPts-1));
      let v=Math.min(h[i],maxDist);
      let y=top+gH-(v/maxDist)*gH;
      if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
    }
    ctx.stroke();

    // Current value label
    let last=h[h.length-1];
    let lx=left+gW+2;
    let ly=top+gH-(Math.min(last,maxDist)/maxDist)*gH;
    ctx.fillStyle=sColors[s];ctx.font='bold 10px monospace';ctx.textAlign='left';
    ctx.fillText(last>=999?'---':last+'cm',lx-8,ly+3);
  }

  // Legend
  ctx.font='bold 10px sans-serif';ctx.textAlign='center';
  for(let s=0;s<4;s++){
    let x=left+gW*(s+0.5)/4;
    ctx.fillStyle=sColors[s];
    ctx.fillText(sNames[s],x,16);
    let last=sHist[s].length?sHist[s][sHist[s].length-1]:999;
    ctx.font='11px monospace';
    ctx.fillText(last>=999?'---':last+'cm',x,H-6);
    ctx.font='bold 10px sans-serif';
  }
}

// Poll sensors
function poll(){
  fetch('/sensor').then(r=>r.json()).then(d=>{
    let s=d.ultrasonic;
    let vals=[s.s1,s.s2,s.s3,s.s4];
    document.getElementById('sensors').textContent=
      'S1:'+s.s1+'cm  S2:'+s.s2+'cm  S3:'+s.s3+'cm  S4:'+s.s4+'cm';
    for(let i=0;i<4;i++){
      sHist[i].push(vals[i]);
      if(sHist[i].length>maxPts)sHist[i].shift();
    }
    drawGraph();
    let lim='';
    if(d.limits.sw1) lim+='SW1(36) PRESSED  ';
    if(d.limits.sw2) lim+='SW2(39) PRESSED  ';
    document.getElementById('limStatus').textContent=lim;
    // Pickup status
    var pu=d.pickup||'idle';
    var puEl=document.getElementById('puStatus');
    if(puEl){
      var puLabels={idle:'Idle',lowering:'Lowering arm...',scooping:'Scooping bottle...',
        lifting:'Lifting arm...',dropping:'Dropping bottle...',done:'Done!'};
      var puColors={idle:'#78909c',lowering:'#e65100',scooping:'#1565c0',
        lifting:'#2e7d32',dropping:'#7b1fa2',done:'#2e7d32'};
      puEl.textContent=puLabels[pu]||pu;
      puEl.style.color=puColors[pu]||'#78909c';
    }
  }).catch(()=>{});
}
setInterval(poll,1000);
poll();
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ==================== OTA ====================

void handleOtaPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>OTA Update</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#f0f4f8;color:#1a2332;
  display:flex;flex-direction:column;align-items:center;
  justify-content:center;min-height:100vh;padding:24px}
.card{background:#fff;border-radius:20px;padding:40px 32px;
  max-width:480px;width:100%;text-align:center;
  box-shadow:0 4px 24px rgba(0,0,0,.08);border:1px solid #e0e8f0}
h1{font-size:1.5rem;margin-bottom:8px;color:#1565c0;font-weight:800}
p{color:#78909c;font-size:.9rem;margin-bottom:24px}
input[type=file]{display:none}
label.btn{display:inline-block;background:#1976d2;color:#fff;
  font-size:1.1rem;font-weight:700;border-radius:14px;
  padding:16px 36px;cursor:pointer;margin:8px;
  box-shadow:0 2px 8px rgba(25,118,210,.25)}
button{font-size:1.1rem;font-weight:700;border:none;border-radius:14px;
  padding:16px 36px;cursor:pointer;background:#43a047;color:#fff;margin:8px;
  box-shadow:0 2px 8px rgba(67,160,71,.2)}
button:disabled{opacity:.4;cursor:not-allowed}
.back{background:#e8eef3;color:#546e7a;font-size:.9rem;font-weight:600;
  padding:10px 24px;margin-top:16px;border-radius:10px;
  text-decoration:none;display:inline-block}
#fname{color:#1565c0;font-family:monospace;margin:12px 0;font-size:.85rem;font-weight:600}
#prog{width:100%;height:24px;border-radius:12px;overflow:hidden;
  background:#e0e8f0;margin:16px 0;display:none}
#bar{height:100%;background:#1976d2;width:0%;transition:width .3s}
#status{font-size:1rem;margin:12px 0;min-height:24px;font-weight:600}
.warn{color:#e65100;font-size:.8rem;margin-top:12px;font-weight:600}
</style>
</head>
<body>
<div class="card">
  <h1>Firmware Update</h1>
  <p>PET Bottle Robot - OTA</p>
  <form id="form" method="POST" enctype="multipart/form-data">
    <label class="btn" for="file">Choose .bin File</label>
    <input type="file" id="file" name="update" accept=".bin"
      onchange="document.getElementById('fname').textContent=this.files[0].name;
      document.getElementById('upload').disabled=false">
    <div id="fname">No file selected</div>
    <button type="submit" id="upload" disabled>Upload & Flash</button>
  </form>
  <div id="prog"><div id="bar"></div></div>
  <div id="status"></div>
  <p class="warn">Do NOT power off during update!</p>
  <a class="back" href="/">Back to Controls</a>
</div>
<script>
document.getElementById('form').addEventListener('submit', function(e) {
  e.preventDefault();
  var f = document.getElementById('file').files[0];
  if (!f) return;
  var xhr = new XMLHttpRequest();
  var prog = document.getElementById('prog');
  var bar = document.getElementById('bar');
  var status = document.getElementById('status');
  prog.style.display = 'block';
  status.textContent = 'Uploading...';
  document.getElementById('upload').disabled = true;
  xhr.open('POST', '/ota', true);
  xhr.upload.addEventListener('progress', function(e) {
    if (e.lengthComputable) {
      var pct = Math.round((e.loaded / e.total) * 100);
      bar.style.width = pct + '%';
      status.textContent = 'Uploading... ' + pct + '%';
    }
  });
  xhr.onload = function() {
    if (xhr.status === 200) {
      bar.style.width = '100%';
      status.textContent = 'Update complete! Rebooting...';
      status.style.color = '#2e7d32';
      setTimeout(function() { location.href = '/'; }, 5000);
    } else {
      bar.style.background = '#e53935';
      status.textContent = 'FAILED: ' + xhr.responseText;
      status.style.color = '#c62828';
      document.getElementById('upload').disabled = false;
    }
  };
  xhr.onerror = function() {
    status.textContent = 'Connection lost. Rebooting...';
    status.style.color = '#e65100';
    setTimeout(function() { location.href = '/'; }, 5000);
  };
  var fd = new FormData();
  fd.append('update', f);
  xhr.send(fd);
});
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleOtaUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA: Receiving %s\n", upload.filename.c_str());
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("OTA UPDATE...");
    lcd.setCursor(0, 1); lcd.print("DO NOT POWER OFF");
    stopAll();

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Serial.printf("OTA: Begin failed: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Serial.printf("OTA: Write failed: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA: Success! %u bytes\n", upload.totalSize);
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("OTA OK!");
      lcd.setCursor(0, 1); lcd.print("Rebooting...");
    } else {
      Serial.printf("OTA: End failed: %s\n", Update.errorString());
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("OTA FAILED!");
    }
  }
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  delay(500);

  // ─── STEP 1: Disable ALL driver enables FIRST ───
  pinMode(L_EN, OUTPUT);    digitalWrite(L_EN, LOW);
  pinMode(R_EN, OUTPUT);    digitalWrite(R_EN, LOW);

  // ─── STEP 2: Set ALL PWM/direction pins LOW before LEDC ───
  // BTS7960 wheel pins
  pinMode(L_RPWM, OUTPUT);  digitalWrite(L_RPWM, LOW);
  pinMode(L_LPWM, OUTPUT);  digitalWrite(L_LPWM, LOW);
  pinMode(R_RPWM, OUTPUT);  digitalWrite(R_RPWM, LOW);
  pinMode(R_LPWM, OUTPUT);  digitalWrite(R_LPWM, LOW);
  // Hybrid Drive 2R0 arm pins (PWM + single DIR; opto-isolated inputs)
  pinMode(ARM_PWM, OUTPUT);  digitalWrite(ARM_PWM, LOW);
  pinMode(ARM_DIR, OUTPUT);  digitalWrite(ARM_DIR, LOW);
  // L298N swing pins
  pinMode(SWING_EN, OUTPUT);  digitalWrite(SWING_EN, LOW);
  pinMode(SWING_IN1, OUTPUT); digitalWrite(SWING_IN1, LOW);
  pinMode(SWING_IN2, OUTPUT); digitalWrite(SWING_IN2, LOW);

  // ─── STEP 3: Attach LEDC PWM channels ───
  // BTS7960 wheels (ch0-3, Timers 0-1)
  ledcSetup(L_FWD_CH, WHEEL_PWM_FREQ, WHEEL_PWM_RES);
  ledcSetup(L_REV_CH, WHEEL_PWM_FREQ, WHEEL_PWM_RES);
  ledcSetup(R_FWD_CH, WHEEL_PWM_FREQ, WHEEL_PWM_RES);
  ledcSetup(R_REV_CH, WHEEL_PWM_FREQ, WHEEL_PWM_RES);
  ledcAttachPin(L_RPWM, L_FWD_CH);
  ledcAttachPin(L_LPWM, L_REV_CH);
  ledcAttachPin(R_RPWM, R_FWD_CH);
  ledcAttachPin(R_LPWM, R_REV_CH);
  // L298N EN pins (ch4-5, Timer 2)
  ledcSetup(ARM_CH, ARM_PWM_FREQ, ARM_PWM_RES);
  ledcSetup(SWING_CH, ARM_PWM_FREQ, ARM_PWM_RES);
  ledcAttachPin(ARM_PWM, ARM_CH);
  ledcAttachPin(SWING_EN, SWING_CH);
  // Servos (ch6-7, Timer 3)
  ledcSetup(SERVO_L_CH, SERVO_FREQ, SERVO_RES);
  ledcSetup(SERVO_R_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_L, SERVO_L_CH);
  ledcAttachPin(SERVO_R, SERVO_R_CH);

  // ─── STEP 4: Zero ALL PWM channels, servos open ───
  ledcWrite(L_FWD_CH, 0); ledcWrite(L_REV_CH, 0);
  ledcWrite(R_FWD_CH, 0); ledcWrite(R_REV_CH, 0);
  ledcWrite(ARM_CH, 0);   ledcWrite(SWING_CH, 0);
  setServo(SERVO_L_CH, PU_SERVO_OPEN_L);
  setServo(SERVO_R_CH, PU_SERVO_OPEN_R);

  // ─── STEP 5: Other peripherals ───
  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(echoPins[i], INPUT);
  }
  // Limit switches — LIMIT_1 (GPIO 36) needs external 10k pull-up; LIMIT_2 (GPIO 15) uses internal pull-up
  pinMode(LIMIT_1, INPUT);
  pinMode(LIMIT_2, INPUT_PULLUP);
  // E18-D80NK IR proximity sensor (GPIO 39, needs external 10k pull-up — no internal pull-up available)
  pinMode(IR_PROX, INPUT);
  // Buzzer
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  // --- LCD ---
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Component Test");

  // --- WiFi STA (connect to mobile hotspot) ---
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32_petbottle");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to WiFi '%s'...", WIFI_SSID);
  lcd.setCursor(0, 1);
  lcd.print("WiFi...");

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi FAILED — check hotspot");
    lcd.setCursor(0, 1);
    lcd.print("WiFi FAILED!");
  }

  // --- Web server ---
  server.on("/", handleWebRoot);
  server.on("/cmd", handleWebCmd);
  server.on("/sensor", handleWebSensor);
  server.on("/cmdlog", handleWebCmdLog);
  server.on("/ota", HTTP_GET, handleOtaPage);
  server.on("/ota", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    if (Update.hasError()) {
      server.send(500, "text/plain", "FAIL: " + String(Update.errorString()));
    } else {
      server.send(200, "text/html",
        "<h1>OK! Rebooting...</h1><script>setTimeout(()=>location='/',5000)</script>");
      delay(500);
      ESP.restart();
    }
  }, handleOtaUpload);
  server.begin();

  // --- All off ---
  stopAll();

  beep(50);
  Serial.println("=========================================");
  Serial.println("  PET Bottle Robot - Component Test");
  Serial.printf("  WiFi: %s / %s\n", WIFI_SSID, WIFI_PASS);
  Serial.printf("  Web:  http://%s\n", WiFi.localIP().toString().c_str());
  Serial.printf("  OTA:  http://%s/ota\n", WiFi.localIP().toString().c_str());
  Serial.println("=========================================");
  printMenu();
}

// ==================== MAIN LOOP ====================

void loop() {
  server.handleClient();

  // Non-blocking buzzer
  buzzerUpdate();

  // Pickup state machine (non-blocking)
  pickupUpdate();

  // Background ultrasonic reads — ONE sensor per tick (round-robin).
  // Shared TRIG needs ~60ms between fires to let echoes decay.
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();
    cachedDist[nextSensorIdx] = readDistance(echoPins[nextSensorIdx]);
    nextSensorIdx = (nextSensorIdx + 1) % NUM_SENSORS;
  }

  // Ultrasonic auto-reads on serial when in that mode
  if (currentMode == MODE_ULTRASONIC) {
    static unsigned long lastRead = 0;
    if (millis() - lastRead > 500) {
      lastRead = millis();
      Serial.print("| ");
      for (int i = 0; i < NUM_SENSORS; i++) {
        long dist = readDistance(echoPins[i]);
        Serial.print(echoLabels[i]);
        Serial.print(": ");
        if (dist >= 999) Serial.print("---");
        else { Serial.print(dist); Serial.print("cm"); }
        Serial.print(" | ");
        delay(50);
      }
      Serial.println();
    }
  }

  // Arm limit switch check
  // NOTE: GPIO 36/39 have no internal pull-up. Without external 10kΩ
  // pull-ups to 3.3V they float LOW (always "pressed") and block the arm.
  // Only enforce limit stops when a pin reads HIGH at least once (pull-up installed).
  // limitsReady is file-scope so setArm() can also consult it.
  if (!limitsReady) {
    if (digitalRead(LIMIT_1) == HIGH || digitalRead(LIMIT_2) == HIGH)
      limitsReady = true;
  }
  // Direction-aware stop: only kill the motor if the limit being pressed
  // is the one in the current direction of travel.
  // Physical wiring: SW1 (GPIO 36) = BOTTOM/DOWN limit, SW2 (GPIO 39) = TOP/UP limit.
  if (limitsReady && currentMode == MODE_ARM) {
    bool lim_bottom = digitalRead(LIMIT_1) == LOW;
    bool lim_top    = digitalRead(LIMIT_2) == LOW;
    bool hit_going_up   = lim_top    && lastArmDir > 0;
    bool hit_going_down = lim_bottom && lastArmDir < 0;
    if (hit_going_up || hit_going_down) {
      stopArm();
      armManualPulseActive = false;
      if (hit_going_up)   Serial.println("*** TOP LIMIT (SW2) hit while going UP — stopped (DOWN still works) ***");
      if (hit_going_down) Serial.println("*** BOTTOM LIMIT (SW1) hit while going DOWN — stopped (UP still works) ***");
    }
  }

  // Manual ARM pulse loop (UP and DOWN) — runs only outside the pickup state machine.
  // Toggles motor on/off so arm moves in small controlled steps.
  if (armManualPulseActive && puState == PU_IDLE) {
    unsigned long now = millis();
    int onMs  = (armManualPulseDir > 0) ? MANUAL_DOWN_ON_MS  : MANUAL_DOWN_ON_MS;
    int offMs = (armManualPulseDir > 0) ? MANUAL_DOWN_OFF_MS : MANUAL_DOWN_OFF_MS;
    int spd   = (armManualPulseDir > 0) ? MANUAL_UP_SPEED    : MANUAL_DOWN_SPEED;
    if (armManualPulseOn && (now - armManualPulseTimer >= onMs)) {
      stopArm();
      armManualPulseOn = false;
      armManualPulseTimer = now;
    } else if (!armManualPulseOn && (now - armManualPulseTimer >= offMs)) {
      armSpeed = spd;
      setArm(armManualPulseDir);
      armManualPulseOn = true;
      armManualPulseTimer = now;
    }
  }

  // Serial input — debug only (Arduino IDE Serial Monitor).
  static String serialBuf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        executeCmd(serialBuf);
        Serial.print("OK ");
        Serial.println(serialBuf);
        serialBuf = "";
      }
    } else {
      serialBuf += c;
      if (serialBuf.length() > 64) serialBuf = "";
    }
  }

  // ── Wheel watchdog ───────────────────────────────────────
  // If the Pi stops sending commands (disconnect, navigator crash),
  // force wheels to zero after WHEEL_WATCHDOG_MS of silence.
  #define WHEEL_WATCHDOG_MS 1000
  static bool watchdogTripped = false;
  if (lastPiCmdMs > 0 && (millis() - lastPiCmdMs) > WHEEL_WATCHDOG_MS) {
    if (!watchdogTripped) {
      stopWheels();
      Serial.println("WATCHDOG: no Pi command for >1s — wheels stopped");
      watchdogTripped = true;
    }
  } else if (lastPiCmdMs > 0) {
    watchdogTripped = false;  // re-armed when commands resume
  }

}
