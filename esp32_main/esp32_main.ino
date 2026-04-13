/*
 * PET Bottle Robot - Combined Drive + Arm Controller
 * ESP32 WiFi Access Point + USB Serial + LCD I2C
 *
 * HARDWARE:
 *   - 24V 30000mAh battery (ALEAIVY)
 *   - DC-DC converter (24V -> 5V for ESP32/servos)
 *   - 2x 24V DC drive motors (left/right wheels) via BTS7960B
 *   - 1x 58SW31ZY DC geared motor (arm lift) via BTS7960B
 *   - 1x NEMA 17 stepper (base turning platform) via L298N
 *   - 2x MG996R servo (left/right arm)
 *   - 1x LCD I2C 16x2 (status display)
 *   - 1x Buzzer
 *
 * WIRING:
 *   BTS7960B #1 (Left Wheel):   RPWM=4,  LPWM=5,  EN=26
 *   BTS7960B #2 (Right Wheel):  RPWM=2,  LPWM=25, EN=19
 *   BTS7960B #3 (Arm Lift):     RPWM=16, LPWM=17, EN=27
 *   L298N (Stepper Base):       IN1=32, IN2=33, IN3=22, IN4=21
 *   Servo Left:                 Signal=13
 *   Servo Right:                Signal=18
 *   LCD I2C:                    SDA=14, SCL=15
 *   Buzzer:                     GPIO23
 *
 * COMMANDS:
 *   F/J<spd>    Forward/Back        TL/TR<spd>  Turn
 *   W<l>,<r>    Raw wheels          X           Stop wheels
 *   B<ang>      Base rotate         BH          Home base
 *   U/D<spd>    Lift up/down        S           Stop lift
 *   L<ang>      Left servo (0-180)  R<ang>      Right servo (0-180)
 *   O           Both servos open    C           Both servos close
 *   P           Pickup sequence     H           Home all
 *   E           Emergency stop      ?           Help & status
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
// NOTE: ESP32Servo REMOVED - causes LEDC channel conflicts with motor PWM
// Servos are controlled directly via LEDC channels 6 & 7

// ===================== FORWARD DECLARATIONS =====================
void beep(int count, int dur);
void wheels_stop();
void lift_stop();
void lift_up(int speed);
void lift_down(int speed);
void lift_up_timed(int speed, int ms);
void lift_down_timed(int speed, int ms);
void servos_open();
void servos_close();
void servo_write(int pin, int angle);
void set_wheels(int left, int right);
void drive_forward(int speed);
void drive_backward(int speed);
void turn_left(int speed);
void turn_right(int speed);
void rotate_base(int degrees);
void home_base();
void stepper_release();
void bts_drive(int rpwm, int lpwm, int en, int speed);
void executeCommand(String cmd);
void pickup_sequence();
void home_position();
void emergency_stop();
void printHelp();
void printStatus();
void handleRoot();
void handleCommand();
void processSerial();
void lcd_update();
void handleOtaPage();
void handleOtaUpload();
void handleSensor();
long readUltrasonic(int echoPin);

// ===================== WiFi ACCESS POINT =====================

const char* AP_SSID = "PetBottle_Robot";
const char* AP_PASS = "petbottle123";
WebServer server(80);

// ===================== PIN DEFINITIONS =====================

// BTS7960B #1 - Left Drive Wheel
#define LEFT_RPWM    2     // was GPIO18 (boot pull-up caused motor runaway)
#define LEFT_LPWM    25     // was GPIO23 (not producing PWM)
#define LEFT_EN      19

// BTS7960B #2 - Right Drive Wheel  (swapped with Arm Lift pins)
#define RIGHT_RPWM   16
#define RIGHT_LPWM   17
#define RIGHT_EN     27

// BTS7960B #3 - Arm Lift (58SW31ZY)  (swapped with Right Wheel pins)
#define LIFT_RPWM    4
#define LIFT_LPWM    5
#define LIFT_EN      26

// L298N - NEMA 17 Stepper (Base Turning Platform)
#define STEP_IN1     32
#define STEP_IN2     33
#define STEP_IN3     22
#define STEP_IN4     21

// Servo Left
#define SERVO_LEFT_PIN   13

// Servo Right
#define SERVO_RIGHT_PIN  18    // swapped from GPIO4 (now used by Left RPWM)

// LCD I2C
#define LCD_SDA      14
#define LCD_SCL      15

// Buzzer
#define BUZZER_PIN   23

// Ultrasonic Sensors (obstacle avoidance — read by Pi via /sensor)
#define US_TRIG      12    // shared trigger (pulse one at a time)
#define US_ECHO_L    34    // left echo  (input-only pin)
#define US_ECHO_R    35    // right echo (input-only pin)

// ===================== MOTOR CONSTANTS =====================

#define PWM_FREQ       1000
#define PWM_RES        8
#define DRIVE_DEFAULT  60
#define DRIVE_MAX      80     // hard cap — drivers explode above 100
#define LIFT_DEFAULT   80

// LEDC channel assignments (paired by hardware timer: 0/1, 2/3, 4/5, 6/7)
// Each pair shares one timer — both channels must use same freq/res.
#define LEFT_RPWM_CH    0
#define LEFT_LPWM_CH    1   // pair with ch0 (timer 0) - same 1kHz/8-bit
#define RIGHT_RPWM_CH   2
#define RIGHT_LPWM_CH   3   // pair with ch2 (timer 1) - same 1kHz/8-bit
#define LIFT_RPWM_CH    4
#define LIFT_LPWM_CH    5   // pair with ch4 (timer 2) - same 1kHz/8-bit

// Stepper
#define STEPS_PER_REV      200
#define STEPS_PER_DEGREE   (STEPS_PER_REV / 360.0)
#define STEPPER_MAX_SPEED  60      // reduced for L298N full-step (was 150)
#define STEPPER_ACCEL      30      // gentle acceleration (was 80)
#define BASE_MAX_ANGLE     270

// Servos
#define SERVO_OPEN_ANGLE   0
#define SERVO_CLOSE_ANGLE  90
#define SERVO_MIN_US       544
#define SERVO_MAX_US       2400

// Timing
#define LIFT_TRAVEL_MS     3000
#define SETTLE_DELAY_MS    500

// ===================== SERVO LEDC CONFIG =====================
// Servos use 50Hz, 12-bit resolution on dedicated channels 6 & 7
// 12-bit = 4096 ticks per 20ms period
// 544us  = 544/20000 * 4096 = ~111 ticks (0 degrees)
// 2400us = 2400/20000 * 4096 = ~491 ticks (180 degrees)
#define SERVO_FREQ     50
#define SERVO_RES      12
#define SERVO_CH_LEFT  6
#define SERVO_CH_RIGHT 7
#define SERVO_TICK_MIN 111
#define SERVO_TICK_MAX 491

void servo_write(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  int duty = map(angle, 0, 180, SERVO_TICK_MIN, SERVO_TICK_MAX);
  ledcWrite(channel, duty);
}

// ===================== OBJECTS =====================

// L298N FULL4WIRE: IN1, IN2, IN3, IN4 = coilA+, coilA-, coilB+, coilB-
AccelStepper stepper(AccelStepper::FULL4WIRE, STEP_IN1, STEP_IN2, STEP_IN3, STEP_IN4);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===================== STATE =====================

float currentBaseAngle = 0;
int currentLeftServo = 0;
int currentRightServo = 0;
bool liftMoving = false;
bool motorsEnabled = true;
int leftWheelSpeed = 0;
int rightWheelSpeed = 0;
String inputBuffer = "";
unsigned long lastLcdUpdate = 0;

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);

  // --- STEP 1: Disable all drivers FIRST ---
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(LIFT_EN, OUTPUT);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(LIFT_EN, LOW);

  // --- STEP 2: Set PWM pins LOW before LEDC ---
  pinMode(LEFT_RPWM, OUTPUT);  digitalWrite(LEFT_RPWM, LOW);
  pinMode(LEFT_LPWM, OUTPUT);  digitalWrite(LEFT_LPWM, LOW);
  pinMode(RIGHT_RPWM, OUTPUT); digitalWrite(RIGHT_RPWM, LOW);
  pinMode(RIGHT_LPWM, OUTPUT); digitalWrite(RIGHT_LPWM, LOW);
  pinMode(LIFT_RPWM, OUTPUT);  digitalWrite(LIFT_RPWM, LOW);
  pinMode(LIFT_LPWM, OUTPUT);  digitalWrite(LIFT_LPWM, LOW);

  // --- STEP 3: Attach LEDC PWM (ESP32 Arduino core v2 API) ---
  // Channels paired in natural order so each motor's fwd/rev share one timer:
  //   LEFT  : ch0+ch1 -> timer 0
  //   RIGHT : ch2+ch3 -> timer 1
  //   LIFT  : ch4+ch5 -> timer 2
  //   SERVOS: ch6+ch7 -> timer 3 (50Hz)
  ledcSetup(LEFT_RPWM_CH,  PWM_FREQ, PWM_RES); ledcAttachPin(LEFT_RPWM,  LEFT_RPWM_CH);
  ledcSetup(LEFT_LPWM_CH,  PWM_FREQ, PWM_RES); ledcAttachPin(LEFT_LPWM,  LEFT_LPWM_CH);
  ledcSetup(RIGHT_RPWM_CH, PWM_FREQ, PWM_RES); ledcAttachPin(RIGHT_RPWM, RIGHT_RPWM_CH);
  ledcSetup(RIGHT_LPWM_CH, PWM_FREQ, PWM_RES); ledcAttachPin(RIGHT_LPWM, RIGHT_LPWM_CH);
  ledcSetup(LIFT_RPWM_CH,  PWM_FREQ, PWM_RES); ledcAttachPin(LIFT_RPWM,  LIFT_RPWM_CH);
  ledcSetup(LIFT_LPWM_CH,  PWM_FREQ, PWM_RES); ledcAttachPin(LIFT_LPWM,  LIFT_LPWM_CH);

  // --- STEP 4: Zero all PWM ---
  wheels_stop();
  lift_stop();

  // --- STEP 5: Enable drivers ---
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);
  digitalWrite(LIFT_EN, HIGH);

  // --- Stepper ---
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCEL);
  stepper.setCurrentPosition(0);

  // --- Servos via raw LEDC (channels 6 & 7, no library conflicts) ---
  ledcSetup(SERVO_CH_LEFT,  SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_LEFT_PIN, SERVO_CH_LEFT);
  ledcSetup(SERVO_CH_RIGHT, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_RIGHT_PIN, SERVO_CH_RIGHT);
  servos_open();

  // --- LCD I2C ---
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PET Bottle Robot");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // --- Buzzer ---
  pinMode(BUZZER_PIN, OUTPUT);

  // --- Ultrasonic sensors ---
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO_L, INPUT);
  pinMode(US_ECHO_R, INPUT);

  // --- WiFi AP ---
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("WiFi AP: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // --- Web Server ---
  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.on("/sensor", handleSensor);
  server.on("/ota", HTTP_GET, handleOtaPage);
  server.on("/ota", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    if (Update.hasError()) {
      server.send(500, "text/plain", "FAIL: " + String(Update.errorString()));
    } else {
      server.send(200, "text/html",
        "<html><body style='background:#111;color:#66bb6a;font-family:sans-serif;"
        "display:flex;align-items:center;justify-content:center;height:100vh;"
        "font-size:2rem'>Update OK! Rebooting...</body></html>");
      delay(1000);
      ESP.restart();
    }
  }, handleOtaUpload);
  server.begin();

  // --- Startup ---
  beep(2, 100);
  Serial.println("=== PET Bottle Robot Ready ===");
  Serial.println("Drivers: 3x BTS7960B + L298N + 2x MG996R + LCD");
  Serial.println("Control: USB Serial + WiFi AP (192.168.4.1)");
  Serial.println("OTA: http://192.168.4.1/ota");
  Serial.println("Send '?' for commands");
  printStatus();

  // LCD ready message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready! IP:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.softAPIP().toString());
}

// ===================== MAIN LOOP =====================

void loop() {
  stepper.run();
  server.handleClient();
  processSerial();

  // Update LCD every 500ms
  if (millis() - lastLcdUpdate > 500) {
    lastLcdUpdate = millis();
    lcd_update();
  }
}

// ===================== LCD =====================

void lcd_update() {
  lcd.setCursor(0, 0);
  if (!motorsEnabled) {
    lcd.print("!! E-STOP !!    ");
    lcd.setCursor(0, 1);
    lcd.print("Send H to reset ");
    return;
  }

  // Line 1: Wheels
  char line1[17];
  snprintf(line1, 17, "W:L%-4dR%-4d   ", leftWheelSpeed, rightWheelSpeed);
  lcd.print(line1);

  // Line 2: Base + Lift + Servos
  lcd.setCursor(0, 1);
  char line2[17];
  snprintf(line2, 17, "B:%-4d %s S:%d",
    (int)currentBaseAngle,
    liftMoving ? "LFT" : "   ",
    currentLeftServo);
  lcd.print(line2);
}

// ===================== SERIAL PARSER =====================

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        executeCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// ===================== WEB SERVER =====================

void handleCommand() {
  if (server.hasArg("c")) {
    String cmd = server.arg("c");
    cmd.trim();
    if (cmd.length() > 0) {
      executeCommand(cmd);
      server.send(200, "text/plain", "OK: " + cmd);
      return;
    }
  }
  server.send(400, "text/plain", "Missing ?c=<command>");
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>PET Bottle Robot</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:#111;color:#eee;padding:12px;
  -webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
h2{text-align:center;margin:8px 0;font-size:1.1rem;color:#888}
h1{text-align:center;font-size:1.4rem;margin-bottom:4px}
.section{background:#1a1a2e;border-radius:14px;padding:14px;margin:10px 0}
.grid{display:grid;gap:8px}
.g3{grid-template-columns:1fr 1fr 1fr}
.g2{grid-template-columns:1fr 1fr}
.g4{grid-template-columns:1fr 1fr 1fr 1fr}
button{font-size:1rem;font-weight:700;border:none;border-radius:10px;
  padding:14px 8px;cursor:pointer;transition:transform .08s;color:#111;
  touch-action:manipulation}
button:active{transform:scale(.94)}
.bf{background:#4fc3f7}.bb{background:#ff8a65}
.bl{background:#ffd54f}.br{background:#ffd54f}
.bx{background:#ef5350;color:#fff}
.bu{background:#66bb6a}.bd{background:#ff8a65}
.bs{background:#aaa}
.bo{background:#81c784}.bc{background:#e57373}
.bg{background:#4db6ac}
.bp{background:#ab47bc;color:#fff}
.bh{background:#78909c;color:#fff}
.be{background:#f44336;color:#fff;font-size:1.2rem}
.slider-row{display:flex;align-items:center;gap:8px;margin:8px 0}
.slider-row label{width:70px;font-size:.85rem;color:#aaa}
.slider-row input[type=range]{flex:1;height:28px}
.slider-row span{width:36px;text-align:right;font-family:monospace}
#log{background:#000;border-radius:8px;padding:8px;margin-top:10px;
  font-family:monospace;font-size:.75rem;color:#4fc3f7;height:80px;
  overflow-y:auto;white-space:pre-wrap}
</style>
</head>
<body>
<h1>PET Bottle Robot</h1>

<div class="section">
<h2>Drive Wheels</h2>
<div class="slider-row">
  <label>Speed</label>
  <input type="range" id="dspd" min="30" max="80" value="60">
  <span id="dspdV">80</span>
</div>
<div class="grid g3">
  <div></div>
  <button class="bf" onpointerdown="cmd('F'+ds())">Forward</button>
  <div></div>
  <button class="bl" onpointerdown="cmd('TL'+ds())">Left</button>
  <button class="bx" onpointerdown="cmd('X')">STOP</button>
  <button class="br" onpointerdown="cmd('TR'+ds())">Right</button>
  <div></div>
  <button class="bb" onpointerdown="cmd('J'+ds())">Back</button>
  <div></div>
</div>
</div>

<div class="section">
<h2>Arm Lift</h2>
<div class="slider-row">
  <label>Speed</label>
  <input type="range" id="lspd" min="50" max="255" value="80">
  <span id="lspdV">80</span>
</div>
<div class="grid g3">
  <button class="bu" onpointerdown="cmd('U'+ls())">Lift Up</button>
  <button class="bs" onpointerdown="cmd('S')">Stop</button>
  <button class="bd" onpointerdown="cmd('D'+ls())">Lift Down</button>
</div>
</div>

<div class="section">
<h2>Base Rotation</h2>
<div class="slider-row">
  <label>Angle</label>
  <input type="range" id="bang" min="-270" max="270" value="0">
  <span id="bangV">0</span>
</div>
<div class="grid g4">
  <button class="bg" onpointerdown="cmd('B'+document.getElementById('bang').value)">Go To</button>
  <button class="bg" onpointerdown="cmd('B-90')">-90</button>
  <button class="bg" onpointerdown="cmd('B90')">+90</button>
  <button class="bh" onpointerdown="cmd('BH')">Home</button>
</div>
</div>

<div class="section">
<h2>Servos (Left & Right Arm)</h2>
<div class="grid g2">
  <button class="bo" onpointerdown="cmd('O')">Both Open</button>
  <button class="bc" onpointerdown="cmd('C')">Both Close</button>
</div>
<div class="slider-row">
  <label>Left Srv</label>
  <input type="range" id="sl" min="0" max="180" value="0"
    oninput="document.getElementById('slV').textContent=this.value;cmd('L'+this.value)">
  <span id="slV">0</span>
</div>
<div class="slider-row">
  <label>Right Srv</label>
  <input type="range" id="sr" min="0" max="180" value="0"
    oninput="document.getElementById('srV').textContent=this.value;cmd('R'+this.value)">
  <span id="srV">0</span>
</div>
</div>

<div class="section">
<h2>System</h2>
<div class="grid g3">
  <button class="bp" onpointerdown="cmd('P')">Pickup</button>
  <button class="bh" onpointerdown="cmd('H')">Home All</button>
  <button class="be" onpointerdown="cmd('E')">EMERGENCY STOP</button>
</div>
</div>

<div id="log">Ready.\n</div>

<script>
const dspd=document.getElementById('dspd'),dspdV=document.getElementById('dspdV');
const lspd=document.getElementById('lspd'),lspdV=document.getElementById('lspdV');
const bang=document.getElementById('bang'),bangV=document.getElementById('bangV');
dspd.oninput=()=>dspdV.textContent=dspd.value;
lspd.oninput=()=>lspdV.textContent=lspd.value;
bang.oninput=()=>bangV.textContent=bang.value;
function ds(){return dspd.value}
function ls(){return lspd.value}
const log=document.getElementById('log');
function cmd(c){
  log.textContent+='> '+c+'\n';
  log.scrollTop=log.scrollHeight;
  fetch('/cmd?c='+encodeURIComponent(c))
    .then(r=>r.text()).then(t=>{log.textContent+=t+'\n';log.scrollTop=log.scrollHeight})
    .catch(e=>{log.textContent+='ERR: '+e+'\n'});
}
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ===================== COMMAND EXECUTOR =====================

void executeCommand(String cmd) {
  cmd.toUpperCase();
  char first = cmd.charAt(0);

  Serial.print("> "); Serial.println(cmd);

  switch (first) {
    // --- Drive Wheels ---
    case 'F':
      {
        int speed = cmd.length() > 1 ? cmd.substring(1).toInt() : DRIVE_DEFAULT;
        drive_forward(constrain(speed, 0, 255));
      }
      break;

    case 'J':
      {
        int speed = cmd.length() > 1 ? cmd.substring(1).toInt() : DRIVE_DEFAULT;
        drive_backward(constrain(speed, 0, 255));
      }
      break;

    case 'T':
      if (cmd.length() >= 2) {
        char dir = cmd.charAt(1);
        int speed = cmd.length() > 2 ? cmd.substring(2).toInt() : DRIVE_DEFAULT;
        speed = constrain(speed, 0, 255);
        if (dir == 'L') turn_left(speed);
        else if (dir == 'R') turn_right(speed);
      }
      break;

    case 'W':
      {
        int ci = cmd.indexOf(',');
        if (ci > 1) {
          int l = cmd.substring(1, ci).toInt();
          int r = cmd.substring(ci + 1).toInt();
          set_wheels(l, r);
        }
      }
      break;

    case 'X':
      wheels_stop();
      break;

    // --- Base Rotation ---
    case 'B':
      if (cmd == "BH") home_base();
      else rotate_base(cmd.substring(1).toInt());
      break;

    // --- Arm Lift ---
    case 'U':
      {
        int speed = cmd.length() > 1 ? cmd.substring(1).toInt() : LIFT_DEFAULT;
        lift_up(constrain(speed, 0, 255));
      }
      break;

    case 'D':
      {
        int speed = cmd.length() > 1 ? cmd.substring(1).toInt() : LIFT_DEFAULT;
        lift_down(constrain(speed, 0, 255));
      }
      break;

    case 'S':
      lift_stop();
      break;

    // --- Servos ---
    case 'L':  // Left servo
      {
        int angle = constrain(cmd.substring(1).toInt(), 0, 180);
        servo_write(SERVO_CH_LEFT, angle);
        currentLeftServo = angle;
        Serial.printf("Left Servo: %d deg\n", angle);
      }
      break;

    case 'R':  // Right servo
      {
        int angle = constrain(cmd.substring(1).toInt(), 0, 180);
        servo_write(SERVO_CH_RIGHT, angle);
        currentRightServo = angle;
        Serial.printf("Right Servo: %d deg\n", angle);
      }
      break;

    case 'O':  // Both open
      servos_open();
      break;

    case 'C':  // Both close
      servos_close();
      break;

    // --- System ---
    case 'P': pickup_sequence(); break;
    case 'H': home_position(); break;
    case 'E': emergency_stop(); break;
    case '?': printHelp(); printStatus(); break;
    default: Serial.println("Unknown. Send '?' for help."); break;
  }
}

// ===================== DRIVE WHEELS (BTS7960B #1 & #2) =====================

// Note: rpwm_ch / lpwm_ch are LEDC channel numbers (not pins).
void bts_drive(int rpwm_ch, int lpwm_ch, int en, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(en, HIGH);
  if (speed >= 0) {
    ledcWrite(rpwm_ch, speed);
    ledcWrite(lpwm_ch, 0);
  } else {
    ledcWrite(rpwm_ch, 0);
    ledcWrite(lpwm_ch, -speed);
  }
}

void set_wheels(int left, int right) {
  if (!motorsEnabled) { Serial.println("Motors disabled! Send 'H' to re-enable."); return; }
  left = constrain(left, -DRIVE_MAX, DRIVE_MAX);
  right = constrain(right, -DRIVE_MAX, DRIVE_MAX);
  bts_drive(LEFT_RPWM_CH, LEFT_LPWM_CH, LEFT_EN, left);
  bts_drive(RIGHT_RPWM_CH, RIGHT_LPWM_CH, RIGHT_EN, right);
  leftWheelSpeed = left;
  rightWheelSpeed = right;
  Serial.printf("Wheels: L=%d R=%d\n", left, right);
}

void drive_forward(int speed)  { set_wheels(speed, speed); }
void drive_backward(int speed) { set_wheels(-speed, -speed); }
void turn_left(int speed)      { set_wheels(-speed, speed); }
void turn_right(int speed)     { set_wheels(speed, -speed); }

void wheels_stop() {
  ledcWrite(LEFT_RPWM_CH, 0);  ledcWrite(LEFT_LPWM_CH, 0);
  ledcWrite(RIGHT_RPWM_CH, 0); ledcWrite(RIGHT_LPWM_CH, 0);
  leftWheelSpeed = 0;
  rightWheelSpeed = 0;
  Serial.println("Wheels: STOPPED");
}

// ===================== BASE ROTATION (Stepper via L298N) =====================

void rotate_base(int degrees) {
  if (!motorsEnabled) { Serial.println("Motors disabled!"); return; }
  degrees = constrain(degrees, -BASE_MAX_ANGLE, BASE_MAX_ANGLE);
  long target = (long)(degrees * STEPS_PER_DEGREE);
  Serial.printf("Base: -> %d deg (%ld steps)\n", degrees, target);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Base rotating...");
  lcd.setCursor(0, 1);
  char buf[17];
  snprintf(buf, 17, "Target: %d deg", degrees);
  lcd.print(buf);

  stepper.moveTo(target);
  while (stepper.distanceToGo() != 0) stepper.run();
  currentBaseAngle = degrees;
  Serial.printf("Base: at %d deg\n", degrees);
  stepper_release();
}

void home_base() {
  Serial.println("Base: homing");
  rotate_base(0);
}

void stepper_release() {
  digitalWrite(STEP_IN1, LOW); digitalWrite(STEP_IN2, LOW);
  digitalWrite(STEP_IN3, LOW); digitalWrite(STEP_IN4, LOW);
}

// ===================== ARM LIFT (BTS7960B #3) =====================

void lift_up(int speed) {
  if (!motorsEnabled) { Serial.println("Motors disabled!"); return; }
  bts_drive(LIFT_RPWM_CH, LIFT_LPWM_CH, LIFT_EN, constrain(speed, 0, 255));
  liftMoving = true;
  Serial.printf("Lift: UP %d\n", speed);
}

void lift_down(int speed) {
  if (!motorsEnabled) { Serial.println("Motors disabled!"); return; }
  bts_drive(LIFT_RPWM_CH, LIFT_LPWM_CH, LIFT_EN, -constrain(speed, 0, 255));
  liftMoving = true;
  Serial.printf("Lift: DOWN %d\n", speed);
}

void lift_stop() {
  ledcWrite(LIFT_RPWM_CH, 0); ledcWrite(LIFT_LPWM_CH, 0);
  liftMoving = false;
  Serial.println("Lift: STOPPED");
}

void lift_up_timed(int speed, int ms) { lift_up(speed); delay(ms); lift_stop(); }
void lift_down_timed(int speed, int ms) { lift_down(speed); delay(ms); lift_stop(); }

// ===================== SERVOS (Left & Right) =====================

void servos_open() {
  servo_write(SERVO_CH_LEFT, SERVO_OPEN_ANGLE);
  servo_write(SERVO_CH_RIGHT, SERVO_OPEN_ANGLE);
  currentLeftServo = SERVO_OPEN_ANGLE;
  currentRightServo = SERVO_OPEN_ANGLE;
  Serial.println("Servos: BOTH OPEN");
}

void servos_close() {
  servo_write(SERVO_CH_LEFT, SERVO_CLOSE_ANGLE);
  servo_write(SERVO_CH_RIGHT, SERVO_CLOSE_ANGLE);
  currentLeftServo = SERVO_CLOSE_ANGLE;
  currentRightServo = SERVO_CLOSE_ANGLE;
  Serial.println("Servos: BOTH CLOSED");
}

// ===================== SEQUENCES =====================

void pickup_sequence() {
  if (!motorsEnabled) { Serial.println("Motors disabled!"); return; }
  Serial.println("=== PICKUP START ===");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PICKUP SEQUENCE");

  beep(1, 200);

  wheels_stop();
  servos_open();
  delay(SETTLE_DELAY_MS);

  lcd.setCursor(0, 1); lcd.print("Lowering arm... ");
  lift_down_timed(150, LIFT_TRAVEL_MS);
  delay(SETTLE_DELAY_MS);

  lcd.setCursor(0, 1); lcd.print("Grabbing...     ");
  servos_close();
  delay(SETTLE_DELAY_MS);

  lcd.setCursor(0, 1); lcd.print("Lifting...      ");
  lift_up_timed(200, LIFT_TRAVEL_MS);
  delay(SETTLE_DELAY_MS);

  lcd.setCursor(0, 1); lcd.print("Rotating base...");
  rotate_base(180);
  delay(SETTLE_DELAY_MS);

  lcd.setCursor(0, 1); lcd.print("Releasing...    ");
  lift_down_timed(120, 1000);
  delay(300);

  servos_open();
  delay(SETTLE_DELAY_MS);

  lcd.setCursor(0, 1); lcd.print("Returning home..");
  lift_up_timed(200, 1500);
  delay(SETTLE_DELAY_MS);

  home_position();
  beep(2, 100);
  Serial.println("=== PICKUP DONE ===");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PICKUP COMPLETE!");
}

void home_position() {
  Serial.println("Homing...");
  motorsEnabled = true;
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);
  digitalWrite(LIFT_EN, HIGH);

  wheels_stop();
  lift_up_timed(200, 2000);
  delay(SETTLE_DELAY_MS);
  servos_open();
  delay(SETTLE_DELAY_MS);
  home_base();
  Serial.println("All homed.");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("All Homed");
}

void emergency_stop() {
  Serial.println("!!! EMERGENCY STOP !!!");
  wheels_stop();
  lift_stop();
  stepper.stop();
  stepper.setCurrentPosition(stepper.currentPosition());
  stepper_release();
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(LIFT_EN, LOW);
  motorsEnabled = false;
  beep(5, 50);
  Serial.println("All stopped. Send 'H' to re-enable.");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("!! E-STOP !!   ");
  lcd.setCursor(0, 1);
  lcd.print("Send H to reset");
}

// ===================== UTILITY =====================

void beep(int count, int dur) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(dur);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(dur);
  }
}

void printStatus() {
  Serial.println("--- STATUS ---");
  Serial.printf("Wheels:     L=%d R=%d\n", leftWheelSpeed, rightWheelSpeed);
  Serial.printf("Base:       %.1f deg\n", currentBaseAngle);
  Serial.printf("Left Srv:   %d deg\n", currentLeftServo);
  Serial.printf("Right Srv:  %d deg\n", currentRightServo);
  Serial.printf("Lift:       %s\n", liftMoving ? "MOVING" : "STOPPED");
  Serial.printf("Motors:     %s\n", motorsEnabled ? "ENABLED" : "DISABLED");
  Serial.printf("WiFi AP:    %s @ %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("LCD:        SDA=%d SCL=%d\n", LCD_SDA, LCD_SCL);
  Serial.println("--------------");
}

void printHelp() {
  Serial.println("=== COMMANDS ===");
  Serial.println("F/J<spd>  Forward/Back   TL/TR<spd> Turn");
  Serial.println("W<l>,<r>  Raw wheels     X  Stop wheels");
  Serial.println("B<ang>    Base rotate    BH Home base");
  Serial.println("U/D<spd>  Lift up/down   S  Stop lift");
  Serial.println("L<ang>    Left servo     R<ang> Right servo");
  Serial.println("O  Both open   C  Both close");
  Serial.println("P Pickup  H Home all  E Emergency  ? Help");
  Serial.println("OTA: http://192.168.4.1/ota");
  Serial.println("================");
}

// ===================== ULTRASONIC SENSORS =====================

long readUltrasonic(int echoPin) {
  // Shared trigger pin — pulse once, read the specified echo
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
  if (duration <= 0) return 999;
  return duration * 0.034 / 2;  // cm
}

void handleSensor() {
  long distL = readUltrasonic(US_ECHO_L);
  delay(15);  // prevent sonar overlap
  long distR = readUltrasonic(US_ECHO_R);

  // JSON response for Pi navigator
  String json = "{\"left\":" + String(distL)
              + ",\"right\":" + String(distR)
              + ",\"wheels\":{\"left\":" + String(leftWheelSpeed)
              + ",\"right\":" + String(rightWheelSpeed) + "}"
              + ",\"base\":" + String((int)currentBaseAngle)
              + ",\"lift\":" + String(liftMoving ? 1 : 0)
              + ",\"motors\":" + String(motorsEnabled ? 1 : 0)
              + "}";
  server.send(200, "application/json", json);
}

// ===================== OTA FIRMWARE UPDATE =====================

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
body{font-family:sans-serif;background:#111;color:#eee;
  display:flex;flex-direction:column;align-items:center;
  justify-content:center;min-height:100vh;padding:24px}
.card{background:#1a1a2e;border-radius:20px;padding:40px 32px;
  max-width:480px;width:100%;text-align:center;
  box-shadow:0 8px 40px rgba(0,0,0,.5)}
h1{font-size:1.5rem;margin-bottom:8px}
p{color:#888;font-size:.9rem;margin-bottom:24px}
input[type=file]{display:none}
label.btn{display:inline-block;background:#4fc3f7;color:#111;
  font-size:1.1rem;font-weight:700;border-radius:14px;
  padding:16px 36px;cursor:pointer;margin:8px}
button{font-size:1.1rem;font-weight:700;border:none;border-radius:14px;
  padding:16px 36px;cursor:pointer;background:#66bb6a;color:#111;margin:8px}
button:disabled{opacity:.4;cursor:not-allowed}
.back{background:#2a2a3e;color:#aaa;font-size:.9rem;
  padding:10px 24px;margin-top:16px;border-radius:10px;
  text-decoration:none;display:inline-block}
#fname{color:#4fc3f7;font-family:monospace;margin:12px 0;font-size:.85rem}
#prog{width:100%;height:24px;border-radius:12px;overflow:hidden;
  background:#333;margin:16px 0;display:none}
#bar{height:100%;background:#66bb6a;width:0%;transition:width .3s}
#status{font-size:1rem;margin:12px 0;min-height:24px}
.warn{color:#ffd54f;font-size:.8rem;margin-top:12px}
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
      bar.style.background = '#66bb6a';
      status.textContent = 'Update complete! Rebooting...';
      status.style.color = '#66bb6a';
      setTimeout(function() { location.href = '/'; }, 5000);
    } else {
      bar.style.background = '#ef5350';
      status.textContent = 'FAILED: ' + xhr.responseText;
      status.style.color = '#ef5350';
      document.getElementById('upload').disabled = false;
    }
  };
  xhr.onerror = function() {
    status.textContent = 'Connection lost. Rebooting...';
    status.style.color = '#ffd54f';
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
    lcd.setCursor(0, 0);
    lcd.print("OTA UPDATE...");
    lcd.setCursor(0, 1);
    lcd.print("DO NOT POWER OFF");

    // Stop all motors for safety during OTA
    wheels_stop();
    lift_stop();
    stepper_release();

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
      lcd.setCursor(0, 0);
      lcd.print("OTA OK!");
      lcd.setCursor(0, 1);
      lcd.print("Rebooting...");
    } else {
      Serial.printf("OTA: End failed: %s\n", Update.errorString());
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("OTA FAILED!");
    }
  }
}
