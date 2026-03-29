/*
 * PET Bottle Robot - ESP32 Motor Controller
 * Motors: MY1016Z 24V DC via BTS7960 drivers
 * Control: WiFi Web Controller + Bluetooth Serial from Raspberry Pi
 *
 * WiFi: Connect to same network, open ESP32 IP in browser
 * Bluetooth commands from Pi:
 *   F<speed>   - Forward   (e.g., F200)
 *   B<speed>   - Backward  (e.g., B150)
 *   L<speed>   - Turn Left (e.g., L180)
 *   R<speed>   - Turn Right(e.g., R180)
 *   S          - Stop
 *   P<L>,<R>   - Direct PWM (e.g., P200,150)
 */

#include <WiFi.h>
#include <WebServer.h>
#include "BluetoothSerial.h"

// ── WiFi Settings (CHANGE THESE) ──
const char* WIFI_SSID     = "eMathrixTech";
const char* WIFI_PASSWORD = "eM@thr1x2026";

BluetoothSerial SerialBT;
WebServer server(80);

// ── BTS7960 #1 - Left Motor ──
#define LEFT_RPWM   25
#define LEFT_LPWM   26
#define LEFT_EN     32

// ── BTS7960 #2 - Right Motor ──
#define RIGHT_RPWM  27
#define RIGHT_LPWM  14
#define RIGHT_EN    33

// ── LED ──
#define LED_PIN     2

// ── PWM Settings ──
#define PWM_FREQ      20000
#define PWM_RESOLUTION 8

// ── Safety ──
#define TIMEOUT_MS    500
unsigned long lastCmdTime = 0;
bool motorsActive = false;
int currentSpeed = 150;
String lastAction = "STOP";

String inputBuffer = "";

// ── Web Page ──
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>PET Bottle Robot</title>
  <style>
    * { margin:0; padding:0; box-sizing:border-box; }
    body {
      font-family: Arial, sans-serif;
      background: #1a1a2e;
      color: #fff;
      display: flex;
      flex-direction: column;
      align-items: center;
      height: 100vh;
      padding: 15px;
      user-select: none;
      -webkit-user-select: none;
    }
    h1 { font-size: 20px; color: #00d4ff; margin-bottom: 10px; }
    .status {
      background: #16213e;
      padding: 8px 20px;
      border-radius: 8px;
      margin-bottom: 15px;
      font-size: 14px;
      text-align: center;
    }
    .status span { color: #0f0; font-weight: bold; }
    .speed-section {
      width: 100%;
      max-width: 320px;
      margin-bottom: 15px;
      text-align: center;
    }
    .speed-section label { font-size: 14px; color: #aaa; }
    .speed-val { color: #00d4ff; font-size: 28px; font-weight: bold; }
    input[type=range] {
      width: 100%;
      height: 30px;
      margin: 5px 0;
      accent-color: #00d4ff;
    }
    .controls {
      display: grid;
      grid-template-columns: 90px 90px 90px;
      grid-template-rows: 90px 90px 90px;
      gap: 8px;
      margin-bottom: 15px;
    }
    .btn {
      border: none;
      border-radius: 12px;
      font-size: 28px;
      font-weight: bold;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: transform 0.1s;
      -webkit-tap-highlight-color: transparent;
    }
    .btn:active { transform: scale(0.92); }
    .btn-fwd  { grid-column:2; grid-row:1; background:#00c853; color:#fff; }
    .btn-left { grid-column:1; grid-row:2; background:#ff9800; color:#fff; }
    .btn-stop { grid-column:2; grid-row:2; background:#f44336; color:#fff; font-size:16px; }
    .btn-right{ grid-column:3; grid-row:2; background:#ff9800; color:#fff; }
    .btn-back { grid-column:2; grid-row:3; background:#2196f3; color:#fff; }
    .log {
      width: 100%;
      max-width: 320px;
      background: #0d1117;
      border-radius: 8px;
      padding: 10px;
      font-family: monospace;
      font-size: 12px;
      color: #8b949e;
      height: 100px;
      overflow-y: auto;
      flex-shrink: 0;
    }
  </style>
</head>
<body>
  <h1>PET BOTTLE ROBOT</h1>
  <div class="status">
    Status: <span id="st">CONNECTED</span> | Action: <span id="act">STOP</span>
  </div>
  <div class="speed-section">
    <label>SPEED</label><br>
    <span class="speed-val" id="sv">150</span>
    <input type="range" id="spd" min="50" max="255" value="150"
           oninput="document.getElementById('sv').innerText=this.value">
  </div>
  <div class="controls">
    <button class="btn btn-fwd"   ontouchstart="send('F')" onmousedown="send('F')">&#9650;</button>
    <button class="btn btn-left"  ontouchstart="send('L')" onmousedown="send('L')">&#9664;</button>
    <button class="btn btn-stop"  ontouchstart="send('S')" onmousedown="send('S')">STOP</button>
    <button class="btn btn-right" ontouchstart="send('R')" onmousedown="send('R')">&#9654;</button>
    <button class="btn btn-back"  ontouchstart="send('B')" onmousedown="send('B')">&#9660;</button>
  </div>
  <div class="log" id="log">Ready...<br></div>
  <script>
    function send(cmd) {
      var s = document.getElementById('spd').value;
      var url = '/cmd?c=' + cmd + '&s=' + s;
      fetch(url)
        .then(r => r.text())
        .then(t => {
          document.getElementById('act').innerText = t;
          var log = document.getElementById('log');
          var d = new Date().toLocaleTimeString();
          log.innerHTML = '[' + d + '] ' + cmd + ' speed=' + s + ' -> ' + t + '<br>' + log.innerHTML;
        })
        .catch(e => {
          document.getElementById('st').innerText = 'ERROR';
          document.getElementById('st').style.color = 'red';
        });
    }
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Enable pins
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);

  // Attach PWM to motor pins (ESP32 core v3.x API)
  ledcAttach(LEFT_RPWM,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LEFT_LPWM,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_LPWM, PWM_FREQ, PWM_RESOLUTION);

  // Start stopped
  stopMotors();

  // WiFi
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Web server routes
    server.on("/", []() {
      server.send(200, "text/html", HTML_PAGE);
    });

    server.on("/cmd", []() {
      String cmd = server.arg("c");
      String spd = server.arg("s");
      int speed = spd.toInt();
      speed = constrain(speed, 0, 255);
      currentSpeed = speed;

      String result = "OK";
      if (cmd == "F") {
        setMotors(speed, speed);
        result = "FORWARD";
      } else if (cmd == "B") {
        setMotors(-speed, -speed);
        result = "BACKWARD";
      } else if (cmd == "L") {
        setMotors(-speed, speed);
        result = "LEFT";
      } else if (cmd == "R") {
        setMotors(speed, -speed);
        result = "RIGHT";
      } else if (cmd == "S") {
        stopMotors();
        result = "STOP";
      }
      lastAction = result;
      lastCmdTime = millis();
      Serial.printf("[WEB] %s speed=%d\n", result.c_str(), speed);
      server.send(200, "text/plain", result);
    });

    server.begin();
    Serial.println("Web server started!");
  } else {
    Serial.println("\nWiFi failed - continuing with Bluetooth only");
  }

  // Bluetooth
  SerialBT.begin("ESP32_BOTTLE_COLLECTOR");

  Serial.println("=================================");
  Serial.println("PET Bottle Robot Motor Controller");
  Serial.println("Bluetooth: ESP32_BOTTLE_COLLECTOR");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Web Control: http://%s\n", WiFi.localIP().toString().c_str());
  }
  Serial.println("=================================");

  // Quick LED flash to confirm boot
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

void loop() {
  // Handle web requests
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  // Read Bluetooth commands
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  // Safety timeout
  if (motorsActive && (millis() - lastCmdTime > TIMEOUT_MS)) {
    stopMotors();
    Serial.println("[SAFETY] Timeout - motors stopped");
    SerialBT.println("TIMEOUT");
  }
}

void processCommand(String cmd) {
  lastCmdTime = millis();
  cmd.trim();

  char type = cmd.charAt(0);
  int speed = 0;

  if (cmd.length() > 1) {
    speed = cmd.substring(1).toInt();
  }
  speed = constrain(speed, 0, 255);

  switch (type) {
    case 'F': case 'f':
      setMotors(speed, speed);
      Serial.printf("[FWD] speed=%d\n", speed);
      SerialBT.printf("OK:F%d\n", speed);
      break;

    case 'B': case 'b':
      setMotors(-speed, -speed);
      Serial.printf("[REV] speed=%d\n", speed);
      SerialBT.printf("OK:B%d\n", speed);
      break;

    case 'L': case 'l':
      setMotors(-speed, speed);
      Serial.printf("[LEFT] speed=%d\n", speed);
      SerialBT.printf("OK:L%d\n", speed);
      break;

    case 'R': case 'r':
      setMotors(speed, -speed);
      Serial.printf("[RIGHT] speed=%d\n", speed);
      SerialBT.printf("OK:R%d\n", speed);
      break;

    case 'S': case 's':
      stopMotors();
      Serial.println("[STOP]");
      SerialBT.println("OK:S");
      break;

    case 'P': case 'p':
    {
      int commaIdx = cmd.indexOf(',', 1);
      if (commaIdx > 0) {
        int leftPWM  = cmd.substring(1, commaIdx).toInt();
        int rightPWM = cmd.substring(commaIdx + 1).toInt();
        leftPWM  = constrain(leftPWM, -255, 255);
        rightPWM = constrain(rightPWM, -255, 255);
        setMotors(leftPWM, rightPWM);
        Serial.printf("[PWM] L=%d R=%d\n", leftPWM, rightPWM);
        SerialBT.printf("OK:P%d,%d\n", leftPWM, rightPWM);
      }
      break;
    }

    default:
      Serial.printf("[UNKNOWN] %s\n", cmd.c_str());
      SerialBT.println("ERR:UNKNOWN");
      break;
  }
}

void setMotors(int left, int right) {
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);
  motorsActive = true;

  if (left >= 0) {
    ledcWrite(LEFT_RPWM, left);
    ledcWrite(LEFT_LPWM, 0);
  } else {
    ledcWrite(LEFT_RPWM, 0);
    ledcWrite(LEFT_LPWM, -left);
  }

  if (right >= 0) {
    ledcWrite(RIGHT_RPWM, right);
    ledcWrite(RIGHT_LPWM, 0);
  } else {
    ledcWrite(RIGHT_RPWM, 0);
    ledcWrite(RIGHT_LPWM, -right);
  }

  digitalWrite(LED_PIN, (left != 0 || right != 0) ? HIGH : LOW);
}

void stopMotors() {
  ledcWrite(LEFT_RPWM, 0);
  ledcWrite(LEFT_LPWM, 0);
  ledcWrite(RIGHT_RPWM, 0);
  ledcWrite(RIGHT_LPWM, 0);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(LED_PIN, LOW);
  motorsActive = false;
}
