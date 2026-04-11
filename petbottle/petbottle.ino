#include "BluetoothSerial.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// --- Sensor Pins (Safe Pins for Uploading) ---
#define L_TRIG 5
#define L_ECHO 18
#define R_TRIG 32
#define R_ECHO 33
#define BUZZER_PIN 19

// --- WiFi AP ---
const char* AP_SSID = "PetBottle_Sensor";
const char* AP_PASS = "petbottle123";

BluetoothSerial SerialBT;
WebServer server(80);

// Latest sensor readings (updated each loop, served over HTTP)
volatile long lastDistL = 999;
volatile long lastDistR = 999;

// Forward declarations
void handleRoot();
void handleSensor();
void handleOtaPage();
void handleOtaUpload();

void setup() {
  Serial.begin(115200);

  pinMode(L_TRIG, OUTPUT); pinMode(L_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT); pinMode(R_ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  SerialBT.begin("ESP32_Chat");

  // --- WiFi AP ---
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("WiFi AP: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // --- Web Server Routes ---
  server.on("/", handleRoot);
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

  // Quick beep to confirm setup is done
  digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Robot Ready. Monitoring Left (5/18) and Right (32/33)");
  Serial.println("OTA: http://192.168.4.1/ota");
}

long readDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Increased timeout to 30ms
  long duration = pulseIn(echo, HIGH, 30000);

  if (duration <= 0) return 999;
  return duration * 0.034 / 2;
}

void loop() {
  // Service web server (handles /sensor polls and OTA uploads)
  server.handleClient();

  long distL = readDistance(L_TRIG, L_ECHO);
  delay(20); // Gap to prevent sonar waves overlapping
  long distR = readDistance(R_TRIG, R_ECHO);

  lastDistL = distL;
  lastDistR = distR;

  // Send to Bluetooth
  SerialBT.print("L:"); SerialBT.print(distL);
  SerialBT.print(",R:"); SerialBT.println(distR);

  // Serial Debug
  Serial.printf("L: %ld cm | R: %ld cm\n", distL, distR);

  // Variable Beep
  long closest = min(distL, distR);
  if (closest < 30) {
    int speed = map(closest, 2, 30, 50, 400);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(30);
    digitalWrite(BUZZER_PIN, LOW);
    delay(constrain(speed, 50, 400));
  } else {
    delay(100);
  }
}

// ===================== WEB HANDLERS =====================

void handleRoot() {
  String html = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>PET Bottle Sensor</title>"
                "<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:24px}"
                "a{color:#4fc3f7;display:inline-block;margin:12px;padding:12px 24px;background:#1a1a2e;"
                "border-radius:10px;text-decoration:none}</style></head><body>"
                "<h1>PET Bottle Sensor ESP32</h1>"
                "<p>L: " + String(lastDistL) + " cm &nbsp; R: " + String(lastDistR) + " cm</p>"
                "<a href='/sensor'>JSON /sensor</a>"
                "<a href='/ota'>OTA Update</a>"
                "</body></html>";
  server.send(200, "text/html", html);
}

void handleSensor() {
  String json = "{\"left\":" + String(lastDistL) +
                ",\"right\":" + String(lastDistR) + "}";
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
  <p>PET Bottle Sensor - OTA</p>
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
  <a class="back" href="/">Back</a>
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
    // Buzzer off during update
    digitalWrite(BUZZER_PIN, LOW);
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
    } else {
      Serial.printf("OTA: End failed: %s\n", Update.errorString());
    }
  }
}
