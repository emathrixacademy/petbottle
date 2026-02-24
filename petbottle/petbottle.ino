#include "BluetoothSerial.h"

// --- Sensor Pins ---
#define L_TRIG 5
#define L_ECHO 18
#define R_TRIG 17
#define R_ECHO 16
#define BUZZER_PIN 19

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  pinMode(L_TRIG, OUTPUT); pinMode(L_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT); pinMode(R_ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  SerialBT.begin("ESP32_Chat");
}

long readDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 30000); // 30ms timeout
  return (d == 0) ? 999 : d * 0.034 / 2;
}

void loop() {
  long distL = readDistance(L_TRIG, L_ECHO);
  long distR = readDistance(R_TRIG, R_ECHO);

  // Send combined data: "L:15,R:22"
  SerialBT.print("L:"); SerialBT.print(distL);
  SerialBT.print(",R:"); SerialBT.println(distR);

  // Beep logic: Beep if either sensor is close
  long closest = min(distL, distR);
  if (closest < 30) {
    int beepDelay = map(closest, 2, 30, 40, 400);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(30);
    digitalWrite(BUZZER_PIN, LOW);
    delay(constrain(beepDelay, 40, 400));
  } else {
    delay(60); 
  }
}