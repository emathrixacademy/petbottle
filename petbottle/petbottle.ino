#include "BluetoothSerial.h"

// --- Sensor Pins (Safe Pins for Uploading) ---
#define L_TRIG 5
#define L_ECHO 18
#define R_TRIG 32 
#define R_ECHO 33 
#define BUZZER_PIN 19

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  
  pinMode(L_TRIG, OUTPUT); pinMode(L_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT); pinMode(R_ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  SerialBT.begin("ESP32_Chat");
  
  // Quick beep to confirm setup is done
  digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Robot Ready. Monitoring Left (5/18) and Right (32/33)");
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
  long distL = readDistance(L_TRIG, L_ECHO);
  delay(20); // Gap to prevent sonar waves overlapping
  long distR = readDistance(R_TRIG, R_ECHO);

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