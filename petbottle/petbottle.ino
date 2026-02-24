#include "BluetoothSerial.h"

#define TRIG_PIN 5
#define ECHO_PIN 18
#define BUZZER_PIN 19

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  SerialBT.begin("ESP32_Chat");
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
}

void loop() {
  long distance = getDistance();
  
  // Stream data to Pi constantly for real-time visualization
  if (distance > 0 && distance < 100) {
    SerialBT.print("D:");
    SerialBT.println(distance);
  }

  // Dynamic Beeping Logic
  if (distance > 0 && distance < 30) {
    int beepDelay = map(distance, 2, 30, 30, 400);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(30); 
    digitalWrite(BUZZER_PIN, LOW);
    delay(constrain(beepDelay, 30, 400));
  } else {
    delay(50); // High-speed refresh rate
  }
}