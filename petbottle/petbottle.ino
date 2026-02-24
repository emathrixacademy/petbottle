#include "BluetoothSerial.h"

// --- Hardware Pins ---
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
  Serial.println("Robot Ready: Dynamic Proximity Beeping Active.");
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return (distance == 0) ? 999 : distance; // Return 999 if out of range
}

void loop() {
  long distance = getDistance();
  
  // Only beep if the object is closer than 30cm
  if (distance < 30) {
    // Alert the Pi
    SerialBT.print("PROXIMITY:");
    SerialBT.println(distance);

    // Calculate beep speed
    // 2cm = 50ms delay (fast), 30cm = 500ms delay (slow)
    int beepDelay = map(distance, 2, 30, 50, 500);
    beepDelay = constrain(beepDelay, 50, 500); 

    // Beep
    digitalWrite(BUZZER_PIN, HIGH);
    delay(40); // Constant short beep duration
    digitalWrite(BUZZER_PIN, LOW);
    
    // Wait based on distance
    delay(beepDelay); 
  } else {
    delay(100); // Slow down loop if nothing is near
  }

  // Manual command check
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd == "BEEP") {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}