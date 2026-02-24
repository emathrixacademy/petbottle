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
  Serial.println("Robot Ready: Ultrasonic + Buzzer Active.");
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  // Calculate distance in cm
  return duration * 0.034 / 2;
}

void loop() {
  long distance = getDistance();
  
  // If an object (bottle) is detected within 10cm
  if (distance > 0 && distance < 10) {
    Serial.print("Object Detected: ");
    Serial.print(distance);
    Serial.println("cm");
    
    // Alert the Raspberry Pi via Bluetooth
    SerialBT.print("BOTTLE_DETECTED:");
    SerialBT.println(distance);
    
    // Beep the buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    
    delay(1000); // Prevent spamming the Pi
  }

  // Check if Pi sent a manual buzzer command
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd == "BEEP") {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      SerialBT.println("Buzzer Triggered Manually.");
    }
  }
  
  delay(100); 
}