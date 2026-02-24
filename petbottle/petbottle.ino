#include "BluetoothSerial.h"
#include <Stepper.h>

// --- Hardware Pins ---
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

// --- Stepper Config ---
const int stepsPerRevolution = 200; 
// Lowered speed to 30 RPM to provide more torque at 5V
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  
  // 30 RPM is safer for 5V batteries
  myStepper.setSpeed(30); 
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  SerialBT.begin("ESP32_Chat"); 
  Serial.println("Robot Ready (5V Low Power Mode)");
}

void releaseStepper() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  if (SerialBT.available()) {
    String data = SerialBT.readStringUntil('\n');
    data.trim();

    if (data.length() > 0) {
      if (isDigit(data[0]) || (data[0] == '-' && isDigit(data[1]))) {
        float degrees = data.toFloat();
        int stepsToMove = (int)((degrees * stepsPerRevolution) / 360.0);
        
        SerialBT.print("Moving ");
        SerialBT.print(degrees);
        SerialBT.println(" deg @ 5V...");

        myStepper.step(stepsToMove);
        
        SerialBT.println("Done.");
        // CRITICAL: Always release at 5V to save battery
        releaseStepper(); 
      }
      else if (data == "OFF") {
        releaseStepper();
        SerialBT.println("Motors released.");
      }
    }
  }
}