#include "BluetoothSerial.h"

// Check if Bluetooth is properly enabled in the ESP32 config
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  // Serial monitor for debugging
  Serial.begin(115200);
  
  // The name that will show up on your Raspberry Pi
  SerialBT.begin("ESP32_Chat"); 
  
  Serial.println("The device started, now you can pair it with Bluetooth!");
}

void loop() {
  // 1. Read from Raspberry Pi and print to Serial Monitor
  if (SerialBT.available()) {
    Serial.print("Received from Pi: ");
    while (SerialBT.available()) {
      char incomingChar = SerialBT.read();
      Serial.print(incomingChar);
    }
    Serial.println(); // New line for readability

    // 2. Send an automatic response back to the Pi
    SerialBT.println("ESP32: Message received successfully!");
  }

  // 3. Optional: Send typing from Serial Monitor to the Pi
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  
  delay(20);
}