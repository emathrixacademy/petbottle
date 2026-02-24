
#define D80NK_PIN 23
#define TRIG_PIN 22
#define ECHO_PIN 21
#define BUZZER_PIN 19

void setup() {
  Serial.begin(115200);
  pinMode(D80NK_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  Serial.println("PRIORITY LOGIC MODE");
  
  // Startup
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);
}

void loop() {
  bool d80nkDetected = (digitalRead(D80NK_PIN) == LOW);
  
  if(d80nkDetected) {
    // PRIMARY: D80NK detected - use ultrasonic for distance
    int distance = getUltrasonicDistance();
    
    Serial.print("🔴 PRIMARY Detection | Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    proximityBuzz(distance);
    
  } else {
    // SECONDARY: Check ultrasonic for far objects
    int distance = getUltrasonicDistance();
    
    if(distance > 0 && distance < 100) {
      Serial.print("📡 SECONDARY Detection | Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      
      proximityBuzz(distance);
    } else {
      // All clear
      digitalWrite(BUZZER_PIN, LOW);
      delay(50);
    }
  }
}

int getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  
  if(distance == 0 || distance > 400) return 999;
  return distance;
}

void proximityBuzz(int dist) {
  if(dist < 10) {
    digitalWrite(BUZZER_PIN, HIGH); delay(15);
    digitalWrite(BUZZER_PIN, LOW); delay(15);
  } else if(dist < 30) {
    digitalWrite(BUZZER_PIN, HIGH); delay(40);
    digitalWrite(BUZZER_PIN, LOW); delay(40);
  } else if(dist < 60) {
    digitalWrite(BUZZER_PIN, HIGH); delay(80);
    digitalWrite(BUZZER_PIN, LOW); delay(80);
  } else {
    digitalWrite(BUZZER_PIN, HIGH); delay(150);
    digitalWrite(BUZZER_PIN, LOW); delay(150);
  }
}