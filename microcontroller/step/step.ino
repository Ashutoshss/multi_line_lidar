#define STEP_PIN 15
#define DIR_PIN 2    


void setup() {
    Serial.begin(115200);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
}


void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    int direction = input.toInt();

    if (direction > 0) {
      digitalWrite(DIR_PIN, HIGH);  
    } else if (direction < 0) {
      digitalWrite(DIR_PIN, LOW);   
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);  
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(10);
  }
}

