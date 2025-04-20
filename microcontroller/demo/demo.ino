const int dirPin = 2;     // Direction pin
const int stepPin = 15;    // Step pin
const int steps = 50;     // Number of steps

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10);
  }

  digitalWrite(dirPin, LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10);
  }
}
