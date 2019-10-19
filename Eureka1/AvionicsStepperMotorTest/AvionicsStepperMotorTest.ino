int stepPin = 9;
int dirPin = 7;
//int enablePin = 8;
int dir = HIGH;

void setup() {
  pinMode(stepPin, OUTPUT);
//  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(enablePin, LOW);
  
}

void loop() {
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, dir);

  for (int i=0; i<2510; i++) {
    digitalWrite(stepPin, HIGH);
    //digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(stepPin, LOW);
    //digitalWrite(LED_BUILTIN, LOW);
    delayMicroseconds(400);
  }
  //dir = !dir;
  //digitalWrite(dirPin, dir);
  delay(10);
}
