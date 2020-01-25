int stepPin = 3;
int dirPin = 4;
//int enablePin = 8;
int dir = LOW;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, dir);
  digitalWrite(stepPin, LOW);

}

//dir low is to close the motor without the writing
// +++++++++++++++++++++++++++------------

void loop() {

  for (int i=0; i<2510; i++) {
    digitalWrite(stepPin, HIGH);
    //digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(400);
    digitalWrite(stepPin, LOW);
    //digitalWrite(LED_BUILTIN, LOW);
    delayMicroseconds(400);
  }

  delay(1000);
}
