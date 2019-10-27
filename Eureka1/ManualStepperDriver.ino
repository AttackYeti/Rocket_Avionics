
// ___ Pin Definitions ___
const int stepPin = 9;
const int directionPin = 8;
const int enablePin = 12;

const int directionButtonPin = 3;
const int enableButtonPin = 5;

// ___ State Variable Definitions ___
int lastMotorDirection = HIGH;
int lastMotorEnable = LOW;
int motorPosition = 0;

int lastTime = micros();
int lastPulse = LOW;

int lastLog = 0;
int logFrequency = 10;

int now = 0;
int sensorValue = 0;

// ___Physical Parameters ___
int startSpeed = 600;
int minimumStepDelayMicros = 450;
int stepAcceleration = 5;
int stepTime = startSpeed;

void getInput() {
  int motorDirection = digitalRead(directionButtonPin);
  if (motorDirection != lastMotorDirection) {
    lastMotorDirection = motorDirection;
    digitalWrite(directionPin, lastMotorDirection);
    digitalWrite(LED_BUILTIN, lastMotorDirection);
  }
  
  int motorEnable = digitalRead(enableButtonPin);
  if (motorEnable != lastMotorEnable) {
    stepTime = startSpeed;
    lastMotorEnable = motorEnable;
    digitalWrite(enablePin, lastMotorEnable);
  }
}

void updateStepper() {
  now = micros();

  if (lastMotorEnable == LOW) {
    if (abs(now-lastTime) >= stepTime) {
      stepTime -= stepAcceleration;
      if (stepTime < minimumStepDelayMicros) {
        stepTime = minimumStepDelayMicros;
      }
      lastTime = now;
        if (lastPulse == HIGH) {
          lastPulse = LOW;
        }
        else {
          lastPulse = HIGH;
        }
      if (lastMotorDirection == LOW) {
        motorPosition += 1;
      }
      else if (lastMotorDirection == HIGH) {
        motorPosition -= 1;
      }
      digitalWrite(stepPin, lastPulse);
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(enableButtonPin, INPUT_PULLUP);
  pinMode(directionButtonPin, INPUT_PULLUP);
  
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(enablePin, lastMotorEnable);
  digitalWrite(directionPin, lastMotorDirection);
  digitalWrite(stepPin, LOW);
}

void loop() {
  getInput();
  sensorValue = analogRead(A0);
  now = millis();
  if (now - lastLog >= logFrequency) {
    lastLog = now;
    char buffer[20];
    sprintf(buffer, "%d,%d", motorPosition, sensorValue);
    Serial.println(buffer);
  }
  updateStepper();
}
