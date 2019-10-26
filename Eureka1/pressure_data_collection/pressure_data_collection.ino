
#define PRESSURE_INPUT A1

#define dirPin 2
#define stepPin 3

#define OPEN_DIR HIGH
#define CLOSE_DIR LOW

#define STEP_DELAY 400

int motor_pulse_state = LOW;

int motor_pos = 0;
int pressure = 0;

unsigned long lastTime = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(PRESSURE_INPUT, INPUT);

  Serial.begin(9600);
  lastTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  pressure = analogRead(PRESSURE_INPUT);

  // change delay
  unsigned long stepDelay = 400;
  change_state(stepDelay);
  print_state()
}

void change_state(long delayTime){
  unsigned long now = micros();
  if (abs(now - lastTime) > delayTime){
    update_motor();
    lastTime = micros();
  }
}

void update_motor(){
  motor_pulse_state = (motor_pulse_state == LOW)? HIGH : LOW;
  digitalWrite(stepPin, motor_pulse_state);
}

void print_state(){
  Serial.print(micros());
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.println(motor_pos);
}
