
#define PRESSURE_INPUT A1

#define dirPin 2
#define stepPin 3

#define OPEN_DIR HIGH
#define CLOSE_DIR LOW

#define MIN_STEP_DELAY 400

// int to psi:
// 258 - 500
// 330 - 1000
// 410 - 1500 ~ little bit off

int motor_pulse_state = LOW;

int motor_pos = 0;
int pressure = 0;

int dir;

unsigned long lastTime = 0;
//300 ~ 900PSI
//350 ~ 1100PSI

void setup() {
  // put your setup code here, to run once:
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(PRESSURE_INPUT, INPUT);
  digitalWrite(dirPin, OPEN_DIR);
  Serial.begin(9600);
  lastTime = micros();
  dir = CLOSE_DIR;
}

// 0.88V - 4.4V : ?? - 5000 PSI

void loop() {
  // put your main code here, to run repeatedly:
  pressure = analogRead(PRESSURE_INPUT);

  // change delay
  //unsigned long stepDelay = 400;
  byte incomingByte = Serial.read(); 
  if(incomingByte == 'w'){
    for (int i = 0; i < 20000; i++){
    change_state(MIN_STEP_DELAY);
    }
  } else if (incomingByte == 'r'){
    dir = (dir == CLOSE_DIR)? OPEN_DIR : CLOSE_DIR;
    digitalWrite(dirPin, dir);
  }
  print_state();
}

void change_state(long delayTime){
  unsigned long now = micros();
  if (abs(now - lastTime) > delayTime){
    update_motor();
    lastTime = micros();
  }
}

void update_motor(){
  if (motor_pulse_state == LOW){
    motor_pulse_state = HIGH;
  } else if(motor_pulse_state == HIGH){
    motor_pulse_state = LOW;
  }
  digitalWrite(stepPin, motor_pulse_state);  
}

void print_state(){
  Serial.print(micros());
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.println(motor_pos);
}
