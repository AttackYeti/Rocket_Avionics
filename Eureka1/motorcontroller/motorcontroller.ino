// This code will be deployed on an Arduino Nano that communicates with the Rocket brain over an RS485
// signal and controls two stepper motors. This same code is used for the pressurant release, and for
// lox, propane release to the engine with the sole difference being whether it responds to the key 
// "PRESSURANT" or "TANKS".


#include <SoftwareSerial.h>

#define serialRX 1
#define serialTX 2
#define serialControl 3
#define serialTransmit HIGH
#define serialReceive LOW

//lox
#define motorAstep 4
#define motorAdir 5
#define motorAEnable 6

//propane
#define motorBstep 7
#define motorBdir 8
#define motorBEnable 9

#define stepAngle 0.036; //1.8 / 50
#define STEP_DELAY 400

// ~~~~~~~ COMMUNICATION CODES ~~~~~~~~~~~ ##
#define PRESSURANT 1 // this one
#define TANKS 2
#define MOTORA 3
#define MOTORB 4

float position_a = 0.0;
int A_state = LOW;
float position_b = 0.0;
int B_state = LOW;

int pos_a_desired = 0;
int pos_b_desired = 0;

unsigned long lastTime = 0;

SoftwareSerial rs485(serialRX, serialTX);

void setup() {
  pinMode(serialControl, OUTPUT);

  pinMode(motorAstep, OUTPUT);
  pinMode(motorAdir, OUTPUT);
  pinMode(motorAEnable, OUTPUT);

  pinMode(motorBstep, OUTPUT);
  pinMode(motorBdir, OUTPUT);
  pinMode(motorBEnable, OUTPUT);

  digitalWrite(serialControl, serialReceive); 
  rs485.begin(9600);

  Serial.begin(9600);
  lastTime = micros();
}

unsigned long t = 0;

void loop() {
  read_serial();
  change_state();
}

void change_state(){
  unsigned long now = micros();
  if (abs(now - lastTime) > STEP_DELAY){
    update_motor('A');
    update_motor('B');
  }
  lastTime = micros();
}

void read_serial(){
    if(rs485.available()){
    int message = rs485.read();
    if(message == PRESSURANT){ //Change this line for other motor control
      message = rs485.read();
      
      if(message == MOTORA){
        message = rs485.read();
        transmit_received(true);
        pos_a_desired = message;      
      } else if (message == MOTORB){
        message = rs485.read();
        transmit_received(true);
        pos_b_desired = message;
      }
    }
  }
}

void transmit_received(bool success){
  digitalWrite(serialControl, serialTransmit);
  rs485.write(success);
  digitalWrite(serialControl, serialReceive);
}

void update_motor(char motor){
  int dirPin = (motor == 'A')? motorAdir : motorBdir;
  int stepPin = (motor == 'A')? motorAstep : motorBstep;
  int pos = (motor == 'A')? pos_a_desired : pos_b_desired;
  float curr_position = (motor == 'A')? position_a : position_b;
  int dir = (pos - curr_position)? LOW : HIGH;  //===== DEPENDS ON HOW WE WIRE IT ======== 
  digitalWrite(dirPin, dir);
  if(motor == 'A'){
    digitalWrite(stepPin, !A_state);
    A_state = !A_state;
  } else if (motor == 'B'){
    digitalWrite(stepPin, !B_state);
    B_state = !B_state;
  }
}



void enable_motor(char motor) {
  if (motor == 'A'){
    digitalWrite(motorAEnable, LOW);
  } else if (motor == 'B'){
    digitalWrite(motorBEnable, LOW);
  }
}

void disable_motor(char motor) {
  if (motor == 'A'){
    digitalWrite(motorAEnable, HIGH);
  } else if (motor == 'B'){
    digitalWrite(motorBEnable, HIGH);
  }
}


void stop_motor(char motor){
  int stepPin = (motor == 'A')? motorAstep : motorBstep;
  digitalWrite(stepPin, LOW);
}
