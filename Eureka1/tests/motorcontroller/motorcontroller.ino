#include <SoftwareSerial.h>

#define SerialRX 
#define SerialTX
#define SerialControl
#define serialTransmit HIGH
#define serialTransmit LOW

#lox
#define motorAstep
#define motorAdir
#define motorAEnable

#propane
#define motorBstep
#define motorBdir
#define motorBEnable

## ~~~~~~~ COMMUNICATION CODES ~~~~~~~~~~~ ##
#define PRESSURE 1 # this one
#define TANKS 2
#define MOTORA 3
#define MOTORB 4

SoftwareSerial rs485(SerialRX, SerialTX);

void setup() {
  pinMode(serialControl, OUTPUT);

  pinMode(motorAstep, OUTPUT);
  pinMode(motorAdir, OUTPUT);
  pinMode(motorAEnable, OUTPUT);

  pinMode(motorBstep, OUTPUT);
  pinMode(motorBdir, OUTPUT);
  pinMode(motorBEnable, OUTPUT);

  digitalWrite(serialControl, 
  
}

void loop() {
  // put your main code here, to run repeatedly:

  int message = digitalRead(serialinputPin);
  if(message == PRESSURE){
    message = digitalRead(serialinputPin);
    
  }
}
