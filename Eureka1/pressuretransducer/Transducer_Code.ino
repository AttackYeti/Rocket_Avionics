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

#define sensorPin A1

// ~~~~~~~ COMMUNICATION CODES ~~~~~~~~~~~ ##
#define READ_SENSOR 6 //this number can be changed


int rawPressure = 0; //will return 10 bit number from 0-1023

SoftwareSerial rs485(serialRX, serialTX);

void setup() {
  pinMode(serialControl, OUTPUT);
  pinMode(sensorPin, INPUT);

  digitalWrite(serialControl, serialReceive); 
  rs485.begin(9600);

  Serial.begin(9600);
}

unsigned long t = 0;

void loop() {
  read_sensor();
  read_serial();
}


void read_sensor(){
  rawPressure = analogRead(sensorPin);
}

void respond(){
  digitalWrite(serialControl, serialTransmit); 
  rs485.write(rawPressure/4); //scaled data by 4 to convert to 8 bit. must be rescaled once received
  digitalWrite(serialControl, serialReceive); 
  Serial.print("sending value: ");
  Serial.println(rawPressure);
}

void read_serial(){

    if(rs485.available()){
      int message = rs485.read();
      if(message == READ_SENSOR){
        transmit_received(true);
        respond();
    }
  }
}

void transmit_received(bool success){
  digitalWrite(serialControl, serialTransmit);
  rs485.write(success);
  digitalWrite(serialControl, serialReceive);
}
