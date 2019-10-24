
#include <SoftwareSerial.h>

#define serialRX 1
#define serialTX 2
#define serialControl 3
#define serialTransmit HIGH
#define serialReceive LOW

// ~~~~~~~~~~~~~~~ COMMUNICATION CODES ~~~~~~~~~~~~~~~~~~
#define PRESSURANT 1 // for pressurant motor controller 
#define TANKS 2 // for tanks motor controller
#define MOTORA 3
#define MOTORB 4

#define N_LOX_TRANSDUCER 11 //high
#define N_PROP_TRANSDUCER 12 //high
#define LOX_TRANSDUCER 13 //low
#define PROP_TRANSDUCER 14 //low

int n_lox_pressure = 0;
int n_prop_pressure = 0;

int lox_pressure = 0;
int prop_pressure = 0;

SoftwareSerial serial(serialRX, serialTX);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  // read pressure data
  read_all_pressure_values();
  
  // calculate desired flow rate
  
  // calculate appropriate control for desired flow rate

  // apply control

  // transmit and store state
  // @SQ networking stuff
  // special board for sd storage

}

void read_all_pressure_values(){
  n_lox_pressure = read_pressure(N_LOX_TRANSDUCER);
  n_lox_pressure = high_pressure_sensor_conversion(n_lox_pressure);
  
  n_prop_pressure = read_pressure(N_PROP_TRANSDUCER);
  n_prop_pressure = high_pressure_sensor_conversion(n_prop_pressure);
  
  lox_pressure = read_pressure(LOX_TRANSDUCER);
  lox_pressure = low_pressure_sensor_conversion(lox_pressure);
  
  prop_pressure = read_pressure(PROP_TRANSDUCER);
  prop_pressure = low_pressure_sensor_conversion(prop_pressure);
}

int _read_pressure(int sensor_num){
  digitalWrite(serialControl, serialTransmit);
  serial.write(sensor_num);
  digitalWrite(serialControl, serialReceive);
  int pressure_val = serial.read();
  return pressure_val;
}

float low_pressure_sensor_conversion(int raw){
  // fill this out based on characterization of transducer
  pressure = ;
  return pressure;
}

float high_pressure_sensor_conversion(int raw){
  // fill this out based on characterization of transducer
  pressure = ;
  return pressure;
}
