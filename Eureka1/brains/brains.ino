
#include <SoftwareSerial.h>

#define serialRX 1
#define serialTX 2
#define serialControl 3
#define serialTransmit HIGH
#define serialReceive LOW

// ~~~~~~~~~~~~~~~ COMMUNICATION CODES ~~~~~~~~~~~~~~~~~~
#define PRESSURANT_MOTOR 1 // for pressurant motor controller 
#define TANKS_MOTOR 2 // for tanks motor controller
#define LOX_MOTOR 3
#define PROP_MOTOR 4

#define N_LOX_TRANSDUCER 11 //high
#define N_PROP_TRANSDUCER 12 //high
#define LOX_TRANSDUCER 13 //low
#define PROP_TRANSDUCER 14 //low

int motor_lox_pressurant_pos = 0;
int motor_prop_pressurant_pos = 0;

int n_lox_pressure = 0;
int n_prop_pressure = 0;

int lox_pressure = 0;
int prop_pressure = 0;

SoftwareSerial serial(serialRX, serialTX);

void setup() {
  // put your setup code here, to run once:
  pinMode(serialControl, OUTPUT);
  digitalWrite(serialControl, serialReceive);

  serial.begin(9600);
  Serial.begin(9600);
  
  Serial.println("time, pressure, pos");
  print_state();
}

void loop() {
  // put your main code here, to run repeatedly:

  // read pressure data
  read_all_pressure_values();
  
  // calculate desired flow rate
  
  // calculate appropriate control for desired flow rate

  // apply control
  //~~~~~~~~~CHANGE THIS FOR DATA COLLECTION - NEED DIFFERENT RATES, EVEN CONSTANT POS ~~~~~~~~~~~~~~~`
  motor_lox_pressurant_pos = adjust_motor(PRESSURANT_MOTOR, LOX_MOTOR, 1)

  // transmit and store state
  print_state();
  // @SQ networking stuff
  // special board for sd storage


}

void print_state(){
  Serial.print(micros());
  Serial.print(",");
  Serial.print(n_lox_pressure);
  Serial.print(",");
  Serial.println(motor_lox_pressurant_pos);
}

int desired_choke_area_given_pressure_differential(int curr_p_tank, int curr_p_n2){
  
}

int adjust_motor(int board_id, int motor_id, int amount){
  send_message([board_id, amount, motor_id]);
  return serial.read();
}

void send_message(int[] data){
  digitalWrite(serialControl, serialTransmit);
  serial.write(data[0]);
  serial.write(data[1]);
  serial.write(data[2]);
  digitalWrite(serialControl, serialReceive);
}


void read_all_pressure_values(){
  n_lox_pressure = read_pressure(N_LOX_TRANSDUCER);
  n_lox_pressure = high_pressure_sensor_conversion(n_lox_pressure);
  
  //n_prop_pressure = read_pressure(N_PROP_TRANSDUCER);
  //n_prop_pressure = high_pressure_sensor_conversion(n_prop_pressure);
  
  //lox_pressure = read_pressure(LOX_TRANSDUCER);
  //lox_pressure = low_pressure_sensor_conversion(lox_pressure);
  
  //prop_pressure = read_pressure(PROP_TRANSDUCER);
  //prop_pressure = low_pressure_sensor_conversion(prop_pressure);
}

int _read_pressure(int sensor_num){ // modify this using general communications method.
  digitalWrite(serialControl, serialTransmit);
  serial.write(sensor_num);
  digitalWrite(serialControl, serialReceive);
  int pressure_val = serial.read();
  return pressure_val;
}

float low_pressure_sensor_conversion(int raw){
  // fill this out based on characterization of transducer
  pressure = raw;
  return pressure;
}

float high_pressure_sensor_conversion(int raw){
  // fill this out based on characterization of transducer
  pressure = raw;
  return pressure;
}

#define desired_exit_mass_flow_rate_pressurant = 0.02938; //kg / s
#define BETA = 5.78396 * pow(10, -6);

float curr_mass_flow_rate(float curr_pressure, int tank_id){
  if (tank_id == LOX){
    int motor_position = lox_pressurant_motor_position;
  } else if (tank_id == PROPELLANT){
    int motor_position = prop_pressurant_motor_position;
  }
  float CdA = cda(curr_pressure, motor_position);
  return CdA * curr_pressure * sqrt(BETA);
}

float cda(float pressure, float motor_position){
  return 0;
}
