#include <SoftwareSerial.h>

// This ID Code should be set to a value specific to each module
#define ID_CODE 1

// Define pin parameters for each module
#define SSRX 5
#define SSTX 6
#define SSDE 12

#define SENSOR_INPUT_AMP_1 A7 //amplified
#define SENSOR_INPUT_AMP_2 A6 //amplified
#define SENSOR_INPUT_1 A3 //not amplified
#define SENSOR_INPUT_2 A2 //not amplified

#define HI_PRESSURE_SENSOR 1
#define LO_PRESSURE_SENSOR 2

// Define MACROs to be used throughout the codes
// Message structure = ID_CODE/VALUE/CONNECTION
#define RECEIVE 1
#define TRANSMIT 0
#define CONFIRM 200
#define FAULT_CODE 111
//
#define COMMAND_1 1
#define COMMAND_2 2
#define COMMAND_3 3
#define COMMAND_4 4
#define COMMAND_5 5
#define COMMAND_6 6
#define COMMAND_7 7
#define COMMAND_8 8
#define COMMAND_9 9
#define COMMAND_10 10

// Forward declaration of support functions and structures
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
typedef struct {
    int value;
    int input_pin;
    int sensor_type;
    int buffer[5];
} Sensor;

void recieve_message(Stream &port);
void send_message(char message, Stream &port);
void initializeSensor(Sensor &Sensor, input_pin, sensor_type)

void command1(int value);
void command2(int value);
void command3(int value);
void command4(int value);
void command5(int value);
void command6(int value);
void command7(int value);
void command8(int value);
void command9(int value);
void command10(int value);

// Define abstractions
SoftwareSerial rs485serial(SSRX, SSTX); //rx, tx
Sensor Sensor1;
Sensor Sensor2;
Sensor Sensor3;
Sensor Sensor4;

// STATE VARIABLES

void setup(){
    // initialize the serial ports
    rs485serial.begin(4800);
    pinMode(SSDE, OUTPUT);
    digitalWrite(SSDE, LOW);
    Serial.begin(9600);

    Serial.println("Serial connections initialized.");

    // initialize the motor abstractions
    initializeSensor(Sensor1, SENSOR_INPUT_AMP_1, HI_PRESSURE_SENSOR); //edit for different sensors
    initializeSensor(Sensor2, SENSOR_INPUT_AMP_2, HI_PRESSURE_SENSOR); //edit for different sensors
    initializeSensor(Sensor3, SENSOR_INPUT_1, LO_PRESSURE_SENSOR); //edit for different sensors
    initializeSensor(Sensor4, SENSOR_INPUT_2, LO_PRESSURE_SENSOR); //edit for different sensors

    Serial.println("Sensor(s) initialized...");

    // User friendly status message for debugging
    Serial.println("setup done");
}

void loop() {

    updateSensor(Sensor1);
    updateSensor(Sensor2);
    updateSensor(Sensor3);
    updateSensor(Sensor4);
    
    recieve_message(rs485serial, SSDE);
    recieve_message(Serial, 255);
}

void recieve_message(Stream &port, int controlPin){
    if (port.available()) {
        int identifier = port.read();
        //Check if message is intended for motor controller
        if (identifier == ID_CODE) {
            int value = port.read();
            int connection = port.read();

            switch(connection) {

                case COMMAND_1:
                    command1(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_2:
                    command2(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_3:
                    command3(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_4:
                    command4(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_5:
                    command5(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_6:
                    command6(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_7:
                    command7(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_8:
                    command8(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_9:
                    command9(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                case COMMAND_10:
                    command10(value);
                    send_message(port, CONFIRM, controlPin);
                    break;

                default:
                    send_message(port, FAULT_CODE, controlPin);
                    break;
            }
        }
    }
}

void send_message(Stream &port, char message, int controlPin){

    // Allow for flow control to be bypassed.
    if (controlPin == 255) {
        port.write(message);
    }
    // Implement flow control for serial connections which require it.
    else {
        digitalWrite(controlPin, TRANSMIT);
        port.write(message);
        digitalWrite(controlPin, RECEIVE);
    }
}

void updateSensor(Sensor &Sensor) {

  int sum = 0;
  for(int i = sizeof(Sensor.buffer) - 1; i > 0 ; i--){
    Sensor.buffer[i] = Sensor.buffer[i-1];
    sum += Sensor.buffer[i];
  }

    int current_reading = adjustValue(Sensor, analogRead(Sensor.input_pin));
  Sensor.buffer[0] = current_reading;
  sum += Sensor.buffer[0];

    Sensor.value = int(sum / sizeof(Sensor.buffer));

}

void initializeSensor(Sensor &Sensor, input_pin, sensor_type){

  Sensor.value = 0;
  Sensor.input_pin = input_pin;
  Sensor.sensor_type = sensor_type;
  for(int i = 0; i < sizeof(Sensor.buffer); i++){
    Sensor.buffer[i] = 0;
  }
  pinMode(input_pin, INPUT);
}

int adjustValue(Sensor &Sensor, int raw_value) {

  int rval = 0;

  if(Sensor.sensor_type == HI_PRESSURE_SENSOR){
    rval = raw_value; // EDIT THIS
  } 
  else if(Sensor.sensor_type == LO_PRESSURE_SENSOR){
    rval = raw_value; // EDIT THIS
  }
  else{
    Serial.println("Sensor Type not recongized")
  }
  return rval;
}

// ~~~~ Primary commands ~~~~
// Return reading from sensor input 1
void command1(int value) {
    char message = itoa(Sensor1.value);
    send_message(rs485serial, message, SSDE);
}
// Return reading from sensor input 2
void command2(int value) {
    char message = itoa(Sensor2.value);
    send_message(rs485serial, message, SSDE);
}
// Return reading from sensor input 3
void command3(int value) {
    char message = itoa(Sensor3.value);
    send_message(rs485serial, message, SSDE);
}
// Return reading from sensor input 4
void command4(int value) {
    char message = itoa(Sensor4.value);
    send_message(rs485serial, message, SSDE);
}
void command5(int value) {

}
void command6(int value) {

}
void command7(int value) {

}

// ~~~~ Debugging Commands ~~~~
// Enable LED on pin 13 for debugging
void command8(int value) {
    digitalWrite(LED_BUILTIN, HIGH);
}
// Return debugging message on hardware serial
void command9(int value) {
    char message = "Hello World.";
    send_message(Serial, message, 255);
}
// Return debugging message on RS485 Software serial
void command10(int value) {
    char message = "Hello RS485 World.";
    send_message(rs485serial, message, SSDE);
}
