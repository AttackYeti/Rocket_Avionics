#include <SoftwareSerial.h>

// This ID Code should be set to a value specific to each module
#define ID_CODE 1

// Define pin parameters for each module
#define SSRX 5
#define SSTX 6
#define SSDE 12

#define MOTOR_ENABLE_A      1
#define MOTOR_STEP_A        2
#define MOTOR_DIRECTION_A   3

#define MOTOR_ENABLE_B      4
#define MOTOR_STEP_B        5
#define MOTOR_DIRECTION_B   6


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
struct stepperMotor {
    int minimumStepPeriod;
    int desiredPosition;
    int position;
    int direction;
    int enable;
    int pulse;
    int lastTime;
    int enablePin;
    int directionPin;
    int stepPin;
};

void initializeStepperMotor(stepperMotor &Motor, int enablePin, int directionPin, int stepPin);
void setMotorEnable(stepperMotor &Motor, int status);
void setDirectionMotor(stepperMotor &Motor, int direction);
void setDesiredPositionMotor(stepperMotor &Motor, int goal);

void stepMotor(stepperMotor &Motor, int stepPeriod);
void recieve_message(Stream &port);
void send_message(char message, Stream &port);

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
stepperMotor MotorA;
stepperMotor MotorB;

// STATE VARIABLES

void setup(){
    // initialize the serial ports
    rs485serial.begin(4800);
    pinMode(SSDE, OUTPUT);
    digitalWrite(SSDE, LOW);
    Serial.begin(9600);

    Serial.println("Serial connections initialized.");

    // initialize the motor abstractions
    initializeStepperMotor(MotorA, MOTOR_ENABLE_A, MOTOR_DIRECTION_A, MOTOR_STEP_A);
    initializeStepperMotor(MotorB, MOTOR_ENABLE_B, MOTOR_DIRECTION_B, MOTOR_STEP_B);

    Serial.println("Motors initialized...");

    // User friendly status message for debugging
    Serial.println("setup done");
}

void loop() {
    recieve_message(rs485serial, SSDE);
    recieve_message(Serial, 255);

    controlMotor(MotorA);
    controlMotor(MotorB);
}

void controlMotor(stepperMotor &Motor) {
    if (Motor.position != Motor.desiredPosition) {

        if (Motor.position > Motor.desiredPosition) {
            setDirectionMotor(Motor, HIGH);
        }
        if (Motor.position < Motor.desiredPosition) {
            setDirectionMotor(Motor, LOW);
        }
        if (Motor.enable != HIGH) {
            setMotorEnable(Motor, HIGH);
        }

        stepMotor(Motor, Motor.minimumStepPeriod);
    }
    else {
        if (Motor.enable) {
            setMotorEnable(Motor, LOW);
        }
    }
}

void initializeStepperMotor(stepperMotor &Motor, int enablePin, int directionPin, int stepPin) {
    // This function should be used to initialize a stepperMotor after it has been defined.
    Motor.desiredPosition = 0;
    Motor.minimumStepPeriod = 2*400;
    Motor.position = 0;
    Motor.direction = LOW;
    Motor.enable = LOW;
    Motor.pulse = LOW;
    Motor.lastTime = 0;
    Motor.enablePin = enablePin;
    Motor.directionPin = directionPin;
    Motor.stepPin = stepPin;

    pinMode(Motor.enablePin, OUTPUT);
    pinMode(Motor.directionPin, OUTPUT);
    pinMode(Motor.stepPin, OUTPUT);
    setMotorEnable(Motor, Motor.enable);
    setDirectionMotor(Motor, Motor.direction);

};

void setMotorEnable(stepperMotor &Motor, int status) {
    // A status of HIGH should enable the driver and allow the motor to spin.
    // A status of LOW should disable the driver and not allow motor to spin.
    if ((status != HIGH) && (status != LOW)) {
        status = LOW;
    }

    if (Motor.enable != status) {
        Motor.enable = status;

        if (Motor.enable == HIGH) {
            digitalWrite(Motor.enablePin, LOW);
        }

        else {
            digitalWrite(Motor.enablePin, HIGH);
        }
    }
}

void setDirectionMotor(stepperMotor &Motor, int direction) {
    // This function should take in an initialized motor structure and set the direction.
    // This function takes in either HIGH or LOW, all other values are interpretted as LOW.
    if ((direction != HIGH) && (direction != LOW)) {
        direction = LOW;
    }

    if (Motor.direction != direction) {
        Motor.direction = direction;
        digitalWrite(Motor.directionPin, Motor.direction);
    }
}

void setDesiredPositionMotor(stepperMotor &Motor, int goal) {
    Motor.desiredPosition = goal;
}

void stepMotor(stepperMotor &Motor, int stepPeriod) {
    // This function takes in an initialized motor structure and steps it. The step period should be the length
    // of time for both an on and off pulse. Motor.MinimumStepPeriod can be used for maximum speed.
    int thisTime = micros();

    if (Motor.enable) {
        if (abs(thisTime-Motor.lastTime) >= stepPeriod/2) {
            Motor.lastTime = thisTime;
            switch (Motor.pulse) {
                case HIGH:
                    Motor.pulse = LOW;
                    break;

                case LOW:
                    Motor.pulse = HIGH;
                    break;

                default:
                    Motor.pulse = LOW;
                    break;
            }

            digitalWrite(Motor.stepPin, Motor.pulse);
        }
    }
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


// Enable MotorA and start spinning it
void command1(int value) {
    setDesiredPositionMotor(MotorA, value);
}
// Enable MotorB and start spinning it
void command2(int value) {
    setDesiredPositionMotor(MotorA, value);
}
// Enable LED on pin 13 for debugging
void command3(int value) {
    digitalWrite(LED_BUILTIN, HIGH);
}
// Print Hello World to the Serial
void command4(int value) {
    Serial.println("Hello World.");
}
// Print Hello World to the RS485 Interface
void command5(int value) {
    rs485serial.println("Hello RS485 World.");
}
void command6(int value) {

}
void command7(int value) {

}
void command8(int value) {

}
void command9(int value) {

}
void command10(int value) {

}
