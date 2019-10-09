/*
 AnalogReadSerial
 Reads an analog input on pin 0, prints the result to the Serial Monitor.
 Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
 Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 This example code is in the public domain.
 http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/
#include <SoftwareSerial.h>

#define RX 5
#define TX 6

SoftwareSerial rs485serial(RX,TX); //rx, tx

# define DE 12

// the setup routine runs once when you press reset:
void setup() {
 // initialize serial communication at 9600 bits per second:
 rs485serial.begin(4800);
 Serial.begin(9600);
 pinMode(DE, OUTPUT);
 digitalWrite(DE, LOW);
 Serial.println("setup done");
}
int msg;
// the loop routine runs over and over again forever:
void loop() {
 if(rs485serial.available())
 {
   Serial.println("received");
   msg = rs485serial.read();
   Serial.println(msg, DEC);
 }
 // print out the value you read:
 delay(500);        // delay in between reads for stability
}
