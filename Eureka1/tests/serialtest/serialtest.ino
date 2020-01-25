/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/
#include <SoftwareSerial.h>

SoftwareSerial rs485serial(3,4); //rx, tx

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  rs485serial.begin(4800);
  Serial.begin(9600);
}

char msg;
// the loop routine runs over and over again forever:
void loop() {
  if(rs485serial.available())
  {
    Serial.println("received");
    msg = rs485serial.read();
  }
  // print out the value you read:
  Serial.println(msg, DEC);
  delay(10);        // delay in between reads for stability
}
