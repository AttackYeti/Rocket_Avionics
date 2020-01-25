#include<SoftwareSerial.h>

# define RX 5
# define TX 6

SoftwareSerial serial(RX, TX);

void setup(){
  serial.begin(4800);
  Serial.begin(9600);
}

int val = 5;
void loop(){
  serial.write(val);
  Serial.println(val);
  delay(500);
  serial.write(2*val);
  Serial.println(2*val);
  delay(500);
  serial.write(3*val);
  Serial.println(3*val);
  delay(500);
}
