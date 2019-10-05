#include<SoftwareSerial.h>

# define TX 1
# define RX 2

SoftwareSerial Serial(RX, TX);

void setup(){
  Serial.begin(9600);
}

void loop(){
  Serial.print(78);
  delay(1000);
}
