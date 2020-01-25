#include<SoftwareSerial.h>

# define TX 1
# define RX 2

SoftwareSerial Serial(RX, TX);

void setup(){
  Serial.begin(9600);
}

int val = 5;
void loop(){
  Serial.print(val);
  delay(1000);
}
