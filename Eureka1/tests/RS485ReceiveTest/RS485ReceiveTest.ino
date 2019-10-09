/*-----( Import needed libraries )-----*/
#include <SoftwareSerial.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define SSerialRX        3  //Serial Receive pin
#define SSerialTX        4  //Serial Transmit pin
#define SSerialTxControl 12   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

/*-----( Declare objects )-----*/
SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
/*-----( Declare Variables )-----*/
unsigned int byte1Received;
unsigned int byte2Received;
unsigned int byteSend;
int pressure;
void setup(){
  pinMode(SSerialTxControl, OUTPUT);
  // Start the software serial port, to another device
  //digitalWrite(SSerialTxControl, RS485Receive);     // Init Transceiver
  digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit  
  RS485Serial.begin(4800);                          // set the data rate 
  Serial.begin(9600);
}
void loop() {
  digitalWrite(SSerialTxControl, RS485Transmit);
  RS485Serial.write('P');
  digitalWrite(SSerialTxControl, RS485Receive);
  delay(100);
  // Read the analog voltage from the transducer
  // byteSend = analogRead(signalPin);
  // Look for a message from the brain
  if (RS485Serial.available())  {
     byte1Received = RS485Serial.read();                   // Read the byte   
     //pressure = (byteReceived - 102)*1000/(921-102);
     //Serial.println("byteReceived:");
     Serial.println(4*byte1Received); 
     //Serial.println("pressure:");
     //Serial.println(pressure);
     //Serial.println("---------");
     }
  // Send the transducer value if the brain sends the write command
  //if (byteReceived == 'P') {
    //digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit    
    // RS485Serial.write(byteSend);                    // Send the byte back
    //digitalWrite(SSerialTxControl, RS485Receive);   // Disable RS485 Transmit   

  digitalWrite(SSerialTxControl, RS485Transmit);
  RS485Serial.write('A');
  digitalWrite(SSerialTxControl, RS485Receive);
  delay(1);
  //}
}
