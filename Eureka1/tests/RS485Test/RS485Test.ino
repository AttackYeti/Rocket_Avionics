/*-----( Import needed libraries )-----*/
#include <SoftwareSerial.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define SSerialRX        5  //Serial Receive pin
#define SSerialTX        6  //Serial Transmit pin

#define SSerialTxControl 12   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

#define signalPin         A1

/*-----( Declare objects )-----*/
SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX

/*-----( Declare Variables )-----*/
unsigned int byteReceived;
unsigned int byteSend;

void setup()  {
  pinMode(SSerialTxControl, OUTPUT);
  pinMode(signalPin, INPUT);

  // Start the software serial port, to another device
  digitalWrite(SSerialTxControl, RS485Receive);     // Init Transceiver
  //digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit  
  RS485Serial.begin(4800);                          // set the data rate
  Serial.begin(9600);
}

void loop() {

  byteSend = analogRead(signalPin);

  // Look for a message from the brain
  if (RS485Serial.available())  {
    byteReceived = RS485Serial.read();  
  }

  // Send the transducer value if the brain sends the write command
  if (byteReceived == 'P') {
    //byteReceived = 0;
    //for (int i = 0; i < 600; i++){
    //  byteSend += analogRead(signalPin);
    //}
    //byteSend /= 600;
    digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit
    RS485Serial.write(byteSend/4); // Send the data
    digitalWrite(SSerialTxControl, RS485Receive);   // Disable RS485 Transmit
    Serial.print("sending value: ");
    Serial.println(byteSend);
    byteSend = 0;
  }
  delay(1);
}
