/*
Developer Notes
~~~~~
-The SD library uses an obnoxious amount of memory.
-IMU data will be fairly complicated and may take the longest time depending on method used
-Should propably use established libraries for IMU
-Need Bluetooth Implemented
-Need GPS implemented
-Would be cool to have pwm to play sounds via the buzzer
*/
#include <Adafruit_ATParser.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>

#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>

// Set the pins
#define gpsInterruptPin 3
#define BLETx 5
#define BLERx 6
#define GPSRx 7
#define GPSTx 8
#define SDCS 10
#define SDMOSI 11
#define SDMISO 12
#define SDCLK 13
#define parachuteDeployPin A1
#define sirenPin A2

#define filename "LOGDATA.txt"

int parachuteDeployed = false;
bool FORCE_PARA_DEPLOY = false;

File logfile;
SoftwareSerial gpsSerial(GPSTx, GPSRx);
SoftwareSerial bleSerial(BLETx, BLERx);
Adafruit_GPS GPS(&gpsSerial);
Adafruit_BMP085 bmp;

#define LOG_FIXONLY false  //set to true to only log to SD when GPS has a fix, for debugging, keep it false

void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  bleSerial.begin(9600);
  bmp.begin();

  sdConfigure();
  gpsConfigure();
}

void loop() {
  parachuteDeployed = manageParachuteDeploy(parachuteDeployed);
  Recovery(parachuteDeployed);
  
  char gpsData = getGPSdata();
  int imuData = getIMUdata();
  int temperatureData = bmp.readTemperature();
  int altitudeData = bmp.readAltitude();

  logData(gpsData, altitudeData, temperatureData, imuData);
  }

// ================================================================
// ===                   SUPPORT FUNCTIONS                      ===
// ================================================================

void logData(char gpsData, int altitudeData, int temperatureData, int imuData) {
  /* 
  * Need to format all arguments from int -> String and then format into a standard csv format
  * Then write the standardized string to the file all at once. logString should hold all of the converted
  * data in the specified format. The procedure for data being formatted should be fairly easy to modify if more info
  * is needed.
  */
  logfile = SD.open(filename, FILE_WRITE);
  
  String logString = String(altitudeData) + ", " + char(imuData) + ", " + char(gpsData) + ", " + char(temperatureData);

  logfile.println(logString);
  logfile.close();
}

void gpsConfigure() {
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);   // Turn off updates on antenna status, if the firmware permits it
}

char getGPSdata() {
  gpsSerial.listen();
  GPS.read();
  
  if (GPS.newNMEAreceived()) {    // If a new sentence is recieved  
    char *stringptr = GPS.lastNMEA();  
  
    if (!GPS.parse(stringptr)){   // this sets the newNMEAreceived() flag to false
      return char("Parse Fail");  // we can fail to parse a sentence in which case we should just wait for another
    }
  
    if (LOG_FIXONLY && !GPS.fix) {
      Serial.print("No Fix");
      return char("No Fix");
    }
  
    else {
      return stringptr;
    }
  }
}

int getIMUdata() {  // Incomplete

}

bool manageParachuteDeploy(bool parachuteDeployed) {
   /* 
   * This function needs to check relevant data as well as wireless coms in order to see if parachute deployment should be activated, and if so then activate it.
   * You will most likely need to change your input variables in order to get useful info. You might try checking for patterns in the altitude or accelerations 
   * using the IMU. ie freefall or something like that. It could be good to have multiple checks that would be activated just for redundancy. In order to actually 
   * deploy the parachute you should use the command below... as you can see, it returns 1. The format that I envision being used would be for the datalogger to log
   * the state of the parachute as 0 until the parachute
   *    
   *    digitalWrite(ParachuteDeployPin, HIGH);
   *    delay(100);
   *    return 1
   */
  if (FORCE_PARA_DEPLOY) {
    digitalWrite(parachuteDeployPin, HIGH);
    return true;
  }
  
  if (parachuteDeployed) {
    return true;
  }
  
  // if ( !parachuteDeployed && (conditional1 || conditional2 || conditional3 || etc) ) {
  //   digitalWrite(ParachuteDeployPin, HIGH);
  //   return true
  // }
  
  else {
    return false;
  }
}

void Recovery(bool parachuteDeployed) {
  // This function handles the recovery buzzer as well as broadcasting recovery data and any other recovery procedures.
  
  if (parachuteDeployed) {
    digitalWrite(sirenPin, HIGH);
    // Add recovery code here
  }
}

void bleConfigure() {
  
}

char getBLEdata() {
  bleSerial.listen();
  int i = 1;
  char output[32] = {0};
  while (bleSerial.available() > 0 && i <= 32) {
    char inByte = bleSerial.read();
    output[i] = inByte;
    i += 1;
  }
  return output;
}

void sdConfigure() {
  Serial.print("Initializing SD card...");

  if (!SD.begin(SDCS)) {
    Serial.println("initialization failed!");
  }
  else {
    Serial.println("initialization done.");
  }
}
