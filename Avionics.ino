/*
Developer Notes
~~~~~
-The SD library uses an obnoxious amount of memory.
-IMU data will be fairly complicated and may take the longest time depending on method used
-
*/

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>

// Set the pins
#define chipSelect 10
#define ledPin 13
#define sirenPin 2
#define filename "LOGDATA.txt"

int parachuteDeployed = 0;
bool FORCE_PARA_DEPLOY = false;

File logfile;
SoftwareSerial gpsSerial(8, 7);
Adafruit_GPS GPS(&gpsSerial);
Adafruit_BMP085 bmp;

#define LOG_FIXONLY false  //set to true to only log to SD when GPS has a fix, for debugging, keep it false

void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  SD.begin();
  bmp.begin();
  
  pinMode(10, OUTPUT);    // Chip Select pin must be set as output even if unused.
  
  logfile = SD.open(filename, FILE_WRITE);

  gpsConfigure();
}

void loop() {
  radioData = getRadioData();
  parachuteDeployed = manageParachuteDeploy(parachuteDeployed);
  checkRecovery(parachuteDeployed);
  
  String gpsData = getGPSdata();
  int imuData = getIMUdata();
  int temperatureData = bmp.readTemperature();
  int altitudeData = bmp.readAltitude();

  logData(gpsData, altitudeData, temperatureData, imuData);
  }

// ================================================================
// ===                   SUPPORT FUNCTIONS                      ===
// ================================================================

void logData(int gpsData, int altitudeData, int imuData) {  //Incomplete
  // Need to format all data from int -> String and then format into a standard csv format
  // Then write the standardized string to the file all at once
  char logString = "abc";

  logfile.write(logString);
}

void gpsConfigure() {
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);   // Turn off updates on antenna status, if the firmware permits it
}

String getGPSdata() {
  GPS.read();
  
  if (GPS.newNMEAreceived()) {    // If a new sentence is recieved  
    char *stringptr = GPS.lastNMEA();  
  
    if (!GPS.parse(stringptr)){   // this sets the newNMEAreceived() flag to false
      return "Parse Fail";  // we can fail to parse a sentence in which case we should just wait for another
    }
  
    if (LOG_FIXONLY && !GPS.fix) {
      Serial.print("No Fix");
      return "No Fix";
    }
  
    else {
      return stringptr;
    }
  }
}

int getIMUdata() {  // Incomplete
  
}

int manageParachuteDeploy(parachuteDeployed) {   // Incomplete
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
    digitalWrite(ParachuteDeployPin, HIGH);
    delay(100);
    return 1
  }
  
  if (parachuteDeployed) {
    return 1
  }
  
  // if ( !parachuteDeployed && (conditional1 || conditional2 || conditional3 || etc) ) {
  //   digitalWrite(ParachuteDeployPin, HIGH);
  //   delay(100);
  //   return 1
  // }
  
  else {
    return 0
  }
}

void Recovery(parachuteDeployed) {  // Incomplete
  // This function handles the recovery buzzer as well as broadcasting recovery data and any other recovery procedures
   if (parachuteDeployed) {
    digitalWrite(sirenPin, HIGH);
    // Add code here
   }
}

char getRadioData() {   // Incomplete
  
}

