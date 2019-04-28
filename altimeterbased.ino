#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include <SD.h>

//~~~~~~~~~~~~~~~~~~~~~~~~Define Constants~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//---Pin Definitions
const int primaryCharge = A3;                                    // Pin to which the charge will be connected
const int secondaryCharge = A2;
const int chipSelect = 10;                                            // Used for the SD card

//---Strings
const char filename[] = "LOG.TXT";                                          // Name of log file on SD card

//---Timing Values
const int log_time = 1000;                                                  // How many ms between opening and closing file
const int max_time = 1800000;                                               // Time before Datalog Stops (1800000ms=30min)
const int primary_offset = 30000;                                         // Time before backup timer sends primary ejection signal

//---Memory Registers
const int MPU_addr=0x68;                                              // MPU6050 (Accelerometer,Gyro,Temp)

//~~~~~~~~~~~~~~~~~~~~~~~~Initialize Objects~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SimpleKalmanFilter kf1 = SimpleKalmanFilter(0.1,0.1,0.01);            // Kalman Filter which is currently unused
File logfile;                                                         // File object for logging to SD
Adafruit_BMP085 bmp;                                                  // Adafruit Barometer

//~~~~~~~~~~~~~~~~~~~~~~~~Define Variables~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//---Accelerometer Readings
float AcX;                                    //Acceleration X
float AcY;                                    //Acceleration Y
float AcZ;                                    //Acceleration Z
double arrAcc[7] = {0,0,0,0,0,0,0};           //Acceleration LPF
double avgAcc = 0.0;                          //Filtered Average Net Acceleration

//---Gyroscope Readings
float GyX;                                    //Gyroscope X
float GyY;                                    //Gyroscope Y
float GyZ;                                    //Gyroscope Z

//---Altimeter Readings
int altitudeData = 0;                         //Altimeter Computed Altitude
int pressureData = 0;                         //Raw Barometric Pressure
int temperatureData = 0;                      //Temperature
double arrAlt[7] = {0,0,0,0,0,0,0};           //Altitude LPF
double avgAlt = 0.0;                          //Filtered Altimeter Computed Altitude
double startAlt = 0.0;                        //Measured Starting Altitude
double maxAlt = 0.0;                          //Maximum Measured Altitude

//---Boolean State Variables
bool inFlight = false;                        // Stores whether launch has been detected
bool apogeeReached = false;                   // Stores whether apogee has been detected
bool chuteFired = false;                      // Stores whether attempt has been made to fire primary charge
bool backupArmed = false;                     //
bool backupFired = false;

//---Numerical State Values
int countLoop = 0;                            //Counter used for finding starting altitude, records number of times loop has executed
int last_log;                                 //Timer used to manage frequency of log file reopening
int launchTime;                               //Offset used for timer based deployment
int primaryIgnitionTime;                      //Offset used for secondary/backup charge ignition

void setup() {
  initializeMPU();                                             //Send codes to initialize IMU
  bmp.begin();                                                 //Setup Barometer
  Serial.begin(115200);
  pinMode(primaryCharge, OUTPUT);
  pinMode(secondaryCharge, OUTPUT);

//---Initialize the SD card
  pinMode(chipSelect, OUTPUT);
  SD.begin(chipSelect);
  logfile = SD.open(filename, FILE_WRITE);  
  last_log = millis();
}

void loop() {
  MPU();
  BARO();
  
  if (countLoop < 50) {
    startAlt = avgAlt;
    countLoop += 1;
  }
  
  else if (countLoop >= 50) {
    updateMaxAltReached();
    detectLaunch();
    detectApogee();
    manageParachuteDeploy();
  }
  logData();
}

// ================================================================
// ===                   SUPPORT FUNCTIONS                      ===
// ================================================================

void logData() {
  if (((millis() - last_log) <= log_time) && (millis() < max_time)) {
  String logString=String(millis())+","+String(startAlt)+","+String(maxAlt)+","+String(avgAlt)+","+String(altitudeData)
    +","+String(inFlight) +","+String(launchTime)+","+String(apogeeReached)+","+String(backupArmed)+","+String(chuteFired) 
    +","+String(backupFired)+","+String(avgAcc); 
  logfile.println(logString);
  }
  
  else {
    logfile.close(); 
    logfile = SD.open(filename, FILE_WRITE); 
    last_log = millis();
  }
}

void manageParachuteDeploy() {
//---Manage in-flight arming of seconary charge
  if (avgAlt-startAlt >= 300) {
    backupArmed = true; 
  }
  
//----Primary Charge Ignition----
//-------------------------------

//---Send Primary Ignition Signal                                             //Sends signal if apogee detected and chute not deployed
  if (apogeeReached && !chuteFired) {                                         //dependent on detectlaunch and detect apogee (correct altimeter data)
    digitalWrite(primaryCharge, HIGH);
    chuteFired = true;
    primaryIgnitionTime = millis();
  }
  
//---Send Primary Ignition Signal                                             //Sends signal if launch detected, backup timer, and primary detection failed
  if (inFlight && ((millis()-launchTime))>primary_offset && !chuteFired) {    //dependent on detectlaunch (launchtime and correct altimeter data)
    digitalWrite(primaryCharge, HIGH);
    chuteFired = true;
    primaryIgnitionTime = millis();
  }
  
//---End Primary Ignition Signal                                              //Sends signal 2000ms after primary ignition
  if (chuteFired && ((millis()-primaryIgnitionTime)>2000)) {                  //dependent on accurate timekeeping
    digitalWrite(primaryCharge, LOW);
  }
  
//---Secondary Charge Ignition---
//-------------------------------

//---Send Secondary Ignition Signal                                           //Sends signal at 300m after primary ignition
  if (backupArmed && (avgAlt - startAlt) <= 300) {                            //dependent on correct altimeter data                 
    digitalWrite(secondaryCharge, HIGH);
    backupFired = true;
  }

//---End Secondary Ignition Signal
  if (backupFired && ((millis()-primaryIgnitionTime)>2000)) {
    digitalWrite(secondaryCharge, LOW);
  }
}

void initializeMPU() {
//---Handle initialization commands for MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(B00011000);   //here is the byte for sensitivity (16g here) check datasheet for the one u want
  Wire.endTransmission(true);
}

void MPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers

  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  double junk1 = Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  AcX = AcX / 2048.0 * 9.81;        //Convert accelerometer values to m/s^2
  AcY = AcY / 2048.0 * 9.81;
  AcZ = AcZ / 2048.0 * 9.81;
  GyX = GyX / 131.0;                  //Concert gyroscope values to degrees/s
  GyY = GyY / 131.0;
  GyZ = GyZ / 131.0;
  double accelMag = kf1.updateEstimate(sqrt(sq(AcX)+sq(AcY)+sq(AcZ)));
  avgAcc = rollingAverage(accelMag, arrAcc);
  
}

void BARO(){
  temperatureData = bmp.readTemperature();
  altitudeData = bmp.readAltitude();
  pressureData = bmp.readPressure();
  avgAlt = rollingAverage(altitudeData, arrAlt);
}

double rollingAverage(double data, double avg[]){
   double sum = 0.0;
   int arrlength = sizeof(avg);
   for(int i = 0; arrlength; i++){
    avg[i] = avg[i+1];
    sum += avg[i];
   }
   avg[arrlength - 1] = data;
   sum += avg[arrlength - 1];
   return sum / arrlength;
}

void detectLaunch(){                            //launched if 10m above start altitude
  if (avgAlt - startAlt > 10 && !inFlight) {
    inFlight = true;
    launchTime = millis();
  }
}

void updateMaxAltReached() {                    
  if (avgAlt > maxAlt) {
    maxAlt = avgAlt;
  }
}

void detectApogee(){                            //apogee reached if current alitutde is 5 meters below the maximum recorded
  if (inFlight && (maxAlt - avgAlt) > 5) {
    apogeeReached = true;
  } 
}






