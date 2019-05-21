#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <SimpleKalmanFilter.h>
#include <analogWrite.h>
#include <mySD.h>


// ================================================================
// ===                   GLOBAL DEFINITIONS                     ===
// ================================================================

//~~~~~~~~~~~~~~~~~~~~Define Structures~~~~~~~~~~~~~~~~~~~~~~~
struct RTC {
  byte second, minute, hour, weekDay, monthDay, month, year;
};

//~~~~~~~~~~~~~~~~~~~~~Define Constants~~~~~~~~~~~~~~~~~~~~~~~
//---Barometer Definitions
#define SEA_LEVEL_CALIBRATION_HPA (1013.25)                           // Calibration of barometer must be done before every use for accurate readings

//---Pin Definitions
#define primaryCharge 27                                              // Pin to which primary charge will be connected
#define secondaryCharge 33                                            // Pin to which secondary charge will be connected
#define buzzerPin LED_BUILTIN                                         // Used for auditory signals
#define BMP388interruptPin 14                                         // Interrupt used for signaling data-ready from BMP388
#define CS 4                                                          // Used for the SD card
#define SCK 5                                                         // Labeled as SCK on ESP32
#define MISO 19                                                       // Labeled as MI on ESP32
#define MOSI 18                                                       // Labeled as MO on ESP32
#define SCL 22                                                        // Labeled as SCL on ESP32
#define SDA 23                                                        // Labeled as SDA on ESP32
#define LedR 15
#define LedG 33
#define LedB 27

//---Strings
const char filename[] = "/LOG.TXT";                                    // Name of log file on SD card

//---Timing Values
const int log_time = 1000;                                            // How many ms between opening and closing file
const int max_time = 1800000;                                         // Time before Datalog Stops (1800000ms=30min)
const int primary_offset = 30000;                                     // Time before backup timer sends primary ejection signal

//---Memory Registers
const int DS3231_addr=0x68;                                           // DS3231  (Real Time Clock)
const int BNO055_addr=0x68;                                           // MPU6050 (Accelerometer,Gyro,Temp)
const int BMP388_addr=0x77;                                           // BNO055  (Accel,Gyro,Temp,Mag)

//~~~~~~~~~~~~~~~~~~~~~Initialize Objects~~~~~~~~~~~~~~~~~~~~~
SimpleKalmanFilter kf1 = SimpleKalmanFilter(0.1,0.1,0.01);            // Kalman Filter which is currently unused
File logFile;                                                         // File object for logging to SD
Adafruit_BMP3XX bmp388;                                               // (I2C) Adafruit Barometer
Adafruit_BNO055 bno055 = Adafruit_BNO055();                           // (I2C) Adafruit Accelerometer, Gyroscope, Magnetometer 
hw_timer_t*buzzerTimer = NULL;
struct RTC timeRTC;

//~~~~~~~~~~~~~~~~~~~~~Define Variables~~~~~~~~~~~~~~~~~~~~~~~
//---BNO055 Readings
imu::Vector<3> euler;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linAcc;
byte byte_accFilterLen = 7;
double arrAcc[7];                             //Acceleration LPF
double avgAcc = 0.0;                          //Filtered Average Net Acceleration

//---Altimeter Readings
volatile bool BMP388_DATA_READY = false;      //Used to control when measurements are collected from BMP388
double altitudeData = 0.0;                    //Altimeter Computed Altitude
double pressureData = 0.0;                    //Raw Barometric Pressure
double temperatureData = 0.0;                 //Temperature
byte byte_altFilterLen = 5;
double arrAlt[5];                             //Altitude LPF
double avgAlt = 0.0;                          //Filtered Altimeter Computed Altitude
double startAlt = 0.0;                        //Measured Starting Altitude
double maxAlt = 0.0;                          //Maximum Measured Altitude
int interruptTime = 0;
int last_print = 0;

//---Boolean State Variables
bool inFlight = false;                        // Stores whether launch has been detected
bool apogeeReached = false;                   // Stores whether apogee has been detected
bool chuteFired = false;                      // Stores whether attempt has been made to fire primary charge
bool backupArmed = false;                     // Stores whether secondary charge is armed to fire
bool backupFired = false;                     // Stores whether attempt has been made to fire secondary charge

//---Buzzer Handling
volatile byte intervals = 0;
volatile int beepRate = 0;

//---Numerical State Values
int countLoop = 0;                            //Counter used for finding starting altitude, records number of times loop has executed
int last_log;                                 //Timer used to manage frequency of log file reopening
int launchTime;                               //Offset used for timer based deployment
int primaryIgnitionTime;                      //Offset used for secondary/backup charge ignition

// ================================================================
// ===                      ISR FUNCTIONS                       ===
// ================================================================

//---0.125s timer used for buzzer timing
void IRAM_ATTR beep_Timer() {
  
  if (intervals == 255){
    intervals = 0;
  }
  
  else {
    intervals ++;
  }
}

void IRAM_ATTR BMP388_DataReady() {
  BMP388_DATA_READY = true;
  interruptTime = millis();
}
// ================================================================
// ===                     BODY FUNCTIONS                       ===
// ================================================================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  Serial.println("Initializing Controller");
  initializeLogger();
  //setDS3231time(30,59,16,7,12,5,19);        // Used to recalibrate RTC
  
  initializeBNO055();                         //Setup IMU
  initializeBMP388();                         // Set up Altimeter
  initializeBuzzer();
  initializeLed();
  
  pinMode(primaryCharge, OUTPUT);
  pinMode(secondaryCharge, OUTPUT);
  Serial.println("Controller Initialized");
}

void loop() {
  readDS3231time();
  BNO055();
  BMP388();
  //debugBNO055();
  //debugBMP388();
  //debugDS3231();
  
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
  beep();
}

// ================================================================
// ===                       ALGORITHMS                         ===
// ================================================================

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

double rollingAverage(double data, double *average, byte length){
  double sum = 0.0;
 
  for(int i = 0; i < length-1; i++){
    average[i] = average[i+1];
    sum += average[i];
  }
  
  average[length - 1] = data;
  sum += data;
  
  return sum / length;
}

void detectLaunch() {                            //launched if 10m above start altitude
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

// ================================================================
// ===                    HARDWARE DRIVERS                      ===
// ================================================================

void initializeBNO055() {
  if (!bno055.begin()) {
    Serial.println("Unable to connect to BNO055 module");
    delay(1000);
  }
}

void BNO055() {
  // Get sensor readings from the BNO055 IMU module to be used by other functions
  euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);                  // Units - degrees
  gyro = bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);               // Units - rad/s
  accel = bno055.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);          // Units - m/s^2
  linAcc = bno055.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);           // Units - m/s^2
  
  double accelMag = kf1.updateEstimate(sqrt(sq(accel.x())+sq(accel.y())+sq(accel.z())));
  avgAcc = rollingAverage(accelMag, arrAcc, byte_accFilterLen);
}

void debugBNO055() {
  
    String logString=
                      String(avgAcc)       +","+
                      String(euler.x())    +","+
                      String(euler.y())    +","+
                      String(euler.z())    +","+
                      String(gyro.x())     +","+
                      String(gyro.y())     +","+
                      String(gyro.z())     +","+
                      String(accel.x())    +","+
                      String(accel.y())    +","+
                      String(accel.z()) ;   
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  Serial.println("            BNO055 Debugging            ");
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    
  Serial.println(logString);
  //delay(1000);
  }


void initializeBMP388() {
  if (!bmp388.begin()) {
    Serial.println("Unable to connect to BMP388 module");
    delay(1000);
  }
  else {
    pinMode(BMP388interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BMP388interruptPin), BMP388_DataReady, RISING);
    // Set up oversampling and filter initialization
    bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp388.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    initializeBMP388Int();
  }
}

void BMP388(){
  if (BMP388_DATA_READY) {
      temperatureData = bmp388.temperature;                                   // Units - degrees Celcius
      // Right here in assignment to altitudeData causes a core panic
      altitudeData = bmp388.readAltitude(SEA_LEVEL_CALIBRATION_HPA);          // Units - Meters
      pressureData = bmp388.pressure;                                         // Units - Pa      
      avgAlt = rollingAverage(altitudeData, arrAlt, byte_altFilterLen);       // Units - Meters
      BMP388_DATA_READY = false;
  }
}

void debugBMP388() {
  if (millis() - last_print > 100) {
    String logString=
                      String(avgAlt)             +","+
                      String(altitudeData)       +","+
                      String(pressureData)       +","+
                      String(millis()-interruptTime) +","+
                      String(temperatureData);
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  Serial.println("            BMP388 Debugging            ");
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
   
  Serial.println(logString);
  last_print = millis();
  }
}

void initializeBMP388Int() { 
  Wire.beginTransmission(BMP388_addr);
  Wire.write(0x1B);       // Address of mode selection
  Wire.write(0b00110011); // Enable Normal Mode
  
  Wire.write(0x1D); // Address of output data rate setting
  Wire.write(0x01);     // Set output data rate to 100Hz
  
  Wire.write(0x19); // Address to write to for enabling data-ready interrupt
  Wire.write(0b01000010); // Enable data-ready interrupt with bit 6
  Wire.endTransmission();
}


void initializeBuzzer() {
  pinMode(buzzerPin, OUTPUT);
  buzzerTimer = timerBegin(0, 80, true); 
  timerAttachInterrupt(buzzerTimer, &beep_Timer, true);
  timerAlarmWrite(buzzerTimer, 125000, true);
  timerAlarmEnable(buzzerTimer);
  setBeepRate(3);
}

void setBeepRate(byte rate) {
  if (rate != beepRate) {
    beepRate = rate;
    intervals = 0;
  } 
}

void beep() {

  switch (beepRate) {
    
    case 0:                                         // Set to 0 __ continuous
      analogWrite(buzzerPin, 0);
      break;
  
    case 1:                                         // Set to 1 **__ continuous
      if (intervals == 0) {
        analogWrite(buzzerPin, 255);
        break;
      }
      if (intervals == 2) {
        analogWrite(buzzerPin, 0);
        intervals = 0;
        break;
      }
    
    case 2:                                         // Set to 2 *_*_
      if (intervals == 1) {
        analogWrite(buzzerPin, 0);
        break;
      }
      if (intervals == 0 || intervals == 2) {
        analogWrite(buzzerPin, 255);
        break;
      }
      if (intervals == 3) {
        analogWrite(buzzerPin, 0);
        setBeepRate(0);
        break;
      } 
    
    case 3:                                         // Set to 3 *_*_*_ (Startup Tone)
      if (intervals == 1 || intervals == 3) {
        analogWrite(buzzerPin, 0);
        break;
      }
      if (intervals == 0 || intervals == 2 || intervals == 4) {
        analogWrite(buzzerPin, 255);
        break;
      }
      if (intervals == 5) {
        analogWrite(buzzerPin, 0);
        setBeepRate(0);
        break;
      }
    
    case 4:                                         // Set to 4 *_**__*_**_
      if (intervals == 1 || intervals == 4 || intervals == 7) {
        analogWrite(buzzerPin, 0);
        break;
      }
      if (intervals == 0 || intervals == 2 || intervals == 6 || intervals == 8) {
        analogWrite(buzzerPin, 255);
        break;
      }
      if (intervals == 10) {
        analogWrite(buzzerPin, 0);
        setBeepRate(0);
        break;
      } 
    
    case 5:                                         // Set to 5 *_ continuous
      if (intervals == 0) {
        analogWrite(buzzerPin, 255);
        break;
      }
      if (intervals == 1) {
        analogWrite(buzzerPin, 0);
        intervals = 0;
        break;
      }
    
    case 6:                                         // Set to ** continuous
      analogWrite(buzzerPin, 255);
      break;

    default:                                        // If unprogrammed input, turn off the pin and set to 0
      analogWrite(buzzerPin, 0);
      setBeepRate(0);
      break;
  }
}


void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year) {
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0);
  Wire.write(decToBcd(second));     // set seconds
  Wire.write(decToBcd(minute));     // set minutes
  Wire.write(decToBcd(hour));       // set hours
  Wire.write(decToBcd(dayOfWeek));  // set day of week (1=sunday)
  Wire.write(decToBcd(dayOfMonth)); // set day of month
  Wire.write(decToBcd(month));      // set month
  Wire.write(decToBcd(year));       // set year (0-99)
  Wire.endTransmission();
}

void readDS3231time() {
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0); 
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, 7);
  
  timeRTC.second   = bcdToDec(Wire.read() & 0x7f);   // seconds
  timeRTC.minute   = bcdToDec(Wire.read());          // minute
  timeRTC.hour     = bcdToDec(Wire.read() & 0x3f);   // hour
  timeRTC.weekDay  = bcdToDec(Wire.read());          // weekday
  timeRTC.monthDay = bcdToDec(Wire.read());          // month day
  timeRTC.month    = bcdToDec(Wire.read());          // month
  timeRTC.year     = bcdToDec(Wire.read());          // year
}

byte decToBcd(byte val) {
  return( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val) {
  return( (val/16*10) + (val%16) );
}

void debugDS3231() {
  Serial.print(timeRTC.hour, DEC);
    // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (timeRTC.minute<10) {
    Serial.print("0");
  }
  Serial.print(timeRTC.minute, DEC);
  Serial.print(":");
  if (timeRTC.second<10) {
    Serial.print("0");
  }
  Serial.print(timeRTC.second, DEC);
  Serial.print(" ");
  Serial.print(timeRTC.monthDay, DEC);
  Serial.print("/");
  Serial.print(timeRTC.month, DEC);
  Serial.print("/");
  Serial.print(timeRTC.year, DEC);
  Serial.print(" Day of week: ");
  switch(timeRTC.weekDay){
    case 1:
      Serial.println("Sunday");
      break;
    case 2:
      Serial.println("Monday");
      break;
    case 3:
      Serial.println("Tuesday");
      break;
    case 4:
      Serial.println("Wednesday");
      break;
    case 5:
      Serial.println("Thursday");
      break;
    case 6:
      Serial.println("Friday");
      break;
    case 7:
      Serial.println("Saturday");
      break;
  }
  delay(1000);
}


void initializeLogger() {
  pinMode(CS, OUTPUT);

  if(!SD.begin(CS)){
        Serial.println("Card Mount Failed");
        return;
    }
  
  logFile = SD.open(filename, FILE_WRITE);  

  if(!logFile){
        Serial.println("Failed to open file for writing");
        return;
    }
  if (logFile.print("Begin Datalog")){
      Serial.println("Message written");
  } else {
      Serial.println("write failed");
  }
  logFile.flush();
  logFile.close();
  last_log = millis();
}

void logData() {
  if (((millis() - last_log) <= log_time) && (millis() < max_time)) {

    logFile.print(timeRTC.second);
    logFile.print(',');
    logFile.print(timeRTC.minute);
    logFile.print(',');
    logFile.print(timeRTC.hour);
    logFile.print(',');
    logFile.print(timeRTC.monthDay);
    logFile.print(',');
    logFile.print(timeRTC.month);
    logFile.print(',');
    logFile.print(timeRTC.year);
    logFile.print(',');
    logFile.print(startAlt);
    logFile.print(',');
    logFile.print(maxAlt);
    logFile.print(',');
    logFile.print(avgAlt);
    logFile.print(',');
    logFile.print(altitudeData);
    logFile.print(',');
    logFile.print(inFlight);
    logFile.print(',');
    logFile.print(launchTime);
    logFile.print(',');
    logFile.print(apogeeReached);
    logFile.print(',');
    logFile.print(backupArmed);
    logFile.print(',');
    logFile.print(chuteFired);
    logFile.print(',');
    logFile.print(backupFired);
    logFile.print(',');
    logFile.print(avgAcc);
    logFile.print(',');
    logFile.print(euler.x());
    logFile.print(',');
    logFile.print(euler.y());
    logFile.print(',');
    logFile.print(euler.z());
    logFile.print(',');
    logFile.print(gyro.x());
    logFile.print(',');
    logFile.print(gyro.y());
    logFile.print(',');
    logFile.print(gyro.z());
    logFile.print(',');
    logFile.print(accel.x());
    logFile.print(',');
    logFile.print(accel.y());
    logFile.print(',');
    logFile.print(accel.z());
    logFile.println('$');
  }
  
  else {
    Serial.println("Closing file");
    logFile.close(); 
    logFile = SD.open(filename, FILE_WRITE); 
    last_log = millis();
  }
}


void initializeLed() {
  pinMode(LedR, OUTPUT);
  pinMode(LedG, OUTPUT);
  pinMode(LedB, OUTPUT);
  setLedColor('g');
}

void setLedColor(char color) {
  if (color == 'r') {           // red
    analogWrite(LedR, 255);
    analogWrite(LedG, 0);
    analogWrite(LedB, 0);
  }

  if (color == 'g') {           // green
    analogWrite(LedR, 0);
    analogWrite(LedG, 255);
    analogWrite(LedB, 0);
  }

  if (color == 'b') {           // blue
    analogWrite(LedR, 0);
    analogWrite(LedG, 0);
    analogWrite(LedB, 255);
  }

  if (color == 'w') {           // white
    analogWrite(LedR, 255);
    analogWrite(LedG, 255);
    analogWrite(LedB, 255);
  }

  if (color == 'c') {           // cyan
    analogWrite(LedR, 0);
    analogWrite(LedG, 255);
    analogWrite(LedB, 255);
  }

  if (color == 'y') {           // yellow
    analogWrite(LedR, 255);
    analogWrite(LedG, 255);
    analogWrite(LedB, 0);
  }

  if (color == 'm') {           // magenta
    analogWrite(LedR, 255);
    analogWrite(LedG, 0);
    analogWrite(LedB, 255);
  }
}








