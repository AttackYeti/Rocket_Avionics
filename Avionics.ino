/*
 * 
 * Recruiter on campus April 17
Developer Notes
~~~~~
-The SD library uses an obnoxious amount of memory.
$-IMU data will be fairly complicated and may take the longest time depending on method used
$-Should propably use established libraries for IMU
-Need Bluetooth Implemented
-Need GPS implemented
-Would be cool to have pwm to play sounds via the buzzer
*/

#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <AltSoftSerial.h>
#include <NMEAGPS.h>

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>

// Set the pins
#define gpsInterruptPin 3
#define GPSRx 7
#define GPSTx 8
#define SDCS 10
#define SDMOSI 11
#define SDMISO 12
#define SDCLK 13
#define parachuteDeployPin A1
#define sirenPin A2

int parachuteDeployed = false;
bool FORCE_PARA_DEPLOY = false;

//File logfile;
SoftwareSerial gpsSerial(GPSTx, GPSRx);
Adafruit_GPS GPS(&gpsSerial);
Adafruit_BMP085 bmp;
MPU6050 mpu;

// Variables Specifically for MPU
// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
int16_t gyro[3];
float ypr[3];
float motiondata[6];

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
float base_x_gyro = 0;
float base_y_gyro = 0;
float base_z_gyro = 0;
float base_x_accel = 0;
float base_y_accel = 0;
float base_z_accel = 0;

// This global variable tells us how to scale gyroscope data
float GYRO_FACTOR = 131.0;

// This global varible tells how to scale acclerometer data
float ACCEL_FACTOR = 1;

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LOG_FIXONLY false  //set to true to only log to SD when GPS has a fix, for debugging, keep it false

void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  //bleSerial.begin(9600);
  bmp.begin();
  Wire.begin();

  //Serial.println(F("Configuring BLE Module"));
  //bleConfigure();
  Serial.println(F("Initializing MPU6050"));
  initialize_MPU();
  Serial.println(F("Configuring Data logger"));
  //sdConfigure();
  Serial.println(F("Configuring GPS"));
  gpsConfigure();
  Serial.println(F("Systems Initialized"));
  inputString.reserve(200); // Reserve 200 bytes for recieving data over serial
}

void loop() {
  while (fifoCount < packetSize) {

    // !!! MAIN LOOP CODE START!!!
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    parachuteDeployed = manageParachuteDeploy(parachuteDeployed);
    Recovery(parachuteDeployed);

    char gpsData = getGPSdata();
    int temperatureData = bmp.readTemperature();
    int altitudeData = bmp.readAltitude();

    logData(gpsData, altitudeData, temperatureData, ypr);

    // !!! MAIN LOOP CODE END !!!
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    fifoCount = mpu.getFIFOCount();
  }
  
  if (fifoCount == 1024) {
  
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));      
  }
  
  else{  
    if (fifoCount % packetSize != 0) {       
      mpu.resetFIFO();           
    }
  
    else{  
        while (fifoCount >= packetSize) {         
          mpu.getFIFOBytes(fifoBuffer,packetSize);
          fifoCount -= packetSize;               
        } 
                 
        mpu.dmpGetQuaternion(&q,fifoBuffer);
        mpu.dmpGetGravity(&gravity,&q);
        mpu.dmpGetYawPitchRoll(ypr,&q,&gravity); 
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        motiondata[0] = ax - base_x_accel;
        motiondata[1] = ay - base_y_accel;
        motiondata[2] = az - base_z_accel;
        motiondata[3] = gx - base_x_gyro;
        motiondata[4] = gy - base_y_gyro;
        motiondata[5] = gz - base_z_gyro;
        ypr[0] *= 180/PI;
        ypr[1] *= 180/PI;
        ypr[2] *= 180/PI;                      
    }
  }
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
//  logfile = SD.open(filename, FILE_WRITE);
  
  String logString = String(altitudeData) +  ", " + char(temperatureData) + "\n" + char(imuData) + "\n " + char(gpsData);

  Serial.println(logString);
  
//  logfile.println(logString);
//  logfile.close();
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

void initialize_MPU() {
  //TWBR = 24;
    Serial.println(F("Initializing MPU..."));
    mpu.initialize();
    Serial.println(F("Initializing DMP..."));
    int devStatus = mpu.dmpInitialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    if (devStatus == 0) {
      Serial.println(F("Setting Offsets"));
      mpu.setXAccelOffset(-1343);
      mpu.setYAccelOffset(-1155);
      mpu.setZAccelOffset(1033);
      mpu.setXGyroOffset(19);
      mpu.setYGyroOffset(-27);
      mpu.setZGyroOffset(16);
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      packetSize = mpu.dmpGetFIFOPacketSize();
      fifoCount = mpu.getFIFOCount();
      Serial.println(F("Calibrating Sensor, Do Not Move"));
      calibrate_MPU();
    }

    else {
    // ERROR!
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
}

void calibrate_MPU() {
  int num_readings = 10;

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu.setZGyroOffset(-85);
  
  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}

void serialEvent() {
    String inputString = "";      // a String to hold incoming data
    bool stringComplete = false;  // whether the string is complete
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  if (inputString == "FORCE_DEPLOY" && stringComplete) {
      FORCE_PARA_DEPLOY == true;

      // clear the string:
      inputString = "";
      stringComplete = false;
    }
}
