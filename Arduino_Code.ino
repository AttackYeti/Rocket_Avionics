/*
Developer Notes
~~~~~
-Would be cool to have pwm to play sounds via the buzzer
-Need RTC implemented

*/

#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
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
#define GPSRx 8
#define GPSTx 7
#define SDCS 10
#define SDMOSI 11
#define SDMISO 12
#define SDCLK 13
#define parachuteDeployPin A3
#define sirenPin A2

#define LOG_FIXONLY false  //set to true to only log to SD when GPS has a fix, for debugging, keep it false

bool parachuteDeployed = false;
bool FORCE_PARA_DEPLOY = false;
bool STATUS_REPORT = false;
bool BEGIN_LAUNCH_RECORD = false;
bool GPS_ENABLED = false;
bool MPU_CALIBRATED = false;
bool ALTIMETER_ENABLED = false;

SoftwareSerial gpsSerial(GPSRx, GPSTx);
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

char SerInput;
int temperatureData = 0;
int altitudeData = 0;
//char gpsData[50];
char logString[200];

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Connection Successful...");
  GPS.begin(9600);
  Serial.println("GPS Connection Successful...");
  bmp.begin();
  // Need some sort of way to check if bmp successfully initialized
  ALTIMETER_ENABLED = true;
  Serial.println("Altimeter Successfully Enabled...");
  Wire.begin();

  Serial.println(F("Initializing MPU6050"));
  initialize_MPU();

  Serial.println(F("Configuring GPS"));
  gpsConfigure();
  Serial.println(F("Systems Initialized"));
  //SerInput.reserve(250); // Reserve 200 bytes for recieving data over serial


}

void loop() {
  //while (fifoCount < packetSize) {

    // !!! MAIN LOOP CODE START!!!
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    parachuteDeployed = manageParachuteDeploy(parachuteDeployed);
    Recovery(parachuteDeployed);
    parseGPS();

    altitudeData = bmp.readAltitude();
    temperatureData = bmp.readTemperature();
    
    if ((STATUS_REPORT == true) || (BEGIN_LAUNCH_RECORD == true || true)) {
      logData(altitudeData, 0, temperatureData, 0); //ypr);
      STATUS_REPORT = false;
    }
    // !!! MAIN LOOP CODE END !!!
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~

  /*  fifoCount = mpu.getFIFOCount();
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
  */
}

// ================================================================
// ===                   SUPPORT FUNCTIONS                      ===
// ================================================================

void logData(int altitudeData, int gpsData, int temperatureData, int imuData) {
  /*
  * Need to format all arguments from int -> String and then format into a standard csv format
  * Then write the standardized string to the serial all at once. logString should hold all of the converted
  * data in the specified format. The procedure for data being formatted should be fairly easy to modify if more info
  * is needed.
  */
  if (STATUS_REPORT) {
    logString[0] = (char)0;
    //logString.strcat 'STATUS~'+char(altitudeData)+'~'+char(temperatureData)+'~'+char(imuData)+'~'+char(parachuteDeployed)+'~'+char(MPU_CALIBRATED)+'~'+char(ALTIMETER_ENABLED)+'~'+char(GPS_ENABLED)+'\n';//+String(gpsData);
    logString.strcat('status');
    STATUS_REPORT = false;
  }

  else {
    //logString ="LOG~"+String(altitudeData)+"~"+String(temperatureData)+"~"+String(imuData)+"\n";//+String(gpsData);
  }

  Serial.println(logString);
  Serial.print(gpsData);
}

void gpsConfigure() {
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  GPS.sendCommand(PGCMD_NOANTENNA);   // Turn off updates on antenna status, if the firmware permits it
  GPS_ENABLED = true;
}

bool manageParachuteDeploy(bool parachuteDeployed) {
   /*
   * This function needs to check relevant data as well as wireless coms in order to see if parachute deployment should be activated, and if so then activate it.
   * You will most likely need to change your input variables in order to get useful info. You might try checking for patterns in the altitude or accelerations
   * using the IMU. ie freefall or something like that. It could be good to have multiple checks that would be activated just for redundancy. In order to actually
   * deploy the parachute you should use the command below... as you can see, it returns 1. The format that I envision being used would be for the datalogger to log
   * the state of the parachute as 0 until the parachute
   */
  if (FORCE_PARA_DEPLOY) {
    digitalWrite(parachuteDeployPin, HIGH);
    Serial.println("Parachute Deployed");
    FORCE_PARA_DEPLOY = false;
    return true;
  }

  if (parachuteDeployed) {
    delay(0.5);
    digitalWrite(parachuteDeployPin, LOW);
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
    digitalWrite(parachuteDeployPin, LOW);
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
  MPU_CALIBRATED = true;
}

void serialEvent() {
  char SerInput = (char)Serial.read();

  // Code below this point handles commands sent from the Pi to the Arduino
  if (SerInput == 'f') {
      FORCE_PARA_DEPLOY = true;
    }

  else if (SerInput == 'b') {
    BEGIN_LAUNCH_RECORD = true;
  }

  else if (SerInput == 's') {
      STATUS_REPORT = true;
  }
}

//char readGPS() {
//  char DATA = 'NULL';
//  if (gpsSerial.available()) {
//    char c = gpsSerial.readBytesUntil('$',DATA, );
//    if (c) {
//      DATA = "$"+c;
//    }
//  }
//  return DATA;
//}

void parseGPS() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }
}
