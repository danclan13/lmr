// lmr arduino imu - new bootlogger

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// uncomment one combo 9-DoF!
//#include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...
//#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
//#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout
#include "ISM330DHCX_LIS3MDL.h"

// pick your filter! slower == better quality output
//Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 1
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  //accelerometer->printSensorDetails();
  //gyroscope->printSensorDetails();
  //magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
}


void loop() {
  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");
#endif

  // print the heading, pitch and roll
  roll = filter.getRoll()-3.1;
  pitch = filter.getPitch()+2.7;
  heading = filter.getYaw();

  //float headingN, headingW; 
  //headingN = fabs(heading);
  //headingW = 57.3*acos(sin((heading)/57.3));
  //Serial.print("Heading:");
  //Serial.print(heading);
  //Serial.print("HeadingN:");
  //Serial.print(headingN);
  //Serial.print(",HeadingW:");
  //Serial.println(headingW);
  //Serial.print("Pitch(y):");
  //Serial.print(pitch);
  //Serial.print(", Roll(x):");
  //Serial.println(roll);

float p = pitch*0.0174533; // Phi
float k = roll*0.0174533;  // Ksi

float leaning,direct;
  if (abs(k)>=0.5*PI || abs(p)>=0.5*PI)
  { leaning = 180.0-57.2958*atan(sqrt(sq(tan(p))+sq(tan(k))));   }
  else
  { leaning = 57.2958*atan(sqrt(sq(tan(p))+sq(tan(k))));  }

  if (k>=0)
   {  direct = 90.0-57.2958*atan((tan(p))/(tan(k)));   }
   else
   {  direct = 270.0-57.2958*atan((tan(p))/(tan(k)));  }

  
  //Serial.print(",Leaning:");
  //Serial.print(leaning);
  //Serial.print(",Direction:");
  //Serial.println(direct);


  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  //Serial.print("qw:");
  //Serial.print(qw, 4);
  //Serial.print(",qx:");
  //Serial.print(qx, 4);
  //Serial.print(",qy:");
  //Serial.print(qy, 4);
  //Serial.print(",qz:");
  //Serial.println(qz, 4);  
  
int h,l,d;
h = 10.0*heading+0.5; 
l = 10.0*leaning+0.5;
d = 10.0*direct+0.5;

Serial.print("hld,");
Serial.print(h);
Serial.print(",");
Serial.print(l);
Serial.print(",");
Serial.print(d);
Serial.println(",end");

  
  //float attitude, bank;
  //heading = 57.3*atan2(2.0 * (qx*qy + qz*qw),(sq(qx) - sq(qy) - sq(qz) + sq(qw)));
  //bank = 57.3*atan2(2.0 * (qy*qz + qx*qw),(-sq(qx) - sq(qy) + sq(qz) + sq(qw)));
  //attitude = 57.3*asin(-2.0 * (qx*qz - qy*qw)/(sq(qx) + sq(qy) + sq(qz) + sq(qw)));

  
  //Serial.print(",Heading:");
  //Serial.print(heading);
  //Serial.print(",Attitude:");
  //Serial.print(attitude);
  //Serial.print(",Bank:");
  //Serial.println(bank);
  
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif
}
