
#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;

#include <Adafruit_ISM330DHCX.h>
Adafruit_ISM330DHCX ism330dhcx;

bool init_sensors(void) {
  if (!ism330dhcx.begin_I2C() || !lis3mdl.begin_I2C()) {
    return false;
  }
  accelerometer = ism330dhcx.getAccelerometerSensor();
  gyroscope = ism330dhcx.getGyroSensor();
  magnetometer = &lis3mdl;

  return true;
}


void setup_sensors(void) {
  // set lowest range
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  // set slightly above refresh rate
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_104_HZ);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}
