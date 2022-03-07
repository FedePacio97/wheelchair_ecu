#include "IMU.h"

IMU::IMU(){
  icmSettings =
  {
    .cs_pin = 33,                   // SPI chip select pin
    .spi_speed = 7000000,           // SPI clock speed in Hz, max speed is 7MHz
    .mode = 1,                      // 0 = low power mode, 1 = high performance mode
    .enable_gyroscope = false,      // Enables gyroscope output
    .enable_accelerometer = false,  // Enables accelerometer output
    .enable_magnetometer = false,   // Enables magnetometer output
    .enable_quaternion = true,      // Enables quaternion output
    .gyroscope_frequency = 1,       // Max frequency = 225, min frequency = 1
    .accelerometer_frequency = 1,   // Max frequency = 225, min frequency = 1
    .magnetometer_frequency = 1,    // Max frequency = 70, min frequency = 1
    .quaternion_frequency = 225     // Max frequency = 225, min frequency = 50
  };

  icm20948.init(icmSettings); //Add #include<Arduino.h> in case of ESP32 usage
}

void IMU::updateSensors(){
  float quat_w, quat_x, quat_y, quat_z;
  // Must call this often in main loop -- updates the sensor values
  icm20948.task();

  if (icm20948.quatDataIsReady())
  {
    icm20948.readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);

    // roll (x-axis rotation)
    double t0 = +2.0 * (quat_w * quat_x + quat_y * quat_z);
    double t1 = +1.0 - 2.0 * (quat_x * quat_x + quat_y * quat_y);
    double roll = atan2(t0, t1) * 180.0 / PI;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (quat_w * quat_y - quat_z * quat_x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch = asin(t2) * 180.0 / PI;

    // yaw (z-axis rotation)
    double t3 = +2.0 * (quat_w * quat_z + quat_x * quat_y);
    double t4 = +1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
    double yaw = atan2(t3, t4) * 180.0 / PI;

    myMedianRoll.addValue(int(roll));
    myMedianPitch.addValue(int(pitch));
    myMedianYaw.addValue(int(yaw));

    ahsr.roll = myMedianRoll.getMedian(); // retieves the median
    ahsr.pitch = myMedianPitch.getMedian();
    ahsr.yaw = myMedianYaw.getMedian();

  }
}

uint16_t IMU::check_stability(){
  if( (abs(ahsr.pitch) >= WHEELCHAIR_RISK_OVERTURNING_PITCH_threshold) && (abs(ahsr.pitch) < WHEELCHAIR_OVERTURNED_PITCH_threshold))
    return WHEELCHAIR_RISK_OVERTURNING_PITCH;
  if(abs(ahsr.pitch) >= WHEELCHAIR_OVERTURNED_PITCH_threshold)
    return WHEELCHAIR_OVERTURNED_PITCH;

  if( (abs(ahsr.roll) >= WHEELCHAIR_RISK_OVERTURNING_ROLL_threshold) && (abs(ahsr.roll) < WHEELCHAIR_OVERTURNED_ROLL_threshold))
    return WHEELCHAIR_RISK_OVERTURNING_ROLL;
  if(abs(ahsr.roll) >= WHEELCHAIR_OVERTURNED_ROLL_threshold)
    return WHEELCHAIR_OVERTURNED_ROLL;

  return WHEELCHAIR_STABLE;
}

int IMU::get_pitch(){
  return ahsr.pitch;
}

int IMU::get_roll(){
  return ahsr.roll;
}

int IMU::get_yaw(){
  return ahsr.yaw;
}

AHSR IMU::get_ahsr(){
  return ahsr;
}