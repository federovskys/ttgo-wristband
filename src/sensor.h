#ifndef SENSOR_H
#define SENSOR_H

#include <MPU9250.h>
#include "quaternionFilters.h"

// original source: https://github.com/Xinyuan-LilyGO/LilyGO-T-Wristband/blob/master/sensor.h

void readMPU9250(MPU9250* imu) {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (imu->readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    imu->readAccelData(imu->accelCount);  // Read the x/y/z adc values
    imu->getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    imu->ax = (float)imu->accelCount[0] * imu->aRes;  // - accelBias[0];
    imu->ay = (float)imu->accelCount[1] * imu->aRes;  // - accelBias[1];
    imu->az = (float)imu->accelCount[2] * imu->aRes;  // - accelBias[2];

    imu->readGyroData(imu->gyroCount);  // Read the x/y/z adc values
    imu->getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    imu->gx = (float)imu->gyroCount[0] * imu->gRes;
    imu->gy = (float)imu->gyroCount[1] * imu->gRes;
    imu->gz = (float)imu->gyroCount[2] * imu->gRes;

    imu->readMagData(imu->magCount);  // Read the x/y/z adc values
    imu->getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    imu->magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    imu->magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    imu->magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    imu->mx = (float)imu->magCount[0] * imu->mRes * imu->magCalibration[0] - imu->magbias[0];
    imu->my = (float)imu->magCount[1] * imu->mRes * imu->magCalibration[1] - imu->magbias[1];
    imu->mz = (float)imu->magCount[2] * imu->mRes * imu->magCalibration[2] - imu->magbias[2];
  }  // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  imu->updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(imu->ax, imu->ay, imu->az, imu->gx * DEG_TO_RAD, imu->gy * DEG_TO_RAD, imu->gz * DEG_TO_RAD,
                         imu->my, imu->mx, imu->mz, imu->deltat);
  // Serial print and/or display at 0.5 s rate independent of data rates
  imu->delt_t = millis() - imu->count;

  if (imu->delt_t > 20) {
    imu->yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)),
                     *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) -
                         *(getQ() + 3) * *(getQ() + 3));
    imu->pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
    imu->roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)),
                      *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) +
                          *(getQ() + 3) * *(getQ() + 3));
    imu->pitch *= RAD_TO_DEG;
    imu->yaw *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    //  8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    imu->yaw -= 8.5;
    imu->roll *= RAD_TO_DEG;
    imu->count = millis();
    imu->sumCount = 0;
    imu->sum = 0;

  }  // if (imu->delt_t > 20)
}

#endif