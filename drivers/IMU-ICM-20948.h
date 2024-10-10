#ifndef IMU_ICM_20948_H
#define IMU_ICM_20948_H

#include "../vector.h"

typedef struct imu_data {
    vec3 acc;
    vec3 accOffsetVal;
    vec3 accCorrFactor;
    float accRangeFactor;

    vec3 gyro;
    vec3 gyroOffsetVal;
    float gyroRangeFactor;
} imu_data;

void imu_autoOffsets();
void imu_setAccSampleRateDivider(unsigned char value);
void imu_readData();
void imu_enableAcc();

vec3 imu_getAccRawValues();
vec3 imu_getAccCorrectedValues();
vec3 imu_getGyroRawValues();
vec3 imu_getGyroValues();
float imu_getTemperature();

#endif
