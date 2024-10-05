#ifndef IMU_ICM_20948_H
#define IMU_ICM_20948_H

typedef struct vec3 {
    float x;
    float y;
    float z;
} vec3;

typedef struct imu_data {
    vec3 acc;
    vec3 accOffsetVal;
    vec3 accCorrFactor;
    float accRangeFactor;

    vec3 gyro;
    vec3 gyroOffsetVal;
    float gyroRangeFactor;
} imu_data;

#endif
