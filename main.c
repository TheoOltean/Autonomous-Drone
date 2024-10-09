#include <Fusion/FusionAhrs.h>
#include <Fusion/FusionMath.h>
#include <stdio.h>
#include <Fusion/Fusion.h>
#include "i2c/i2c.h"
#include "drivers/IMU-ICM-20948.h"
#include <unistd.h>

int main() {
    int result;
    result = i2c_init("/dev/i2c-1");
    if (result != 0) {
        perror("Failed to open i2c buss");
        return -1;
    }
    
    imu_autoOffsets();
    imu_setAccSampleRateDivider(10);
    
    //initialize fusion
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * 1.0, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    for (;;) {
        imu_readData();
        vec3 acc = imu_getAccRawValues();
        vec3 gyro = imu_getGyroRawValues();

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyro.fusionVec, acc.fusionVec, 1.0f);

        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        printf("x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f\n",earth.axis.x,earth.axis.y,earth.axis.z,euler.angle.roll,euler.angle.pitch,euler.angle.yaw);
    }
}
