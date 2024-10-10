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
    
    uint8_t whoami;
    i2c_readRegister(0x69,0,&whoami);
    printf("%d\n",whoami);
    result = i2c_writeRegister(0x69,15,12);
    if (result != 0) {
        perror("Failed to write i2c");
        return -1;
    }
    i2c_readRegister(0x69,15,&whoami);
    printf("%d\n",whoami);
    imu_enableAcc();
    imu_autoOffsets();
    //imu_setAccSampleRateDivider(1);
    FILE *outputcsv = fopen("output.csv","w");    
    //initialize fusion
    //FusionAhrs ahrs;
    //FusionAhrsInitialise(&ahrs);

    //const FusionAhrsSettings settings = {
    //        .convention = FusionConventionNwu,
    //        .gain = 0.5f,
    //        .gyroscopeRange = 250.0f,
    //        .accelerationRejection = 10.0f,
    //        .magneticRejection = 10.0f,
    //        .recoveryTriggerPeriod = 5 * 1.0, /* 5 seconds */
    //};
    //FusionAhrsSetSettings(&ahrs, &settings);

    double time = 0;
    fprintf(outputcsv,"time,ax,ay,az,gx,gy,gz\n");
    for (;;) {
        imu_readData();
        vec3 acc = imu_getAccCorrectedValues();
        vec3 gyro = imu_getGyroValues();

        //FusionAhrsUpdateNoMagnetometer(&ahrs, gyro.fusionVec, acc.fusionVec, 1.0f);

        //FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        //FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        //printf("accx: %f, accy: %f, accz: %f, x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f\n",acc.x,acc.y,acc.z,earth.axis.x,earth.axis.y,earth.axis.z,euler.angle.roll,euler.angle.pitch,euler.angle.yaw);
        printf("x: % 7.4f, y: % 7.4f, z: % 7.4f, roll: % 7.2f, pitch: % 7.2f, yaw: % 7.2f\n",acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z);
        fprintf(outputcsv,"%f,%f,%f,%f,%f,%f,%f\n",time,acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z);
        usleep(10000);
        time += 10;
    }
    fclose(outputcsv);
    i2c_close();
}
