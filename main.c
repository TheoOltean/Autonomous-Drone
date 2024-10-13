#include <Fusion/FusionAhrs.h>
#include <Fusion/FusionMath.h>
#include <stdio.h>
#include <Fusion/Fusion.h>
#include "i2c/i2c.h"
#include "drivers/IMU-ICM-20948.h"
#include <unistd.h>
#include <stdlib.h>
#include <lgpio.h>
#include <stdatomic.h>

void runIMU() {
    int result;
    uint8_t whoami;
    i2c_readRegister(0x69,0,&whoami);
    printf("%d\n",whoami);
    result = i2c_writeRegister(0x69,15,12);
    if (result != 0) {
        perror("Failed to write i2c");
        exit(-1);
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
        float temperature = imu_getTemperature();
        //FusionAhrsUpdateNoMagnetometer(&ahrs, gyro.fusionVec, acc.fusionVec, 1.0f);

        //FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        //FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        //printf("accx: %f, accy: %f, accz: %f, x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f\n",acc.x,acc.y,acc.z,earth.axis.x,earth.axis.y,earth.axis.z,euler.angle.roll,euler.angle.pitch,euler.angle.yaw);
        printf("x: % 7.4f, y: % 7.4f, z: % 7.4f, roll: % 7.2f, pitch: % 7.2f, yaw: % 7.2f, temperature: %f\n",acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,temperature);
        //fprintf(outputcsv,"%f,%f,%f,%f,%f,%f,%f\n",time,acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z);
        usleep(10000);
        time += 10;
    }
    fclose(outputcsv);
}

atomic_int power;

#define PWM_FLAGS 0
#define PWM_PIN_0 17 
#define PWM_PIN_1 16
#define PWM_PIN_2 17
#define PWM_PIN_3 24

void *pwm_run() {
    int handle = lgGpiochipOpen(0);
    lgGpioClaimOutput(handle,0,PWM_PIN_0,0);
    
    lgTxPwm(handle,PWM_PIN_0,50.0,5.0,0,0);
    usleep(100000);
    lgTxPwm(handle,PWM_PIN_0,50.0,10.0,0,0);
    usleep(100000);

    for (;;) {
        printf("%d\n",atomic_load(&power));
        lgTxPwm(handle,PWM_PIN_0,50.0,5.0+((float)atomic_load(&power)/(float)100)*5.0,PWM_FLAGS,0);
        //lgTxPwm(handle,PWM_PIN_0,50.0,5.5,0,0);
        usleep(20000);
    }
    return NULL;
}

void pwm_setPower(int p) {
    if (p > 100 || p < 0) return;
    atomic_store(&power,p);
}

int main() {
    //int result;
    //result = i2c_init("/dev/i2c-1");
    //if (result != 0) {
    //    perror("Failed to open i2c buss");
    //    return -1;
    //}

    //int phases = pwm_getChannel(12);
    //printf("%d %d\n",phases&0xFFFF,phases>>16);
    //i2c_close();

    pwm_setPower(0);
    pthread_t pwmthread;
    struct sched_param param;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    param.sched_priority = 99;

    pthread_attr_setschedpolicy(&attr,SCHED_FIFO);
    pthread_attr_setschedparam(&attr,&param);

    pthread_create(&pwmthread,&attr,pwm_run,NULL);
    usleep(2000000);
    pwm_setPower(8);
    usleep(5000000);
    pwm_setPower(15);
    usleep(5000000);
    pwm_setPower(30);
    usleep(100000000);
    return 0;
}
