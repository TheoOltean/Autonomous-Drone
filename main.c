#include <stdint.h>
#include <stdio.h>
#include "i2c/i2c.h"

int main() {
    int result;
    result = i2c_init("/dev/i2c-1");
    if (result != 0) {
        perror("Failed to open i2c buss");
        return -1;
    }

//    uint8_t value;
//    result = i2c_readRegister(TEST_ADDR,1,&value);
//    if (result != 0) {
//        perror("Failed to read register from device");
//        return -1; 
//    }
//    uint8_t buffer;
//    if (i2c_readRegister(0x69, 0, &buffer)) {
//        perror("Failed to open i2c bus");
//        return -1;
//    }
    
    imu_autoOffsets();
    imu_setAccRange(ICM20948_ACC_RANGE_2G);
    imu_setAccDLPF(ICM20948_DLPF_6);
    imu_setAccSampleRateDivider(10);

    for (;;) {
        imu_data data = imu_readSenser();
         
    }
}
