#include "IMU-ICM-20948.h"
#include "../i2c/i2c.h"
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>


#define AK09916_ADDRESS                0x0C;
#define ICM20948_ADDR                  0x69
/* Registers ICM20948 USER BANK 0 */
#define ICM20948_WHO_AM_I              0x00
#define ICM20948_USER_CTRL             0x03
#define ICM20948_LP_CONFIG             0x05
#define ICM20948_PWR_MGMT_1            0x06
#define ICM20948_PWR_MGMT_2            0x07
#define ICM20948_INT_PIN_CFG           0x0F
#define ICM20948_INT_ENABLE            0x10
#define ICM20948_INT_ENABLE_1          0x11
#define ICM20948_INT_ENABLE_2          0x12
#define ICM20948_INT_ENABLE_3          0x13
#define ICM20948_I2C_MST_STATUS        0x17
#define ICM20948_INT_STATUS            0x19
#define ICM20948_INT_STATUS_1          0x1A
#define ICM20948_INT_STATUS_2          0x1B
#define ICM20948_INT_STATUS_3          0x1C
#define ICM20948_DELAY_TIME_H          0x28
#define ICM20948_DELAY_TIME_L          0x29
#define ICM20948_ACCEL_OUT             0x2D // accel data registers begin
#define ICM20948_GYRO_OUT              0x33 // gyro data registers begin
#define ICM20948_TEMP_OUT              0x39
#define ICM20948_EXT_SLV_SENS_DATA_00  0x3B
#define ICM20948_EXT_SLV_SENS_DATA_01  0x3C
#define ICM20948_FIFO_EN_1             0x66
#define ICM20948_FIFO_EN_2             0x67
#define ICM20948_FIFO_RST              0x68
#define ICM20948_FIFO_MODE             0x69
#define ICM20948_FIFO_COUNT            0x70
#define ICM20948_FIFO_R_W              0x72
#define ICM20948_DATA_RDY_STATUS       0x74
#define ICM20948_FIFO_CFG              0x76

/* Registers ICM20948 USER BANK 1 */
#define ICM20948_SELF_TEST_X_GYRO      0x02
#define ICM20948_SELF_TEST_Y_GYRO      0x03
#define ICM20948_SELF_TEST_Z_GYRO      0x04
#define ICM20948_SELF_TEST_X_ACCEL     0x0E
#define ICM20948_SELF_TEST_Y_ACCEL     0x0F
#define ICM20948_SELF_TEST_Z_ACCEL     0x10
#define ICM20948_XA_OFFS_H             0x14
#define ICM20948_XA_OFFS_L             0x15
#define ICM20948_YA_OFFS_H             0x17
#define ICM20948_YA_OFFS_L             0x18
#define ICM20948_ZA_OFFS_H             0x1A
#define ICM20948_ZA_OFFS_L             0x1B
#define ICM20948_TIMEBASE_CORR_PLL     0x28

/* Registers ICM20948 USER BANK 2 */
#define ICM20948_GYRO_SMPLRT_DIV       0x00
#define ICM20948_GYRO_CONFIG_1         0x01
#define ICM20948_GYRO_CONFIG_2         0x02
#define ICM20948_XG_OFFS_USRH          0x03
#define ICM20948_XG_OFFS_USRL          0x04
#define ICM20948_YG_OFFS_USRH          0x05
#define ICM20948_YG_OFFS_USRL          0x06
#define ICM20948_ZG_OFFS_USRH          0x07
#define ICM20948_ZG_OFFS_USRL          0x08
#define ICM20948_ODR_ALIGN_EN          0x09
#define ICM20948_ACCEL_SMPLRT_DIV_1    0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2    0x11
#define ICM20948_ACCEL_INTEL_CTRL      0x12
#define ICM20948_ACCEL_WOM_THR         0x13
#define ICM20948_ACCEL_CONFIG          0x14
#define ICM20948_ACCEL_CONFIG_2        0x15
#define ICM20948_FSYNC_CONFIG          0x52
#define ICM20948_TEMP_CONFIG           0x53
#define ICM20948_MOD_CTRL_USR          0x54

/* Registers ICM20948 USER BANK 3 */
#define ICM20948_I2C_MST_ODR_CFG       0x00
#define ICM20948_I2C_MST_CTRL          0x01
#define ICM20948_I2C_MST_DELAY_CTRL    0x02
#define ICM20948_I2C_SLV0_ADDR         0x03
#define ICM20948_I2C_SLV0_REG          0x04
#define ICM20948_I2C_SLV0_CTRL         0x05
#define ICM20948_I2C_SLV0_DO           0x06

/* Registers ICM20948 ALL BANKS */
#define ICM20948_REG_BANK_SEL          0x7F

/* Registers AK09916 */
#define AK09916_WIA_1                  0x00 // Who I am, Company ID
#define AK09916_WIA_2                  0x01 // Who I am, Device ID
#define AK09916_STATUS_1               0x10
#define AK09916_HXL                    0x11
#define AK09916_HXH                    0x12
#define AK09916_HYL                    0x13
#define AK09916_HYH                    0x14
#define AK09916_HZL                    0x15
#define AK09916_HZH                    0x16
#define AK09916_STATUS_2               0x18
#define AK09916_CNTL_2                 0x31
#define AK09916_CNTL_3                 0x32

/* Register Bits */
#define ICM20948_RESET                 0x80
#define ICM20948_I2C_MST_EN            0x20
#define ICM20948_SLEEP                 0x40
#define ICM20948_LP_EN                 0x20
#define ICM20948_BYPASS_EN             0x02
#define ICM20948_GYR_EN                0x07
#define ICM20948_ACC_EN                0x38
#define ICM20948_FIFO_EN               0x40
#define ICM20948_INT1_ACTL             0x80
#define ICM20948_INT_1_LATCH_EN        0x20
#define ICM20948_ACTL_FSYNC            0x08
#define ICM20948_INT_ANYRD_2CLEAR      0x10
#define ICM20948_FSYNC_INT_MODE_EN     0x06
#define AK09916_16_BIT                 0x10
#define AK09916_OVF                    0x08
#define AK09916_READ                   0x80

/* Others */
#define AK09916_WHO_AM_I_1             0x4809
#define AK09916_WHO_AM_I_2             0x0948
#define ICM20948_WHO_AM_I_CONTENT      0xEA
#define ICM20948_ROOM_TEMP_OFFSET      0.0
#define ICM20948_T_SENSITIVITY         333.87
#define AK09916_MAG_LSB                0.1495
#define ICM20948_I2C_MST_RST           0x02

#define ICM20948_GYRO_RANGE            0 
#define ICM20948_ACC_RANGE             0

#define CALIB_SAMPLE_CNT               50
imu_data state;
uint8_t data[20];

int imu_setBank(uint8_t bank) {
    return i2c_writeRegister(ICM20948_ADDR, 127, bank<<4);
}

int imu_init() {
    imu_setBank(0);
    if (i2c_writeRegister(ICM20948_ADDR, ICM20948_PWR_MGMT_1, ICM20948_RESET)){
        perror("Failed to open i2c bus");
        return -1;
    }
    imu_setBank(2);
    i2c_writeRegister(ICM20948_ADDR, ICM20948_ODR_ALIGN_EN, 1);

    state = (imu_data){};
    state.accCorrFactor = (vec3){.x=1,.y=1,.z=1};
    return 0;
}

void imu_enableAcc() {
    imu_setBank(0);
    uint8_t regVal;
    i2c_readRegister(ICM20948_ADDR, ICM20948_PWR_MGMT_2, &regVal);
    regVal &= ~ICM20948_ACC_EN;
    i2c_writeRegister(ICM20948_ADDR, ICM20948_PWR_MGMT_2, regVal);
    i2c_writeRegister(ICM20948_ADDR, ICM20948_USER_CTRL, 0);
    i2c_writeRegister(ICM20948_ADDR, ICM20948_PWR_MGMT_1, 0);
    imu_setBank(2);
    i2c_writeRegister(ICM20948_ADDR, ICM20948_ACCEL_SMPLRT_DIV_1, 0);
    i2c_writeRegister(ICM20948_ADDR, ICM20948_ACCEL_SMPLRT_DIV_2, 0);
}

void imu_readData() {
    imu_setBank(0);

    for (int i = 0; i < 20; ++i) {
        i2c_readRegister(ICM20948_ADDR, ICM20948_ACCEL_OUT+i, data+i);
//        printf("%d ", data[i]);
    }
//    printf("\n");
}

vec3 imu_getAccRawValues() {
    vec3 result;
    result.x = (int16_t)((data[0] << 8) | data[1]);
    result.y = (int16_t)((data[2] << 8) | data[3]);
    result.z = (int16_t)((data[4] << 8) | data[5]);
    return result;
}

vec3 imu_getAccCorrectedValues() {
    vec3 result = imu_getAccRawValues();
    result.x -= state.accOffsetVal.x;
    result.y -= state.accOffsetVal.y;
    result.z -= state.accOffsetVal.z;
    result.x = 2*result.x/32768;
    result.y = 2*result.y/32768;
    result.z = 2*result.z/32768;
    return result;
}

vec3 imu_getGyroRawValues() {
    vec3 result;
    result.x = (int16_t)((data[6] << 8) | data[7]);
    result.y = (int16_t)((data[8] << 8) | data[9]);
    result.z = (int16_t)((data[10] << 8) | data[11]);
    return result;
}

vec3 imu_getGyroValues() {
    vec3 result = imu_getGyroRawValues();
    result.x = 250*result.x/32768;
    result.y = 250*result.y/32768;
    result.z = 250*result.z/32768;
    return result;
}

float imu_getTemperature() {
    int16_t rawTemp = ((data[12] << 8) | data[13]);
    return (rawTemp*1.0 - ICM20948_ROOM_TEMP_OFFSET)/ICM20948_T_SENSITIVITY + 21.0;
}

void imu_setAccSampleRateDivider(uint8_t value) {
    imu_setBank(2);
    i2c_writeRegister(ICM20948_ADDR, ICM20948_ACCEL_SMPLRT_DIV_1, value);
}

void imu_autoOffsets() {
    imu_setBank(2);
    //setGyroDLPF(ICM20948_DLPF_6)
    uint8_t regVal;
    i2c_readRegister(ICM20948_ADDR, ICM20948_GYRO_CONFIG_1, &regVal);
    regVal |= 0x01;
    regVal &= 0xC7;
    regVal |= (6<<3);
    //setGyroRange(ICM20948_GYRO_RANGE)
    regVal &= ~(0x06);
    regVal |= (ICM20948_GYRO_RANGE<<1);
    i2c_writeRegister(ICM20948_ADDR, ICM20948_GYRO_CONFIG_1, regVal);
    state.gyroRangeFactor = (1<<ICM20948_GYRO_RANGE);
    //setAccDLPF(ICM20948_DLPF_6)
    i2c_readRegister(ICM20948_ADDR, ICM20948_ACCEL_CONFIG, &regVal);
    regVal |= 0x01;
    regVal &= 0xC7;
    regVal |= (6<<3);
    //setAccRange(ICM20948_ACC_RANGE)
    regVal &= ~(0x06);
    regVal |= (ICM20948_ACC_RANGE<<1);
    i2c_writeRegister(ICM20948_ADDR, ICM20948_ACCEL_CONFIG, regVal);
    state.accRangeFactor = (1<<ICM20948_ACC_RANGE);
    //
    usleep(1000000);
    for (int i = 0; i < 10; ++i) {
        imu_readData();
        usleep(10000);
    }
    
    state.accOffsetVal = (vec3){};
    for (int i = 0; i < CALIB_SAMPLE_CNT; ++i) {
        imu_readData();
        state.accOffsetVal.x += ((data[0] << 8) | data[1]);
        state.accOffsetVal.y += ((data[2] << 8) | data[3]);
        state.accOffsetVal.z += ((data[4] << 8) | data[5]);
        usleep(10000);
    }

    state.accOffsetVal.x /= CALIB_SAMPLE_CNT;
    state.accOffsetVal.y /= CALIB_SAMPLE_CNT;
    state.accOffsetVal.z /= CALIB_SAMPLE_CNT;

    state.gyroOffsetVal = (vec3){};
    for (int i = 0; i < CALIB_SAMPLE_CNT; ++i) {
        imu_readData();
        state.gyroOffsetVal.x += ((data[6] << 8) | data[7]);
        state.gyroOffsetVal.y += ((data[8] << 8) | data[9]);
        state.gyroOffsetVal.z += ((data[10] << 8) | data[11]);
        usleep(1000);
    }

    state.gyroOffsetVal.x /= CALIB_SAMPLE_CNT;
    state.gyroOffsetVal.y /= CALIB_SAMPLE_CNT;
    state.gyroOffsetVal.z /= CALIB_SAMPLE_CNT;
}
