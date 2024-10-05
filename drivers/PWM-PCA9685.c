#include "../i2c/i2c.h"
#include <unistd.h>
#include <stdint.h>

#define PWM_ADDR 0x40

#define MODE1_REG_ADDR 0x00
#define MODE2_REG_ADDR 0x01
#define MODE1_RESTART 0x80
#define MODE1_SLEEP 0x10
#define MODE1_EXTCLK 0x40
#define PRESCALE_REG 0xFE
#define ALLLED_CHANNEL -1
#define LED0_REG 0x06
#define ALLLED_REG 0xFA
#define MODE1_AUTOINC 0x20

int setPWMFrequency(float pwmFrequency) {
    uint8_t preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3) preScalerVal = 3;
    
    uint8_t mode1Reg;
    int result = 0;
    result += i2c_readRegister(PWM_ADDR,MODE1_REG_ADDR,&mode1Reg);
    mode1Reg = (mode1Reg & ~MODE1_RESTART) | MODE1_SLEEP;
    mode1Reg |= MODE1_EXTCLK;
    result += i2c_writeRegister(PWM_ADDR,MODE1_REG_ADDR,mode1Reg);
    result += i2c_writeRegister(PWM_ADDR,PRESCALE_REG,preScalerVal);

    usleep(5000);
    mode1Reg = (mode1Reg & ~MODE1_RESTART) | MODE1_SLEEP;
    result += i2c_writeRegister(PWM_ADDR,MODE1_REG_ADDR,mode1Reg);
    usleep(5000);
    return result;
}

int writeChannelPWM(int channel, uint16_t phaseBegin, uint16_t phaseEnd) {
    uint8_t regAddress;

    if (channel != ALLLED_CHANNEL) {
        regAddress = LED0_REG + channel * 0x04;
    } else {
        regAddress = ALLLED_REG;
    }

    int result = 0;
    result += i2c_writeRegister(PWM_ADDR,regAddress,(uint8_t)(phaseBegin&0xFF));
    result += i2c_writeRegister(PWM_ADDR,regAddress+1,(uint8_t)(phaseBegin>>8));
    result += i2c_writeRegister(PWM_ADDR,regAddress+2,(uint8_t)(phaseEnd&0xFF));
    result += i2c_writeRegister(PWM_ADDR,regAddress+3,(uint8_t)(phaseEnd>>8));

    return result;
    
}
