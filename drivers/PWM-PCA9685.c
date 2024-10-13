#include "../i2c/i2c.h"
#include <unistd.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define PCA9685_ADDRESS                 0x40
#define PCA9685_ADRMASK                 0x3F
#define PCA9685_PROXY_ADDRESS           0xE0
#define PCA9685_PROXY_ADRMASK           0xFE

// Register addresses from data sheet
#define PCA9685_MODE1_REG               0x00
#define PCA9685_MODE2_REG               0x01
#define PCA9685_SUBADR1_REG             0x02
#define PCA9685_SUBADR2_REG             0x03
#define PCA9685_SUBADR3_REG             0x04
#define PCA9685_ALLCALL_REG             0x05
#define PCA9685_LED0_REG                0x06          // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG            0xFE
#define PCA9685_ALLLED_REG              0xFA

// Mode1 register values
#define PCA9685_MODE1_RESTART           0x80
#define PCA9685_MODE1_EXTCLK            0x40
#define PCA9685_MODE1_AUTOINC           0x20
#define PCA9685_MODE1_SLEEP             0x10
#define PCA9685_MODE1_SUBADR1           0x08
#define PCA9685_MODE1_SUBADR2           0x04
#define PCA9685_MODE1_SUBADR3           0x02
#define PCA9685_MODE1_ALLCALL           0x01

// Mode2 register values
#define PCA9685_MODE2_OUTDRV_TPOLE      0x04
#define PCA9685_MODE2_INVRT             0x10
#define PCA9685_MODE2_OUTNE_TPHIGH      0x01
#define PCA9685_MODE2_OUTNE_HIGHZ       0x02
#define PCA9685_MODE2_OCH_ONACK         0x08

#define PCA9685_SW_RESET                0x06          // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_PWM_FULL                (uint16_t)0x1000    // Special value for full on/full off LEDx modes
#define PCA9685_PWM_MASK                (uint16_t)0x0FFF    // Mask for 12-bit/4096 possible phase positions

#define PCA9685_CHANNEL_COUNT           16
#define PCA9685_MIN_CHANNEL             0
#define PCA9685_MAX_CHANNEL             (PCA9685_CHANNEL_COUNT - 1)
#define PCA9685_ALLLED_CHANNEL          -1                  // Special value for ALLLED registers

enum PCA9685_OutputDriverMode {
    PCA9685_OutputDriverMode_OpenDrain,         // Module outputs in an open-drain (aka direct connection) style structure with 400mA @5v total sink current, useful for LEDs and low-power Servos
    PCA9685_OutputDriverMode_TotemPole,         // Module outputs in a totem-pole (aka push-pull) style structure with 400mA @5v total sink current and 160mA total source current, useful for external drivers (default)

    PCA9685_OutputDriverMode_Count,             // Internal use only
    PCA9685_OutputDriverMode_Undefined = -1     // Internal use only
};

enum PCA9685_OutputEnabledMode {
    PCA9685_OutputEnabledMode_Normal,           // When OE is enabled/LOW, channels output a normal signal, useful for N-type external drivers (default)
    PCA9685_OutputEnabledMode_Inverted,         // When OE is enabled/LOW, channels output an inverted signal, useful for P-type external drivers or direct connection

    PCA9685_OutputEnabledMode_Count,            // Internal use only
    PCA9685_OutputEnabledMode_Undefined = -1    // Internal use only
};

enum PCA9685_OutputDisabledMode {
    PCA9685_OutputDisabledMode_Low,             // When OE is disabled/HIGH, channels output a LOW signal (default)
    PCA9685_OutputDisabledMode_High,            // When OE is disabled/HIGH, channels output a HIGH signal (only available in totem-pole mode)
    PCA9685_OutputDisabledMode_Floating,        // When OE is disabled/HIGH, channel outputs go into a floating (aka high-impedance/high-Z) state, which may be further refined via external pull-up/pull-down resistors

    PCA9685_OutputDisabledMode_Count,           // Internal use only
    PCA9685_OutputDisabledMode_Undefined = -1   // Internal use only
};

enum PCA9685_ChannelUpdateMode {
    PCA9685_ChannelUpdateMode_AfterStop,        // Channel updates commit after full-transmission STOP signal (default)
    PCA9685_ChannelUpdateMode_AfterAck,         // Channel updates commit after individual channel update ACK signal

    PCA9685_ChannelUpdateMode_Count,            // Internal use only
    PCA9685_ChannelUpdateMode_Undefined = -1    // Internal use only
};

enum PCA9685_PhaseBalancer {
    PCA9685_PhaseBalancer_None,                 // Disables software-based phase balancing, relying on installed hardware to handle current sinkage (default)
    PCA9685_PhaseBalancer_Linear,               // Uses linear software-based phase balancing, with each channel being a preset 16 steps (out of the 4096/12-bit value range) away from previous channel (may cause LED flickering/skipped-cycles on PWM changes)

    PCA9685_PhaseBalancer_Count,                // Internal use only
    PCA9685_PhaseBalancer_Undefined = -1        // Internal use only
};

void pwm_init() {
    //reset
    i2c_writeRegister(PCA9685_ADDRESS,PCA9685_MODE1_REG,PCA9685_SW_RESET);
    usleep(1000);

    enum PCA9685_OutputDriverMode driverMode = PCA9685_OutputDriverMode_TotemPole;
    enum PCA9685_OutputEnabledMode enabledMode = PCA9685_OutputEnabledMode_Normal;
    enum PCA9685_OutputDisabledMode disabledMode = PCA9685_OutputDisabledMode_Low;
    enum PCA9685_ChannelUpdateMode updateMode = PCA9685_ChannelUpdateMode_AfterStop;
    enum PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_None;

    uint8_t mode2Val = 0;

    if (driverMode == PCA9685_OutputDriverMode_TotemPole) {
        mode2Val |= PCA9685_MODE2_OUTDRV_TPOLE;
    }

    if (enabledMode == PCA9685_OutputEnabledMode_Inverted) {
        mode2Val |= PCA9685_MODE2_INVRT;
    } 

    if (disabledMode == PCA9685_OutputDisabledMode_High) {
        mode2Val |= PCA9685_MODE2_OUTNE_TPHIGH;
    } else if (disabledMode == PCA9685_OutputDisabledMode_Floating) {
        mode2Val |= PCA9685_MODE2_OUTNE_HIGHZ;
    }

    if (updateMode == PCA9685_ChannelUpdateMode_AfterAck) {
        mode2Val |= PCA9685_MODE2_OCH_ONACK;
    }

    i2c_writeRegister(PCA9685_ADDRESS,PCA9685_MODE1_REG,PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC);
    i2c_writeRegister(PCA9685_ADDRESS,PCA9685_MODE2_REG,mode2Val);
}

void pwm_setFrequency(float pwmFrequency) {
    //23.84Hz-1525.88Hz
    int preScaleVal = (25000000 / (4096 * pwmFrequency)) - 1;
    if (preScaleVal > 255) preScaleVal = 255;
    if (preScaleVal < 3) preScaleVal = 3;
    uint8_t mode1Reg;
    i2c_readRegister(PCA9685_ADDRESS,PCA9685_MODE1_REG,&mode1Reg);
    i2c_writeRegister(PCA9685_ADDRESS, PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    i2c_writeRegister(PCA9685_ADDRESS, PCA9685_PRESCALE_REG, (uint8_t)preScaleVal);

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    i2c_writeRegister(PCA9685_ADDRESS, PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    usleep(500);
    //uint8_t preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
    //if (preScalerVal > 255) preScalerVal = 255;
    //if (preScalerVal < 3) preScalerVal = 3;
    //
    //uint8_t mode1Reg;
    //int result = 0;
    //result += i2c_readRegister(PCA9685_ADDRESS,MODE1_REG_ADDR,&mode1Reg);
    //mode1Reg = (mode1Reg & ~MODE1_RESTART) | MODE1_SLEEP;
    //mode1Reg |= MODE1_EXTCLK;
    //result += i2c_writeRegister(PCA9685_ADDRESS,MODE1_REG_ADDR,mode1Reg);
    //result += i2c_writeRegister(PCA9685_ADDRESS,PRESCALE_REG,preScalerVal);

    //usleep(5000);
    //mode1Reg = (mode1Reg & ~MODE1_RESTART) | MODE1_SLEEP;
    //result += i2c_writeRegister(PCA9685_ADDRESS,MODE1_REG_ADDR,mode1Reg);
    //usleep(5000);
    //return result;
}

int pwm_getChannel(int channel) {
    uint8_t reg= 4*channel + 6;
    uint8_t address = PCA9685_ADDRESS;

    struct i2c_msg messages[5];
    union {
        uint8_t raw[4];
        int value;
    } result;
    messages[0] = (struct i2c_msg){.addr = address, .flags = 0, .len = 1, .buf = &reg};
    messages[1] = (struct i2c_msg){.addr = address, .flags = I2C_M_RD | I2C_M_NOSTART, .len = 1, .buf = result.raw};
    messages[2] = (struct i2c_msg){.addr = address, .flags = I2C_M_RD | I2C_M_NOSTART, .len = 1, .buf = result.raw+1};
    messages[3] = (struct i2c_msg){.addr = address, .flags = I2C_M_RD | I2C_M_NOSTART, .len = 1, .buf = result.raw+2};
    messages[4] = (struct i2c_msg){.addr = address, .flags = I2C_M_RD | I2C_M_NOSTART, .len = 1, .buf = result.raw+3};

    struct i2c_rdwr_ioctl_data transmission[1];
    transmission[0] = (struct i2c_rdwr_ioctl_data){.msgs = messages, .nmsgs = 5};

    if (ioctl(i2c_getfd(),I2C_RDWR,&transmission) < 0) {
        return -1;
    }

    return result.value;
}

int pwm_writeChannel(int channel, uint16_t phaseBegin, uint16_t phaseEnd) {
    uint8_t regAddress = 4*channel + 6;

    int result = 0;
    result += i2c_writeRegister(PCA9685_ADDRESS,regAddress,(uint8_t)(phaseBegin&0xFF));
    result += i2c_writeRegister(PCA9685_ADDRESS,regAddress+1,(uint8_t)(phaseBegin>>8));
    result += i2c_writeRegister(PCA9685_ADDRESS,regAddress+2,(uint8_t)(phaseEnd&0xFF));
    result += i2c_writeRegister(PCA9685_ADDRESS,regAddress+3,(uint8_t)(phaseEnd>>8));

    return result;
}
