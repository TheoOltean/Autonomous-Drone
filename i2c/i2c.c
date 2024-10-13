#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

int i2c_fd;
typedef struct i2c_msg i2c_msg;
typedef struct i2c_rdwr_ioctl_data i2c_ioctl_data;

int i2c_init(const char *fname) {
   i2c_fd = open(fname,O_RDWR);
   if (i2c_fd < 0) {
       return -1;
   }
   return 0;
}

void i2c_close(void) {
    close(i2c_fd);
}

int i2c_readRegister(unsigned char address, unsigned char reg, unsigned char *result) {
    struct i2c_msg messages[2];
    messages[0] = (i2c_msg){.addr = address, .flags = 0, .len = 1, .buf = &reg};
    messages[1] = (i2c_msg){.addr = address, .flags = I2C_M_RD | I2C_M_NOSTART, .len = 1, .buf = result};

    i2c_ioctl_data transmission[1];
    transmission[0] = (i2c_ioctl_data){.msgs = messages, .nmsgs = 2};

    if (ioctl(i2c_fd,I2C_RDWR,&transmission) < 0) {
        return -1;
    }
    
    return 0;
}

int i2c_writeRegister(unsigned char address, unsigned char reg, unsigned char value) {
    uint8_t out[2] = {reg, value};
    i2c_msg messages[1];
    messages[0] = (i2c_msg){.addr = address, .flags = 0, .len = 2, .buf = out};

    i2c_ioctl_data transmission[1];
    transmission[0] = (i2c_ioctl_data){.msgs = messages, .nmsgs = 1};

    if (ioctl(i2c_fd,I2C_RDWR,&transmission) < 0) {
        return -1;
    }

    return 0;
}

int i2c_getfd() {
    return i2c_fd;
}
