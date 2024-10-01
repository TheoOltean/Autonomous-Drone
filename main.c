#include <stdio.h>
#include "i2c/i2c.h"

#define TEST_ADDR 0x60

int main() {
    int result;
    result = i2c_init("/dev/i2c-1");
    if (result != 0) {
        perror("Failed to open i2c buss");
        return -1;
    }

    unsigned char value;
    result = i2c_readRegister(TEST_ADDR,1,&value);
    if (result != 0) {
        perror("Failed to read register from device");
        return -1; 
    }

    return 0;
}
