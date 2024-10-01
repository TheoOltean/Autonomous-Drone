#ifndef I2C
#define I2C

int i2c_init(const char *fname);

//theoretically not necessary
int i2c_close(void);

int i2c_readRegister(unsigned char address, unsigned char reg, unsigned char *result);

int i2c_writeRegister(unsigned char address, unsigned char reg, unsigned char value); 

#endif
