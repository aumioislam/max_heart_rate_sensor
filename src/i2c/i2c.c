/*
 * Adapted from the wiringPi library included with the Raspbian OS
 * See: https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPiI2C.c
 * This library is used to read and write data between the Raspberry Pi and
 * MAX30101 sensor
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

#include "i2c.h"

/*
 * The SMBus code is taken from the wiringPi library
 * There is not much of a difference between Linux I2C and SMBus
 * Due to the byte limit specified in the SMBus standard, an i2c_read function
 * was implemented using the i2c standard
 * https://www.kernel.org/doc/Documentation/i2c/ was used as reference for the 
 * Linux code
 */
static inline int i2c_smbus_access (int fd, char rw, uint8_t addr,
                                    int size, i2c_smbus_data *data)
{
    i2c_smbus_ioctl_data args;

    args.read_write     = rw;
    args.command        = addr;
    args.size           = size;
    args.data           = data;

    return ioctl(fd, I2C_SMBUS, &args);
}

/*
 * 8-bit and 16-bit read and write functions implemented according to the
 * SMBus standard
 */
int i2c_smbus_read_register8(int fd, uint8_t reg, uint8_t *dest)
{
    i2c_smbus_data data;

    if (i2c_smbus_access(fd, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data)) {
        return -1; 
    } else { 
        *dest = data.byte & 0xFF; 
        return 1; 
    }
}

int i2c_smbus_read_register16(int fd, uint8_t reg, uint16_t *dest)
{
    i2c_smbus_data data;

    if (i2c_smbus_access(fd, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data)) {
        return -1;
    } else {
        *dest = data.word & 0xFFFF;
        return 1;
    }
}

int i2c_smbus_write_register8(int fd, uint8_t reg, uint8_t value)
{
    i2c_smbus_data data;

    data.byte = value;

    return i2c_smbus_access(fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data);
}

int i2c_smbus_write_register16(int fd, uint8_t reg, uint16_t value)
{
    i2c_smbus_data data;

    data.word = value;

    return i2c_smbus_access(fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, &data);
}

/*
 * i2c_read function implemented to read N bytes of data
 * structs used are from the i2c-dev.h header
 * See https://www.kernel.org/doc/Documentation/i2c/writing-clients for more
 * information
 */
int i2c_read(int fd, uint8_t slave_addr, uint8_t reg, uint8_t *buf,
             uint8_t cnt)
{
   uint8_t outbuf;
   i2c_rdwr_ioctl_data data;
   i2c_msg msgs[2];

   outbuf = reg;

   msgs[0].addr  = slave_addr;
   msgs[0].flags = 0;
   msgs[0].len   = sizeof(outbuf);
   msgs[0].buf   = &outbuf;

   msgs[1].addr  = slave_addr;
   msgs[1].flags = I2C_M_RD;
   msgs[1].len   = cnt;
   msgs[1].buf   = buf;

   data.msgs    = msgs;
   data.nmsgs   = 2;

   if (ioctl(fd, I2C_RDWR, &data) < 0)
       return -1;
   else
       return 1;
}

/*
 * I2C initialization
 * Open I2C device, register the target device
 */
int i2c_init(const char *device, const uint8_t device_id)
{
    int fd;

    if ((fd = open(device, O_RDWR)) < 0)
        return -1;
    if (ioctl(fd, I2C_SLAVE, device_id) < 0)
        return -1;

    return fd;
}
