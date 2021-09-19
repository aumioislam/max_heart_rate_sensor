#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

// I2C definitions

#define I2C_SLAVE                   0x0703
#define I2C_SMBUS                   0x0720

#define I2C_SMBUS_READ              1
#define I2C_SMBUS_WRITE             0

// SMBus transaction types

#define I2C_SMBUS_QUICK             0
#define I2C_SMBUS_BYTE              1
#define I2C_SMBUS_BYTE_DATA         2 
#define I2C_SMBUS_WORD_DATA         3
#define I2C_SMBUS_PROC_CALL         4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7
#define I2C_SMBUS_I2C_BLOCK_DATA    8

// SMBus sizes

#define I2C_SMBUS_BLOCK_MAX         32

// ioctl structs and union

typedef union i2c_smbus_data_t
{
    uint8_t     byte;
    uint16_t    word;
    uint8_t     block[I2C_SMBUS_BLOCK_MAX + 2];
} i2c_smbus_data;

typedef struct i2c_smbus_ioctl_data_t
{
    char            read_write;
    uint8_t         command;
    int             size;
    i2c_smbus_data  *data;
} i2c_smbus_ioctl_data;

typedef struct i2c_rdwr_ioctl_data i2c_rdwr_ioctl_data;

typedef struct i2c_msg i2c_msg;

extern int i2c_smbus_read_register8(int, uint8_t, uint8_t*);
extern int i2c_smbus_read_register16(int, uint8_t, uint16_t*);
extern int i2c_smbus_write_register8(int, uint8_t, uint8_t);
extern int i2c_smbus_write_register16(int, uint8_t, uint16_t);
extern int i2c_read(int, uint8_t, uint8_t, uint8_t*, uint8_t);
extern int i2c_init(const char*, const uint8_t);

#ifdef __cplusplus
}
#endif

#endif
