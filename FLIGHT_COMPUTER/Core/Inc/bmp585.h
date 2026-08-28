#ifndef INC_BMP585_H_
#define INC_BMP585_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* I2C address — SDO=GND → 7-bit 0x46, SDO=VDD → 7-bit 0x47. HAL takes (addr << 1) */
#define BMP585_I2C_ADDR_SDO_GND   (0x46 << 1)
#define BMP585_I2C_ADDR_SDO_VDD   (0x47 << 1)

/* Register addresses (from datasheet Rev 1.2) */
#define BMP585_REG_CHIP_ID        0x01
#define BMP585_REG_REV_ID         0x02
#define BMP585_REG_CHIP_STATUS    0x11
#define BMP585_REG_DRIVE_CONFIG   0x13
#define BMP585_REG_INT_CONFIG     0x14
#define BMP585_REG_INT_SOURCE     0x15
#define BMP585_REG_FIFO_CONFIG    0x16
#define BMP585_REG_FIFO_COUNT     0x17
#define BMP585_REG_FIFO_SEL       0x18
#define BMP585_REG_TEMP_XLSB      0x1D   /* burst-read 6 bytes from here */
#define BMP585_REG_PRESS_XLSB     0x20
#define BMP585_REG_INT_STATUS     0x27   /* clear-on-read */
#define BMP585_REG_STATUS         0x28
#define BMP585_REG_DSP_CONFIG     0x30
#define BMP585_REG_DSP_IIR        0x31
#define BMP585_REG_OOR_THR_LSB    0x32
#define BMP585_REG_OOR_THR_MSB    0x33
#define BMP585_REG_OOR_RANGE      0x34
#define BMP585_REG_OOR_CONFIG     0x35
#define BMP585_REG_OSR_CONFIG     0x36
#define BMP585_REG_ODR_CONFIG     0x37
#define BMP585_REG_OSR_EFF        0x38
#define BMP585_REG_CMD            0x7E

/* Chip ID (Rev 1.1+ = 0x51, earlier silicon = 0x50) */
#define BMP585_CHIP_ID_V1         0x50
#define BMP585_CHIP_ID_V2         0x51

/* Soft reset command */
#define BMP585_CMD_SOFT_RESET     0xB6

/*
 * OSR_CONFIG (0x36):
 *   bit[7]   = reserved
 *   bit[6]   = press_en   (1 = enable pressure)
 *   bits[5:3]= osr_p      (000=1x, 001=2x, 010=4x, 011=8x, ...)
 *   bits[2:0]= osr_t      (000=1x, 001=2x, ...)
 *
 * 0x40 = press_en=1, osr_p=1x, osr_t=1x  (fastest, lowest noise)
 */
#define BMP585_OSR_PRESS_EN_1X_1X 0x40

/*
 * ODR_CONFIG (0x37):
 *   bit[7]   = deep_dis
 *   bits[6:2]= odr  (0x14 = 25 Hz)
 *   bits[1:0]= pwr_mode  (00=Standby, 01=Normal, 10=Forced, 11=Non-Stop)
 *
 * 0x51 = deep_dis=0, odr=0x14(25Hz), pwr_mode=01(Normal)
 */
#define BMP585_ODR_NORMAL_25HZ    0x51

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            addr;
    float              temperature;   /* degrees C */
    float              pressure;      /* Pa */
    uint8_t            chip_id;       /* captured during init */
    uint8_t            boot_status;   /* STATUS(0x28) right after reset:
                                         bit1=NVM ready, bit2=NVM error —
                                         healthy boot shows xxxx x01x */
} BMP585_t;

typedef enum {
    BMP585_OK       = 0,
    BMP585_ERR      = 1,
    BMP585_WRONG_ID = 2,
} BMP585_Status;

BMP585_Status BMP585_Init(BMP585_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);
BMP585_Status BMP585_ReadData(BMP585_t *dev);
BMP585_Status BMP585_ScanI2C(I2C_HandleTypeDef *hi2c, uint8_t *found_addr);

#endif /* INC_BMP585_H_ */
