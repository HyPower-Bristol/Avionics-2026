#include "adxl375.h"
#include <math.h>

/* Registers (ADXL345/375 family) */
#define REG_DEVID        0x00
#define REG_BW_RATE      0x2C
#define REG_POWER_CTL    0x2D
#define REG_DATA_FORMAT  0x31
#define REG_DATAX0       0x32   /* X0,X1,Y0,Y1,Z0,Z1 — 6 bytes, int16 LE */

/* ADXL375 sensitivity: 20.5 LSB/g typical (fixed ±200 g full-res) */
#define G_PER_LSB   (1.0f / 20.5f)

#define I2C_TIMEOUT  30u

static ADXL375_Status w8(ADXL375_t *d, uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(d->hi2c, d->addr, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, I2C_TIMEOUT) == HAL_OK)
           ? ADXL375_OK : ADXL375_ERR;
}

ADXL375_Status ADXL375_Init(ADXL375_t *dev, I2C_HandleTypeDef *hi2c,
                            uint8_t addr7)
{
    dev->hi2c = hi2c;
    dev->addr = (uint8_t)(addr7 << 1);
    dev->raw[0] = dev->raw[1] = dev->raw[2] = 0;
    dev->g[0] = dev->g[1] = dev->g[2] = 0.0f;
    dev->mag_g = 0.0f;

    /* Verify device ID */
    dev->dev_id = 0;
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->addr, REG_DEVID, I2C_MEMADD_SIZE_8BIT,
                         &dev->dev_id, 1, I2C_TIMEOUT) != HAL_OK)
        return ADXL375_ERR;
    if (dev->dev_id != ADXL375_DEVID)
        return ADXL375_WRONG_ID;

    /* DATA_FORMAT = 0x0B: FULL_RES=1, range=11 (±200 g), right-justified */
    if (w8(dev, REG_DATA_FORMAT, 0x0B) != ADXL375_OK) return ADXL375_ERR;
    /* BW_RATE = 0x0D: 800 Hz output data rate (fresh data at any poll) */
    if (w8(dev, REG_BW_RATE, 0x0D) != ADXL375_OK) return ADXL375_ERR;
    /* POWER_CTL = 0x08: Measure mode (leave standby) */
    if (w8(dev, REG_POWER_CTL, 0x08) != ADXL375_OK) return ADXL375_ERR;
    HAL_Delay(10);

    return ADXL375_OK;
}

ADXL375_Status ADXL375_Read(ADXL375_t *dev)
{
    uint8_t b[6];
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->addr, REG_DATAX0, I2C_MEMADD_SIZE_8BIT,
                         b, 6, I2C_TIMEOUT) != HAL_OK)
        return ADXL375_ERR;

    dev->raw[0] = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    dev->raw[1] = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
    dev->raw[2] = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));

    dev->g[0] = dev->raw[0] * G_PER_LSB;
    dev->g[1] = dev->raw[1] * G_PER_LSB;
    dev->g[2] = dev->raw[2] * G_PER_LSB;
    dev->mag_g = sqrtf(dev->g[0]*dev->g[0] +
                       dev->g[1]*dev->g[1] +
                       dev->g[2]*dev->g[2]);
    return ADXL375_OK;
}
