#include "bno055.h"

/* Registers (page 0) */
#define REG_CHIP_ID    0x00   /* reads 0xA0 */
#define REG_PAGE_ID    0x07
#define REG_EUL_DATA   0x1A   /* heading, roll, pitch — 6 bytes, int16 LE /16 */
#define REG_CALIB      0x35
#define REG_UNIT_SEL   0x3B
#define REG_OPR_MODE   0x3D
#define REG_PWR_MODE   0x3E
#define REG_SYS_TRIG   0x3F

#define CHIP_ID        0xA0
#define MODE_CONFIG    0x00
#define MODE_NDOF      0x0C   /* 9-DoF fusion, absolute heading */

#define I2C_TIMEOUT    30u

static BNO055_Status w8(BNO055_t *d, uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(d->hi2c, d->addr, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, I2C_TIMEOUT) == HAL_OK)
           ? BNO055_OK : BNO055_ERR;
}

static uint8_t r8(BNO055_t *d, uint8_t reg)
{
    uint8_t v = 0;
    HAL_I2C_Mem_Read(d->hi2c, d->addr, reg, I2C_MEMADD_SIZE_8BIT,
                     &v, 1, I2C_TIMEOUT);
    return v;
}

BNO055_Status BNO055_Init(BNO055_t *dev, I2C_HandleTypeDef *hi2c,
                          uint8_t addr7, uint8_t ext_crystal)
{
    dev->hi2c    = hi2c;
    dev->addr    = (uint8_t)(addr7 << 1);
    dev->heading = dev->roll = dev->pitch = 0.0f;
    dev->cal_sys = dev->cal_gyro = dev->cal_acc = dev->cal_mag = 0;

    /* Wait up to 1 s for the chip to answer with its ID */
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 1000) {
        if (r8(dev, REG_CHIP_ID) == CHIP_ID) break;
        HAL_Delay(20);
    }
    if (r8(dev, REG_CHIP_ID) != CHIP_ID) return BNO055_WRONG_ID;

    w8(dev, REG_PAGE_ID, 0x00);
    w8(dev, REG_OPR_MODE, MODE_CONFIG);
    HAL_Delay(25);

    /* Soft reset, then wait for it to re-enumerate (~650 ms boot) */
    w8(dev, REG_SYS_TRIG, 0x20);
    HAL_Delay(700);
    t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 1000) {
        if (r8(dev, REG_CHIP_ID) == CHIP_ID) break;
        HAL_Delay(10);
    }

    w8(dev, REG_PWR_MODE, 0x00);                            /* normal power  */
    HAL_Delay(10);
    w8(dev, REG_SYS_TRIG, ext_crystal ? 0x80 : 0x00);       /* clock source  */
    HAL_Delay(10);
    w8(dev, REG_UNIT_SEL, 0x00);        /* accel m/s^2, gyro dps, euler deg  */
    HAL_Delay(10);
    w8(dev, REG_OPR_MODE, MODE_NDOF);                       /* fusion on     */
    HAL_Delay(25);

    return BNO055_OK;
}

BNO055_Status BNO055_ReadEuler(BNO055_t *dev)
{
    uint8_t b[6];
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->addr, REG_EUL_DATA,
                         I2C_MEMADD_SIZE_8BIT, b, 6, I2C_TIMEOUT) != HAL_OK)
        return BNO055_ERR;

    int16_t h = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    int16_t r = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
    int16_t p = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    dev->heading = h / 16.0f;
    dev->roll    = r / 16.0f;
    dev->pitch   = p / 16.0f;
    return BNO055_OK;
}

void BNO055_ReadCalib(BNO055_t *dev)
{
    uint8_t c = r8(dev, REG_CALIB);
    dev->cal_sys  = (c >> 6) & 3;
    dev->cal_gyro = (c >> 4) & 3;
    dev->cal_acc  = (c >> 2) & 3;
    dev->cal_mag  =  c       & 3;
}
