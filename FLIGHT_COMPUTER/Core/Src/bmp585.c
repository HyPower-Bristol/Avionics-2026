#include "bmp585.h"

#define I2C_TIMEOUT_MS  100

static BMP585_Status reg_read(BMP585_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT,
                         data, len, I2C_TIMEOUT_MS) != HAL_OK)
        return BMP585_ERR;
    return BMP585_OK;
}

static BMP585_Status reg_write(BMP585_t *dev, uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT,
                          &val, 1, I2C_TIMEOUT_MS) != HAL_OK)
        return BMP585_ERR;
    return BMP585_OK;
}

BMP585_Status BMP585_ScanI2C(I2C_HandleTypeDef *hi2c, uint8_t *found_addr)
{
    *found_addr = 0;
    for (uint16_t addr = 2; addr < 254; addr += 2) {
        if (HAL_I2C_IsDeviceReady(hi2c, addr, 1, 10) == HAL_OK) {
            *found_addr = (uint8_t)addr;
            return BMP585_OK;
        }
    }
    return BMP585_ERR;
}

BMP585_Status BMP585_Init(BMP585_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    dev->hi2c        = hi2c;
    dev->addr        = addr;
    dev->temperature = 0.0f;
    dev->pressure    = 0.0f;

    /* Soft reset — note: no ACK returned for this write, ignore result */
    reg_write(dev, BMP585_REG_CMD, BMP585_CMD_SOFT_RESET);
    HAL_Delay(5);  /* tsoft_res = 2 ms, 5 ms is safe */

    /* After reset, poll INT_STATUS for POR bit (bit 4 = 0x10) */
    uint8_t int_status = 0;
    for (int i = 0; i < 10; i++) {
        if (reg_read(dev, BMP585_REG_INT_STATUS, &int_status, 1) == BMP585_OK) {
            if (int_status & 0x10) break;  /* POR complete */
        }
        HAL_Delay(2);
    }

    /* Capture STATUS while still in post-reset standby — the only moment
       the NVM-ready/NVM-error bits are meaningful for boot diagnosis */
    dev->boot_status = 0;
    reg_read(dev, BMP585_REG_STATUS, &dev->boot_status, 1);

    /* Verify chip ID — accepts 0x50 (older silicon) and 0x51 (Rev 1.1+) */
    uint8_t chip_id = 0;
    if (reg_read(dev, BMP585_REG_CHIP_ID, &chip_id, 1) != BMP585_OK)
        return BMP585_ERR;
    dev->chip_id = chip_id;
    if (chip_id != BMP585_CHIP_ID_V1 && chip_id != BMP585_CHIP_ID_V2)
        return BMP585_WRONG_ID;

    /*
     * DSP_CONFIG (0x30) bits[1:0] = compensation enable:
     *   00 = none (raw ADC out — garbage pressure), 01 = T only, 11 = T + P.
     * Must be set or the data registers deliver uncompensated values.
     * Bits[7:4] select IIR shadow/FIFO routing — leave at 0 (unfiltered).
     */
    if (reg_write(dev, BMP585_REG_DSP_CONFIG, 0x03) != BMP585_OK)
        return BMP585_ERR;

    /* IIR filter bypass for both channels (raw 25 Hz data; add filtering
       in software where the flight code can control it) */
    if (reg_write(dev, BMP585_REG_DSP_IIR, 0x00) != BMP585_OK)
        return BMP585_ERR;

    /*
     * OSR_CONFIG = 0x40:
     *   press_en = 1 (bit 6)
     *   osr_p    = 000 → 1x oversampling (bits [5:3])
     *   osr_t    = 000 → 1x oversampling (bits [2:0])
     */
    if (reg_write(dev, BMP585_REG_OSR_CONFIG, BMP585_OSR_PRESS_EN_1X_1X) != BMP585_OK)
        return BMP585_ERR;

    /*
     * ODR_CONFIG = 0x51:
     *   deep_dis = 0
     *   odr      = 0x14 → 25 Hz (bits [6:2])
     *   pwr_mode = 01   → Normal continuous mode (bits [1:0])
     */
    if (reg_write(dev, BMP585_REG_ODR_CONFIG, BMP585_ODR_NORMAL_25HZ) != BMP585_OK)
        return BMP585_ERR;

    HAL_Delay(50);  /* wait for first measurement at 25 Hz (40 ms/sample) */
    return BMP585_OK;
}

BMP585_Status BMP585_ReadData(BMP585_t *dev)
{
    /*
     * Burst-read 6 bytes starting at TEMP_XLSB (0x1D):
     *   raw[0] = TEMP_XLSB  (temp[7:0])
     *   raw[1] = TEMP_LSB   (temp[15:8])
     *   raw[2] = TEMP_MSB   (temp[23:16])
     *   raw[3] = PRESS_XLSB (press[7:0])
     *   raw[4] = PRESS_LSB  (press[15:8])
     *   raw[5] = PRESS_MSB  (press[23:16])
     *
     * Temperature: signed 24-bit fixed-point, 2^16 per °C
     *   T [°C] = raw_temp / 65536.0
     *
     * Pressure: signed 24-bit fixed-point, 2^6 per Pa
     *   P [Pa] = raw_press / 64.0
     */
    uint8_t raw[6] = {0};
    if (reg_read(dev, BMP585_REG_TEMP_XLSB, raw, 6) != BMP585_OK)
        return BMP585_ERR;

    /* Temperature (signed 24-bit, sign-extend to 32-bit) */
    int32_t raw_temp = (int32_t)(((uint32_t)raw[2] << 16) |
                                  ((uint32_t)raw[1] << 8)  |
                                   (uint32_t)raw[0]);
    if (raw_temp & 0x800000)
        raw_temp |= (int32_t)0xFF000000;
    dev->temperature = (float)raw_temp / 65536.0f;

    /* Pressure (signed 24-bit — positive in operational range) */
    int32_t raw_press = (int32_t)(((uint32_t)raw[5] << 16) |
                                   ((uint32_t)raw[4] << 8)  |
                                    (uint32_t)raw[3]);
    if (raw_press & 0x800000)
        raw_press |= (int32_t)0xFF000000;
    dev->pressure = (float)raw_press / 64.0f;

    return BMP585_OK;
}
