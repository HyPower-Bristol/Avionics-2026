#ifndef INC_ADXL375_H_
#define INC_ADXL375_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*
 * ADXL375 — ±200 g high-g 3-axis accelerometer (I2C).
 * Shares I2C1 with the BMP585 (no address clash: 0x53 vs 0x46).
 *
 * Purpose on the flight computer: measure boost/deployment accelerations
 * far beyond the SCH16T's range — used for launch detection and event
 * logging. Fixed ±200 g range, ~20.5 LSB/g (49 mg/LSB), 13-bit.
 */

#define ADXL375_ADDR_LOW   0x53   /* 7-bit; ALT ADDRESS pin low (default) */
#define ADXL375_ADDR_HIGH  0x1D   /* 7-bit; ALT ADDRESS pin high          */
#define ADXL375_DEVID      0xE5   /* DEVID register value                 */

typedef enum {
    ADXL375_OK       = 0,
    ADXL375_ERR      = 1,   /* I2C transaction failed */
    ADXL375_WRONG_ID = 2,   /* DEVID != 0xE5          */
} ADXL375_Status;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t  addr;          /* 8-bit HAL address (7-bit << 1) */
    int16_t  raw[3];        /* X,Y,Z raw counts */
    float    g[3];          /* X,Y,Z in g       */
    float    mag_g;         /* vector magnitude, g (≈1 at rest, >>1 on boost) */
    uint8_t  dev_id;        /* captured at init */
} ADXL375_t;

ADXL375_Status ADXL375_Init(ADXL375_t *dev, I2C_HandleTypeDef *hi2c,
                            uint8_t addr7);
ADXL375_Status ADXL375_Read(ADXL375_t *dev);

#endif /* INC_ADXL375_H_ */
