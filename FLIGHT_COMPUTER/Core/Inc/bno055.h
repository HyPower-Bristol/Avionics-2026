#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*
 * BNO055 9-axis IMU with on-chip sensor fusion (I2C).
 *
 * Used here for ABSOLUTE HEADING only: the flight computer takes roll/pitch
 * from the SCH16T (K10) Kalman filter (better sensor, higher rate, immune to
 * magnetic interference) and uses this chip's magnetometer-backed fused
 * heading for yaw — the one thing a 6-axis IMU cannot provide.
 *
 * Runs in NDOF mode (accel+mag+gyro fusion, auto mag calibration).
 * I2C must be <= 100 kHz — the BNO055 clock-stretches.
 *
 * Heading is only trustworthy once cal_mag reaches 3: wave the board through
 * a few slow figure-8s at startup and watch cal_mag climb 0 -> 3.
 */

#define BNO055_ADDR_LOW    0x28   /* 7-bit; COM3/ADR low  */
#define BNO055_ADDR_HIGH   0x29   /* 7-bit; COM3/ADR high */

typedef enum {
    BNO055_OK       = 0,
    BNO055_ERR      = 1,   /* I2C transaction failed */
    BNO055_WRONG_ID = 2,   /* chip ID != 0xA0        */
} BNO055_Status;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t  addr;         /* 8-bit HAL address (7-bit << 1) */
    float    heading;      /* fused yaw, degrees 0..360 */
    float    roll;         /* fused roll (chip), degrees  */
    float    pitch;        /* fused pitch (chip), degrees */
    uint8_t  cal_sys;      /* 0..3 */
    uint8_t  cal_gyro;
    uint8_t  cal_acc;
    uint8_t  cal_mag;      /* heading only reliable at 3 */
} BNO055_t;

/* addr7 = BNO055_ADDR_LOW/HIGH. ext_crystal: 1 ONLY if the breakout has the
   32 kHz crystal (Adafruit does); selecting it without one stalls the clock
   and all data reads 0. Most bare modules: pass 0. */
BNO055_Status BNO055_Init(BNO055_t *dev, I2C_HandleTypeDef *hi2c,
                          uint8_t addr7, uint8_t ext_crystal);

/* Read fused Euler angles (heading/roll/pitch) into dev. */
BNO055_Status BNO055_ReadEuler(BNO055_t *dev);

/* Refresh calibration status bytes into dev (sys/gyro/acc/mag). */
void BNO055_ReadCalib(BNO055_t *dev);

#endif /* INC_BNO055_H_ */
