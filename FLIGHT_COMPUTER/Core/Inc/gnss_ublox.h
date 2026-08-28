#ifndef GNSS_UBLOX_H
#define GNSS_UBLOX_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*
 * GPS state exposed to C callers.
 * Updated by GNSS_UBX_Update(); safe to read from the main loop.
 *
 * status values (GPS_Status enum from qqqlab_GPS_UBLOX.h):
 *   0 = NO_GPS     — no UBX messages received yet
 *   1 = NO_FIX     — receiving messages, no position lock
 *   2 = FIX_2D
 *   3 = FIX_3D     — normal operation
 *   4 = FIX_3D_DGPS
 *   5 = RTK_FLOAT
 *   6 = RTK_FIXED
 */
typedef struct {
    int32_t  lat;        /* 1e-7 degrees: 515432100 = 51.5432100° N */
    int32_t  lng;        /* 1e-7 degrees: -25987000 = -2.5987000° E */
    int32_t  alt_mm;     /* altitude above MSL, mm                  */
    int32_t  speed_mms;  /* ground speed, mm/s                      */
    int32_t  course_e5;  /* course over ground, 1e-5 degrees        */
    uint16_t hdop;       /* HDOP × 100 (155 = 1.55)                 */
    uint8_t  status;     /* GPS_Status (see above)                  */
    uint8_t  num_sats;   /* satellites used in solution             */
} GNSS_UBX_State;

/* Watch these in Live Expressions */
extern GNSS_UBX_State gnss_ubx;
extern volatile uint32_t gnss_ubx_rx_count;   /* increments each UART burst  */
extern volatile uint32_t gnss_uart_err_count; /* framing/noise/overrun count */

/**
 * Initialise the UBX driver.
 * - Releases RESET_N (active-low) on reset_port/reset_pin
 * - Enables UART7 IRQ and starts interrupt-driven receive
 * - Configures M10S for airborne-4g dynamics at 10 Hz
 * Call once after MX_UART7_Init(), before the main loop.
 */
void GNSS_UBX_Init(UART_HandleTypeDef *huart,
                   GPIO_TypeDef *reset_port, uint16_t reset_pin);

/**
 * Drive the UBX state machine.
 * Must be called at >= 10 Hz (the 50 Hz main loop is fine).
 * Handles baud auto-detection, module configuration and parsing.
 */
void GNSS_UBX_Update(void);

#endif /* GNSS_UBLOX_H */
