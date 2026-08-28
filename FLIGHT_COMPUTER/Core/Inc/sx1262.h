#ifndef INC_SX1262_H_
#define INC_SX1262_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * SX1262 LoRa transceiver — SPI1 with hardware NSS (PA4).
 *
 * Board specifics (from schematic):
 *   PC4  = NRESET (active-low, GPIO init leaves it LOW)
 *   PC5  = BUSY   (input; high while the chip processes a command)
 *   DIO2 = RF switch control (SetDio2AsRfSwitchCtrl)
 *   DIO1 = not routed to MCU → IRQs are polled over SPI
 *   32 MHz crystal (no TCXO), DC-DC inductor fitted (15 µH)
 *
 * Every command is ONE HAL_SPI_TransmitReceive call so the hardware
 * NSS frames it correctly.
 */

/* UK 868 MHz ISM band. NOTE: the 32 MHz crystal runs ~57 ppm slow
   (see XTAL trim in sx1262.c), so the actual on-air frequency is
   ≈ 867.950 MHz — receivers must tune 867.95, not 868.00. */
#define SX1262_FREQ_HZ      868000000UL

/* Max +22 dBm. Higher settings demand ~120 mA supply bursts during
   TX — verify the 3.3 V rail holds up before raising this. */
#define SX1262_TX_DBM       14
#define SX1262_MAX_PAYLOAD  128   /* rich flight-telemetry packet */

typedef enum {
    SX1262_OK          = 0,
    SX1262_ERR         = 1,  /* SPI transaction failed        */
    SX1262_BUSY_STUCK  = 2,  /* BUSY never went low           */
    SX1262_BAD_SYNC    = 3,  /* verification register wrong   */
    SX1262_TX_TIMEOUT  = 4,  /* TxDone IRQ never arrived      */
} SX1262_Status;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;      /* PA4 reconfigured as GPIO (soft CS) */
    uint16_t      cs_pin;
    GPIO_TypeDef *nreset_port;
    uint16_t      nreset_pin;
    GPIO_TypeDef *busy_port;
    uint16_t      busy_pin;
    uint8_t       chip_status;  /* last GetStatus byte: bits[6:4] = mode */
    uint8_t       sync_msb;     /* verify read, expect 0x14 */
    int16_t       rx_rssi_dbm;  /* RSSI of last received packet */
    int8_t        rx_snr_db;    /* SNR of last received packet  */
} SX1262_t;

SX1262_Status SX1262_Init(SX1262_t *dev, SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port, uint16_t cs_pin,
                          GPIO_TypeDef *nreset_port, uint16_t nreset_pin,
                          GPIO_TypeDef *busy_port, uint16_t busy_pin);

/* Blocking LoRa transmit (SF7/BW125/CR4-5). A ~10-byte packet takes
   ~30 ms on air; the call polls TxDone up to timeout_ms. */
SX1262_Status SX1262_Transmit(SX1262_t *dev, const uint8_t *data,
                              uint8_t len, uint32_t timeout_ms);

/* Enter continuous receive mode (stays listening until next command).
   Call after each Transmit to open the command uplink window. */
SX1262_Status SX1262_StartRx(SX1262_t *dev);

/* TEST: transmit an unmodulated carrier for duration_ms (blocking),
   then return to standby + RX. For RF path probing with a meter or
   spectrum tool — never call without an antenna or load fitted. */
SX1262_Status SX1262_TestCW(SX1262_t *dev, uint32_t duration_ms);

/* Non-blocking: returns true if a CRC-valid packet is waiting and
   copies it into buf (max *len in, actual out). Radio remains in RX. */
bool SX1262_CheckRx(SX1262_t *dev, uint8_t *buf, uint8_t *len);

/* Refresh dev->chip_status (GetStatus command) */
SX1262_Status SX1262_GetStatus(SX1262_t *dev);

/* GetDeviceErrors: bit0 RC64k cal, bit1 RC13M cal, bit2 PLL cal,
   bit3 ADC cal, bit4 image cal, bit5 XOSC start, bit6 PLL lock,
   bit8 PA ramp. Nonzero = chip-detected fault. */
SX1262_Status SX1262_GetErrors(SX1262_t *dev, uint16_t *errors);

/* Where the last Transmit got to (for diagnosis):
   1=write buffer, 2=packet params, 3=clear irq, 4=SetTx, 5=polling irq,
   0=completed */
extern volatile int sx1262_tx_stage;

#endif /* INC_SX1262_H_ */
