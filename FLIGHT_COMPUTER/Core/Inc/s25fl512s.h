#ifndef INC_S25FL512S_H_
#define INC_S25FL512S_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*
 * S25FL512S — 512 Mbit (64 MB) SPI NOR flash (Cypress/Infineon).
 *
 * Geometry (this part is unusual — check before porting to other chips):
 *   Page size   : 512 bytes  (program granularity)
 *   Sector size : 256 KB     (erase granularity)
 *   Capacity    : 64 MB → 256 sectors, 4-byte addressing required
 *
 * All commands used here are the native 4-byte-address variants,
 * so no bank/EXTADD mode juggling is needed.
 */

#define S25FL_PAGE_SIZE      512u
#define S25FL_SECTOR_SIZE    (256u * 1024u)
#define S25FL_CAPACITY       (64u * 1024u * 1024u)
#define S25FL_NUM_SECTORS    (S25FL_CAPACITY / S25FL_SECTOR_SIZE)  /* 256 */

/* Expected JEDEC ID (cmd 0x9F): manufacturer 0x01, device 0x0220 */
#define S25FL_JEDEC_MFR      0x01
#define S25FL_JEDEC_ID_MSB   0x02
#define S25FL_JEDEC_ID_LSB   0x20

typedef enum {
    S25FL_OK        = 0,
    S25FL_ERR       = 1,  /* SPI transaction failed          */
    S25FL_WRONG_ID  = 2,  /* JEDEC ID mismatch               */
    S25FL_TIMEOUT   = 3,  /* WIP never cleared               */
    S25FL_PROG_ERR  = 4,  /* SR1 P_ERR set after program     */
    S25FL_ERASE_ERR = 5,  /* SR1 E_ERR set after erase       */
    S25FL_VERIFY_ERR= 6,  /* self-test readback mismatch     */
} S25FL_Status;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint8_t            jedec[3];   /* captured during init */
} S25FL_t;

/* Init: fixes SPI config (8-bit, safe clock), verifies JEDEC ID */
S25FL_Status S25FL_Init(S25FL_t *dev, SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin);

S25FL_Status S25FL_Read(S25FL_t *dev, uint32_t addr,
                        uint8_t *buf, uint32_t len);

/* Write any range; handles 512-byte page boundaries internally.
   Target range must be erased first (NOR: program only clears bits). */
S25FL_Status S25FL_Write(S25FL_t *dev, uint32_t addr,
                         const uint8_t *buf, uint32_t len);

/* Erase the 256 KB sector containing addr (blocks up to ~3 s) */
S25FL_Status S25FL_EraseSector(S25FL_t *dev, uint32_t addr);

uint8_t S25FL_ReadStatus1(S25FL_t *dev);

/* Destructive self-test on the LAST sector: erase → pattern write →
   readback verify. Safe for a fresh chip; do not call once logging.
   On failure, s25fl_test_step tells which stage died:
   1=erase 2=blank-read 3=blank-check 4=write 5=readback 6=compare */
S25FL_Status S25FL_SelfTest(S25FL_t *dev);
extern volatile int s25fl_test_step;

#endif /* INC_S25FL512S_H_ */
