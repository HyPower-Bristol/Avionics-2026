#ifndef INC_FLASH_LOG_H_
#define INC_FLASH_LOG_H_

#include "s25fl512s.h"
#include <stdint.h>

/*
 * Telemetry logger for the S25FL512S NOR flash.
 *
 * Workflow:
 *   prep     — erase the log region on the pad (blocks ~seconds; NEVER
 *              erase in flight: a 256KB sector erase stalls ~0.5 s)
 *   logon    — start appending records each loop
 *   ...fly...
 *   logoff   — stop, flush the partial page
 *   dump     — stream all records to USB as CSV for capture + plotting
 *
 * Records are 64 bytes → 8 per 512-byte page. In-flight writes are pure
 * page-programs (~0.4 ms), safe inside the control loop. The write cursor
 * survives power cycles (boot scans for the first erased page).
 */

typedef struct __attribute__((packed)) {
    uint32_t t_ms;          /* ms since boot                    */
    int16_t  roll_x10;      /* deg ×10                          */
    int16_t  pitch_x10;
    int16_t  yaw_x10;
    int16_t  hg_x_x10;      /* high-g accel, g ×10              */
    int16_t  hg_y_x10;
    int16_t  hg_z_x10;
    int16_t  hg_mag_x10;
    int32_t  baro_pa;       /* pressure, Pa                     */
    int32_t  baro_alt_cm;   /* altitude AGL, cm (0 until armed) */
    int32_t  gps_lat;       /* 1e-7 deg                         */
    int32_t  gps_lng;
    int16_t  gps_alt_m;
    uint8_t  sats;
    uint8_t  fix;
    uint8_t  state;         /* flight state                     */
    uint8_t  flags;         /* b0 armed, b1 drogue, b2 main     */
    uint8_t  _pad[24];      /* pad to 64 bytes                  */
} FlashLogRecord;

void         FlashLog_Init(S25FL_t *flash);              /* boot: find cursor */
S25FL_Status FlashLog_Prep(uint32_t region_bytes);       /* erase + reset     */
void         FlashLog_SetActive(uint8_t on);
uint8_t      FlashLog_IsActive(void);
void         FlashLog_Write(const FlashLogRecord *rec);  /* buffered append   */
void         FlashLog_Flush(void);                       /* write partial page*/
uint32_t     FlashLog_BytesUsed(void);
void         FlashLog_Dump(void);                        /* CSV to USB        */

#endif /* INC_FLASH_LOG_H_ */
