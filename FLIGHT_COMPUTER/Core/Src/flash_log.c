#include "flash_log.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

/* Log region layout: contiguous from address 0. Prep erases this many
   sectors; 2 MB = 8 × 256KB sectors holds ~30 min at 50 Hz. */
#define LOG_BASE        0u
#define LOG_REGION      (2u * 1024u * 1024u)
#define PAGE_BYTES      512u
#define REC_BYTES       64u
#define RECS_PER_PAGE   (PAGE_BYTES / REC_BYTES)   /* 8 */

static S25FL_t *s_flash = NULL;
static uint32_t s_cursor = LOG_BASE;   /* flash addr of next page to write */
static uint8_t  s_active = 0;
static uint8_t  s_pagebuf[PAGE_BYTES];
static uint8_t  s_recs_in_buf = 0;

void FlashLog_Init(S25FL_t *flash)
{
    s_flash = flash;
    s_active = 0;
    s_recs_in_buf = 0;
    memset(s_pagebuf, 0xFF, sizeof(s_pagebuf));

    /* Find the first erased page (first 4 bytes == 0xFFFFFFFF) so logging
       resumes after a power cycle without overwriting existing data. */
    uint32_t addr = LOG_BASE;
    uint8_t  head[4];
    while (addr < LOG_BASE + LOG_REGION) {
        if (S25FL_Read(s_flash, addr, head, 4) != S25FL_OK) break;
        if (head[0] == 0xFF && head[1] == 0xFF &&
            head[2] == 0xFF && head[3] == 0xFF)
            break;                       /* erased page = resume point */
        addr += PAGE_BYTES;
    }
    s_cursor = addr;
}

S25FL_Status FlashLog_Prep(uint32_t region_bytes)
{
    if (!s_flash) return S25FL_ERR;
    if (region_bytes == 0 || region_bytes > LOG_REGION)
        region_bytes = LOG_REGION;

    for (uint32_t a = LOG_BASE; a < LOG_BASE + region_bytes;
         a += S25FL_SECTOR_SIZE) {
        S25FL_Status st = S25FL_EraseSector(s_flash, a);
        if (st != S25FL_OK) return st;
    }
    s_cursor = LOG_BASE;
    s_recs_in_buf = 0;
    memset(s_pagebuf, 0xFF, sizeof(s_pagebuf));
    return S25FL_OK;
}

void    FlashLog_SetActive(uint8_t on) { s_active = on ? 1 : 0; }
uint8_t FlashLog_IsActive(void)        { return s_active; }
uint32_t FlashLog_BytesUsed(void)      { return s_cursor - LOG_BASE
                                              + s_recs_in_buf * REC_BYTES; }

void FlashLog_Write(const FlashLogRecord *rec)
{
    if (!s_flash) return;
    if (s_cursor >= LOG_BASE + LOG_REGION) { s_active = 0; return; } /* full */

    memcpy(&s_pagebuf[s_recs_in_buf * REC_BYTES], rec, REC_BYTES);
    s_recs_in_buf++;

    if (s_recs_in_buf >= RECS_PER_PAGE) {
        S25FL_Write(s_flash, s_cursor, s_pagebuf, PAGE_BYTES);
        s_cursor += PAGE_BYTES;
        s_recs_in_buf = 0;
        memset(s_pagebuf, 0xFF, sizeof(s_pagebuf));
    }
}

void FlashLog_Flush(void)
{
    if (!s_flash || s_recs_in_buf == 0) return;
    /* pad remainder of page with 0xFF (already memset), program it */
    S25FL_Write(s_flash, s_cursor, s_pagebuf, PAGE_BYTES);
    s_cursor += PAGE_BYTES;
    s_recs_in_buf = 0;
    memset(s_pagebuf, 0xFF, sizeof(s_pagebuf));
}

static void emit(const char *s)
{
    CDC_Transmit_FS((uint8_t *)s, (uint16_t)strlen(s));
    HAL_Delay(3);   /* CDC has no flow control — let each line drain */
}

void FlashLog_Dump(void)
{
    if (!s_flash) return;
    emit("t_ms,roll_x10,pitch_x10,yaw_x10,hgx_x10,hgy_x10,hgz_x10,"
         "hgmag_x10,baro_pa,alt_cm,lat_1e7,lng_1e7,gps_alt_m,"
         "sats,fix,state,flags\r\n");

    uint8_t page[PAGE_BYTES];
    char line[160];
    for (uint32_t a = LOG_BASE; a < s_cursor; a += PAGE_BYTES) {
        if (S25FL_Read(s_flash, a, page, PAGE_BYTES) != S25FL_OK) break;
        for (uint32_t i = 0; i < RECS_PER_PAGE; i++) {
            FlashLogRecord r;
            memcpy(&r, &page[i * REC_BYTES], REC_BYTES);
            if (r.t_ms == 0xFFFFFFFFu) continue;   /* unwritten slot */
            int n = snprintf(line, sizeof(line),
                "%lu,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%d,%u,%u,%u,%u\r\n",
                (unsigned long)r.t_ms,
                r.roll_x10, r.pitch_x10, r.yaw_x10,
                r.hg_x_x10, r.hg_y_x10, r.hg_z_x10, r.hg_mag_x10,
                (long)r.baro_pa, (long)r.baro_alt_cm,
                (long)r.gps_lat, (long)r.gps_lng, r.gps_alt_m,
                r.sats, r.fix, r.state, r.flags);
            if (n > 0) emit(line);
        }
    }
    emit("--- end of log ---\r\n");
}
