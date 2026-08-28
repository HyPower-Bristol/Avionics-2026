#include "s25fl512s.h"
#include <string.h>

/* Command set — 4-byte address variants (native on S25FL512S) */
#define CMD_RDID    0x9F  /* read JEDEC ID                     */
#define CMD_RDSR1   0x05  /* read status register 1            */
#define CMD_CLSR    0x30  /* clear P_ERR / E_ERR               */
#define CMD_WREN    0x06  /* write enable                      */
#define CMD_READ4   0x13  /* read, 4-byte address (≤50 MHz)    */
#define CMD_PP4     0x12  /* page program, 4-byte address      */
#define CMD_SE4     0xDC  /* 256 KB sector erase, 4-byte addr  */

/* SR1 bits */
#define SR1_WIP     0x01  /* write in progress   */
#define SR1_WEL     0x02  /* write enable latch  */
#define SR1_E_ERR   0x20  /* erase error         */
#define SR1_P_ERR   0x40  /* program error       */

#define SPI_TIMEOUT_MS       100u
#define PROG_TIMEOUT_MS      100u   /* 512 B page: typ 0.34 ms      */
#define ERASE_TIMEOUT_MS     5000u  /* 256 KB sector: typ 0.5 s      */

static inline void cs_low(S25FL_t *d)  { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(S25FL_t *d) { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);   }

/* One framed transaction: send hdr, then optionally write or read data.
   Reads use full-duplex TransmitReceive with dummy 0xFF bytes — the H7
   HAL's master-mode HAL_SPI_Receive switches to RX-only mode and fails
   on longer transfers, so it is avoided entirely. */
static S25FL_Status xfer(S25FL_t *d, const uint8_t *hdr, uint16_t hdr_len,
                         const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    static uint8_t dummy[64];
    HAL_StatusTypeDef st;

    cs_low(d);
    st = HAL_SPI_Transmit(d->hspi, (uint8_t *)hdr, hdr_len, SPI_TIMEOUT_MS);
    if (st == HAL_OK && len) {
        if (tx) {
            st = HAL_SPI_Transmit(d->hspi, (uint8_t *)tx, len, SPI_TIMEOUT_MS);
        } else if (rx) {
            memset(dummy, 0xFF, sizeof(dummy));
            while (len && st == HAL_OK) {
                uint16_t chunk = (len < sizeof(dummy)) ? (uint16_t)len
                                                       : (uint16_t)sizeof(dummy);
                st = HAL_SPI_TransmitReceive(d->hspi, dummy, rx, chunk,
                                             SPI_TIMEOUT_MS);
                rx  += chunk;
                len -= chunk;
            }
        }
    }

    cs_high(d);
    return (st == HAL_OK) ? S25FL_OK : S25FL_ERR;
}

static S25FL_Status cmd_only(S25FL_t *d, uint8_t cmd)
{
    return xfer(d, &cmd, 1, NULL, NULL, 0);
}

uint8_t S25FL_ReadStatus1(S25FL_t *dev)
{
    uint8_t cmd = CMD_RDSR1, sr = 0xFF;
    xfer(dev, &cmd, 1, NULL, &sr, 1);
    return sr;
}

static S25FL_Status wait_ready(S25FL_t *d, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    for (;;) {
        uint8_t sr = S25FL_ReadStatus1(d);
        if (sr & SR1_P_ERR) { cmd_only(d, CMD_CLSR); return S25FL_PROG_ERR; }
        if (sr & SR1_E_ERR) { cmd_only(d, CMD_CLSR); return S25FL_ERASE_ERR; }
        if (!(sr & SR1_WIP)) return S25FL_OK;
        if (HAL_GetTick() - start > timeout_ms) return S25FL_TIMEOUT;
    }
}

static void addr4(uint8_t *out, uint8_t cmd, uint32_t addr)
{
    out[0] = cmd;
    out[1] = (uint8_t)(addr >> 24);
    out[2] = (uint8_t)(addr >> 16);
    out[3] = (uint8_t)(addr >> 8);
    out[4] = (uint8_t)(addr);
}

S25FL_Status S25FL_Init(S25FL_t *dev, SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    memset(dev->jedec, 0, sizeof(dev->jedec));

    cs_high(dev);

    /* CubeMX H7 SPI defaults can be 4-bit/div2 — force 8-bit frames,
       1-byte FIFO threshold, kernel/8 clock. SPE=0 after HAL_SPI_Init
       so CFG1 is writable without a DeInit/Init cycle. */
    MODIFY_REG(hspi->Instance->CFG1,
               SPI_CFG1_DSIZE_Msk | SPI_CFG1_FTHLV_Msk | SPI_CFG1_MBR_Msk,
               (7U << SPI_CFG1_DSIZE_Pos) |   /* 8-bit frames        */
               (0U << SPI_CFG1_FTHLV_Pos) |   /* 1-byte threshold    */
               (2U << SPI_CFG1_MBR_Pos));      /* kernel clk / 8      */

    /* Keep SCK/MOSI under SPI control between HAL calls — without this
       the pins float when HAL disables SPE after each Transmit/Receive,
       and SCK glitches corrupt multi-transfer frames (hdr + data). */
    SET_BIT(hspi->Instance->CFG2, SPI_CFG2_AFCNTR);

    HAL_Delay(1);

    /* JEDEC ID: expect 01 02 20 */
    uint8_t cmd = CMD_RDID;
    if (xfer(dev, &cmd, 1, NULL, dev->jedec, 3) != S25FL_OK)
        return S25FL_ERR;

    if (dev->jedec[0] != S25FL_JEDEC_MFR ||
        dev->jedec[1] != S25FL_JEDEC_ID_MSB ||
        dev->jedec[2] != S25FL_JEDEC_ID_LSB)
        return S25FL_WRONG_ID;

    /* Clear any stale error latches from a previous aborted operation */
    cmd_only(dev, CMD_CLSR);
    return wait_ready(dev, PROG_TIMEOUT_MS);
}

S25FL_Status S25FL_Read(S25FL_t *dev, uint32_t addr,
                        uint8_t *buf, uint32_t len)
{
    uint8_t hdr[5];
    addr4(hdr, CMD_READ4, addr);
    return xfer(dev, hdr, 5, NULL, buf, len);
}

static S25FL_Status program_page(S25FL_t *dev, uint32_t addr,
                                 const uint8_t *buf, uint32_t len)
{
    S25FL_Status st;
    uint8_t hdr[5];

    if ((st = cmd_only(dev, CMD_WREN)) != S25FL_OK) return st;

    addr4(hdr, CMD_PP4, addr);
    if ((st = xfer(dev, hdr, 5, buf, NULL, len)) != S25FL_OK) return st;

    return wait_ready(dev, PROG_TIMEOUT_MS);
}

S25FL_Status S25FL_Write(S25FL_t *dev, uint32_t addr,
                         const uint8_t *buf, uint32_t len)
{
    while (len) {
        /* bytes remaining in the current 512-byte page */
        uint32_t page_room = S25FL_PAGE_SIZE - (addr % S25FL_PAGE_SIZE);
        uint32_t chunk = (len < page_room) ? len : page_room;

        S25FL_Status st = program_page(dev, addr, buf, chunk);
        if (st != S25FL_OK) return st;

        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    return S25FL_OK;
}

S25FL_Status S25FL_EraseSector(S25FL_t *dev, uint32_t addr)
{
    S25FL_Status st;
    uint8_t hdr[5];

    if ((st = cmd_only(dev, CMD_WREN)) != S25FL_OK) return st;

    addr4(hdr, CMD_SE4, addr & ~(S25FL_SECTOR_SIZE - 1u));
    if ((st = xfer(dev, hdr, 5, NULL, NULL, 0)) != S25FL_OK) return st;

    return wait_ready(dev, ERASE_TIMEOUT_MS);
}

volatile int s25fl_test_step = 0;

S25FL_Status S25FL_SelfTest(S25FL_t *dev)
{
    /* last sector, so future log data at the bottom stays untouched */
    const uint32_t test_addr = S25FL_CAPACITY - S25FL_SECTOR_SIZE;
    static uint8_t pattern[S25FL_PAGE_SIZE];
    static uint8_t readback[S25FL_PAGE_SIZE];

    for (uint32_t i = 0; i < sizeof(pattern); i++)
        pattern[i] = (uint8_t)(i * 7u + 13u);

    s25fl_test_step = 1;
    S25FL_Status st = S25FL_EraseSector(dev, test_addr);
    if (st != S25FL_OK) return st;

    /* erased NOR must read all 0xFF */
    s25fl_test_step = 2;
    if ((st = S25FL_Read(dev, test_addr, readback, sizeof(readback))) != S25FL_OK)
        return st;
    s25fl_test_step = 3;
    for (uint32_t i = 0; i < sizeof(readback); i++)
        if (readback[i] != 0xFF) return S25FL_VERIFY_ERR;

    s25fl_test_step = 4;
    if ((st = S25FL_Write(dev, test_addr, pattern, sizeof(pattern))) != S25FL_OK)
        return st;

    s25fl_test_step = 5;
    memset(readback, 0, sizeof(readback));
    if ((st = S25FL_Read(dev, test_addr, readback, sizeof(readback))) != S25FL_OK)
        return st;
    s25fl_test_step = 6;
    if (memcmp(pattern, readback, sizeof(pattern)) != 0)
        return S25FL_VERIFY_ERR;

    s25fl_test_step = 0; /* all passed */
    return S25FL_OK;
}
