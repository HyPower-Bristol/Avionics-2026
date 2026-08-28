#include "sx1262.h"
#include <string.h>

/* Opcodes (SX1261/2 datasheet) */
#define OP_SET_STANDBY          0x80
#define OP_SET_TX               0x83
#define OP_SET_REGULATOR_MODE   0x96
#define OP_CALIBRATE            0x89
#define OP_CALIBRATE_IMAGE      0x98
#define OP_SET_PA_CONFIG        0x95
#define OP_SET_RF_FREQUENCY     0x86
#define OP_SET_PACKET_TYPE      0x8A
#define OP_SET_TX_PARAMS        0x8E
#define OP_SET_BUF_BASE_ADDR    0x8F
#define OP_SET_MOD_PARAMS       0x8B
#define OP_SET_PACKET_PARAMS    0x8C
#define OP_SET_DIO_IRQ_PARAMS   0x08
#define OP_SET_DIO2_RFSWITCH    0x9D
#define OP_GET_STATUS           0xC0
#define OP_GET_IRQ_STATUS       0x12
#define OP_CLEAR_IRQ_STATUS     0x02
#define OP_WRITE_BUFFER         0x0E
#define OP_WRITE_REGISTER       0x0D
#define OP_READ_REGISTER        0x1D
#define OP_SET_RX               0x82
#define OP_GET_RX_BUF_STATUS    0x13
#define OP_READ_BUFFER          0x1E

#define IRQ_TX_DONE             0x0001
#define IRQ_RX_DONE             0x0002
#define IRQ_CRC_ERR             0x0040
#define IRQ_TIMEOUT             0x0200

#define REG_LORA_SYNC_MSB       0x0740  /* reset default 0x14 */
#define REG_OCP                 0x08E7

#define BUSY_TIMEOUT_MS         100u
#define SPI_TIMEOUT_MS          100u

static SX1262_Status busy_wait(SX1262_t *d)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(d->busy_port, d->busy_pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start > BUSY_TIMEOUT_MS)
            return SX1262_BUSY_STUCK;
    }
    return SX1262_OK;
}

/* One command = one SPI frame, CS held low by software for the whole
   frame. rx (if given) receives the full frame including status bytes. */
static SX1262_Status cmd(SX1262_t *d, const uint8_t *tx, uint8_t *rx,
                         uint16_t len)
{
    static uint8_t sink[SX1262_MAX_PAYLOAD + 8];
    if (len > sizeof(sink)) return SX1262_ERR;

    SX1262_Status st = busy_wait(d);
    if (st != SX1262_OK) return st;


    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hst = HAL_SPI_TransmitReceive(d->hspi, (uint8_t *)tx,
                                                    rx ? rx : sink, len,
                                                    SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);
    if (hst != HAL_OK) return SX1262_ERR;

    return busy_wait(d);  /* chip processes command after CS rises */
}

static SX1262_Status write_reg(SX1262_t *d, uint16_t addr, uint8_t val)
{
    uint8_t f[4] = { OP_WRITE_REGISTER,
                     (uint8_t)(addr >> 8), (uint8_t)addr, val };
    return cmd(d, f, NULL, sizeof(f));
}

static SX1262_Status read_reg(SX1262_t *d, uint16_t addr, uint8_t *val)
{
    uint8_t tx[5] = { OP_READ_REGISTER,
                      (uint8_t)(addr >> 8), (uint8_t)addr, 0, 0 };
    uint8_t rx[5] = {0};
    SX1262_Status st = cmd(d, tx, rx, sizeof(tx));
    *val = rx[4];
    return st;
}

SX1262_Status SX1262_GetStatus(SX1262_t *dev)
{
    uint8_t tx[2] = { OP_GET_STATUS, 0 };
    uint8_t rx[2] = {0};
    SX1262_Status st = cmd(dev, tx, rx, 2);
    dev->chip_status = rx[1];
    return st;
}

SX1262_Status SX1262_GetErrors(SX1262_t *dev, uint16_t *errors)
{
    uint8_t tx[4] = { 0x17, 0, 0, 0 };  /* GetDeviceErrors */
    uint8_t rx[4] = {0};
    SX1262_Status st = cmd(dev, tx, rx, 4);
    *errors = ((uint16_t)rx[2] << 8) | rx[3];
    return st;
}

static SX1262_Status get_irq(SX1262_t *d, uint16_t *irq)
{
    uint8_t tx[4] = { OP_GET_IRQ_STATUS, 0, 0, 0 };
    uint8_t rx[4] = {0};
    SX1262_Status st = cmd(d, tx, rx, 4);
    *irq = ((uint16_t)rx[2] << 8) | rx[3];
    return st;
}

static SX1262_Status clear_irq(SX1262_t *d)
{
    uint8_t f[3] = { OP_CLEAR_IRQ_STATUS, 0xFF, 0xFF };
    return cmd(d, f, NULL, sizeof(f));
}

static SX1262_Status set_packet_len(SX1262_t *d, uint8_t len)
{
    /* preamble 12 sym, explicit header, CRC on, standard IQ */
    uint8_t f[7] = { OP_SET_PACKET_PARAMS, 0x00, 0x0C, 0x00, len, 0x01, 0x00 };
    return cmd(d, f, NULL, sizeof(f));
}

SX1262_Status SX1262_Init(SX1262_t *dev, SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port, uint16_t cs_pin,
                          GPIO_TypeDef *nreset_port, uint16_t nreset_pin,
                          GPIO_TypeDef *busy_port, uint16_t busy_pin)
{
    SX1262_Status st;

    dev->hspi        = hspi;
    dev->cs_port     = cs_port;
    dev->cs_pin      = cs_pin;
    dev->nreset_port = nreset_port;
    dev->nreset_pin  = nreset_pin;
    dev->busy_port   = busy_port;
    dev->busy_pin    = busy_pin;
    dev->chip_status = 0;
    dev->sync_msb    = 0;

    /* Hardware NSS asserts too close to the first SCK edge for the
       SX1262 (data arrived bit-shifted) — use software CS instead.
       Re-init the SPI with soft NSS, 8-bit frames, kernel/8
       (80 MHz PLL1Q / 8 = 10 MHz, SX1262 max 16 MHz). */
    hspi->Init.NSS               = SPI_NSS_SOFT;
    hspi->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    hspi->Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi->Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    if (HAL_SPI_Init(hspi) != HAL_OK) return SX1262_ERR;
    SET_BIT(hspi->Instance->CFG2, SPI_CFG2_AFCNTR);

    /* Take PA4 back from the SPI peripheral: plain GPIO output, idle high */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = cs_pin;
        g.Mode  = GPIO_MODE_OUTPUT_PP;
        g.Pull  = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(cs_port, &g);
    }

    /* Reset pulse: ≥100 µs low, then wait for boot (BUSY falls) */
    HAL_GPIO_WritePin(nreset_port, nreset_pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(nreset_port, nreset_pin, GPIO_PIN_SET);
    HAL_Delay(5);
    if ((st = busy_wait(dev)) != SX1262_OK) return st;

    /* Verify SPI: LoRa sync word MSB resets to 0x14 */
    if ((st = read_reg(dev, REG_LORA_SYNC_MSB, &dev->sync_msb)) != SX1262_OK)
        return st;
    if (dev->sync_msb != 0x14) return SX1262_BAD_SYNC;

    { uint8_t f[2] = { OP_SET_STANDBY, 0x00 };            /* STDBY_RC   */
      if ((st = cmd(dev, f, NULL, 2)) != SX1262_OK) return st; }

    /* XTAL trim: chip default is ~13.6 pF internal caps per pin, but the
       board fits its own 10 pF externals — the over-loaded crystal ran
       ~45 ppm slow (TX measured at ~867.96 instead of 868.00 by RX
       frequency sweep). Set internal trim caps to minimum (11.3 pF).
       MUST be written in STDBY_XOSC — in STDBY_RC the values are
       reloaded with defaults when the oscillator starts. */
    { uint8_t f[2] = { OP_SET_STANDBY, 0x01 };            /* STDBY_XOSC */
      if ((st = cmd(dev, f, NULL, 2)) != SX1262_OK) return st; }
    if ((st = write_reg(dev, 0x0911, 0x00)) != SX1262_OK) return st;  /* XTA */
    if ((st = write_reg(dev, 0x0912, 0x00)) != SX1262_OK) return st;  /* XTB */
    { uint8_t f[2] = { OP_SET_STANDBY, 0x00 };            /* STDBY_RC   */
      if ((st = cmd(dev, f, NULL, 2)) != SX1262_OK) return st; }

    { /* Regulator: 0x00 = LDO, 0x01 = DC-DC. DC-DC halves TX current
         but requires a healthy inductor on DCC_SW (≥15 µH, DCR ≤ 2 Ω,
         Isat ≥ 100 mA); with a marginal inductor the chip overheats
         and TX fails intermittently. LDO is the robust default. */
      uint8_t f[2] = { OP_SET_REGULATOR_MODE, 0x00 };
      if ((st = cmd(dev, f, NULL, 2)) != SX1262_OK) return st; }

    { uint8_t f[2] = { OP_SET_DIO2_RFSWITCH, 0x01 };      /* RF switch  */
      if ((st = cmd(dev, f, NULL, 2)) != SX1262_OK) return st; }

    { uint8_t f[2] = { OP_CALIBRATE, 0x7F };              /* all blocks */
      if ((st = cmd(dev, f, NULL, 2)) != SX1262_OK) return st;
      HAL_Delay(5); }

    { uint8_t f[3] = { OP_CALIBRATE_IMAGE, 0xD7, 0xDB };  /* 863–870MHz */
      if ((st = cmd(dev, f, NULL, 3)) != SX1262_OK) return st; }

    { uint8_t f[2] = { OP_SET_PACKET_TYPE, 0x01 };        /* LoRa       */
      if ((st = cmd(dev, f, NULL, 2)) != SX1262_OK) return st; }

    { /* frf = freq × 2^25 / 32 MHz */
      uint32_t frf = (uint32_t)(((uint64_t)SX1262_FREQ_HZ << 25) / 32000000UL);
      uint8_t f[5] = { OP_SET_RF_FREQUENCY, (uint8_t)(frf >> 24),
                       (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf };
      if ((st = cmd(dev, f, NULL, 5)) != SX1262_OK) return st; }

    { uint8_t f[5] = { OP_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 }; /* SX1262 PA */
      if ((st = cmd(dev, f, NULL, 5)) != SX1262_OK) return st; }

    if ((st = write_reg(dev, REG_OCP, 0x38)) != SX1262_OK) /* 140 mA OCP */
        return st;

    { uint8_t f[3] = { OP_SET_TX_PARAMS, (uint8_t)SX1262_TX_DBM, 0x04 }; /* 200µs ramp */
      if ((st = cmd(dev, f, NULL, 3)) != SX1262_OK) return st; }

    { uint8_t f[3] = { OP_SET_BUF_BASE_ADDR, 0x00, 0x00 };
      if ((st = cmd(dev, f, NULL, 3)) != SX1262_OK) return st; }

    { /* SF9, BW125 (0x04), CR 4/5, LDRO off — SF9 gives ~+6 dB link
         margin over SF7 for long-range tests. Both ends MUST use the
         same spreading factor or nothing decodes. (SF7 = 0x07.) */
      uint8_t f[5] = { OP_SET_MOD_PARAMS, 0x09, 0x04, 0x01, 0x00 };
      if ((st = cmd(dev, f, NULL, 5)) != SX1262_OK) return st; }

    if ((st = set_packet_len(dev, 16)) != SX1262_OK) return st;

    { /* IRQ mask: TxDone + RxDone + CrcErr + Timeout; no DIO routing */
      uint8_t f[9] = { OP_SET_DIO_IRQ_PARAMS,
                       0x02, 0x43,  0x00, 0x00,  0x00, 0x00,  0x00, 0x00 };
      if ((st = cmd(dev, f, NULL, 9)) != SX1262_OK) return st; }

    return SX1262_GetStatus(dev);  /* expect mode STDBY_RC (0x2X) */
}

volatile int sx1262_tx_stage = 0;

SX1262_Status SX1262_Transmit(SX1262_t *dev, const uint8_t *data,
                              uint8_t len, uint32_t timeout_ms)
{
    SX1262_Status st;
    if (len == 0 || len > SX1262_MAX_PAYLOAD) return SX1262_ERR;

    sx1262_tx_stage = 1;
    { uint8_t f[2 + SX1262_MAX_PAYLOAD] = { OP_WRITE_BUFFER, 0x00 };
      memcpy(&f[2], data, len);
      if ((st = cmd(dev, f, NULL, (uint16_t)(2 + len))) != SX1262_OK)
          return st; }

    sx1262_tx_stage = 2;
    if ((st = set_packet_len(dev, len)) != SX1262_OK) return st;
    sx1262_tx_stage = 3;
    if ((st = clear_irq(dev)) != SX1262_OK) return st;

    sx1262_tx_stage = 4;
    { /* SetTx, hardware timeout = timeout_ms (15.625 µs/count) */
      uint32_t tocnt = timeout_ms * 64u;
      uint8_t f[4] = { OP_SET_TX, (uint8_t)(tocnt >> 16),
                       (uint8_t)(tocnt >> 8), (uint8_t)tocnt };
      if ((st = cmd(dev, f, NULL, 4)) != SX1262_OK) return st; }

    sx1262_tx_stage = 5;
    uint32_t start = HAL_GetTick();
    for (;;) {
        uint16_t irq = 0;
        if ((st = get_irq(dev, &irq)) != SX1262_OK) return st;
        if (irq & IRQ_TX_DONE) { clear_irq(dev); sx1262_tx_stage = 0; return SX1262_OK; }
        if (irq & IRQ_TIMEOUT) { clear_irq(dev); return SX1262_TX_TIMEOUT; }
        if (HAL_GetTick() - start > timeout_ms + 50u)
            return SX1262_TX_TIMEOUT;
    }
}

SX1262_Status SX1262_StartRx(SX1262_t *dev)
{
    SX1262_Status st;
    if ((st = clear_irq(dev)) != SX1262_OK) return st;
    /* 0xFFFFFF = continuous RX: stays listening across packets */
    uint8_t f[4] = { OP_SET_RX, 0xFF, 0xFF, 0xFF };
    return cmd(dev, f, NULL, 4);
}

SX1262_Status SX1262_TestCW(SX1262_t *dev, uint32_t duration_ms)
{
    SX1262_Status st;
    uint8_t cw = 0xD1;                    /* SetTxContinuousWave */
    if ((st = cmd(dev, &cw, NULL, 1)) != SX1262_OK) return st;

    HAL_Delay(duration_ms);

    { uint8_t f[2] = { OP_SET_STANDBY, 0x00 };
      if ((st = cmd(dev, f, NULL, 2)) != SX1262_OK) return st; }
    return SX1262_StartRx(dev);
}

bool SX1262_CheckRx(SX1262_t *dev, uint8_t *buf, uint8_t *len)
{
    uint16_t irq = 0;
    uint8_t  max = *len;
    *len = 0;

    if (get_irq(dev, &irq) != SX1262_OK) return false;
    if (!(irq & IRQ_RX_DONE))            return false;

    if (irq & IRQ_CRC_ERR) {   /* corrupted frame — drop silently */
        clear_irq(dev);
        return false;
    }

    /* payload length + start offset in the radio's buffer */
    uint8_t st_tx[4] = { OP_GET_RX_BUF_STATUS, 0, 0, 0 };
    uint8_t st_rx[4] = {0};
    if (cmd(dev, st_tx, st_rx, 4) != SX1262_OK) { clear_irq(dev); return false; }
    uint8_t plen   = st_rx[2];
    uint8_t offset = st_rx[3];
    if (plen == 0) { clear_irq(dev); return false; }
    if (plen > max) plen = max;

    /* ReadBuffer: opcode, offset, NOP, then data */
    uint8_t rd_tx[3 + SX1262_MAX_PAYLOAD] = { OP_READ_BUFFER, offset, 0 };
    uint8_t rd_rx[3 + SX1262_MAX_PAYLOAD] = {0};
    if (cmd(dev, rd_tx, rd_rx, (uint16_t)(3 + plen)) != SX1262_OK) {
        clear_irq(dev);
        return false;
    }
    memcpy(buf, &rd_rx[3], plen);
    *len = plen;

    { /* GetPacketStatus (0x14): RssiPkt, SnrPkt of this packet */
      uint8_t ps_tx[5] = { 0x14, 0, 0, 0, 0 };
      uint8_t ps_rx[5] = {0};
      if (cmd(dev, ps_tx, ps_rx, 5) == SX1262_OK) {
          dev->rx_rssi_dbm = -(int16_t)ps_rx[2] / 2;
          dev->rx_snr_db   = (int8_t)ps_rx[3] / 4;
      }
    }

    clear_irq(dev);   /* radio stays in continuous RX */
    return true;
}
