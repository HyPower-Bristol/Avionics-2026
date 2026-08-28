/*
 * gnss_ublox.c — u-blox M10S UBX driver, pure C
 *
 * Boot sequence
 *   1. Release RESET_N (PB2 HIGH), wait 500 ms
 *   2. Send UBX-CFG-VALSET at 9600: set UART1 baud → 115200
 *   3. Reinit host UART to 115200
 *   4. Send UBX-CFG-VALSET at 115200: UBX-only output, NAV-PVT 10 Hz,
 *      airborne-4g dynamic model
 *   5. Parse NAV-PVT messages and fill gnss_ubx
 */

#include "gnss_ublox.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* 1 = diagnostic mode: stay at 9600, echo raw module output to CDC.
   You should see readable NMEA ($GNGGA, $GNRMC...) in the terminal.
   Set back to 0 for normal operation. */
#define GNSS_DEBUG_RAW  0

/* ── Ring buffer (power-of-2, ISR→main) ──────────────────────────────────── */
#define RING_SIZE  1024u
#define RING_MASK  (RING_SIZE - 1u)

static uint8_t           s_ring[RING_SIZE];
static volatile uint32_t s_head = 0;   /* written in ISR */
static volatile uint32_t s_tail = 0;   /* read in main   */
static uint8_t           s_dma[256];
static UART_HandleTypeDef *s_huart = NULL;

/* Public globals (declared extern in gnss_ublox.h) */
GNSS_UBX_State    gnss_ubx          = {0};
volatile uint32_t gnss_ubx_rx_count = 0;
volatile uint32_t gnss_uart_err_count = 0; /* framing/noise/overrun errors */

static uint32_t ring_used(void)
{
    return (s_head - s_tail) & RING_MASK;
}

static void ring_push(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint32_t next = (s_head + 1u) & RING_MASK;
        if (next != s_tail) { s_ring[s_head] = data[i]; s_head = next; }
    }
}

static int ring_pop(void)
{
    if (s_head == s_tail) return -1;
    uint8_t b = s_ring[s_tail];
    s_tail = (s_tail + 1u) & RING_MASK;
    return b;
}

/* ── UBX framing ─────────────────────────────────────────────────────────── */
static void ubx_send(uint8_t cls, uint8_t id,
                     const uint8_t *payload, uint16_t len)
{
    uint8_t hdr[6] = { 0xB5, 0x62, cls, id,
                       (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    uint8_t ck_a = 0, ck_b = 0;

    for (int i = 2; i < 6; i++) { ck_a += hdr[i]; ck_b += ck_a; }
    for (uint16_t i = 0; i < len; i++) { ck_a += payload[i]; ck_b += ck_a; }

    uint8_t ck[2] = { ck_a, ck_b };
    HAL_UART_Transmit(s_huart, hdr,             6,   50);
    if (len) HAL_UART_Transmit(s_huart, (uint8_t *)payload, len, 200);
    HAL_UART_Transmit(s_huart, ck,              2,   10);
}

/* ── UBX-CFG-VALSET helpers ──────────────────────────────────────────────── */
/*
 * VALSET payload layout:
 *   [0]   version  = 0
 *   [1]   layers   = 0x01 (RAM)
 *   [2-3] reserved = 0
 *   then repeating: key (U4 LE) + value (size encoded in key bits[31:28])
 *     0x1... → 1 byte  (L / U1 / I1)
 *     0x2... → 1 byte  (U1 / I1)
 *     0x3... → 2 bytes (U2 / I2)
 *     0x4... → 4 bytes (U4 / I4 / R4)
 */
static void vs_u1(uint8_t *buf, uint16_t *p, uint32_t key, uint8_t  v)
{
    buf[(*p)++] = (uint8_t)(key);       buf[(*p)++] = (uint8_t)(key >> 8);
    buf[(*p)++] = (uint8_t)(key >> 16); buf[(*p)++] = (uint8_t)(key >> 24);
    buf[(*p)++] = v;
}

static void vs_u2(uint8_t *buf, uint16_t *p, uint32_t key, uint16_t v)
{
    buf[(*p)++] = (uint8_t)(key);       buf[(*p)++] = (uint8_t)(key >> 8);
    buf[(*p)++] = (uint8_t)(key >> 16); buf[(*p)++] = (uint8_t)(key >> 24);
    buf[(*p)++] = (uint8_t)(v);         buf[(*p)++] = (uint8_t)(v >> 8);
}

static void vs_u4(uint8_t *buf, uint16_t *p, uint32_t key, uint32_t v)
{
    buf[(*p)++] = (uint8_t)(key);       buf[(*p)++] = (uint8_t)(key >> 8);
    buf[(*p)++] = (uint8_t)(key >> 16); buf[(*p)++] = (uint8_t)(key >> 24);
    buf[(*p)++] = (uint8_t)(v);         buf[(*p)++] = (uint8_t)(v >> 8);
    buf[(*p)++] = (uint8_t)(v >> 16);   buf[(*p)++] = (uint8_t)(v >> 24);
}

/* Tell M10S to switch UART1 to a new baud rate (send at current baud) */
static void send_valset_baud(uint32_t baud)
{
    uint8_t  pl[12];
    uint16_t pos = 0;
    pl[pos++] = 0;     /* version */
    pl[pos++] = 0x01;  /* layers: RAM */
    pl[pos++] = 0;
    pl[pos++] = 0;
    vs_u4(pl, &pos, 0x40520001UL, baud);  /* CFG-UART1-BAUDRATE */
    ubx_send(0x06, 0x8A, pl, pos);
}

/* Configure NAV-PVT output, dynamic model, measurement rate (at 115200) */
static void send_valset_navconfig(void)
{
    uint8_t  pl[64];
    uint16_t pos = 0;
    pl[pos++] = 0;
    pl[pos++] = 0x01;
    pl[pos++] = 0;
    pl[pos++] = 0;

    vs_u1(pl, &pos, 0x10740001UL, 1);   /* CFG-UART1OUTPROT-UBX  = 1 */
    vs_u1(pl, &pos, 0x10740002UL, 0);   /* CFG-UART1OUTPROT-NMEA = 0 */
    vs_u1(pl, &pos, 0x10730001UL, 1);   /* CFG-UART1INPROT-UBX   = 1 */
    vs_u1(pl, &pos, 0x10730002UL, 1);   /* CFG-UART1INPROT-NMEA  = 1 */
    vs_u1(pl, &pos, 0x20910007UL, 1);   /* CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1 epoch */
    vs_u1(pl, &pos, 0x20110021UL, 8);   /* CFG-NAVSPG-DYNMODEL = 8 (airborne 4g) */
    vs_u2(pl, &pos, 0x30210001UL, 100); /* CFG-RATE-MEAS = 100 ms (10 Hz) */
    vs_u2(pl, &pos, 0x30210002UL, 1);   /* CFG-RATE-NAV  = 1 */

    ubx_send(0x06, 0x8A, pl, pos);
}

/* ── NAV-PVT parser ──────────────────────────────────────────────────────── */
/*
 * NAV-PVT (class 0x01, id 0x07) payload offsets (92 bytes):
 *   20  fixType  U1  — 0=no fix, 1=dead-reck, 2=2D, 3=3D, 4=GNSS+DR
 *   23  numSV    U1
 *   24  lon      I4  1e-7 deg
 *   28  lat      I4  1e-7 deg
 *   36  hMSL     I4  mm above MSL
 *   60  gSpeed   I4  mm/s ground speed
 *   64  headMot  I4  1e-5 deg heading of motion
 *   76  pDOP     U2  0.01 units
 */
#define NAV_PVT_LEN  92u

typedef enum { S1, S2, CLS, MID, LL, LH, PL, CA, CB } UbxPs;

static UbxPs   s_ps    = S1;
static uint8_t s_cls, s_id;
static uint16_t s_len, s_idx;
static uint8_t s_buf[NAV_PVT_LEN];
static uint8_t s_cka, s_ckb;

static void process_pvt(const uint8_t *p)
{
    int32_t  lon, lat, hMSL, gSpeed, headMot;
    uint16_t pDOP;

    memcpy(&lon,     p + 24, 4);
    memcpy(&lat,     p + 28, 4);
    memcpy(&hMSL,    p + 36, 4);
    memcpy(&gSpeed,  p + 60, 4);
    memcpy(&headMot, p + 64, 4);
    memcpy(&pDOP,    p + 76, 2);

    gnss_ubx.lat       = lat;
    gnss_ubx.lng       = lon;
    gnss_ubx.alt_mm    = hMSL;
    gnss_ubx.speed_mms = gSpeed;
    gnss_ubx.course_e5 = headMot;
    gnss_ubx.hdop      = pDOP;
    gnss_ubx.num_sats  = p[23];

    /* Any parsed NAV-PVT means the GPS is alive → at least NO_FIX(1).
       status 0 (NO_GPS) is reserved for "no UBX messages at all". */
    switch (p[20]) {
    case 2:  gnss_ubx.status = 2; break;  /* FIX_2D */
    case 3:
    case 4:  gnss_ubx.status = 3; break;  /* FIX_3D */
    default: gnss_ubx.status = 1; break;  /* alive, NO_FIX */
    }
}

static void ubx_parse_byte(uint8_t b)
{
    switch (s_ps) {
    case S1:  if (b == 0xB5) s_ps = S2;  break;
    case S2:  s_ps = (b == 0x62) ? CLS : S1; break;
    case CLS: s_cls = b; s_cka = b; s_ckb = b;        s_ps = MID; break;
    case MID: s_id  = b; s_cka += b; s_ckb += s_cka;  s_ps = LL;  break;
    case LL:  s_len = b; s_cka += b; s_ckb += s_cka;  s_ps = LH;  break;
    case LH:
        s_len |= (uint16_t)(b << 8);
        s_cka += b; s_ckb += s_cka;
        s_idx = 0;
        s_ps = (s_len > 0) ? PL : CA;
        break;
    case PL:
        if (s_idx < sizeof(s_buf)) s_buf[s_idx] = b;
        s_idx++;
        s_cka += b; s_ckb += s_cka;
        if (s_idx >= s_len) s_ps = CA;
        break;
    case CA:
        s_ps = (b == s_cka) ? CB : S1;
        break;
    case CB:
        if (b == s_ckb &&
            s_cls == 0x01 && s_id == 0x07 && s_len == NAV_PVT_LEN)
            process_pvt(s_buf);
        s_ps = S1;
        break;
    }
}

/* ── Driver state machine ────────────────────────────────────────────────── */
typedef enum {
    DRV_WAIT_BOOT,   /* 500 ms after RESET_N release  */
    DRV_SEND_BAUD,   /* VALSET baud=115200 at 9600     */
    DRV_BAUD_SWITCH, /* reinit host + send nav config  */
    DRV_RUNNING,
} DrvState;

static DrvState s_drv  = DRV_WAIT_BOOT;
static uint32_t s_tick = 0;

/* ── UART IRQ callback (overrides weak HAL stub) ─────────────────────────── */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART7) return;
    gnss_ubx_rx_count++;
    ring_push(s_dma, Size);
    HAL_UARTEx_ReceiveToIdle_IT(huart, s_dma, sizeof(s_dma));
}

/* On framing/noise/overrun HAL aborts reception — restart it or the
   port goes silent forever. Counter tells us the line has activity
   even when no clean bytes get through (usually a baud mismatch). */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART7) return;
    gnss_uart_err_count++;
    HAL_UARTEx_ReceiveToIdle_IT(huart, s_dma, sizeof(s_dma));
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void GNSS_UBX_Init(UART_HandleTypeDef *huart,
                   GPIO_TypeDef *reset_port, uint16_t reset_pin)
{
    s_huart = huart;
    HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_SET);
    s_drv  = DRV_WAIT_BOOT;
    s_tick = HAL_GetTick();
    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
    HAL_UARTEx_ReceiveToIdle_IT(huart, s_dma, sizeof(s_dma));
}

void GNSS_UBX_Update(void)
{
    uint32_t now = HAL_GetTick();

#if GNSS_DEBUG_RAW
    /* Diagnostic passthrough: forward raw module bytes to CDC terminal.
       Module should be at its default 9600 sending NMEA text.
       Also transmit PING out PE8 once/sec — jumper PE7↔PE8 to loopback-
       test the MCU UART: PING appearing in the terminal = MCU side OK. */
    {
        static uint32_t ping_tick = 0;
        if (now - ping_tick >= 1000) {
            ping_tick = now;
            HAL_UART_Transmit(s_huart, (uint8_t *)"PING\r\n", 6, 20);
        }
        uint8_t buf[64];
        int n = 0, b;
        while (n < (int)sizeof(buf) && (b = ring_pop()) >= 0)
            buf[n++] = (uint8_t)b;
        if (n > 0)
            CDC_Transmit_FS(buf, (uint16_t)n);
    }
    return;
#endif

    switch (s_drv) {

    case DRV_WAIT_BOOT:
        if (now - s_tick >= 500)
            s_drv = DRV_SEND_BAUD;
        break;

    case DRV_SEND_BAUD:
        send_valset_baud(115200);  /* M10S applies after ACK at 9600 */
        HAL_Delay(100);            /* wait for module to switch       */
        s_drv  = DRV_BAUD_SWITCH;
        s_tick = HAL_GetTick();
        break;

    case DRV_BAUD_SWITCH:
        /* Reinit host side to match module */
        s_huart->Init.BaudRate = 115200;
        HAL_UART_Init(s_huart);
        s_head = s_tail = 0;  /* discard stale ring bytes */
        HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_dma, sizeof(s_dma));
        HAL_Delay(50);
        send_valset_navconfig();
        s_drv = DRV_RUNNING;
        break;

    case DRV_RUNNING:
        {
            int b;
            while ((b = ring_pop()) >= 0)
                ubx_parse_byte((uint8_t)b);
        }
        break;
    }
}
