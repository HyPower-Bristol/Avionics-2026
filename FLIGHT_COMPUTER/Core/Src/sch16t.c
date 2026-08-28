#include "sch16t.h"
#include <stdio.h>

/* ================================================================
   48-bit SPI request codes (MOSI frames with pre-computed CRC).
   Set variants have no CRC — it is appended by the build helpers.
   ================================================================ */
#define REQ_READ_RATE_X1       0x0048000000ACULL
#define REQ_READ_RATE_Y1       0x00880000009AULL
#define REQ_READ_RATE_Z1       0x00C80000006DULL
#define REQ_READ_ACC_X1        0x0108000000F6ULL
#define REQ_READ_ACC_Y1        0x014800000001ULL
#define REQ_READ_ACC_Z1        0x018800000037ULL
#define REQ_READ_ACC_X3        0x01C8000000C0ULL
#define REQ_READ_ACC_Y3        0x02080000002EULL
#define REQ_READ_ACC_Z3        0x0248000000D9ULL
#define REQ_READ_RATE_X2       0x0288000000EFULL
#define REQ_READ_RATE_Y2       0x02C800000018ULL
#define REQ_READ_RATE_Z2       0x030800000083ULL
#define REQ_READ_ACC_X2        0x034800000074ULL
#define REQ_READ_ACC_Y2        0x038800000042ULL
#define REQ_READ_ACC_Z2        0x03C8000000B5ULL
#define REQ_READ_STAT_SUM      0x05080000001CULL
#define REQ_READ_STAT_SUM_SAT  0x0548000000EBULL
#define REQ_READ_STAT_COM      0x0588000000DDULL
#define REQ_READ_STAT_RATE_COM 0x05C80000002AULL
#define REQ_READ_STAT_RATE_X   0x0608000000C4ULL
#define REQ_READ_STAT_RATE_Y   0x064800000033ULL
#define REQ_READ_STAT_RATE_Z   0x068800000005ULL
#define REQ_READ_STAT_ACC_X    0x06C8000000F2ULL
#define REQ_READ_STAT_ACC_Y    0x070800000069ULL
#define REQ_READ_STAT_ACC_Z    0x07480000009EULL
#define REQ_READ_TEMP          0x0408000000B1ULL
#define REQ_READ_SN_ID1        0x0F4800000065ULL
#define REQ_READ_SN_ID2        0x0F8800000053ULL
#define REQ_READ_SN_ID3        0x0FC8000000A4ULL
#define REQ_READ_FILT_RATE     0x0948000000FAULL
#define REQ_READ_FILT_ACC12    0x0988000000CCULL
#define REQ_READ_FILT_ACC3     0x09C80000003BULL
#define REQ_READ_RATE_CTRL     0x0A08000000D5ULL
#define REQ_READ_ACC12_CTRL    0x0A4800000022ULL
#define REQ_READ_ACC3_CTRL     0x0A8800000014ULL
#define REQ_READ_MODE_CTRL     0x0D4800000010ULL
#define REQ_READ_USER_IF_CTRL  0x0CC80000007CULL
/* Write (no-CRC) base frames — caller appends data field then CRC */
#define REQ_SET_FILT_RATE      0x0968000000ULL
#define REQ_SET_FILT_ACC12     0x09A8000000ULL
#define REQ_SET_FILT_ACC3      0x09E8000000ULL
#define REQ_SET_RATE_CTRL      0x0A28000000ULL
#define REQ_SET_ACC12_CTRL     0x0A68000000ULL
#define REQ_SET_ACC3_CTRL      0x0AA8000000ULL
#define REQ_SET_MODE_CTRL      0x0D68000000ULL
#define REQ_SET_USER_IF_CTRL   0x0CE8000000ULL
#define REQ_SOFTRESET          0x0DA800000AC3ULL

/* Frame field masks */
#define TA_FIELD_MASK     0xFFC000000000ULL
#define SA_FIELD_MASK     0x7FE000000000ULL
#define DATA_FIELD_MASK   0x00000FFFFF00ULL
#define ERROR_FIELD_MASK  0x001E00000000ULL

/* Extract signed 20-bit data from a 48-bit MISO frame */
static inline int32_t  SPI48_INT32(uint64_t a)  { return ((int32_t)(((a) << 4)  & 0xFFFFF000UL)) >> 12; }
static inline uint16_t SPI48_UINT16(uint64_t a) { return  (uint16_t)(((a) >> 8) & 0x0000FFFFUL); }

/* ================================================================
   CRC-8 (polynomial 0x2F, init 0xFF)
   ================================================================ */
static uint8_t sch_crc8(uint64_t frame)
{
    uint64_t data = frame & 0xFFFFFFFFFF00ULL;
    uint8_t  crc  = 0xFF;
    for (int i = 47; i >= 0; i--) {
        uint8_t bit = (data >> i) & 0x01;
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x2F) ^ bit
                           : (uint8_t)(crc << 1) | bit;
    }
    return crc;
}

/* ================================================================
   Frame builders — mirror Arduino addTargetAddress / addTargetAddressNoCRC
   ================================================================ */

/* Build full 48-bit read frame (with CRC appended). */
static uint64_t build_read(SCH16T_Handle *h, uint64_t req48)
{
    uint64_t out = (req48 | ((uint64_t)h->ta9_8 << 46)) & 0xFFFFFFFFFF00ULL;
    out |= sch_crc8(out);
    return out;
}

/* Build 40-bit write frame base (no CRC byte yet). */
static uint64_t build_write_base(SCH16T_Handle *h, uint64_t req40)
{
    return (req40 | ((uint64_t)h->ta9_8 << 38)) & 0xFFFFFFFFFFULL;
}

/* Finalise a write frame: append CRC to the low byte. */
static uint64_t finalise_write(uint64_t frame40)
{
    uint64_t out = frame40 << 8;
    out |= sch_crc8(out);
    return out;
}

/* ================================================================
   SPI — full-duplex 48-bit (6-byte) transaction, manual CS
   ================================================================ */
static uint64_t sch_transfer(SCH16T_Handle *h, uint64_t req)
{
    uint8_t  tx[6], rx[6] = {0};
    uint64_t received = 0;

    for (int i = 0; i < 6; i++)
        tx[i] = (req >> ((5 - i) * 8)) & 0xFF;

    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(h->hspi, tx, rx, 6, 10);
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);

    for (int i = 0; i < 6; i++)
        received |= (uint64_t)rx[i] << ((5 - i) * 8);

    return received;
}

static bool check_frame_errors(uint64_t *data, int n)
{
    for (int i = 0; i < n; i++)
        if (data[i] & ERROR_FIELD_MASK) return true;
    return false;
}

/* ================================================================
   Validation helpers
   ================================================================ */
static bool valid_rate_filter(uint32_t f)
{ return (f==13||f==30||f==68||f==235||f==280||f==370||f==0); }

static bool valid_acc_filter(uint32_t f)
{ return (f==13||f==30||f==68||f==210||f==240||f==290||f==0); }

static bool valid_rate_sens(SCH16T_Handle *h, uint32_t s)
{
    if (h->variant == SCH16T_VARIANT_K10)
        return (s==100||s==200||s==400);
    return (s==1600||s==3200||s==6400);
}

static bool valid_acc_sens(uint32_t s)
{ return (s==3200||s==6400||s==12800||s==25600); }

static bool valid_decimation(uint32_t d)
{ return (d==2||d==4||d==8||d==16||d==32); }

/* ================================================================
   Bitfield converters
   ================================================================ */
static uint32_t rate_filter_bits(uint32_t f)
{
    switch (f) {
        case 13:  return 0x092;
        case 30:  return 0x049;
        case 68:  return 0x000;
        case 235: return 0x16D;
        case 280: return 0x0DB;
        case 370: return 0x124;
        case 0:   return 0x1FF;
        default:  return 0x000;
    }
}

static uint32_t acc_filter_bits(uint32_t f)
{
    switch (f) {
        case 13:  return 0x092;
        case 30:  return 0x049;
        case 68:  return 0x000;
        case 210: return 0x16D;
        case 240: return 0x0DB;
        case 290: return 0x124;
        case 0:   return 0x1FF;
        default:  return 0x000;
    }
}

static uint32_t rate_sens_bits(SCH16T_Handle *h, uint32_t s)
{
    if (h->variant == SCH16T_VARIANT_K10) {
        switch (s) { case 100: return 0x02; case 200: return 0x03; case 400: return 0x04; default: return 0x01; }
    }
    switch (s) { case 1600: return 0x02; case 3200: return 0x03; case 6400: return 0x04; default: return 0x01; }
}

static uint32_t acc_sens_bits(uint32_t s)
{
    switch (s) {
        case 3200:  return 0x01;
        case 6400:  return 0x02;
        case 12800: return 0x03;
        case 25600: return 0x04;
        default:    return 0x00;
    }
}

static uint32_t decimation_bits(uint32_t d)
{
    switch (d) {
        case 2:  return 0x00;
        case 4:  return 0x01;
        case 8:  return 0x02;
        case 16: return 0x03;
        case 32: return 0x04;
        default: return 0x00;
    }
}

/* ================================================================
   Internal register-write helpers
   ================================================================ */
static int set_filters(SCH16T_Handle *h, uint32_t fr12, uint32_t fa12, uint32_t fa3)
{
    if (!valid_rate_filter(fr12) || !valid_acc_filter(fa12) || !valid_acc_filter(fa3))
        return SCH16T_ERR_INVALID_PARAM;

    /* Build and send write frames */
    uint64_t wf_rate12 = finalise_write(build_write_base(h, REQ_SET_FILT_RATE)  | rate_filter_bits(fr12));
    uint64_t wf_acc12  = finalise_write(build_write_base(h, REQ_SET_FILT_ACC12) | acc_filter_bits(fa12));
    uint64_t wf_acc3   = finalise_write(build_write_base(h, REQ_SET_FILT_ACC3)  | acc_filter_bits(fa3));

    sch_transfer(h, wf_rate12);
    sch_transfer(h, wf_acc12);
    sch_transfer(h, wf_acc3);

    /* Read back — MISO is always 1 transaction delayed */
    sch_transfer(h, build_read(h, REQ_READ_FILT_RATE));
    uint64_t rsp_rate12 = sch_transfer(h, build_read(h, REQ_READ_FILT_ACC12));
    uint64_t rsp_acc12  = sch_transfer(h, build_read(h, REQ_READ_FILT_ACC3));
    uint64_t rsp_acc3   = sch_transfer(h, build_read(h, REQ_READ_FILT_ACC3));

    if (!rsp_rate12 || rsp_rate12 == 0xFFFFFFFFFFFFULL) return SCH16T_ERR_OTHER;
    if (!rsp_acc12  || rsp_acc12  == 0xFFFFFFFFFFFFULL) return SCH16T_ERR_OTHER;
    if (!rsp_acc3   || rsp_acc3   == 0xFFFFFFFFFFFFULL) return SCH16T_ERR_OTHER;

    if (((wf_rate12 & TA_FIELD_MASK)>>38) != ((rsp_rate12 & SA_FIELD_MASK)>>37)) return SCH16T_ERR_OTHER;
    if (((wf_acc12  & TA_FIELD_MASK)>>38) != ((rsp_acc12  & SA_FIELD_MASK)>>37)) return SCH16T_ERR_OTHER;
    if (((wf_acc3   & TA_FIELD_MASK)>>38) != ((rsp_acc3   & SA_FIELD_MASK)>>37)) return SCH16T_ERR_OTHER;

    if ((wf_rate12 & DATA_FIELD_MASK) != (rsp_rate12 & DATA_FIELD_MASK)) return SCH16T_ERR_OTHER;
    if ((wf_acc12  & DATA_FIELD_MASK) != (rsp_acc12  & DATA_FIELD_MASK)) return SCH16T_ERR_OTHER;
    if ((wf_acc3   & DATA_FIELD_MASK) != (rsp_acc3   & DATA_FIELD_MASK)) return SCH16T_ERR_OTHER;

    return SCH16T_OK;
}

static int set_rate_sens_dec(SCH16T_Handle *h, uint16_t s1, uint16_t s2, uint16_t dec)
{
    if (!valid_rate_sens(h, s1) || !valid_rate_sens(h, s2) || !valid_decimation(dec))
        return SCH16T_ERR_INVALID_PARAM;

    h->sens_rate1 = s1;
    h->sens_rate2 = s2;

    uint32_t d = rate_sens_bits(h, s1);
    d = (d << 3) | rate_sens_bits(h, s2);
    d = (d << 3) | decimation_bits(dec);
    d = (d << 3) | decimation_bits(dec);  /* same decimation all axes */
    d = (d << 3) | decimation_bits(dec);

    uint64_t wf = finalise_write(build_write_base(h, REQ_SET_RATE_CTRL) | d);
    sch_transfer(h, wf);

    sch_transfer(h, build_read(h, REQ_READ_RATE_CTRL));
    uint64_t rsp = sch_transfer(h, build_read(h, REQ_READ_RATE_CTRL));

    if (!rsp || rsp == 0xFFFFFFFFFFFFULL) return SCH16T_ERR_OTHER;
    if (((wf & TA_FIELD_MASK)>>38) != ((rsp & SA_FIELD_MASK)>>37)) return SCH16T_ERR_OTHER;
    if ((wf & DATA_FIELD_MASK) != (rsp & DATA_FIELD_MASK)) return SCH16T_ERR_OTHER;

    return SCH16T_OK;
}

static int set_acc_sens_dec(SCH16T_Handle *h, uint16_t s1, uint16_t s2, uint16_t s3, uint16_t dec)
{
    if (!valid_acc_sens(s1) || !valid_acc_sens(s2) || !valid_acc_sens(s3) || !valid_decimation(dec))
        return SCH16T_ERR_INVALID_PARAM;

    h->sens_acc1 = s1;
    h->sens_acc2 = s2;
    h->sens_acc3 = s3;

    /* Acc12 control: Sens1 | Sens2 | DecX | DecY | DecZ */
    uint32_t d12 = acc_sens_bits(s1);
    d12 = (d12 << 3) | acc_sens_bits(s2);
    d12 = (d12 << 3) | decimation_bits(dec);
    d12 = (d12 << 3) | decimation_bits(dec);
    d12 = (d12 << 3) | decimation_bits(dec);

    uint64_t wf_acc12 = finalise_write(build_write_base(h, REQ_SET_ACC12_CTRL) | d12);
    uint64_t wf_acc3  = finalise_write(build_write_base(h, REQ_SET_ACC3_CTRL)  | acc_sens_bits(s3));

    sch_transfer(h, wf_acc12);
    sch_transfer(h, wf_acc3);

    sch_transfer(h, build_read(h, REQ_READ_ACC12_CTRL));
    uint64_t rsp12 = sch_transfer(h, build_read(h, REQ_READ_ACC3_CTRL));
    uint64_t rsp3  = sch_transfer(h, build_read(h, REQ_READ_ACC3_CTRL));

    if (!rsp12 || rsp12 == 0xFFFFFFFFFFFFULL) return SCH16T_ERR_OTHER;
    if (!rsp3  || rsp3  == 0xFFFFFFFFFFFFULL) return SCH16T_ERR_OTHER;
    if (((wf_acc12 & TA_FIELD_MASK)>>38) != ((rsp12 & SA_FIELD_MASK)>>37)) return SCH16T_ERR_OTHER;
    if (((wf_acc3  & TA_FIELD_MASK)>>38) != ((rsp3  & SA_FIELD_MASK)>>37)) return SCH16T_ERR_OTHER;
    if ((wf_acc12 & DATA_FIELD_MASK) != (rsp12 & DATA_FIELD_MASK)) return SCH16T_ERR_OTHER;
    if ((wf_acc3  & DATA_FIELD_MASK) != (rsp3  & DATA_FIELD_MASK)) return SCH16T_ERR_OTHER;

    return SCH16T_OK;
}

static int enable_meas(SCH16T_Handle *h, bool en_sensor, bool set_eoi)
{
    uint64_t wf = build_write_base(h, REQ_SET_MODE_CTRL);
    if (en_sensor) wf |= 0x01;
    if (set_eoi)   wf |= 0x02;
    wf = finalise_write(wf);
    sch_transfer(h, wf);

    sch_transfer(h, build_read(h, REQ_READ_MODE_CTRL));
    uint64_t rsp = sch_transfer(h, build_read(h, REQ_READ_MODE_CTRL));

    if (!rsp || rsp == 0xFFFFFFFFFFFFULL) return SCH16T_ERR_OTHER;
    if (((wf & TA_FIELD_MASK)>>38) != ((rsp & SA_FIELD_MASK)>>37)) return SCH16T_ERR_OTHER;

    return SCH16T_OK;
}

static int set_dry(SCH16T_Handle *h, int8_t polarity, bool enable)
{
    if (polarity < -1 || polarity > 1) return SCH16T_ERR_INVALID_PARAM;

    /* Read current USER_IF_CTRL content */
    sch_transfer(h, build_read(h, REQ_READ_USER_IF_CTRL));
    uint64_t rsp = sch_transfer(h, build_read(h, REQ_READ_USER_IF_CTRL));
    uint64_t data = (rsp & DATA_FIELD_MASK) >> 8;

    if (polarity == 0)       data &= ~(uint64_t)0x40;  /* active high */
    else if (polarity == 1)  data |=  (uint64_t)0x40;  /* active low  */
    if (enable)              data |=  (uint64_t)0x20;
    else                     data &= ~(uint64_t)0x20;

    uint64_t wf = finalise_write(build_write_base(h, REQ_SET_USER_IF_CTRL) | (data & 0xFF));
    sch_transfer(h, wf);

    sch_transfer(h, build_read(h, REQ_READ_USER_IF_CTRL));
    uint64_t rsp2 = sch_transfer(h, build_read(h, REQ_READ_USER_IF_CTRL));

    if (!rsp2 || rsp2 == 0xFFFFFFFFFFFFULL) return SCH16T_ERR_OTHER;
    if (((wf & TA_FIELD_MASK)>>38) != ((rsp2 & SA_FIELD_MASK)>>37)) return SCH16T_ERR_OTHER;
    if ((wf & DATA_FIELD_MASK) != (rsp2 & DATA_FIELD_MASK)) return SCH16T_ERR_OTHER;

    return SCH16T_OK;
}

/* ================================================================
   Public API
   ================================================================ */

void SCH16T_Reset(SCH16T_Handle *h)
{
    if (!h->reset_port) return;
    HAL_GPIO_WritePin(h->reset_port, h->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(h->reset_port, h->reset_pin, GPIO_PIN_SET);
}

void SCH16T_SPIReset(SCH16T_Handle *h)
{
    sch_transfer(h, build_read(h, REQ_SOFTRESET));
}

int SCH16T_GetStatus(SCH16T_Handle *h, SCH16T_Status *status)
{
    if (!status) return SCH16T_ERR_NULL_PTR;

    /* Pipeline: each sch_transfer(REQ_READ_x) returns the PREVIOUS command's data.
       Send N+1 requests to collect N responses. */
    sch_transfer(h, build_read(h, REQ_READ_STAT_SUM));
    status->Summary      = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_SUM_SAT)));
    status->Summary_Sat  = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_COM)));
    status->Common       = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_RATE_COM)));
    status->Rate_Common  = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_RATE_X)));
    status->Rate_X       = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_RATE_Y)));
    status->Rate_Y       = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_RATE_Z)));
    status->Rate_Z       = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_ACC_X)));
    status->Acc_X        = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_ACC_Y)));
    status->Acc_Y        = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_ACC_Z)));
    status->Acc_Z        = SPI48_UINT16(sch_transfer(h, build_read(h, REQ_READ_STAT_SUM))); /* dummy to clock out last */

    return SCH16T_OK;
}

bool SCH16T_VerifyStatus(SCH16T_Status *status)
{
    if (!status) return false;
    return (status->Summary     == 0xFFFF &&
            status->Summary_Sat == 0xFFFF &&
            status->Common      == 0xFFFF &&
            status->Rate_Common == 0xFFFF &&
            status->Rate_X      == 0xFFFF &&
            status->Rate_Y      == 0xFFFF &&
            status->Rate_Z      == 0xFFFF &&
            status->Acc_X       == 0xFFFF &&
            status->Acc_Y       == 0xFFFF &&
            status->Acc_Z       == 0xFFFF);
}

int SCH16T_Begin(SCH16T_Handle *h,
                 SCH16T_Filter filter,
                 SCH16T_Sensitivity sens,
                 SCH16T_Decimation dec,
                 bool enableDRY)
{
    /* Deassert CS and RESET immediately */
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);
    if (h->reset_port)
        HAL_GPIO_WritePin(h->reset_port, h->reset_pin, GPIO_PIN_SET);

    SCH16T_Status st;
    bool ok = false;

    SCH16T_Reset(h);
    HAL_Delay(10);
    SCH16T_SPIReset(h);

    for (int attempt = 0; attempt < 2; attempt++) {
        HAL_Delay(32);  /* NVM read time */

        set_filters(h, filter.Rate12, filter.Acc12, filter.Acc3);
        set_rate_sens_dec(h, sens.Rate1, sens.Rate2, dec.Rate2);
        set_acc_sens_dec(h, sens.Acc1, sens.Acc2, sens.Acc3, dec.Acc2);
        set_dry(h, 0, enableDRY);  /* polarity = active-high */
        enable_meas(h, true, false);

        HAL_Delay(215);  /* measurement stabilisation */

        SCH16T_GetStatus(h, &st);  /* no-critise read */
        enable_meas(h, true, true); /* set EOI */

        HAL_Delay(3);

        SCH16T_GetStatus(h, &st);
        SCH16T_GetStatus(h, &st);

        if (SCH16T_VerifyStatus(&st)) {
            ok = true;
            break;
        }
        SCH16T_Reset(h);
    }

    return ok ? SCH16T_OK : SCH16T_ERR_SENSOR_INIT;
}

void SCH16T_GetData(SCH16T_Handle *h, SCH16T_RawData *data)
{
    sch_transfer(h, build_read(h, REQ_READ_RATE_X1));
    uint64_t rx1 = sch_transfer(h, build_read(h, REQ_READ_RATE_Y1));
    uint64_t ry1 = sch_transfer(h, build_read(h, REQ_READ_RATE_Z1));
    uint64_t rz1 = sch_transfer(h, build_read(h, REQ_READ_ACC_X1));
    uint64_t ax1 = sch_transfer(h, build_read(h, REQ_READ_ACC_Y1));
    uint64_t ay1 = sch_transfer(h, build_read(h, REQ_READ_ACC_Z1));
    uint64_t az1 = sch_transfer(h, build_read(h, REQ_READ_TEMP));
    uint64_t tmp = sch_transfer(h, build_read(h, REQ_READ_TEMP));

    uint64_t frames[] = {rx1, ry1, rz1, ax1, ay1, az1, tmp};
    data->frame_error = check_frame_errors(frames, 7);

    data->Rate1_raw[SCH16T_AXIS_X] = SPI48_INT32(rx1);
    data->Rate1_raw[SCH16T_AXIS_Y] = SPI48_INT32(ry1);
    data->Rate1_raw[SCH16T_AXIS_Z] = SPI48_INT32(rz1);
    data->Acc1_raw[SCH16T_AXIS_X]  = SPI48_INT32(ax1);
    data->Acc1_raw[SCH16T_AXIS_Y]  = SPI48_INT32(ay1);
    data->Acc1_raw[SCH16T_AXIS_Z]  = SPI48_INT32(az1);
    data->Temp_raw = SPI48_INT32(tmp) >> 4;
}

void SCH16T_GetDataDecimated(SCH16T_Handle *h, SCH16T_RawData *data)
{
    sch_transfer(h, build_read(h, REQ_READ_RATE_X2));
    uint64_t rx2 = sch_transfer(h, build_read(h, REQ_READ_RATE_Y2));
    uint64_t ry2 = sch_transfer(h, build_read(h, REQ_READ_RATE_Z2));
    uint64_t rz2 = sch_transfer(h, build_read(h, REQ_READ_ACC_X2));
    uint64_t ax2 = sch_transfer(h, build_read(h, REQ_READ_ACC_Y2));
    uint64_t ay2 = sch_transfer(h, build_read(h, REQ_READ_ACC_Z2));
    uint64_t az2 = sch_transfer(h, build_read(h, REQ_READ_TEMP));
    uint64_t tmp = sch_transfer(h, build_read(h, REQ_READ_TEMP));

    uint64_t frames[] = {rx2, ry2, rz2, ax2, ay2, az2, tmp};
    data->frame_error = check_frame_errors(frames, 7);

    data->Rate2_raw[SCH16T_AXIS_X] = SPI48_INT32(rx2);
    data->Rate2_raw[SCH16T_AXIS_Y] = SPI48_INT32(ry2);
    data->Rate2_raw[SCH16T_AXIS_Z] = SPI48_INT32(rz2);
    data->Acc2_raw[SCH16T_AXIS_X]  = SPI48_INT32(ax2);
    data->Acc2_raw[SCH16T_AXIS_Y]  = SPI48_INT32(ay2);
    data->Acc2_raw[SCH16T_AXIS_Z]  = SPI48_INT32(az2);
    data->Temp_raw = SPI48_INT32(tmp) >> 4;
}

void SCH16T_GetDataAux(SCH16T_Handle *h, SCH16T_RawData *data)
{
    sch_transfer(h, build_read(h, REQ_READ_ACC_X3));
    uint64_t ax3 = sch_transfer(h, build_read(h, REQ_READ_ACC_Y3));
    uint64_t ay3 = sch_transfer(h, build_read(h, REQ_READ_ACC_Z3));
    uint64_t az3 = sch_transfer(h, build_read(h, REQ_READ_TEMP));
    uint64_t tmp = sch_transfer(h, build_read(h, REQ_READ_TEMP));

    uint64_t frames[] = {ax3, ay3, az3, tmp};
    data->frame_error = check_frame_errors(frames, 4);

    data->Acc3_raw[SCH16T_AXIS_X] = SPI48_INT32(ax3);
    data->Acc3_raw[SCH16T_AXIS_Y] = SPI48_INT32(ay3);
    data->Acc3_raw[SCH16T_AXIS_Z] = SPI48_INT32(az3);
    data->Temp_raw = SPI48_INT32(tmp) >> 4;
}

void SCH16T_ConvertData(SCH16T_Handle *h, SCH16T_RawData *in, SCH16T_Result *out)
{
    float sr1 = (float)h->sens_rate1;
    float sa1 = (float)h->sens_acc1;
    out->Rate1[SCH16T_AXIS_X] = (float)in->Rate1_raw[SCH16T_AXIS_X] / sr1;
    out->Rate1[SCH16T_AXIS_Y] = (float)in->Rate1_raw[SCH16T_AXIS_Y] / sr1;
    out->Rate1[SCH16T_AXIS_Z] = (float)in->Rate1_raw[SCH16T_AXIS_Z] / sr1;
    out->Acc1[SCH16T_AXIS_X]  = (float)in->Acc1_raw[SCH16T_AXIS_X]  / sa1;
    out->Acc1[SCH16T_AXIS_Y]  = (float)in->Acc1_raw[SCH16T_AXIS_Y]  / sa1;
    out->Acc1[SCH16T_AXIS_Z]  = (float)in->Acc1_raw[SCH16T_AXIS_Z]  / sa1;
    out->Temp = (float)in->Temp_raw / 100.0f;
}

void SCH16T_ConvertDataDecimated(SCH16T_Handle *h, SCH16T_RawData *in, SCH16T_Result *out)
{
    float sr2 = (float)h->sens_rate2;
    float sa2 = (float)h->sens_acc2;
    out->Rate2[SCH16T_AXIS_X] = (float)in->Rate2_raw[SCH16T_AXIS_X] / sr2;
    out->Rate2[SCH16T_AXIS_Y] = (float)in->Rate2_raw[SCH16T_AXIS_Y] / sr2;
    out->Rate2[SCH16T_AXIS_Z] = (float)in->Rate2_raw[SCH16T_AXIS_Z] / sr2;
    out->Acc2[SCH16T_AXIS_X]  = (float)in->Acc2_raw[SCH16T_AXIS_X]  / sa2;
    out->Acc2[SCH16T_AXIS_Y]  = (float)in->Acc2_raw[SCH16T_AXIS_Y]  / sa2;
    out->Acc2[SCH16T_AXIS_Z]  = (float)in->Acc2_raw[SCH16T_AXIS_Z]  / sa2;
    out->Temp = (float)in->Temp_raw / 100.0f;
}

void SCH16T_ConvertDataAux(SCH16T_Handle *h, SCH16T_RawData *in, SCH16T_Result *out)
{
    float sa3 = (float)h->sens_acc3;
    out->Acc3[SCH16T_AXIS_X] = (float)in->Acc3_raw[SCH16T_AXIS_X] / sa3;
    out->Acc3[SCH16T_AXIS_Y] = (float)in->Acc3_raw[SCH16T_AXIS_Y] / sa3;
    out->Acc3[SCH16T_AXIS_Z] = (float)in->Acc3_raw[SCH16T_AXIS_Z] / sa3;
    out->Temp = (float)in->Temp_raw / 100.0f;
}
