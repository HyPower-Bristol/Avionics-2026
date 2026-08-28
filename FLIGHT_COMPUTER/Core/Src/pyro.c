#include "pyro.h"
#include "main.h"

PyroState pyro = {0};

static ADC_HandleTypeDef *s_hadc = NULL;

static GPIO_TypeDef *const FIRE_PORT[2] = { FIRE_C_GPIO_Port, FIRE_D_GPIO_Port };
static const uint16_t      FIRE_PIN[2]  = { FIRE_C_Pin,       FIRE_D_Pin       };
static const uint32_t      SENSE_CH[2]  = { ADC_CHANNEL_11,   ADC_CHANNEL_1    };

static void fets_off(void)
{
    HAL_GPIO_WritePin(FIRE_PORT[0], FIRE_PIN[0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FIRE_PORT[1], FIRE_PIN[1], GPIO_PIN_RESET);
    pyro.firing[0] = pyro.firing[1] = false;
}

/* Single polled conversion. The 200k/27k divider is a ~24k source
   impedance — needs the longest sampling window, not CubeMX's 1.5 cyc. */
static uint16_t read_sense_mv(PyroChannel ch)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = SENSE_CH[ch];
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    if (HAL_ADC_ConfigChannel(s_hadc, &cfg) != HAL_OK) return 0;

    if (HAL_ADC_Start(s_hadc) != HAL_OK) return 0;
    if (HAL_ADC_PollForConversion(s_hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(s_hadc);
        return 0;
    }
    uint32_t raw = HAL_ADC_GetValue(s_hadc);   /* 16-bit */
    HAL_ADC_Stop(s_hadc);

    return (uint16_t)((raw * 3300u) / 65535u);
}

void Pyro_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    fets_off();
    pyro.armed = false;

    /* CubeMX clocks ADC3 from PLL2 at 80 MHz with DIV1 — out of spec
       (H750 ADC kernel clock max 50 MHz), which corrupts conversions
       (constant mid-scale reads). Re-init at /4 = 20 MHz. */
    hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    if (HAL_ADC_Init(hadc) != HAL_OK) return;

    /* offset calibration — ADC must be stopped */
    HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    pyro.cont_mv[PYRO_CH_C] = read_sense_mv(PYRO_CH_C);
    pyro.cont_mv[PYRO_CH_D] = read_sense_mv(PYRO_CH_D);
}

void Pyro_Arm(void)
{
    pyro.armed = true;
    pyro.arm_expiry_tick = HAL_GetTick() + PYRO_ARM_TIMEOUT_MS;
}

void Pyro_Disarm(void)
{
    pyro.armed = false;
    fets_off();   /* disarm always kills any active pulse */
}

bool Pyro_Fire(PyroChannel ch, uint32_t duration_ms)
{
    if (!pyro.armed)      return false;
    if (pyro.firing[ch])  return false;
    if (duration_ms == 0) duration_ms = PYRO_FIRE_MS_DEFAULT;
    if (duration_ms > PYRO_FIRE_MS_MAX) duration_ms = PYRO_FIRE_MS_MAX;

    pyro.fire_end_tick[ch] = HAL_GetTick() + duration_ms;
    pyro.firing[ch] = true;
    HAL_GPIO_WritePin(FIRE_PORT[ch], FIRE_PIN[ch], GPIO_PIN_SET);
    return true;
}

bool Pyro_HasContinuity(PyroChannel ch)
{
    return pyro.cont_mv[ch] > PYRO_CONT_THRESH_MV;
}

void Pyro_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* end expired pulses */
    for (int ch = 0; ch < 2; ch++) {
        if (pyro.firing[ch] && (int32_t)(now - pyro.fire_end_tick[ch]) >= 0) {
            HAL_GPIO_WritePin(FIRE_PORT[ch], FIRE_PIN[ch], GPIO_PIN_RESET);
            pyro.firing[ch] = false;
            pyro.fired_count[ch]++;
        }
    }

    /* expire arming */
    if (pyro.armed && (int32_t)(now - pyro.arm_expiry_tick) >= 0)
        Pyro_Disarm();

    /* refresh continuity ~5 Hz; skip a channel mid-pulse (drain is
       pulled low by the FET — reading would be meaningless) */
    static uint32_t next_sense = 0;
    if ((int32_t)(now - next_sense) >= 0) {
        next_sense = now + 200;
        for (int ch = 0; ch < 2; ch++)
            if (!pyro.firing[ch])
                pyro.cont_mv[ch] = read_sense_mv((PyroChannel)ch);
    }
}
