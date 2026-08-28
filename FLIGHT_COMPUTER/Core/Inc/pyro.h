#ifndef INC_PYRO_H_
#define INC_PYRO_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * Pyro channel driver — recovery deployment.
 *
 * Hardware (per channel): MCU pin → 1k → NMOS gate (3.3k pulldown),
 * low-side switch; ematch from battery+ to FET drain; F1206 fuse;
 * 200k/27k divider on the drain node → ADC for continuity.
 *   drain ≈ VBAT   (ematch present, FET off) → sense ≈ VBAT × 27/227
 *   drain ≈ 0      (no ematch / fired / FET on)
 *
 * Channels: C = FIRE_C (PC15) / C_SENSE (PC1,  ADC3_INP11)
 *           D = FIRE_D (PC2)  / D_SENSE (PC3_C, ADC3_INP1)
 *
 * SAFETY MODEL
 *  - Boot state: FETs off (GPIO init low + gate pulldowns).
 *  - Pyro_Fire() refuses unless Pyro_Arm() was called first.
 *  - Arming auto-expires after PYRO_ARM_TIMEOUT_MS.
 *  - Fire pulse is time-limited and ended by Pyro_Update() — call it
 *    every main-loop iteration.
 */

#define PYRO_ARM_TIMEOUT_MS   60000u  /* arm expires after 60 s        */
#define PYRO_FIRE_MS_DEFAULT   1000u  /* ematch pulse length           */
#define PYRO_FIRE_MS_MAX       2000u  /* hard clamp                    */
#define PYRO_CONT_THRESH_MV     300u  /* pin mV above this = continuity */

typedef enum { PYRO_CH_C = 0, PYRO_CH_D = 1 } PyroChannel;

typedef struct {
    bool     armed;
    uint32_t arm_expiry_tick;
    bool     firing[2];
    uint32_t fire_end_tick[2];
    uint32_t fired_count[2];   /* pulses completed since boot */
    uint16_t cont_mv[2];       /* last continuity reading, mV at ADC pin */
} PyroState;

extern PyroState pyro;

void Pyro_Init(ADC_HandleTypeDef *hadc);

/* Software arm/disarm. Arm expires automatically. */
void Pyro_Arm(void);
void Pyro_Disarm(void);

/* Start a fire pulse. Returns false (and does nothing) if not armed
   or already firing. duration_ms is clamped to PYRO_FIRE_MS_MAX. */
bool Pyro_Fire(PyroChannel ch, uint32_t duration_ms);

/* Call every loop: ends expired pulses, expires arming,
   refreshes continuity readings (skipped while a channel fires). */
void Pyro_Update(void);

/* True if the last reading showed an ematch present */
bool Pyro_HasContinuity(PyroChannel ch);

#endif /* INC_PYRO_H_ */
