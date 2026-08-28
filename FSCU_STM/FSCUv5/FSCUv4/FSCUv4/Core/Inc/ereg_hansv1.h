/*
 * ereg_hansv1.h
 *
 * Simulink Coder 25.2 (R2025b) output, model v7.23, with a few bits pulled out
 * for the FSCU build: the MW_target_hardware_resources.h include and the
 * stopRequested / runModel externs, since there is no ERT main here. Also
 * Delay_DSTATE is cut from [10000] to [1] - the delay output is never read
 * back in the step function and the full array was 80 KB of RAM. Interface
 * externs added at the bottom.
 */

#ifndef ereg_hansv1_h_
#define ereg_hansv1_h_

#ifndef ereg_hansv1_COMMON_INCLUDES_
#define ereg_hansv1_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif

#include "ereg_hansv1_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include <stddef.h>

/* Real-time model macros */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif
#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals */
typedef struct {
  real_T Lookup;    /* '<S73>/Lookup', pressure setpoint [bar] */
  real_T Lookup_l;  /* '<S74>/Lookup', outer PID enable */
  real_T Step;      /* '<S3>/Step', 0 at steady state */
  real_T Lookup_f;  /* '<S11>/Lookup', servo PID enable */
  real_T Lookup_c;  /* '<S5>/Lookup', valve Cv, unused on hardware */
} B_ereg_hansv1_T;

/* Block states */
typedef struct {
  /* Tank plant model, not updated now the PTs feed P1/P2 */
  real_T DiscreteTimeIntegrator2_DSTATE; /* '<S4>/Discrete-Time Integrator2' */
  real_T DiscreteTimeIntegrator_DSTATE;  /* '<S4>/Discrete-Time Integrator'  */
  real_T DiscreteTimeIntegrator1_DSTATE; /* '<S4>/Discrete-Time Integrator1' */

  /* Outer pressure PID */
  real_T Integrator_DSTATE;  /* '<S109>/Integrator' */
  real_T Filter_DSTATE;      /* '<S104>/Filter'     */

  /* Output is never read back, so one element instead of 10000 */
  real_T Delay_DSTATE[1];    /* '<S16>/Delay' */

  /* Inner servo PID and position */
  real_T DiscreteTimeIntegrator_DSTATE_f; /* '<S12>/Discrete-Time Integrator', servo position [deg] */
  real_T Filter_DSTATE_o;                 /* '<S48>/Filter'     */
  real_T Integrator_DSTATE_b;             /* '<S53>/Integrator' */
  real_T PrevY;                           /* '<S2>/Backlash', valve angle [deg] */

  /* Repeating sequence counters, kept for the rate scheduler */
  uint16_T Output_DSTATE;    /* '<S127>/Output' */
  uint16_T Output_DSTATE_i;  /* '<S130>/Output' */
  uint16_T Output_DSTATE_c;  /* '<S13>/Output'  */
  uint16_T Output_DSTATE_g;  /* '<S6>/Output'   */
} DW_ereg_hansv1_T;

/* Invariant block signals */
typedef struct {
  const real_T P_HPbar; /* '<S1>/Gain3', nominal HP tank pressure [bar] */
} ConstB_ereg_hansv1_T;

/* Constant parameters, lookup tables in flash */
typedef struct {
  real_T Lookup_tableData[10000];   /* '<S73>/Lookup' */
  real_T pooled9[10000];            /* time axis shared by the lookup blocks */
  real_T pooled10[10000];           /* '<S5>/Lookup', '<S74>/Lookup' */
  real_T Lookup_tableData_j[10000]; /* '<S11>/Lookup' */
} ConstP_ereg_hansv1_T;

/* Real-time model */
struct tag_RTM_ereg_hansv1_T {
  const char_T * volatile errorStatus;
  struct {
    uint32_T clockTick1;
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Global data */
extern B_ereg_hansv1_T ereg_hansv1_B;
extern DW_ereg_hansv1_T ereg_hansv1_DW;
extern const ConstB_ereg_hansv1_T ereg_hansv1_ConstB;
extern const ConstP_ereg_hansv1_T ereg_hansv1_ConstP;
extern RT_MODEL_ereg_hansv1_T *const ereg_hansv1_M;

/* Defined in ereg_hansv1.c */
extern double ereg_P1_hp_bar;
extern double ereg_P2_prop_bar;
extern double ereg_setpoint_bar;
extern unsigned char ereg_enable;
extern double ereg_servo_out_deg;

extern void ereg_hansv1_initialize(void);
extern void ereg_hansv1_step(void);
extern void ereg_hansv1_terminate(void);

#endif /* ereg_hansv1_h_ */
