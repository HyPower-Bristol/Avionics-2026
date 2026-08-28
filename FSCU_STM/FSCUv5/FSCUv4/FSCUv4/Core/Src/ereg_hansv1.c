/*
 * ereg_hansv1.c - EREG controller, hardware build
 *
 * Simulink Coder 25.2 (R2025b) output for model v7.23, trimmed down to run on
 * the FSCU. The tank plant model and the flow solver are gone - P1 and P2 come
 * off the PTs now via ereg_P1_hp_bar / ereg_P2_prop_bar. Setpoint and enable
 * are fed in from ereg_setpoint_bar / ereg_enable instead of the repeating
 * sequence blocks. The delay block update went too; its output was never read
 * back and the 9999-element shift cost about 2 ms per step.
 */

#include "ereg_hansv1.h"
#include "ereg_hansv1_private.h"
#include <math.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"

/* Interface to the FSCU main loop */
double ereg_P1_hp_bar    = 300.0; /* HP/N2 tank pressure [bar]          */
double ereg_P2_prop_bar  =  50.0; /* Prop tank pressure [bar]           */
double ereg_setpoint_bar =  50.0; /* Target prop pressure [bar]         */
unsigned char ereg_enable =    0; /* 0 = hold, 1 = active               */
double ereg_servo_out_deg =   0.0; /* Rate-limited servo angle [0..180] */

/* Simulink model storage */
B_ereg_hansv1_T  ereg_hansv1_B;
DW_ereg_hansv1_T ereg_hansv1_DW;

static RT_MODEL_ereg_hansv1_T ereg_hansv1_M_;
RT_MODEL_ereg_hansv1_T *const ereg_hansv1_M = &ereg_hansv1_M_;

/* Advances the 10 ms sub-rate, called once per base step */
static void rate_scheduler(void)
{
    (ereg_hansv1_M->Timing.TaskCounters.TID[1])++;
    if (ereg_hansv1_M->Timing.TaskCounters.TID[1] > 9)
        ereg_hansv1_M->Timing.TaskCounters.TID[1] = 0;
}

/* Kept for the look1_binlcpw prototype */
real_T look1_binlcpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
    real_T frac;
    real_T yL_0d0;
    uint32_T iLeft;

    if (u0 <= bp0[0U]) {
        iLeft = 0U; frac = 0.0;
    } else if (u0 < bp0[maxIndex]) {
        uint32_T bpIdx, iRght;
        bpIdx = maxIndex >> 1U; iLeft = 0U; iRght = maxIndex;
        while (iRght - iLeft > 1U) {
            if (u0 < bp0[bpIdx]) iRght = bpIdx; else iLeft = bpIdx;
            bpIdx = (iRght + iLeft) >> 1U;
        }
        frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
    } else {
        iLeft = maxIndex - 1U; frac = 1.0;
    }

    yL_0d0 = table[iLeft];
    return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/* Safe power, straight out of the generated code */
real_T rt_powd_snf(real_T u0, real_T u1)
{
    real_T y;
    if (rtIsNaN(u0) || rtIsNaN(u1)) {
        y = (rtNaN);
    } else {
        real_T tmp  = fabs(u0);
        real_T tmp_0 = fabs(u1);
        if (rtIsInf(u1)) {
            if (tmp == 1.0) y = 1.0;
            else if (tmp > 1.0) y = (u1 > 0.0) ? (rtInf) : 0.0;
            else                y = (u1 > 0.0) ? 0.0 : (rtInf);
        } else if (tmp_0 == 0.0) {
            y = 1.0;
        } else if (tmp_0 == 1.0) {
            y = (u1 > 0.0) ? u0 : 1.0 / u0;
        } else if (u1 == 2.0) {
            y = u0 * u0;
        } else if ((u1 == 0.5) && (u0 >= 0.0)) {
            y = sqrt(u0);
        } else if ((u0 < 0.0) && (u1 > floor(u1))) {
            y = (rtNaN);
        } else {
            y = pow(u0, u1);
        }
    }
    return y;
}

/* Call every 1 ms */
void ereg_hansv1_step(void)
{
    real_T rtb_AddConstant;
    real_T rtb_FilterCoefficient;
    real_T rtb_NProdOut;
    real_T rtb_P_1;
    real_T rtb_P_2;
    real_T rtb_Sum;
    real_T rtb_Switch;
    real_T rtb_Switch_h;
    real_T rtb_valve_angle;
    real_T delta_P_pa;

    uint16_T rtb_FixPtSum1;
    uint16_T rtb_FixPtSum1_c;
    uint16_T rtb_FixPtSum1_h;
    uint16_T rtb_FixPtSum1_i;
    boolean_T tmp;

    /* PT readings in place of the old tank plant model */
    rtb_P_1 = ereg_P1_hp_bar  * 1.0E5;  /* bar -> Pa */
    rtb_P_2 = ereg_P2_prop_bar * 1.0E5;  /* bar -> Pa */

    /* Feed-forward gain scaling out of the '<S1>/PT' subsystem:
     *   (P_HP_setpoint - P1) * (1/300) * 5 + 1
     * Winds Kp up as the HP supply drops away. The formula assumes a 300 bar
     * system though, so on the bench at 8 bar or so it comes out near 6 and
     * shakes the loop apart. Cap it at 1 and run the nominal gains. */
    rtb_AddConstant = (ereg_hansv1_ConstB.P_HPbar - 1.0E-5 * rtb_P_1) *
                      0.0033333333333333335 * 5.0 + 1.0;
    if (rtb_AddConstant > 1.0) rtb_AddConstant = 1.0;

    /* 10 ms tasks run on every 10th base step */
    tmp = (ereg_hansv1_M->Timing.TaskCounters.TID[1] == 0);

    if (tmp) {
        rtb_FixPtSum1   = (uint16_T)(ereg_hansv1_DW.Output_DSTATE   + 1);
        rtb_FixPtSum1_c = (uint16_T)(ereg_hansv1_DW.Output_DSTATE_i + 1);

        /* Was a repeating sequence lookup */
        ereg_hansv1_B.Lookup   = ereg_setpoint_bar;
        ereg_hansv1_B.Lookup_l = ereg_enable ? 1.0 : 0.0;
    }

    /* Outer pressure loop error */
    if (ereg_hansv1_B.Lookup_l != 0.0) {
        rtb_Switch = ereg_hansv1_B.Lookup - 1.0E-5 * rtb_P_2;
    } else {
        rtb_Switch = 0.0;
    }

    /* Filtered derivative, Kd = 8, N = 100 */
    rtb_NProdOut = (rtb_Switch * 8.0 - ereg_hansv1_DW.Filter_DSTATE) * 100.0;

    if (tmp) {
        rtb_FixPtSum1_i = (uint16_T)(ereg_hansv1_DW.Output_DSTATE_c + 1);

        ereg_hansv1_B.Step    = 0.0;
        ereg_hansv1_B.Lookup_f = ereg_enable ? 1.0 : 0.0;
    }

    /* Outer PID output is the demanded valve angle in degrees.
     * Kp = 16 * feed-forward, Ki = 17, Kd = 8. */
    if (ereg_hansv1_B.Lookup_f != 0.0) {
        rtb_Switch_h = (16.0 * rtb_AddConstant * rtb_Switch
                        + ereg_hansv1_DW.Integrator_DSTATE
                        + rtb_NProdOut)
                       + ereg_hansv1_B.Step;
    } else {
        rtb_Switch_h = 0.0;

        /* With the controller disabled the servo position integrator drifts
         * negative at 180 deg/s, since the outer loop asks for 0 deg while
         * PrevY sits at 0.5. Leave it be and a 60 s wait before enabling turns
         * into 60 s of dead time. Park the states at neutral so the first step
         * after ereg_enable goes high starts clean. */
        ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE_f = 0.0;
        ereg_hansv1_DW.Integrator_DSTATE_b             = 0.0;
        ereg_hansv1_DW.Filter_DSTATE_o                 = 0.0;
        ereg_hansv1_DW.Integrator_DSTATE               = 0.0;
        ereg_hansv1_DW.Filter_DSTATE                   = 0.0;
        ereg_hansv1_DW.PrevY                           = 0.0;
    }

    if (tmp) {
        rtb_FixPtSum1_h = (uint16_T)(ereg_hansv1_DW.Output_DSTATE_g + 1);
        /* Lookup_c is the valve Cv, unused on hardware */
    }

    /* Valve gear conversion and backlash. PrevY is the angle the gear train
     * actually delivers - 1.7:1 with a 0.25 deg deadband either side. */
    {
        real_T valve_angle_raw = ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE_f
                                 / 1.7;
        if (valve_angle_raw < ereg_hansv1_DW.PrevY - 0.25)
            ereg_hansv1_DW.PrevY = valve_angle_raw + 0.25;
        else if (!(valve_angle_raw <= ereg_hansv1_DW.PrevY + 0.25))
            ereg_hansv1_DW.PrevY = valve_angle_raw - 0.25;
    }

    /* Clamp the demand to the servo travel */
    if (rtb_Switch_h < 0.0)        delta_P_pa = 0.0;
    else if (rtb_Switch_h > 180.0) delta_P_pa = 180.0;
    else                           delta_P_pa = rtb_Switch_h;

    rtb_valve_angle = delta_P_pa - ereg_hansv1_DW.PrevY;

    /* Inner servo loop, Kp = 3, Ki = 2, Kd = 0. Output is a speed demand in
     * deg/s that the integrator below turns into position. */
    rtb_FilterCoefficient = (0.0 * rtb_valve_angle
                             - ereg_hansv1_DW.Filter_DSTATE_o) * 100.0;

    rtb_Sum = (3.0 * rtb_valve_angle
               + ereg_hansv1_DW.Integrator_DSTATE_b)
              + rtb_FilterCoefficient;

    if (tmp) {
        ereg_hansv1_DW.Output_DSTATE_g = (rtb_FixPtSum1_h > 1000) ? 0U : rtb_FixPtSum1_h;
        ereg_hansv1_DW.Output_DSTATE_c = (rtb_FixPtSum1_i > 1000) ? 0U : rtb_FixPtSum1_i;
        ereg_hansv1_DW.Output_DSTATE_i = (rtb_FixPtSum1_c > 1000) ? 0U : rtb_FixPtSum1_c;
        ereg_hansv1_DW.Output_DSTATE   = (rtb_FixPtSum1   > 1000) ? 0U : rtb_FixPtSum1;
    }

    /* State updates */

    /* Outer integrator, Ki = 17, held to +/-20 */
    ereg_hansv1_DW.Integrator_DSTATE += 17.0 * rtb_AddConstant * rtb_Switch * 0.001;
    if      (ereg_hansv1_DW.Integrator_DSTATE >  20.0) ereg_hansv1_DW.Integrator_DSTATE =  20.0;
    else if (ereg_hansv1_DW.Integrator_DSTATE < -20.0) ereg_hansv1_DW.Integrator_DSTATE = -20.0;

    ereg_hansv1_DW.Filter_DSTATE += 0.001 * rtb_NProdOut;

    /* Servo speed limit */
    if      (rtb_Sum >  180.0) rtb_Sum =  180.0;
    else if (rtb_Sum < -180.0) rtb_Sum = -180.0;

    /* Servo position. The state needs clamping as well as the output - let it
     * run past 180 and the servo takes as long to start closing as the fill
     * took to wind it up there. */
    ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE_f += 0.001 * rtb_Sum;
    if      (ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE_f > 180.0)
        ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE_f = 180.0;
    else if (ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE_f <   0.0)
        ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE_f =   0.0;

    /* Inner filter and integrator, Ki held to +/-180 so it cannot fight a
     * close command after a long fill */
    ereg_hansv1_DW.Filter_DSTATE_o     += 0.001 * rtb_FilterCoefficient;
    ereg_hansv1_DW.Integrator_DSTATE_b += 2.0 * rtb_valve_angle * 0.001;
    if      (ereg_hansv1_DW.Integrator_DSTATE_b >  180.0) ereg_hansv1_DW.Integrator_DSTATE_b =  180.0;
    else if (ereg_hansv1_DW.Integrator_DSTATE_b < -180.0) ereg_hansv1_DW.Integrator_DSTATE_b = -180.0;

    if (ereg_hansv1_M->Timing.TaskCounters.TID[1] == 0)
        ereg_hansv1_M->Timing.clockTick1++;

    rate_scheduler();

    /* Servo position out to the hardware. The 1.7 gear ratio stays inside the
     * valve model, main.c gets the motor angle. */
    {
        double out = ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE_f;
        if      (out <   0.0) out =   0.0;
        else if (out > 180.0) out = 180.0;
        ereg_servo_out_deg = out;
    }
}

void ereg_hansv1_initialize(void)
{
    /* Tank model states, left in but unused now the PTs drive P1/P2 */
    ereg_hansv1_DW.DiscreteTimeIntegrator2_DSTATE = 2.2972972972972969;
    ereg_hansv1_DW.DiscreteTimeIntegrator_DSTATE  = 0.2815315315315316;
    ereg_hansv1_DW.DiscreteTimeIntegrator1_DSTATE = 20.0;

    /* Servo starts almost shut */
    ereg_hansv1_DW.PrevY = 0.5;
}

void ereg_hansv1_terminate(void)
{
}
