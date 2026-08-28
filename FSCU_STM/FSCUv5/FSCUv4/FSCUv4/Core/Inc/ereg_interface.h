/*
 * ereg_interface.h
 *
 * What main.c needs to drive the Simulink EREG pressure regulator.
 *
 * Each step: write the two PT pressures, the setpoint and the enable flag,
 * call ereg_hansv1_step(), then read ereg_servo_out_deg back and command the
 * servo with it.
 */

#ifndef INC_EREG_INTERFACE_H_
#define INC_EREG_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Inputs, written before every ereg_hansv1_step() call */

/* HP / N2 supply tank pressure [bar] */
extern double ereg_P1_hp_bar;

/* Propellant tank pressure [bar] */
extern double ereg_P2_prop_bar;

/* Target propellant tank pressure [bar] */
extern double ereg_setpoint_bar;

/* 0 = hold servo where it is, 1 = active */
extern unsigned char ereg_enable;

/* Output, read after ereg_hansv1_step() returns */

/* Commanded servo angle [0..180 deg], rate limited to 180 deg/s by the inner
 * servo loop */
extern double ereg_servo_out_deg;

/* Model entry points */
void ereg_hansv1_initialize(void);
void ereg_hansv1_step(void);
void ereg_hansv1_terminate(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_EREG_INTERFACE_H_ */
