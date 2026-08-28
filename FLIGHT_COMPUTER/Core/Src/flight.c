#include "flight.h"
#include "pyro.h"
#include <math.h>

#define G_MS2   9.80665f

FlightData flight = { .state = FS_IDLE };

static bool     s_active = false;
static uint32_t s_launch_confirm_start = 0;
static uint8_t  s_descent_count = 0;
static float    s_landed_anchor = 0.0f;
static uint32_t s_landed_start = 0;

/* Vertical-axis handling for velocity integration.
   On the pad we find which accel axis holds gravity (the thrust axis) and
   record its rest reading — subtracting that later cancels BOTH gravity and
   the ADXL375's zero-g offset, leaving true kinematic acceleration. */
static uint8_t  s_vaxis = 2;          /* index of vertical axis (default Z) */
static float    s_rest_ms2 = G_MS2;   /* rest reading on vaxis, m/s^2       */
static float    s_up_sign = 1.0f;     /* +1 if boost makes vaxis read up    */
static uint32_t s_last_ms = 0;

/* Altitude AGL from pressure vs the pad reference (barometric formula) */
static float pa_to_agl(float pa, float ground_pa)
{
    if (pa <= 0.0f || ground_pa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(pa / ground_pa, 0.1903f));
}

void Flight_Enable(float ground_pressure_pa, bool dry_run)
{
    flight.state        = FS_PAD;
    flight.ground_pa    = ground_pressure_pa;
    flight.agl_m        = 0.0f;
    flight.max_agl_m    = 0.0f;
    flight.velocity     = 0.0f;
    flight.max_vel      = 0.0f;
    flight.drogue_fired = false;
    flight.main_fired   = false;
    flight.dry_run      = dry_run;
    s_active = true;
    s_launch_confirm_start = 0;
    s_descent_count = 0;
    s_last_ms = 0;
}

void Flight_Disable(void)
{
    s_active = false;
    flight.state = FS_IDLE;
    Pyro_Disarm();
}

bool Flight_IsActive(void) { return s_active; }

/* Fire a channel: just-in-time arm (the state gate has already been passed,
   so this is a trusted deploy), then a time-limited pulse. In dry-run
   (bench test) the state still advances but NO pyro output happens. */
static void deploy(PyroChannel ch)
{
    if (flight.dry_run) return;   /* TEST mode: log the event, don't fire */
    Pyro_Arm();
    Pyro_Fire(ch, PYRO_FIRE_MS_DEFAULT);
}

void Flight_Update(float baro_pa, const float accel_g[3], uint32_t now_ms)
{
    if (!s_active) return;

    /* magnitude for launch detect; per-axis for velocity */
    float mag_ms2 = sqrtf(accel_g[0]*accel_g[0] +
                          accel_g[1]*accel_g[1] +
                          accel_g[2]*accel_g[2]) * G_MS2;

    float dt = 0.0f;
    if (s_last_ms != 0) dt = (float)(now_ms - s_last_ms) * 0.001f;
    s_last_ms = now_ms;
    if (dt < 0.0f || dt > 0.2f) dt = 0.0f;   /* skip bad/first steps */

    flight.agl_m = pa_to_agl(baro_pa, flight.ground_pa);
    if (flight.agl_m > flight.max_agl_m) flight.max_agl_m = flight.agl_m;

    /* Integrate vertical velocity once launched (kinematic accel = current
       vaxis reading minus its pad rest value; oriented so up is positive) */
    if (flight.state == FS_BOOST || flight.state == FS_COAST) {
        float a_v = accel_g[s_vaxis] * G_MS2;          /* signed, m/s^2 */
        float kin = s_up_sign * (a_v - s_rest_ms2);    /* up-positive   */
        flight.velocity += kin * dt;
        if (flight.velocity > flight.max_vel) flight.max_vel = flight.velocity;
    }

    switch (flight.state) {

    case FS_PAD:
        /* keep ground ref fresh + track which axis holds gravity */
        flight.ground_pa = flight.ground_pa * 0.99f + baro_pa * 0.01f;
        {
            uint8_t ax = 0;
            if (fabsf(accel_g[1]) > fabsf(accel_g[ax])) ax = 1;
            if (fabsf(accel_g[2]) > fabsf(accel_g[ax])) ax = 2;
            s_vaxis    = ax;
            s_rest_ms2 = accel_g[ax] * G_MS2;   /* includes gravity+offset */
            s_up_sign  = (accel_g[ax] >= 0.0f) ? 1.0f : -1.0f;
        }
        if (mag_ms2 > FLIGHT_LAUNCH_MS2) {
            if (s_launch_confirm_start == 0)
                s_launch_confirm_start = now_ms;
            else if (now_ms - s_launch_confirm_start >= FLIGHT_LAUNCH_CONFIRM_MS) {
                flight.launch_tick = now_ms;
                flight.velocity = 0.0f;
                flight.state = FS_BOOST;
            }
        } else {
            s_launch_confirm_start = 0;
        }
        break;

    case FS_BOOST:
        /* MACH INHIBIT: deploy disabled until subsonic (velocity gate) AND
           past a minimum time. No sim numbers — self-adapting to the flight. */
        if (flight.velocity < FLIGHT_SUBSONIC_VEL &&
            (now_ms - flight.launch_tick) >= FLIGHT_MIN_LOCKOUT_MS)
            flight.state = FS_COAST;
        break;

    case FS_COAST:
        /* Apogee: baro sustained descent (reliable now that we're slow),
           OR the integrated velocity has gone negative (backup). */
        if (flight.agl_m < flight.max_agl_m - FLIGHT_APOGEE_DROP_M) {
            if (++s_descent_count >= FLIGHT_APOGEE_CONFIRM) {
                deploy(PYRO_CH_C);            /* DROGUE */
                flight.drogue_fired = true;
                flight.state = FS_DROGUE;
            }
        } else {
            s_descent_count = 0;
        }
        /* velocity backup: only valid if it ACTUALLY went fast (guards
           against bench shaking faking an apogee) */
        if (!flight.drogue_fired && flight.velocity <= 0.0f &&
            flight.max_vel > FLIGHT_MIN_APOGEE_VEL) {
            deploy(PYRO_CH_C);                /* DROGUE (velocity backup) */
            flight.drogue_fired = true;
            flight.state = FS_DROGUE;
        }
        break;

    case FS_DROGUE:
        /* MAIN only if the rocket genuinely climbed above the main altitude
           — prevents firing main instantly at low altitude (e.g. on the
           bench, or a drogue that mis-fired low). */
        if (flight.agl_m <= FLIGHT_MAIN_ALT_M &&
            flight.max_agl_m > FLIGHT_MAIN_MIN_APOGEE_M) {
            deploy(PYRO_CH_D);                /* MAIN */
            flight.main_fired = true;
            flight.state = FS_MAIN;
            s_landed_anchor = flight.agl_m;
            s_landed_start  = now_ms;
        }
        break;

    case FS_MAIN:
        /* Landed = altitude stays within a small band for LANDED_MS */
        if (fabsf(flight.agl_m - s_landed_anchor) > FLIGHT_LANDED_BAND_M) {
            s_landed_anchor = flight.agl_m;
            s_landed_start  = now_ms;
        } else if (now_ms - s_landed_start >= FLIGHT_LANDED_MS) {
            flight.state = FS_LANDED;
        }
        break;

    case FS_LANDED:
        Flight_Disable();       /* safe pyros, stop */
        break;

    default:
        break;
    }
}
