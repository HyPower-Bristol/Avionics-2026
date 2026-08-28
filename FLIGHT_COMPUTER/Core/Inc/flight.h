#ifndef INC_FLIGHT_H_
#define INC_FLIGHT_H_

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================== */
/*  SET THESE FOR YOUR ROCKET + MOTOR BEFORE FLIGHT (from your sim).      */
/* ===================================================================== */
/* Boost acceleration to declare launch, in m/s^2 (measured |a|).
   Rest ≈ 9.81 (1 g). 20 m/s^2 ≈ 2 g. Real supersonic boost is far higher,
   so this triggers reliably right at ignition. Raise if pad wind/handling
   ever false-triggers (the CONFIRM_MS sustain guard makes that unlikely). */
#define FLIGHT_LAUNCH_MS2         20.0f
#define FLIGHT_LAUNCH_CONFIRM_MS  100u   /* sustained above threshold */

/* MACH INHIBIT — velocity-based, needs NO sim numbers (self-adapting).
   Vertical speed is integrated from the high-g accelerometer; deployment
   is inhibited while the rocket is faster than this (well subsonic, so
   baro is trustworthy below it). Mach 1 ≈ 340 m/s; 100 m/s ≈ Mach 0.3
   gives wide margin against transonic baro corruption. */
#define FLIGHT_SUBSONIC_VEL       100.0f  /* m/s: deploy allowed below this */
#define FLIGHT_MIN_LOCKOUT_MS     2000u   /* absolute min inhibit after
                                             launch (ignition-glitch guard) */

/* Dual-deploy altitudes / detection */
#define FLIGHT_MAIN_ALT_M         300.0f /* main deploy AGL */
#define FLIGHT_APOGEE_DROP_M      8.0f   /* AGL drop below peak = apogee  */
/* Safety gates (prevent bench/low-flight false deploys):
   - main only if the rocket actually climbed above the main altitude
   - velocity-based apogee backup only if it actually went fast */
#define FLIGHT_MAIN_MIN_APOGEE_M  350.0f /* apogee must exceed this for main */
#define FLIGHT_MIN_APOGEE_VEL     50.0f  /* m/s peak before vel-backup valid */
#define FLIGHT_APOGEE_CONFIRM     5u     /* consecutive descent samples   */
#define FLIGHT_LANDED_BAND_M      3.0f   /* altitude stable within this…  */
#define FLIGHT_LANDED_MS          10000u /* …for this long = landed       */
/* ===================================================================== */

typedef enum {
    FS_IDLE = 0,   /* not armed for flight            */
    FS_PAD,        /* armed, waiting for launch        */
    FS_BOOST,      /* launched — DEPLOY LOCKED OUT      */
    FS_COAST,      /* lockout passed, watching apogee  */
    FS_DROGUE,     /* drogue fired, descending          */
    FS_MAIN,       /* main fired, descending            */
    FS_LANDED
} FlightState;

typedef struct {
    FlightState state;
    float    ground_pa;    /* pad pressure reference          */
    float    agl_m;        /* current altitude AGL            */
    float    max_agl_m;    /* peak (apogee) seen              */
    float    velocity;     /* vertical speed, m/s (accel-integrated) */
    float    max_vel;      /* peak speed seen (for apogee-backup gate) */
    uint32_t launch_tick;
    bool     drogue_fired;
    bool     main_fired;
    bool     dry_run;      /* true = TEST: run logic, do NOT fire pyro */
} FlightData;

extern FlightData flight;

/* Enable the state machine: captures the pad pressure reference and moves
   to FS_PAD. dry_run=true runs the full logic but NEVER fires pyro (bench
   testing). dry_run=false is LIVE flight — pyro will deploy. */
void Flight_Enable(float ground_pressure_pa, bool dry_run);
void Flight_Disable(void);
bool Flight_IsActive(void);

/* Call every loop with fresh sensor data. Drives states + fires pyro.
   accel_g[3] = high-g accelerometer XYZ in g (for velocity integration). */
void Flight_Update(float baro_pa, const float accel_g[3], uint32_t now_ms);

#endif /* INC_FLIGHT_H_ */
