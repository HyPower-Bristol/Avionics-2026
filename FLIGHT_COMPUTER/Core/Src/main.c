/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "bmp585.h"
#include "sch16t.h"
#include "gnss_ublox.h"
#include "kalman_imu.h"
#include "s25fl512s.h"
#include "sx1262.h"
#include "pyro.h"
#include "bno055.h"
#include "adxl375.h"
#include "flash_log.h"
#include "flight.h"
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */
BMP585_t bmp585;
BMP585_Status bmp585_status;

/* Watch these in Live Expressions */
volatile float baro_temp_C    = 0.0f;
volatile float baro_press_Pa  = 0.0f;
volatile float baro_press_hPa = 0.0f;
volatile uint8_t baro_i2c_found_addr = 0;

/* SCH16T IMU on SPI4 — PE3=CS, PE4=RESET */
SCH16T_Handle  imu;
SCH16T_RawData imu_raw;
SCH16T_Result  imu_result = {0};
volatile int   imu_init_status = 1; /* 0 = SCH16T_OK */

/* Kalman-filtered attitude (degrees). Yaw is gyro-integrated only —
   no absolute reference on a 6-axis IMU, drifts slowly. */
KalmanAxis kf_roll_axis, kf_pitch_axis;
float att_roll  = 0.0f;
float att_pitch = 0.0f;
float att_yaw   = 0.0f;
float att_roll_acc  = 0.0f;  /* raw accel tilt angles, for comparing */
float att_pitch_acc = 0.0f;  /* against the filtered output          */

/* Gyro zero-rate bias (dps), measured at boot while stationary */
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

/* BNO055 9-axis on I2C4 — used for ABSOLUTE yaw only (roll/pitch stay on
   the K10). att_yaw_abs = magnetometer-backed fused heading, 0..360°. */
BNO055_t     bno;
volatile int bno_init_status = -1;   /* BNO055_Status; -1 = not run */
float        att_yaw_abs = 0.0f;     /* fused heading from BNO055    */

/* ADXL375 ±200g high-g accel on I2C1 (with baro) — boost/launch detect. */
ADXL375_t    hg;
volatile int hg_init_status = -1;    /* ADXL375_Status; -1 = not run */

/* S25FL512S 64MB NOR flash on SPI2 — PB12 = CS */
S25FL_t flash;
volatile int flash_init_status = -1; /* S25FL_Status; -1 = not run */
volatile int flash_test_status = -1;

/* SX1262 LoRa on SPI1 (HW NSS PA4) — PC4 = NRESET, PC5 = BUSY */
SX1262_t lora;
volatile int lora_init_status = -1;  /* SX1262_Status; -1 = not run */
volatile int lora_tx_status   = -1;
volatile uint32_t lora_tx_count = 0;
volatile uint32_t lora_cmd_count = 0;   /* valid uplink commands executed */
char lora_last_cmd[24] = "-";           /* last uplink command received  */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART7_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Format a 1e-7-degree coordinate as "+51.458023" (6 decimals ≈ 11 cm) */
static void fmt_coord(char *out, size_t n, int32_t v)
{
    char sgn = (v < 0) ? '-' : '+';
    uint32_t a = (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    snprintf(out, n, "%c%lu.%06lu", sgn,
             (unsigned long)(a / 10000000u),
             (unsigned long)((a % 10000000u) / 10u));
}

/* ------------------------------------------------------------------ */
/* USB console: line-based commands typed into the CDC serial port.    */
/* Called from USB interrupt context — only buffers bytes here.        */
/* ------------------------------------------------------------------ */
static volatile char    con_line[32];
static volatile uint8_t con_ready = 0;

void Console_RxBytes(const uint8_t *data, uint32_t len)
{
    static uint8_t pos = 0;
    for (uint32_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n') {
            if (pos > 0 && !con_ready) {
                con_line[pos] = '\0';
                con_ready = 1;
            }
            pos = 0;
        } else if (pos < sizeof(con_line) - 1) {
            con_line[pos++] = c;
        }
    }
}

static void con_print(const char *s)
{
    CDC_Transmit_FS((uint8_t *)s, (uint16_t)strlen(s));
    HAL_Delay(2);
}

/* Scan I2C4 and print every 7-bit address that ACKs (diagnostic for the
   BNO055 bring-up). BNO055 should show at 0x28 (or 0x29 if ADR high). */
static void I2C4_Scan(void)
{
    extern I2C_HandleTypeDef hi2c4;
    char line[40];
    uint8_t found = 0;
    con_print("I2C4 scan:\r\n");
    for (uint8_t a = 1; a < 127; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c4, (uint16_t)(a << 1), 2, 10) == HAL_OK) {
            int n = snprintf(line, sizeof(line), "  ACK 0x%02X\r\n", a);
            CDC_Transmit_FS((uint8_t *)line, n);
            HAL_Delay(3);
            found++;
        }
    }
    if (!found) con_print("  (nothing responded)\r\n");
}

/* Execute a command string; returns the response line.
   Shared by the USB console and the LoRa uplink. */
static const char *exec_cmd(const char *cmd)
{
    if (strcmp(cmd, "arm") == 0) {
        Pyro_Arm();
        FlashLog_SetActive(1);   /* auto-log from arm through the whole
                                    flight (independent of the 60s arm
                                    timeout — logging keeps running) */
        return "PYRO: ARMED (60s) + LOG recording\r\n";
    }
    if (strcmp(cmd, "disarm") == 0) {
        Pyro_Disarm();
        return "PYRO: disarmed\r\n";
    }
    if (strcmp(cmd, "fire c") == 0)
        return Pyro_Fire(PYRO_CH_C, PYRO_FIRE_MS_DEFAULT)
             ? "PYRO: FIRING C\r\n" : "PYRO: refused (arm first)\r\n";
    if (strcmp(cmd, "fire d") == 0)
        return Pyro_Fire(PYRO_CH_D, PYRO_FIRE_MS_DEFAULT)
             ? "PYRO: FIRING D\r\n" : "PYRO: refused (arm first)\r\n";
    if (strcmp(cmd, "cw") == 0) {
        /* 8 s unmodulated carrier for RF path probing (blocking) */
        extern SX1262_t lora;
        extern volatile int lora_init_status;
        if (lora_init_status != 0) return "CW: radio not initialised\r\n";
        SX1262_TestCW(&lora, 8000);
        return "CW: done\r\n";
    }
    if (strcmp(cmd, "prep") == 0) {
        S25FL_Status st = FlashLog_Prep(0);
        return (st == S25FL_OK) ? "LOG: region erased, cursor reset\r\n"
                                : "LOG: erase failed\r\n";
    }
    if (strcmp(cmd, "logon") == 0)  { FlashLog_SetActive(1); return "LOG: recording ON\r\n"; }
    if (strcmp(cmd, "logoff") == 0) { FlashLog_SetActive(0); FlashLog_Flush(); return "LOG: recording OFF (flushed)\r\n"; }
    if (strcmp(cmd, "flighton") == 0) {
        extern volatile float baro_press_Pa;
        extern BMP585_Status bmp585_status;
        if (bmp585_status != BMP585_OK)
            return "FLIGHT: REFUSED — baro not OK (no apogee/main possible)\r\n";
        Flight_Enable(baro_press_Pa, false);   /* LIVE: pyro will fire */
        FlashLog_SetActive(1);
        return "FLIGHT: ARMED LIVE — auto-deploy WILL FIRE. drogue=C main=D. `flightoff` aborts.\r\n";
    }
    if (strcmp(cmd, "flighttest") == 0) {
        extern volatile float baro_press_Pa;
        Flight_Enable(baro_press_Pa, true);    /* DRY RUN: no pyro output */
        FlashLog_SetActive(1);
        return "FLIGHT: TEST MODE (dry run) — logic runs, pyro will NOT fire.\r\n";
    }
    if (strcmp(cmd, "flightoff") == 0) {
        Flight_Disable();
        FlashLog_SetActive(0); FlashLog_Flush();
        return "FLIGHT: aborted, pyros safe, log flushed\r\n";
    }
    if (strcmp(cmd, "help") == 0)
        return "cmds: arm|disarm|fire c|fire d|flighton|flighttest|flightoff|prep|dump|logstat|cw|scan|gs|fc|help\r\n";
    return "? (try: help)\r\n";
}

/* Runtime role: 0 = flight computer (beacons telemetry, fires pyro),
   1 = ground station (listens, relays telemetry to USB, uplinks cmds).
   Both boards run identical firmware; type "gs" / "fc" to switch. */
static volatile uint8_t g_ground_station = 0;

/* Process one pending console command (main-loop context) */
static void Console_Poll(void)
{
    if (!con_ready) return;
    char cmd[32];
    strncpy(cmd, (const char *)con_line, sizeof(cmd));
    cmd[sizeof(cmd) - 1] = '\0';
    con_ready = 0;

    /* Mode switches work in either role */
    if (strcmp(cmd, "gs") == 0) {
        g_ground_station = 1;
        extern SX1262_t lora;
        extern volatile int lora_init_status;
        if (lora_init_status == 0) SX1262_StartRx(&lora);
        con_print("MODE: GROUND STATION (listening; type cmds to uplink)\r\n");
        return;
    }
    if (strcmp(cmd, "fc") == 0) {
        g_ground_station = 0;
        con_print("MODE: flight computer\r\n");
        return;
    }
    if (strcmp(cmd, "scan") == 0) { I2C4_Scan(); return; }
    if (strcmp(cmd, "dump") == 0) { FlashLog_Dump(); return; }
    if (strcmp(cmd, "logstat") == 0) {
        char m[48];
        snprintf(m, sizeof(m), "LOG: %s, %lu bytes used\r\n",
                 FlashLog_IsActive() ? "ON" : "off",
                 (unsigned long)FlashLog_BytesUsed());
        con_print(m);
        return;
    }

    if (g_ground_station) {
        /* Relay the typed command to the flight FC as an uplink packet */
        extern SX1262_t lora;
        extern volatile int lora_init_status;
        if (lora_init_status != 0) { con_print("uplink: radio down\r\n"); return; }
        char pkt[40];
        int n = snprintf(pkt, sizeof(pkt), "HANSCMD %s", cmd);
        if (n > 0) SX1262_Transmit(&lora, (uint8_t *)pkt, (uint8_t)n, 500);
        SX1262_StartRx(&lora);   /* back to listening */
        char msg[48];
        snprintf(msg, sizeof(msg), ">> uplinked: %s\r\n", cmd);
        con_print(msg);
    } else {
        con_print(exec_cmd(cmd));   /* execute locally on the flight FC */
    }
}

/* Ground-station loop task: print any received downlink packet to USB
   in the same "[payload] RSSI= SNR=" format the Arduino bridge used, so
   the existing GUI parses it unchanged. */
static void GroundStation_Task(void)
{
    extern SX1262_t lora;
    extern volatile int lora_init_status;
    static uint32_t rx_count = 0;
    if (lora_init_status != 0) return;

    uint8_t rbuf[132];
    uint8_t rlen = sizeof(rbuf) - 1;
    if (SX1262_CheckRx(&lora, rbuf, &rlen) && rlen > 0) {
        rbuf[rlen] = '\0';
        rx_count++;
        char out[180];
        int n = snprintf(out, sizeof(out), "#%lu [%s] RSSI=%d SNR=%d\r\n",
                         (unsigned long)rx_count, (char *)rbuf,
                         (int)lora.rx_rssi_dbm, (int)lora.rx_snr_db);
        CDC_Transmit_FS((uint8_t *)out, n);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_USB_DEVICE_Init();
  MX_UART7_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_ADC3_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(3000); /* let USB enumerate on the PC before we start transmitting */

  /* --- SCH16T IMU bring-up ---
   * Change SCH16T_VARIANT_K01 to SCH16T_VARIANT_K10 if you have the K10.
   * Adjust filter / sensitivity / decimation values to suit your application.
   */
  imu.hspi        = &hspi4;
  imu.cs_port     = GPIOE;
  imu.cs_pin      = GPIO_PIN_3;
  imu.reset_port  = GPIOE;
  imu.reset_pin   = GPIO_PIN_4;
  imu.ta9_8       = 0;               /* default — no TA solder jumpers set */
  imu.variant     = SCH16T_VARIANT_K10;

  SCH16T_Filter      imu_filter = { .Rate12 = 68,  .Acc12 = 68,  .Acc3 = 68 };
  /* K10 valid rate sensitivities: 100 / 200 / 400 LSB/dps  */
  SCH16T_Sensitivity imu_sens   = { .Rate1 = 200, .Rate2 = 200,
                                    .Acc1  = 6400, .Acc2  = 6400, .Acc3 = 6400 };
  SCH16T_Decimation  imu_dec    = { .Rate2 = 8, .Acc2 = 8 };

  imu_init_status = SCH16T_Begin(&imu, imu_filter, imu_sens, imu_dec, false);

  /* Gyro bias calibration: average 100 samples (~2 s) while stationary.
     KEEP THE BOARD STILL from power-on until the plotter starts.
     Dominates yaw drift — the yaw axis has no absolute reference to
     correct against, so any uncompensated bias integrates forever. */
  if (imu_init_status == SCH16T_OK) {
    float sum[3] = {0};
    int   good = 0;
    for (int i = 0; i < 100; i++) {
      SCH16T_GetData(&imu, &imu_raw);
      if (!imu_raw.frame_error) {
        SCH16T_ConvertData(&imu, &imu_raw, &imu_result);
        sum[0] += imu_result.Rate1[0];
        sum[1] += imu_result.Rate1[1];
        sum[2] += imu_result.Rate1[2];
        good++;
      }
      HAL_Delay(20);
    }
    if (good > 50) {
      gyro_bias[0] = sum[0] / (float)good;
      gyro_bias[1] = sum[1] / (float)good;
      gyro_bias[2] = sum[2] / (float)good;
    }
  }

  /* --- BMP585 bring-up ---
   * Step 1: Scan I2C to confirm a device is present (checks wiring).
   * Step 2: Init with default address (SDO=GND → 0x46<<1).
   *         If init returns BMP585_WRONG_ID, try BMP585_I2C_ADDR_SDO_VDD.
   */
  BMP585_ScanI2C(&hi2c1, &baro_i2c_found_addr);  /* breakpoint here, check baro_i2c_found_addr */

  bmp585_status = BMP585_Init(&bmp585, &hi2c1, BMP585_I2C_ADDR_SDO_GND);
  /* bmp585_status should be BMP585_OK (0) — if BMP585_WRONG_ID, flip SDO address above */

  /* --- U-BLOX M10S GNSS bring-up ---
   * PB2 = RESET_N (active-low).  GPIO init left it LOW (module held in reset).
   * GNSS_UBX_Init releases it, enables UART7 IRQ, and starts receive.
   * Driver switches module 9600→115200 then parses UBX NAV-PVT at 10 Hz.
   */
  GNSS_UBX_Init(&huart7, GPIOB, GPIO_PIN_2);

  /* Attitude Kalman filters — start level; they converge within ~1 s */
  KalmanAxis_Init(&kf_roll_axis, 0.0f);
  KalmanAxis_Init(&kf_pitch_axis, 0.0f);
  uint32_t att_last_tick = HAL_GetTick();

  /* --- S25FL512S flash bring-up ---
   * PD8 = RESET# (active-low): GPIO init leaves it LOW (chip in reset),
   * release it before talking. tRPH ≈ 35 µs — 1 ms is plenty.
   * Init verifies JEDEC ID (expect 01 02 20).
   * Self-test erases/writes/verifies the LAST 256KB sector —
   * remove once real log data lives on the chip. */
  HAL_GPIO_WritePin(FLASH_RESET_GPIO_Port, FLASH_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  flash_init_status = S25FL_Init(&flash, &hspi2, GPIOB, GPIO_PIN_12);
  if (flash_init_status == S25FL_OK)
      flash_test_status = S25FL_SelfTest(&flash);

  /* Telemetry logger — scans flash for resume cursor. Use console:
     prep (erase on pad) / logon / logoff / dump / logstat */
  if (flash_init_status == S25FL_OK)
      FlashLog_Init(&flash);

  /* --- SX1262 LoRa bring-up ---
   * Set LORA_ENABLED to 0 to keep the radio in reset — do this whenever
   * no antenna is fitted (transmitting into an open connector reflects
   * the full PA output back into the chip and can damage it). */
#define LORA_ENABLED 1
#if LORA_ENABLED
  lora_init_status = SX1262_Init(&lora, &hspi1,
                                 GPIOA, GPIO_PIN_4,   /* CS (soft)   */
                                 LORA_NRESET_GPIO_Port, LORA_NRESET_Pin,
                                 GPIOC, GPIO_PIN_5);  /* BUSY        */
#endif

  /* --- Pyro channels (recovery) ---
   * FETs held off; firing requires "arm" then "fire c"/"fire d" over
   * the USB serial console. Continuity sensed via ADC3 at ~5 Hz. */
  Pyro_Init(&hadc3);

  /* --- BNO055 9-axis (I2C4) — absolute yaw source ---
   * Roll/pitch stay on the K10; only the fused heading is used, for yaw.
   * ext_crystal=0 (bare module). If it has a 32 kHz crystal, pass 1.
   * NOTE: hi2c4 must be <= 100 kHz (BNO055 clock-stretches). */
  bno_init_status = BNO055_Init(&bno, &hi2c4, BNO055_ADDR_LOW, 0);

  /* --- ADXL375 ±200g high-g accel (I2C1, shares bus with baro) ---
   * For launch/boost detection. ALT ADDRESS low → 0x53; if it doesn't
   * respond, run `scan` on I2C1 or try ADXL375_ADDR_HIGH. */
  hg_init_status = ADXL375_Init(&hg, &hi2c1, ADXL375_ADDR_LOW);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Ground-station role: listen + relay only, skip all flight tasks */
    if (g_ground_station) {
      GroundStation_Task();
      Console_Poll();
      HAL_Delay(5);   /* fast RX polling */
      continue;
    }

    if (imu_init_status == SCH16T_OK) {
      SCH16T_GetData(&imu, &imu_raw);
      if (!imu_raw.frame_error) {
        SCH16T_ConvertData(&imu, &imu_raw, &imu_result);

        /* Measured timestep — loop is nominally 20 ms but CDC/GNSS work
           makes it jitter; the filter integrates correctly either way */
        uint32_t now_tick = HAL_GetTick();
        float dt = (float)(now_tick - att_last_tick) * 0.001f;
        att_last_tick = now_tick;
        if (dt <= 0.0f || dt > 0.2f) dt = 0.02f; /* guard first pass/stalls */

        /* Tilt angles from accelerometer (absolute reference, noisy) */
        float ax = imu_result.Acc1[0];
        float ay = imu_result.Acc1[1];
        float az = imu_result.Acc1[2];
        float roll_acc  =  atan2f(ay, az)                    * 57.2957795f;
        float pitch_acc =  atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;
        att_roll_acc  = roll_acc;   /* exposed for filter diagnosis */
        att_pitch_acc = pitch_acc;

        /* Kalman: gyro prediction + accel correction, with online gyro
           bias estimation (replaces the complementary filter) */
        att_roll  = KalmanAxis_Update(&kf_roll_axis,  roll_acc,
                                      imu_result.Rate1[0] - gyro_bias[0], dt);
        att_pitch = KalmanAxis_Update(&kf_pitch_axis, pitch_acc,
                                      imu_result.Rate1[1] - gyro_bias[1], dt);

        /* Yaw (relative): bias-corrected gyro integration — kept as a
           fast, drift-prone backup / cross-check for the BNO heading */
        att_yaw += (imu_result.Rate1[2] - gyro_bias[2]) * dt;
        if (att_yaw >  180.0f) att_yaw -= 360.0f;
        if (att_yaw < -180.0f) att_yaw += 360.0f;
      }
    }

    /* Absolute yaw from the BNO055 fused heading (magnetometer-backed).
       Only trustworthy once bno.cal_mag == 3 (do figure-8s at startup). */
    if (bno_init_status == BNO055_OK) {
      if (BNO055_ReadEuler(&bno) == BNO055_OK)
        att_yaw_abs = bno.heading;
    }

    /* High-g accelerometer — reads |a| for launch/boost detection */
    if (hg_init_status == ADXL375_OK)
      ADXL375_Read(&hg);

    /* Flight state machine: launch detect → velocity-gated mach inhibit →
       apogee (drogue) → 300 m (main) → landed. Fires pyro C/D.
       Active only after `flighton`. */
    Flight_Update(baro_press_Pa, hg.g, HAL_GetTick());

    /* Telemetry logging: append one record per loop while active (~50 Hz).
       In-flight writes are page-programs only — the log region is
       pre-erased on the pad via `prep`, never erased here. */
    if (FlashLog_IsActive()) {
      FlashLogRecord r = {0};
      r.t_ms       = HAL_GetTick();
      r.roll_x10   = (int16_t)(att_roll     * 10.0f);
      r.pitch_x10  = (int16_t)(att_pitch    * 10.0f);
      r.yaw_x10    = (int16_t)(att_yaw_abs  * 10.0f);
      r.hg_x_x10   = (int16_t)(hg.g[0]      * 10.0f);
      r.hg_y_x10   = (int16_t)(hg.g[1]      * 10.0f);
      r.hg_z_x10   = (int16_t)(hg.g[2]      * 10.0f);
      r.hg_mag_x10 = (int16_t)(hg.mag_g     * 10.0f);
      r.baro_pa    = (int32_t)baro_press_Pa;
      r.baro_alt_cm= (int32_t)(flight.agl_m * 100.0f);   /* AGL, cm */
      r.gps_lat    = gnss_ubx.lat;
      r.gps_lng    = gnss_ubx.lng;
      r.gps_alt_m  = (int16_t)(gnss_ubx.alt_mm / 1000);
      r.sats       = gnss_ubx.num_sats;
      r.fix        = gnss_ubx.status;
      r.state      = (uint8_t)flight.state;
      r.flags      = (uint8_t)((pyro.armed ? 1 : 0) |
                               (flight.drogue_fired ? 2 : 0) |
                               (flight.main_fired   ? 4 : 0));
      FlashLog_Write(&r);
    }

    /* Drive the UBX GNSS state machine (baud switch → configure → parse) */
    GNSS_UBX_Update();

    /* Pyro: end expired fire pulses, expire arming, refresh continuity;
       then handle any console command (arm / fire / disarm) */
    Pyro_Update();
    Console_Poll();

    /* LoRa uplink: radio sits in continuous RX between beacons.
       Commands are "HANSCMD <cmd>" — prefix guards against foreign
       LoRa traffic; <cmd> goes through the same handler as the USB
       console, so all pyro safety interlocks apply identically. */
    if (lora_init_status == 0) {
        uint8_t rbuf[64];
        uint8_t rlen = sizeof(rbuf) - 1;
        if (SX1262_CheckRx(&lora, rbuf, &rlen) && rlen > 8 &&
            memcmp(rbuf, "HANSCMD ", 8) == 0) {
            rbuf[rlen] = '\0';
            const char *cmd = (const char *)&rbuf[8];
            strncpy(lora_last_cmd, cmd, sizeof(lora_last_cmd) - 1);
            lora_last_cmd[sizeof(lora_last_cmd) - 1] = '\0';
            lora_cmd_count++;
            con_print(exec_cmd(cmd));  /* echo result to USB too */
        }
    }

    /* Read barometer (25 Hz ODR, loop at 50 Hz — reads latest sample) */
    if (bmp585_status == BMP585_OK) {
      if (BMP585_ReadData(&bmp585) == BMP585_OK) {
        baro_temp_C    = bmp585.temperature;
        baro_press_Pa  = bmp585.pressure;
        baro_press_hPa = bmp585.pressure / 100.0f;
      }
    }

    /* Serial Port Plotter, all in degrees — filtered vs raw diagnosis:
       ch1 roll  (K10 Kalman)      ch2 roll  (raw accel)
       ch3 pitch (K10 Kalman)      ch4 pitch (raw accel)
       ch5 yaw   (BNO055 fused heading) */
    char tx[80];
    int  tx_len = snprintf(tx, sizeof(tx), "$%ld %ld %ld %ld %ld;\n",
        (long)att_roll,  (long)att_roll_acc,
        (long)att_pitch, (long)att_pitch_acc,
        (long)att_yaw_abs);
    CDC_Transmit_FS((uint8_t *)tx, tx_len);

    /* Print GNSS status once per second (~50 × 20 ms) via CDC text.
       lat/lng are in 1e-7 degrees; alt is in mm. */
    static uint32_t gnss_ctr = 0;
    if (++gnss_ctr >= 50) {
        gnss_ctr = 0;
        extern volatile uint32_t uart7_irq_count;
        char gps[120];
        int  gps_len = snprintf(gps, sizeof(gps),
            "GPS st=%d sats=%d lat=%ld lng=%ld alt=%ldm irq=%lu rx=%lu err=%lu\r\n",
            gnss_ubx.status, gnss_ubx.num_sats,
            (long)gnss_ubx.lat, (long)gnss_ubx.lng,
            (long)(gnss_ubx.alt_mm / 1000),
            (unsigned long)uart7_irq_count,
            (unsigned long)gnss_ubx_rx_count,
            (unsigned long)gnss_uart_err_count);
        CDC_Transmit_FS((uint8_t *)gps, gps_len);

        /* Barometer status — integers to avoid printf-float dependency.
           T in centi-°C (2345 = 23.45 °C), P in Pa (101325 ≈ sea level).
           regs: STATUS(0x28) OSR_CONFIG(0x36) ODR_CONFIG(0x37) OSR_EFF(0x38)
           raw: 6 data bytes T[xlsb lsb msb] P[xlsb lsb msb] */
        HAL_Delay(2); /* let previous CDC packet drain */
        uint8_t rg[5] = {0}, rw[6] = {0};
        HAL_I2C_Mem_Read(&hi2c1, baro_i2c_found_addr, BMP585_REG_STATUS,
                         I2C_MEMADD_SIZE_8BIT, &rg[0], 1, 50);
        HAL_I2C_Mem_Read(&hi2c1, baro_i2c_found_addr, BMP585_REG_DSP_CONFIG,
                         I2C_MEMADD_SIZE_8BIT, &rg[4], 1, 50);
        HAL_I2C_Mem_Read(&hi2c1, baro_i2c_found_addr, BMP585_REG_OSR_CONFIG,
                         I2C_MEMADD_SIZE_8BIT, &rg[1], 3, 50); /* 0x36..0x38 */
        HAL_I2C_Mem_Read(&hi2c1, baro_i2c_found_addr, BMP585_REG_TEMP_XLSB,
                         I2C_MEMADD_SIZE_8BIT, rw, 6, 50);
        char baro[120];
        int  baro_len = snprintf(baro, sizeof(baro),
            "BARO st=%d id=%02X bs=%02X T=%ld P=%ld sr=%02X dsp=%02X osr=%02X odr=%02X eff=%02X raw=%02X%02X%02X %02X%02X%02X\r\n",
            (int)bmp585_status, bmp585.chip_id, bmp585.boot_status,
            (long)(baro_temp_C * 100.0f), (long)baro_press_Pa,
            rg[0], rg[4], rg[1], rg[2], rg[3],
            rw[2], rw[1], rw[0], rw[5], rw[4], rw[3]);
        CDC_Transmit_FS((uint8_t *)baro, baro_len);

        /* BNO055 status: init 0=OK 2=WRONG_ID. hdg = fused yaw (deg).
           cal S/G/A/M each 0..3 — heading is only trustworthy at M=3
           (wave the board in slow figure-8s at startup to raise it). */
        HAL_Delay(2);
        if (bno_init_status == BNO055_OK) BNO055_ReadCalib(&bno);
        char bn[72];
        int  bn_len = snprintf(bn, sizeof(bn),
            "BNO init=%d hdg=%ld calSGAM=%u%u%u%u\r\n",
            bno_init_status, (long)att_yaw_abs,
            bno.cal_sys, bno.cal_gyro, bno.cal_acc, bno.cal_mag);
        CDC_Transmit_FS((uint8_t *)bn, bn_len);

        /* High-g accel: id=E5 when present. x/y/z and |a| in milli-g
           (1000 = 1 g at rest on one axis, mag ≈ 1000 stationary). */
        HAL_Delay(2);
        char hgs[80];
        int  hgs_len = snprintf(hgs, sizeof(hgs),
            "HG init=%d id=%02X x=%ld y=%ld z=%ld magG=%ld\r\n",
            hg_init_status, hg.dev_id,
            (long)(hg.g[0]*1000.0f), (long)(hg.g[1]*1000.0f),
            (long)(hg.g[2]*1000.0f), (long)(hg.mag_g*1000.0f));
        CDC_Transmit_FS((uint8_t *)hgs, hgs_len);

        /* Flight state machine: state 0=idle 1=pad 2=boost 3=coast
           4=drogue 5=main 6=landed. agl in m, apogee = peak agl. */
        HAL_Delay(2);
        char fl[110];
        int  fl_len = snprintf(fl, sizeof(fl),
            "FLIGHT st=%d %s agl=%ldm vel=%ldm/s apogee=%ldm drogue=%d main=%d\r\n",
            (int)flight.state,
            !Flight_IsActive() ? "off" : (flight.dry_run ? "TEST" : "LIVE"),
            (long)flight.agl_m, (long)flight.velocity,
            (long)flight.max_agl_m, flight.drogue_fired, flight.main_fired);
        CDC_Transmit_FS((uint8_t *)fl, fl_len);

        /* Flash status: init 0=OK 2=WRONG_ID, test 0=OK 6=VERIFY_ERR */
        HAL_Delay(2);
        char fls[64];
        int  fls_len = snprintf(fls, sizeof(fls),
            "FLASH init=%d test=%d step=%d sr1=%02X id=%02X %02X %02X\r\n",
            flash_init_status, flash_test_status, s25fl_test_step,
            (flash_init_status == 0) ? S25FL_ReadStatus1(&flash) : 0xEE,
            flash.jedec[0], flash.jedec[1], flash.jedec[2]);
        CDC_Transmit_FS((uint8_t *)fls, fls_len);

        /* LoRa: 1 Hz telemetry beacon (blocking ~35 ms at SF7).
           Pair any 868 MHz LoRa receiver: SF7 BW125 CR4/5, sync 0x1424,
           preamble 12, explicit header, CRC on. */
        if (lora_init_status == SX1262_OK) {
            /* HANSFC,<n>,<state>,<agl_m>,<vel_ms>,<apogee_m>,<lat>,<lng>,
                      <gps_alt_m>,<sats>,<fix>,<roll>,<pitch>,<yaw>,
                      A<armed>,<contC>,<contD> */
            char lat_s[16], lng_s[16];
            fmt_coord(lat_s, sizeof(lat_s), gnss_ubx.lat);
            fmt_coord(lng_s, sizeof(lng_s), gnss_ubx.lng);
            char pkt[128];
            int  pkt_len = snprintf(pkt, sizeof(pkt),
                "HANSFC,%lu,%d,%ld,%ld,%ld,%s,%s,%ld,%u,%u,%ld,%ld,%ld,A%d,%u,%u",
                (unsigned long)lora_tx_count,
                (int)flight.state,
                (long)flight.agl_m, (long)flight.velocity, (long)flight.max_agl_m,
                lat_s, lng_s,
                (long)(gnss_ubx.alt_mm / 1000),
                gnss_ubx.num_sats, gnss_ubx.status,
                (long)att_roll, (long)att_pitch, (long)att_yaw_abs,
                pyro.armed ? 1 : 0,
                pyro.cont_mv[PYRO_CH_C], pyro.cont_mv[PYRO_CH_D]);
            if (pkt_len > SX1262_MAX_PAYLOAD) pkt_len = SX1262_MAX_PAYLOAD;
            lora_tx_status = SX1262_Transmit(&lora, (uint8_t *)pkt,
                                             (uint8_t)pkt_len, 500);
            if (lora_tx_status == SX1262_OK) {
                lora_tx_count++;
                SX1262_StartRx(&lora);  /* open the command window */
            } else {
                /* A brownout mid-TX resets the SX1262 (config lost, BUSY
                   stuck). After 3 consecutive failures, re-init fully. */
                static uint8_t lora_fails = 0;
                if (++lora_fails >= 3) {
                    lora_fails = 0;
                    lora_init_status = SX1262_Init(&lora, &hspi1,
                        GPIOA, GPIO_PIN_4,
                        LORA_NRESET_GPIO_Port, LORA_NRESET_Pin,
                        GPIOC, GPIO_PIN_5);
                }
            }
        } else if (lora_init_status > 0) {
            /* init previously failed — keep retrying once per second */
            lora_init_status = SX1262_Init(&lora, &hspi1,
                GPIOA, GPIO_PIN_4,
                LORA_NRESET_GPIO_Port, LORA_NRESET_Pin,
                GPIOC, GPIO_PIN_5);
        }

        /* Pyro status: armed flag, continuity mV per channel (ematch
           present ≈ VBAT×27/227 ≈ 1000 mV on 2S; open ≈ 0), fire counts */
        HAL_Delay(2);
        char pyr[80];
        int  pyr_len = snprintf(pyr, sizeof(pyr),
            "PYRO armed=%d contC=%umV contD=%umV fired=%lu/%lu\r\n",
            pyro.armed ? 1 : 0,
            pyro.cont_mv[PYRO_CH_C], pyro.cont_mv[PYRO_CH_D],
            (unsigned long)pyro.fired_count[PYRO_CH_C],
            (unsigned long)pyro.fired_count[PYRO_CH_D]);
        CDC_Transmit_FS((uint8_t *)pyr, pyr_len);

        HAL_Delay(2);
        if (lora_init_status >= 0) { /* skip entirely while disabled */
            uint16_t lora_errs = 0xFFFF;
            if (lora_init_status == 0) {
                SX1262_GetStatus(&lora);
                SX1262_GetErrors(&lora, &lora_errs);
            }
            char lr[128];
            int  lr_len = snprintf(lr, sizeof(lr),
                "LORA init=%d st=0x%02X sync=%02X tx=%d stage=%d err=0x%04X n=%lu cmd=%lu[%s] uprssi=%d upsnr=%d\r\n",
                lora_init_status, lora.chip_status, lora.sync_msb,
                lora_tx_status, sx1262_tx_stage, lora_errs,
                (unsigned long)lora_tx_count,
                (unsigned long)lora_cmd_count, lora_last_cmd,
                (int)lora.rx_rssi_dbm, (int)lora.rx_snr_db);
            CDC_Transmit_FS((uint8_t *)lr, lr_len);
        }
    }

    HAL_Delay(20);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00707CBB;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */
  /* CubeMX generates 4-bit/div2 — patch to 8-bit/div4 (8 MHz ≤ SCH16T max 10 MHz).
     SPE=0 here so CFG1 is writable without a DeInit/Init cycle. */
  MODIFY_REG(SPI4->CFG1,
             SPI_CFG1_DSIZE_Msk | SPI_CFG1_FTHLV_Msk | SPI_CFG1_MBR_Msk,
             (7U << SPI_CFG1_DSIZE_Pos) |   /* DSIZE=7 → 8-bit */
             (0U << SPI_CFG1_FTHLV_Pos) |   /* FTHLV=0 → 1-byte threshold */
             (1U << SPI_CFG1_MBR_Pos));      /* MBR=1 → APB2/4 = 8 MHz */
  hspi4.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */
  /* M10S defaults to 9600 — re-init at the correct rate.
     Board has GNSS TX/RX routed straight-through, so use the H7's
     internal pin swap: PE7 acts as TX, PE8 as RX. */
  huart7.Init.BaudRate = 9600;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart7.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart7) != HAL_OK) { Error_Handler(); }
  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FIRE_C_Pin|FIRE_D_Pin|LORA_NRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_RESET_GPIO_Port, FLASH_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : FIRE_C_Pin FIRE_D_Pin LORA_NRESET_Pin */
  GPIO_InitStruct.Pin = FIRE_C_Pin|FIRE_D_Pin|LORA_NRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_BUSY_Pin */
  GPIO_InitStruct.Pin = LORA_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 FLASH_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_RESET_Pin */
  GPIO_InitStruct.Pin = FLASH_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_RESET_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
