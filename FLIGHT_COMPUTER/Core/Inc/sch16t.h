#ifndef SCH16T_H
#define SCH16T_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Return codes */
#define SCH16T_OK                 0
#define SCH16T_ERR_NULL_PTR      -1
#define SCH16T_ERR_INVALID_PARAM -2
#define SCH16T_ERR_SENSOR_INIT   -3
#define SCH16T_ERR_OTHER         -4

/* Sensor variant */
#define SCH16T_VARIANT_K01  1   /* Rate: 1600/3200/6400 LSB/dps  */
#define SCH16T_VARIANT_K10  2   /* Rate:  100/ 200/ 400 LSB/dps  */

/* Axis indices */
#define SCH16T_AXIS_X  0
#define SCH16T_AXIS_Y  1
#define SCH16T_AXIS_Z  2

typedef struct {
    int32_t Rate1_raw[3];
    int32_t Rate2_raw[3];
    int32_t Acc1_raw[3];
    int32_t Acc2_raw[3];
    int32_t Acc3_raw[3];
    int32_t Temp_raw;
    bool    frame_error;
} SCH16T_RawData;

typedef struct {
    float Rate1[3];
    float Rate2[3];
    float Acc1[3];
    float Acc2[3];
    float Acc3[3];
    float Temp;
} SCH16T_Result;

typedef struct {
    uint16_t Summary;
    uint16_t Summary_Sat;
    uint16_t Common;
    uint16_t Rate_Common;
    uint16_t Rate_X;
    uint16_t Rate_Y;
    uint16_t Rate_Z;
    uint16_t Acc_X;
    uint16_t Acc_Y;
    uint16_t Acc_Z;
} SCH16T_Status;

typedef struct { uint16_t Rate12; uint16_t Acc12; uint16_t Acc3; } SCH16T_Filter;
typedef struct { uint16_t Rate1; uint16_t Rate2; uint16_t Acc1; uint16_t Acc2; uint16_t Acc3; } SCH16T_Sensitivity;
typedef struct { uint16_t Rate2; uint16_t Acc2; } SCH16T_Decimation;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    GPIO_TypeDef      *reset_port;  /* NULL if EXTRESN not connected */
    uint16_t           reset_pin;
    uint8_t            ta9_8;       /* bits 9:8 of target address (TA9/TA8 solder pads), default 0 */
    uint8_t            variant;     /* SCH16T_VARIANT_K01 or SCH16T_VARIANT_K10 */
    /* internal — set by SCH16T_Begin */
    int sens_rate1, sens_rate2;
    int sens_acc1,  sens_acc2, sens_acc3;
} SCH16T_Handle;

/* Public API */
int  SCH16T_Begin(SCH16T_Handle *h,
                  SCH16T_Filter filter,
                  SCH16T_Sensitivity sens,
                  SCH16T_Decimation dec,
                  bool enableDRY);

void SCH16T_GetData(SCH16T_Handle *h, SCH16T_RawData *data);
void SCH16T_GetDataDecimated(SCH16T_Handle *h, SCH16T_RawData *data);
void SCH16T_GetDataAux(SCH16T_Handle *h, SCH16T_RawData *data);
void SCH16T_ConvertData(SCH16T_Handle *h, SCH16T_RawData *in, SCH16T_Result *out);
void SCH16T_ConvertDataDecimated(SCH16T_Handle *h, SCH16T_RawData *in, SCH16T_Result *out);
void SCH16T_ConvertDataAux(SCH16T_Handle *h, SCH16T_RawData *in, SCH16T_Result *out);
int  SCH16T_GetStatus(SCH16T_Handle *h, SCH16T_Status *status);
bool SCH16T_VerifyStatus(SCH16T_Status *status);
void SCH16T_Reset(SCH16T_Handle *h);
void SCH16T_SPIReset(SCH16T_Handle *h);

#endif /* SCH16T_H */
