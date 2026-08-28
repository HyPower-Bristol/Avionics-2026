/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../ads131m0x/ads131m0x.h"
#include "ads_helper.h"
#include "pwm.h"
#include "can.h"
#include "can_ids.h"
#include "ereg_interface.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define HTIM2_ARR 999
#define can_hb_delay 5000

/* ── Board / mode selection: define exactly one ── */
/* #define FSCU */
/* #define ECU  */
/* #define GSCU */
#define EREG  /* Electronic Regulator – PT + servo bench test */

#ifdef ECU
#define STARTID 0x240
#endif

#ifdef FSCU
#define STARTID 0x340
#endif

#ifdef GSCU
#define STARTID 0x140
#endif

#ifdef EREG
#define STARTID        0x240   /* same CAN base as ECU for PT data        */
#define EREG_HP_CH     4       /* ADS131 channel index for HP / N2 PT     */
#define EREG_PROP_CH   5       /* ADS131 channel index for prop-tank PT   */
#define EREG_SERVO_CH  TIM_CHANNEL_1  /* TIM2 channel driving the reg servo */
#endif
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PT_CS_Pin GPIO_PIN_13
#define PT_CS_GPIO_Port GPIOC
#define PT_IQR_Pin GPIO_PIN_14
#define PT_IQR_GPIO_Port GPIOC
#define EGPIO_Pin GPIO_PIN_15
#define EGPIO_GPIO_Port GPIOC
#define LEDC_Pin GPIO_PIN_0
#define LEDC_GPIO_Port GPIOC
#define FLSH_CS_Pin GPIO_PIN_1
#define FLSH_CS_GPIO_Port GPIOC
#define STB_CS_Pin GPIO_PIN_4
#define STB_CS_GPIO_Port GPIOA
#define STA_IQR_Pin GPIO_PIN_4
#define STA_IQR_GPIO_Port GPIOC
#define STA_CS_Pin GPIO_PIN_5
#define STA_CS_GPIO_Port GPIOC
#define STR_SYNC_Pin GPIO_PIN_2
#define STR_SYNC_GPIO_Port GPIOB
#define STXC_Pin GPIO_PIN_14
#define STXC_GPIO_Port GPIOB
#define EGPIOB15_Pin GPIO_PIN_15
#define EGPIOB15_GPIO_Port GPIOB
#define EGPIOC8_Pin GPIO_PIN_8
#define EGPIOC8_GPIO_Port GPIOC
#define EGPIOA9_Pin GPIO_PIN_9
#define EGPIOA9_GPIO_Port GPIOA
#define LEDB_Pin GPIO_PIN_10
#define LEDB_GPIO_Port GPIOA
#define EGPIOA11_Pin GPIO_PIN_11
#define EGPIOA11_GPIO_Port GPIOA
#define LEDA_Pin GPIO_PIN_12
#define LEDA_GPIO_Port GPIOA
#define SV_PT_SYNC_Pin GPIO_PIN_15
#define SV_PT_SYNC_GPIO_Port GPIOA
#define SV_CS_Pin GPIO_PIN_2
#define SV_CS_GPIO_Port GPIOD
#define EGPIOB5_Pin GPIO_PIN_5
#define EGPIOB5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern uint8_t noRxRequiredBuff[16];
extern ads131_t PTAM;


extern uint32_t ChanXRaw;
extern float ChanXFloat;
extern uint8_t wrong;

extern ads131_channels_val_t channels_data;
extern volatile float bar_6;
extern volatile float bar_7;

extern uint8_t trig_int;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
