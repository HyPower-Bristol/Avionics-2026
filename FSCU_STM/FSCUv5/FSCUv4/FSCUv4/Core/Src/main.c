/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
uint8_t noRxRequiredBuff[16];
ads131_t PTAM;


uint32_t ChanXRaw = 0;
float ChanXFloat = 0.0f;
uint8_t wrong = 0;

ads131_channels_val_t channels_data;
volatile float bar_6;
volatile float bar_7;

uint8_t trig_int = 0;
uint8_t null=0x00;

uint16_t id;
float bar;

#ifdef FSCU
// Forward servo 1
Servo_t fs1 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_1,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x321
  };

//forward servo 2
Servo_t fs2 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_2,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x322
  };

//forward servo 3
Servo_t fs3 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_3,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x323
  };


//forward servo 4
Servo_t fs4 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_4,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x324
  };
#endif
#ifdef ECU
//aft servo 1
Servo_t as1 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_1,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x221
  };

//aft servo 2
Servo_t as2 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_2,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x222
  };
#endif
#ifdef EREG
/* Regulator servo – TIM2 CH1, standard 50 Hz hobby servo (1–2 ms pulse) */
Servo_t ereg_sv = {
      .htim = &htim2,
      .channel = EREG_SERVO_CH,
      .us_min = 1000,
      .us_max = 2000,
      .angle_min = 0,
      .angle_max = 180,
      .can_id = 0x000   /* not used in EREG mode */
  };

/* Vent servo – TIM2 CH2, opens when prop pressure exceeds setpoint + threshold */
Servo_t ereg_vent_sv = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_2,
      .us_min = 1000,
      .us_max = 2000,
      .angle_min = 0,
      .angle_max = 180,
      .can_id = 0x000   /* not used in EREG mode */
  };

float ereg_vent_angle = 180.0f;        /* last commanded vent angle – starts closed */
#define EREG_VENT_OPEN_DEG    0.0f     /* 0°   = fully open  */
#define EREG_VENT_CLOSED_DEG  180.0f   /* 180° = fully closed */
#define EREG_VENT_THRESHOLD   0.5f     /* open vent if P2 > setpoint + this value [bar] */
#define EREG_VENT_HYSTERESIS  0.3f     /* close vent only when P2 drops back below setpoint + THRESHOLD - HYSTERESIS [bar] */
#endif

#ifdef GSCU
//Ground statione srvo 1
Servo_t gs1 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_1,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x125
  };

//ground station servo 2
Servo_t gs2 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_2,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x126
  };

//ground station servo 3
Servo_t gs3 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_3,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x127
  };

//ground sation servo 4
Servo_t gs4 = {
      .htim = &htim2,
      .channel = TIM_CHANNEL_4,
      .us_min = 1000,
      .us_max = 2000,
  	//.pulse_min=250,
  	//.pulse_max=500,
      .angle_min = 0,
      .angle_max = 180,
	  .can_id = 0x128
  };
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t TxMailbox;
uint8_t TxBuffer[8]={0};
CAN_TxHeaderTypeDef GenericTxHeader;
uint8_t n_counter =0;

CAN_RxHeaderTypeDef rxHeader;
uint8_t rxMessage[8];
Servo_t* servoId = NULL;
volatile uint8_t recv_angle;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    if(hcan->Instance == CAN2){
    	if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, rxMessage)!=HAL_OK){

    	}
    	else{
    		//uint8_t reading = rxMessage[7] | rxMessage[0]; // not sure if msb or lsb first?

//    		uint8_t rxData[8];  // local copy
//    		memcpy(rxData, rxMessage, rxHeader.DLC);


    		uint16_t can_id_ack;
    		servoId = NULL;
#ifdef EREG
    		/* EREG does not use CAN for servo control — ignore all incoming messages */
    		return;
#endif
#ifdef FSCU
    		can_id_ack = (HPB_CAN_SOURCE_UPPER_TX | 0xF0 | HPB_CAN_INSTANCE_1);
    		if (rxHeader.ExtId == fs1.can_id) {
    			servoId = &fs1;
    			//ServoActuator(&fs1, dataValue);
    		}
    		else if (rxHeader.ExtId == fs2.can_id) {
    			servoId = &fs2;
    			//ServoActuator(&fs2, dataValue);
    		}
    		else if (rxHeader.ExtId == fs3.can_id) {
    			servoId = &fs3;
    			//ServoActuator(&fs3, dataValue);
    		}
    		else if (rxHeader.ExtId == fs4.can_id) {
    			servoId = &fs4;
    			//ServoActuator(&fs4, dataValue);
    		} else {
    			return;
    		}
#endif
#ifdef ECU
    		can_id_ack = (HPB_CAN_SOURCE_LOWER_TX | 0xF0 | HPB_CAN_INSTANCE_1);
    		if (rxHeader.ExtId == as1.can_id) {
    			servoId = &as1;
    			//ServoActuator(&as1, dataValue);
    		}
    		else if (rxHeader.ExtId == as2.can_id) {
    			servoId = &as2;
    			//ServoActuator(&as2, dataValue);
    		} else {
    			return;
    		}
#endif
#ifdef GSCU

    		can_id_ack = (HPB_CAN_SOURCE_GROUND_SYSTEMS | 0xF0 | HPB_CAN_INSTANCE_1);
    		if (rxHeader.ExtId == gs1.can_id) {
    			servoId = &gs1;

    			//ServoActuator(&gs1, dataValue);
    		}
    		else if (rxHeader.ExtId == gs2.can_id) {
    			servoId = &gs2;

    			//ServoActuator(&gs2, dataValue);
    		}
    		else if (rxHeader.ExtId == gs3.can_id) {
    			servoId = &gs3;

    			//ServoActuator(&gs3, dataValue);
    		}
    		else if (rxHeader.ExtId == gs4.can_id) {
    			servoId = &gs4;

    			//ServoActuator(&gs4, dataValue);
    		} else {
    			return;
    		}
#endif

    		recv_angle = rxMessage[3];
			ServoActuator(servoId, recv_angle);
			n_counter++;
			CAN_TxHeaderTypeDef ackTxHeader = {
							.StdId = can_id_ack,
							.ExtId = 0,
							.IDE = CAN_ID_STD,
							.RTR = CAN_RTR_DATA,
							.DLC = 4,
							.TransmitGlobalTime = DISABLE
						};
			rxMessage[0] = rxHeader.ExtId;
			rxHeader.ExtId = 0;
			if (HAL_CAN_AddTxMessage(&hcan2, &ackTxHeader, rxMessage, &TxMailbox) != HAL_OK)
			{
				return;
			}




    	}
    }
}
uint8_t i;
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_DMA_Init();
  MX_CAN2_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  initADS();
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
  if (HAL_CAN_Start(&hcan2) != HAL_OK){
  	  Error_Handler();
    }
  InitDefaultCANHeader(&GenericTxHeader);

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
   sFilterConfig.FilterActivation = ENABLE;
    //  sFilterConfig.SlaveStartFilterBank = 0;

      HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
    //  HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig); //configure CAN filter

    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
  	  Error_Handler();
    }


  #ifdef FSCU
    fs1.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);
    fs2.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);
    fs3.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);

    ServoInit(&fs1);
    ServoInit(&fs2);
    ServoInit(&fs3);
    ServoInit(&fs4);

    HAL_TIM_PWM_Start(fs1.htim, fs1.channel);
    HAL_TIM_PWM_Start(fs2.htim, fs2.channel);
    HAL_TIM_PWM_Start(fs3.htim, fs3.channel);
    HAL_TIM_PWM_Start(fs4.htim, fs4.channel);

  #endif

  #ifdef ECU

    as1.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);
    as2.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);

    ServoInit(&as1);
    ServoInit(&as2);

    HAL_TIM_PWM_Start(as1.htim, as1.channel);
    HAL_TIM_PWM_Start(as2.htim, as2.channel);

  #endif

  #ifdef GSCU

    gs1.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);
    gs2.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);
    gs3.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);
    gs4.timer_clk_hz = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1) / (htim2.Init.Prescaler + 1);

    ServoInit(&gs1);
    ServoInit(&gs2);
    ServoInit(&gs3);
    ServoInit(&gs4);

    HAL_TIM_PWM_Start(gs1.htim, gs1.channel);
    HAL_TIM_PWM_Start(gs2.htim, gs2.channel);
    HAL_TIM_PWM_Start(gs3.htim, gs3.channel);
    HAL_TIM_PWM_Start(gs4.htim, gs4.channel);

  #endif

  #ifdef EREG
    /* Servo init */
    ereg_sv.timer_clk_hz = HAL_RCC_GetPCLK1Freq()
                            * ((RCC->CFGR & RCC_CFGR_PPRE1) != 0 ? 2 : 1)
                            / (htim2.Init.Prescaler + 1);
    ServoInit(&ereg_sv);
    HAL_TIM_PWM_Start(ereg_sv.htim, ereg_sv.channel);

    /* Vent servo init – TIM2 CH2 */
    ereg_vent_sv.timer_clk_hz = ereg_sv.timer_clk_hz;
    ServoInit(&ereg_vent_sv);
    HAL_TIM_PWM_Start(ereg_vent_sv.htim, ereg_vent_sv.channel);

    /* EREG controller: set starting conditions */
    ereg_setpoint_bar = 3.0;    /* target LP tank pressure [bar]                 */
    ereg_P1_hp_bar    = 8.0;    /* initial HP estimate until first PT reading    */
    ereg_P2_prop_bar  = 0.0;    /* initial LP estimate until first PT reading    */
    ereg_enable       = 1;      /* controller runs immediately on power-up */

    ereg_hansv1_initialize();

    /* Explicitly command both servos to known position before loop starts */
    ServoActuator(&ereg_vent_sv, (uint32_t)180);
    ServoActuator(&ereg_sv,      (uint32_t)180);
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#ifdef EREG
  /* ── EREG main loop ──────────────────────────────────────────────────

   ────────────── */
  {
    uint32_t last_ereg_tick = 0;

    while (1)
    {
      /* USER CODE END WHILE */
      /* USER CODE BEGIN 3 */

      uint32_t now = HAL_GetTick();

      if (now - last_ereg_tick >= 1)  /* 1 ms step */
      {
        last_ereg_tick = now;

        /* 1. Read PTs via ADS131 */
        ads131_read_all_channel(&PTAM, &channels_data);
        ereg_P1_hp_bar   = (double)ADCmVToBar(channels_data.ChannelVoltageMv[EREG_HP_CH]); //Can change this to channel using
        ereg_P2_prop_bar = (double)ADCmVToBar(channels_data.ChannelVoltageMv[EREG_PROP_CH]);

        // PID START
        /* 3. Run the EREG pressure controller (1 ms step) */
        ereg_hansv1_step();

        /* 4. Actuate the regulator (fill) with angles chhanged*/
        ServoActuator(&ereg_sv, (uint32_t)(180.0 - ereg_servo_out_deg));
        // PID END

        /* ── BANG-BANG FALLBACK

         */
        // #define EREG_BB_DEADBAND  0.2f
        // static float bb_fill_angle = 180.0f;
        // if      (ereg_P2_prop_bar < (ereg_setpoint_bar - EREG_BB_DEADBAND))
        //     bb_fill_angle = 0.0f;
        // else if (ereg_P2_prop_bar > (ereg_setpoint_bar + EREG_BB_DEADBAND))
        //     bb_fill_angle = 180.0f;
        // ServoActuator(&ereg_sv, (uint32_t)bb_fill_angle);

        /* 5. Vent servo bang bang */
        if (ereg_P2_prop_bar > (ereg_setpoint_bar + EREG_VENT_THRESHOLD)) {
            ereg_vent_angle = EREG_VENT_OPEN_DEG;
        } else if (ereg_P2_prop_bar < (ereg_setpoint_bar + EREG_VENT_THRESHOLD - EREG_VENT_HYSTERESIS)) {
            ereg_vent_angle = EREG_VENT_CLOSED_DEG;
        }
        ServoActuator(&ereg_vent_sv, (uint32_t)ereg_vent_angle);

        if ((now % 1000) < 500)
          HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
        else
          HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
      }
    }
  }

#else /* original non-EREG loop ────────────────────────────────────── */

  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  ads131_read_all_channel(&PTAM, &channels_data);

	  for (i=4; i<8; i++){

		  id = STARTID +i;
		  for (uint8_t j=0; j<8; j++){
				  memcpy(&TxBuffer[j], &null, 1);
			  };
		  bar=ADCmVToBar(channels_data.ChannelVoltageMv[i]);
		  memcpy(&TxBuffer[0], (const void *)&bar, sizeof(float));
		  applyHeaderID(&GenericTxHeader, id);
		  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
		  		  HAL_Delay(1);};
		  HAL_CAN_AddTxMessage(&hcan2, &GenericTxHeader, TxBuffer, &TxMailbox);

		  id = STARTID+i + 0x10;
		  for (uint8_t j=0; j<8; j++){
			  memcpy(&TxBuffer[j], &null, 1);
		  };
		  memcpy(&TxBuffer[0], &channels_data.ChannelVoltageMv[i], sizeof(uint32_t));
		  applyHeaderID(&GenericTxHeader, id);
		  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
			  HAL_Delay(1);
		  };
		  HAL_CAN_AddTxMessage(&hcan2, &GenericTxHeader, TxBuffer, &TxMailbox);
	  };

	  HAL_Delay(100);
  }

#endif /* EREG */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE END CAN2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 319;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  __HAL_TIM_SET_AUTORELOAD(&htim2, HTIM2_ARR);
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 159;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PT_CS_Pin|EGPIO_Pin|LEDC_Pin|FLSH_CS_Pin
                          |STA_CS_Pin|EGPIOC8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STB_CS_Pin|EGPIOA9_Pin|LEDB_Pin|EGPIOA11_Pin
                          |SV_PT_SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STR_SYNC_Pin|STXC_Pin|EGPIOB15_Pin|EGPIOB5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SV_CS_GPIO_Port, SV_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PT_CS_Pin EGPIO_Pin LEDC_Pin FLSH_CS_Pin
                           STA_CS_Pin EGPIOC8_Pin */
  GPIO_InitStruct.Pin = PT_CS_Pin|EGPIO_Pin|LEDC_Pin|FLSH_CS_Pin
                          |STA_CS_Pin|EGPIOC8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PT_IQR_Pin STA_IQR_Pin */
  GPIO_InitStruct.Pin = PT_IQR_Pin|STA_IQR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STB_CS_Pin EGPIOA9_Pin LEDB_Pin EGPIOA11_Pin
                           SV_PT_SYNC_Pin */
  GPIO_InitStruct.Pin = STB_CS_Pin|EGPIOA9_Pin|LEDB_Pin|EGPIOA11_Pin
                          |SV_PT_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STR_SYNC_Pin STXC_Pin EGPIOB15_Pin EGPIOB5_Pin */
  GPIO_InitStruct.Pin = STR_SYNC_Pin|STXC_Pin|EGPIOB15_Pin|EGPIOB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDA_Pin */
  GPIO_InitStruct.Pin = LEDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SV_CS_Pin */
  GPIO_InitStruct.Pin = SV_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SV_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
#ifdef USE_FULL_ASSERT
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
