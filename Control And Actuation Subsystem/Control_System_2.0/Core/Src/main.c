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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "battery.h"
#include "ToF.h"
#include "Sonar.h"
#include <stdio.h>
#include "telemetry.h"
#include "temp_sensor.h"
#include "mission_rx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Telemetry pacing */
#define TELEM_PERIOD_MS  500u
uint32_t last_telem_ms = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN 65 //Minimum position of plunger of 4cm
#define MAX 100 //Maximum position of plunger of 12cm
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t current_time;
BatteryData_t bat;
uint32_t prev_time;
int mission_init = 0;
uint32_t t_begin = 0;
int time_saved = 0;
uint32_t time_elapsed = 0;
uint32_t current = 0;
int mission_complete = 0;

/* ---- Live sensor state: refreshed by UpdateSensors(), readable anywhere ---- */
float    soc          = 0.0f;     /* battery state of charge, % */
uint16_t mm           = 0;        /* ToF (plunger position), mm */
uint16_t sonar_dist   = 0;        /* sonar (obstacle distance), mm */
Sonar_Handle g_sonar;             /* global sonar handle */

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Send a telemetry packet at most once every TELEM_PERIOD_MS.
   Non-blocking: returns immediately if it's not yet time. */
static void Telemetry_Tick(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - last_telem_ms) < TELEM_PERIOD_MS) {
        return;
    }
    last_telem_ms = now;

    /* MCU internal temp isn't wired up here — pass the sentinel.
       If you later sample ADC1 (TEMPSENSOR channel), replace this. */
    float mcu_temp = TempSensor_ReadC();
    Telemetry_Send(&bat, mm, sonar_dist, mcu_temp);
}

static void UpdateSensors(void)
{
    /* Battery — rate-limited internally, safe to call often */
    Battery_Update();
    Battery_GetData(&bat);
    soc = bat.soc_pct;

    /* ToF (plunger position) */
    uint16_t tof = ToF_ReadDistance(&hi2c2);
    if (tof > 0) {
        mm = tof;
    }

    /* Sonar (obstacle distance) — keep last good reading on error */
    uint16_t sd = 0;
    Sonar_GetDistance(&g_sonar, &sd);
        sonar_dist = sd;

}
/*A function that stops motor from over exceeding limits*/
void stop_motor(void)
{
    TIM3->CCR1 = 0;
    TIM4->CCR1 = 0;
    UpdateSensors();   /* keep state fresh even at stop */
    Telemetry_Tick();
}

/*Function to check initial plunger position and force it to minimum so that upon mission start we can pull in water and descend*/
void Force_plunger_min(void)
{
    UpdateSensors();
    while (mm > MIN) {
        TIM4->CCR1 = 1599;
        UpdateSensors();
        Telemetry_Tick();
        /* Safety: bail out if battery dies mid-move */
        if (soc < 20) break;

        HAL_Delay(2);
    }
    stop_motor();
}

/* Push plunger to MAX (pulls water in) → platform sinks. */
void descend(void)
{
    UpdateSensors();
    while (mm < MAX) {
        TIM3->CCR1 = 1599;
        UpdateSensors();
        Telemetry_Tick();
        /* Safety: stop if battery dies */
        if (soc < 20) break;

        HAL_Delay(2);
    }
    stop_motor();
}

/* Pull plunger back toward MIN → platform rises.
   Exits early if an obstacle is detected by the sonar. */
void ascend(void)
{
    UpdateSensors();
    while (mm > MIN && sonar_dist > 1000) {
        TIM4->CCR1 = 1599;
        UpdateSensors();
        Telemetry_Tick();
        HAL_Delay(2);
    }
    stop_motor();
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {

	current_time = HAL_GetTick();
 	if ((GPIO_Pin == Pushbutton_Pin) && (current_time - prev_time > 10))
 	{


 		//TODO: Update mission initiation status
 		mission_init = 1;


 		prev_time = current_time;

 	}
 }

uint32_t to_seconds(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    return (hours * 3600) + (minutes * 60) + seconds;
}

/* ============================================================
 *  ONE-SHOT CALIBRATION
 *  Comment out this whole block after running it once.
 *  ============================================================
 *  Procedure:
 *    1. Place sensor exactly TOF_CAL_NEAR_REF_MM from a flat target.
 *    2. Power on / reset — onboard LED stays ON for 5 s (get ready).
 *    3. LED blinks fast while it captures 50 samples (~3 s).
 *    4. LED stays ON for 5 s — move target to TOF_CAL_FAR_REF_MM.
 *    5. LED blinks fast again — captures 50 more samples.
 *    6. LED gives 3 quick flashes = saved, or stays on solid = failed.
 *    7. Comment out the call and reflash.
 * ============================================================ */
static void RunOneShotCalibration(I2C_HandleTypeDef *hi2c)
{
    /* Stage 1: prompt for NEAR target */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);  /* LED ON (active high on Nucleo) */
    HAL_Delay(5000);

    /* Capture NEAR */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);  /* LED OFF */
    uint16_t measured_near = ToF_AverageRaw(hi2c, TOF_CAL_SAMPLES);

    /* Stage 2: prompt for FAR target */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);  /* LED ON */
    HAL_Delay(5000);

    /* Capture FAR */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);  /* LED OFF */
    uint16_t measured_far = ToF_AverageRaw(hi2c, TOF_CAL_SAMPLES);

    /* Compute + persist */
    uint8_t ok = ToF_Calibrate(measured_near, measured_far,
                               TOF_CAL_NEAR_REF_MM, TOF_CAL_FAR_REF_MM);

    if (ok)
    {
        /* 3 quick flashes = success */
        for (int i = 0; i < 6; i++)
        {
            HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
            HAL_Delay(100);
        }
        HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);  /* LED OFF */
    }
    else
    {
        /* Solid ON = failure indication */
        HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
        while (1) {}  /* halt so you notice */
    }
}

static void ErrorBlink(void)
{
    while (1)
    {
    	HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
        HAL_Delay(200);
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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);




    HAL_GPIO_WritePin(XShunt_GPIO_Port, XShunt_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(XShunt_GPIO_Port, XShunt_Pin, GPIO_PIN_SET);
    HAL_Delay(10);


    if (ToF_Init(&hi2c2) != HAL_OK)
        {
            ErrorBlink();
        }

  	if(Battery_Init(&hi2c1) != HAL_OK){
  		Error_Handler();
  	}

  	if (Sonar_Init(&g_sonar, &huart1) != SONAR_OK) {
  	      /* Sonar init failed — hang. Swap for ErrorBlink() if you'd rather see it. */
  	      while (1) { }
  	  }

  	Telemetry_Init(&huart3);

  	TempSensor_Init(&hadc1);

  	MissionRx_Init(&huart3);

  	HAL_Delay(1000);

  	HAL_GPIO_WritePin(GPIOA, LD2_Pin, 0);

    	    	/* Infinite loop */
  	  /* USER CODE BEGIN WHILE */
  	  while (1)
  	  {
  	      /* RTC snapshot — GetDate must follow GetTime on F4 */
  	      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  	      HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  	      /* Refresh all sensors */
  	      UpdateSensors();

  	      Telemetry_Tick();

  	      /* ---- Flow diagram begins here ---- */

  	      /* Gate: Mission Initialized? */
  	      if (!missionInit) {
  	          HAL_Delay(50);
  	          continue;
  	      }

  	    HAL_GPIO_WritePin(GPIOA, LD2_Pin, 1);

  	  /* Gate: Battery Enough? */
  	    	      if (soc <= 30 || mission_complete) {
  	    	          stop_motor();
  	    	          /* If mission is done AND no obstacles, ascend home.
  	    	             Also ascend if battery got low. */
  	    	          if ((mission_complete && sonar_dist > 1000) || soc < 20) {
  	    	              ascend();
  	    	          }
  	    	          HAL_Delay(100);
  	    	          continue;
  	    	      }

  	    	      /* Gate: Plunger at MIN? */
  	    	      if (mm >= MIN) {
  	    	          Force_plunger_min();   /* falls through to DESCEND on next iter */
  	    	          continue;
  	    	      }

  	    	      /* DESCEND */
  	    	      descend();

  	    	      /* Mission timer starts the first time we successfully descended */
  	    	      if (time_saved == 0) {
  	    	          t_begin = to_seconds(sTime.Hours, sTime.Minutes, sTime.Seconds);
  	    	          time_saved = 1;
  	    	      }

  	    	      /* While submerged, keep refreshing time + sensors until mission timeout */
  	    	      while (!mission_complete && soc >= 20) {
  	    	          HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  	    	          HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  	    	          UpdateSensors();
  	    	          Telemetry_Tick();

  	    	          current = to_seconds(sTime.Hours, sTime.Minutes, sTime.Seconds);
  	    	          time_elapsed = (current < t_begin)
  	    	                       ? (86400 - t_begin) + current
  	    	                       : (current - t_begin);

  	    	          if (time_elapsed >= 30) {
  	    	              mission_complete = 1;
  	    	          }

  	    	          HAL_Delay(100);
  	    	      }

  	    	      /* Mission done (or battery low) → ASCEND if path is clear */
  	    	      stop_motor();
  	    	      if ((mission_complete && sonar_dist > 1000) || soc < 20) {
  	    	          ascend();
  	    	      }

  	         /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 23;
  sTime.Minutes = 59;
  sTime.Seconds = 45;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_MAY;
  sDate.Date = 20;
  sDate.Year = 26;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim3.Init.Period = 1599;
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
  htim4.Init.Period = 1599;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|XShunt_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Pushbutton_Pin */
  GPIO_InitStruct.Pin = Pushbutton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pushbutton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin XShunt_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|XShunt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
