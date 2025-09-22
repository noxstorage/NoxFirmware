/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
#include "app_sigfox.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_seq.h"
#include "app_lorawan.h"
#include "sgfx_eeprom_if.h"
#include "sys_app.h"
#include "sgfx_app.h"
#include "lora_app.h" //edit
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACTIVE_APP_SIGFOX               1
#define ACTIVE_APP_LORAWAN              0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*place variable inside retention RAM2 and no init to keep value after reset*/
//UTIL_MEM_PLACE_IN_SECTION("MB_MEM2") uint32_t active_app;
UTIL_MEM_PLACE_IN_SECTION("MB_MEM2") uint32_t nb_wakeup;
#if defined(SIGFOX_DOWNLINK)
uint32_t active_app = ACTIVE_APP_SIGFOX;
#elif !defined(SIGFOX_DOWNLINK)
uint32_t active_app = ACTIVE_APP_LORAWAN;
#endif /* Sigfox_ID */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
//	if (active_app == ACTIVE_APP_LORAWAN) {
//		MX_LoRaWAN_Init();
//	} else {
//		MX_Sigfox_Init();
//	}
  #if defined(SIGFOX_DOWNLINK)
  MX_Sigfox_Init();
  #elif !defined(SIGFOX_DOWNLINK)
  MX_LoRaWAN_Init();
  #endif /* Sigfox_ID */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

	//	if (active_app == ACTIVE_APP_LORAWAN) {
	//		MX_LoRaWAN_Process();
	//	} else {
    #if defined(SIGFOX_DOWNLINK)

    /* USER CODE END WHILE */
    MX_Sigfox_Process();

    /* USER CODE BEGIN 3 */
	//	}
    #elif !defined(SIGFOX_DOWNLINK)
    MX_LoRaWAN_Process();
    #endif /* Sigfox_ID */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Note: Current MX does not support EXTI IP neither BSP. */
/* In order to get a push button IRS by code automatically generated */
/* this function is today the only available possibility. */
/* Calling BSP_PB_Callback() from here it shortcuts the BSP. */
/* If users wants to go through the BSP, it can remove BSP_PB_Callback() from here */
/* and add a call to BSP_PB_IRQHandler() in the USER CODE SESSION of the */
/* correspondent EXTIn_IRQHandler() in the stm32wlxx_it.c */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* USER CODE BEGIN HAL_GPIO_EXTI_Callback_1 */

  /* USER CODE END HAL_GPIO_EXTI_Callback_1 */
  switch (GPIO_Pin)
  {
    case  BUT1_Pin:
      /* swap application type */
      if (active_app == ACTIVE_APP_LORAWAN)
      {
    	  Lora_GPIO_EXTI_Callback(1);
      }
      else
      {
    	  Sigfox_GPIO_EXTI_Callback(1);
      }
      /*restart the mcu*/
      //NVIC_SystemReset();
      break;

    /* USER CODE BEGIN EXTI_Callback_Switch_case */
    case  BUT2_Pin:
    	if (active_app == ACTIVE_APP_LORAWAN)
    	{
    	   Lora_GPIO_EXTI_Callback(2);
    	}
    	else
    	{
    	    Sigfox_GPIO_EXTI_Callback(2);
    	}
    break;
    /* USER CODE END EXTI_Callback_Switch_case */
    default:
    /* USER CODE BEGIN EXTI_Callback_Switch_default */
    /* USER CODE END EXTI_Callback_Switch_default */
      break;
  }
  /* USER CODE BEGIN HAL_GPIO_EXTI_Callback_Last */

  /* USER CODE END HAL_GPIO_EXTI_Callback_Last */
}


void PullDown_I2C_A9(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOA_CLK_ENABLE();
  	GPIO_InitStruct.Pin = GPIO_PIN_9;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Output
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_Delay(50);
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

}

void PullDown_I2C_A10(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOA_CLK_ENABLE();
  	GPIO_InitStruct.Pin = GPIO_PIN_10;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Output
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_Delay(50);
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void My_Error_Handler(void)
{
	Init_LED3();
  __disable_irq();
  while (1)
  {
	  HAL_Delay(2000);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(100);
  }
}

void My_Error_Handler_EEPROM(void)
{
	Init_LED3();
  __disable_irq();
  while (1)
  {
	  HAL_Delay(700);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  }
}

void Init_Switch(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOA_CLK_ENABLE();
  	GPIO_InitStruct.Pin = PIN_SWITCH;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Output
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Switch_Set(void)
{
  	HAL_GPIO_WritePin(GPIOA, PIN_SWITCH, GPIO_PIN_SET);
  	HAL_Delay(50);
}

void Switch_Reset(void)
{
	HAL_Delay(50);
  	HAL_GPIO_WritePin(GPIOA, PIN_SWITCH, GPIO_PIN_RESET);
}





void Init_LED3(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOA_CLK_ENABLE();
  	GPIO_InitStruct.Pin = LED3_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Output
  	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);
}

void Init_LED2(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOA_CLK_ENABLE();
  	GPIO_InitStruct.Pin = LED2_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Output
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);
}

void Init_LED1(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOA_CLK_ENABLE();
  	GPIO_InitStruct.Pin = LED1_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Output
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	Init_LED3();
	__disable_irq();
	while (1) {
		  HAL_Delay(100);
		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		  HAL_Delay(100);
		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
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
