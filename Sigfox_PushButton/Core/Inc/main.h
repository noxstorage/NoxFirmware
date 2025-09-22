/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

void My_Error_Handler(void);
void My_Error_Handler_EEPROM(void);

void Init_Switch(void);
void Switch_Set(void);
void Switch_Reset(void);

void Init_LED3(void);
void Init_LED2(void);
void Init_LED1(void);

void PullDown_I2C_A10(void);
void PullDown_I2C_A9(void);

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define SWITCH_Pin GPIO_PIN_11
#define SWITCH_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
#define BUT1_Pin GPIO_PIN_0
#define BUT1_GPIO_Port GPIOA
#define BUT1_EXTI_IRQn EXTI0_IRQn
#define PROB2_Pin GPIO_PIN_13
#define PROB2_GPIO_Port GPIOB
#define PROB1_Pin GPIO_PIN_12
#define PROB1_GPIO_Port GPIOB
#define BUT2_Pin GPIO_PIN_1
#define BUT2_GPIO_Port GPIOA
#define BUT2_EXTI_IRQn EXTI1_IRQn
#define LED3_OLD_Pin GPIO_PIN_11
#define LED3_OLD_GPIO_Port GPIOB
#define USARTx_RX_Pin GPIO_PIN_3
#define USARTx_RX_GPIO_Port GPIOA
#define USARTx_TX_Pin GPIO_PIN_2
#define USARTx_TX_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

// I2C_SDA = PA10
// I2C_SCK = PA9
// ADC (4) = PB2
// PA0 = INT accelerometer
// PA1 = INT BLE


#define PIN_SWITCH	              GPIO_PIN_11   // (PORT A) GPIO_PIN_11 =>  PIN 5 CN10
#define DELAYSENSOR               1200000       // ms. 20min. N times this value.
#define DELAYACC                  60000         // ms. 1min. N times this value.
#define DELAYSENSOR_DEFAULT       1200000     // ms. 20min. Default value.
#define DELAYACC_DEFAULT          60000         // ms. 1min. Default value.
#define DELAYSENSOR_FIRST         600000       // ms. 10min First value.
#define DELAYACC_FIRST            60000         // ms. 1min. First value.
#define PIN_BLE                   11 //11 (9 only in V.1.2)
#define TEST_MODE                 0  // 0 : disable some specific parts for the test. 1 : active some specific parts for the test
#define O2_SENSOR                 0  // 0 : no 02 sensor. 1 : active some specific parts for O2 sensor
#define SENSIBILITY_DEFAULT       6  // 1 to 6


/* CHOICE BETWEEN LORAWAN AND SIGFOX */
/* comment SIGFOX_DOWNLINK or LORA_DOWNLINK define */
#define SIGFOX_DOWNLINK           SFX_TRUE  // SFX_FALSE : no downlink. SFX_TRUE : downlink
//#define LORA_DOWNLINK             1            // 0: no downlink // 1 : downlink

#define MY_VDD_SUPPLY                3000 //mV

#define RH_T_WIRED	                  0
#define P_WIRED	                      0
#define CO2_WIRED			  		  0
#define SENSOR_ACC_WIRED			  0
#define MEM_EEPROM_WIRED			  0
#define BLE_WIRED                     0


#define SECRET_CODE_1			  1
#define SECRET_CODE_2			  5
#define SECRET_CODE_3			  2
#define SECRET_CODE_4			  3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
