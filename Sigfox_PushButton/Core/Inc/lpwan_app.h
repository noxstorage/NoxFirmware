/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lpwan_app.h
  * @author  Louis LABROT
  * @brief   provides code for the application of the SIGFOX Middleware
  ******************************************************************************
  * @attention
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LPWAN_APP_H__
#define __LPWAN_APP_H__

#ifdef __cplusplus
extern "C" {
#endif



#include "sigfox_types.h"


////////////////////////////////////////////////////////////////
// EEPROM and parameters
extern uint64_t delay_timer_acc;
extern uint64_t delay_timer;
extern uint8_t preceive_command[8];

//float Norm_g=0;
extern uint8_t pREAD_FIFO[32];
extern uint8_t MC3635_sensibility;
extern uint8_t MC3635_sensibility_msg;

// buffer uart
extern uint8_t aRxBuffer1[20]; //16+4
extern uint8_t aRxBuffer2[8];

//Sigfox downlink
extern sfx_bool Sigfox_Downlink;
extern uint8_t LPWAN_Downlink;

// BLE data
extern uint8_t send_ble_data;

extern uint32_t nb_wakeup;
/////////////////////////////////////////////////////////////////////////////////////////

extern void SendSigfox(void);

void LPWAN_init1(void);
float LPWAN_pressure(void);
void LPWAN_Init_Acc(void);
uint8_t LPWAN_CO2(float SHTC3_humidity, float SHTC3_temperature, float MS5637_pressure);
void BLElink(void);
HAL_StatusTypeDef Store_Uplink (HAL_StatusTypeDef my_ret_eeprom, uint8_t * ul_msg, uint8_t ul_size);

// UART DEFINE
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

#define RXBUFFERSIZE1                      (COUNTOF(aRxBuffer1) - 1)
#define RXBUFFERSIZE2                      (COUNTOF(aRxBuffer2) - 1)

#ifdef __cplusplus
}
#endif

#endif /*__LPWAN_APP_H__*/
