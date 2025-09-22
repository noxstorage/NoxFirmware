/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHTC3_H
#define __SHTC3_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"


/* USER CODE BEGIN */
#define SHTC3_ADDRESS_W  0xE0 //write address (0x70<<1)
#define SHTC3_ADDRESS_R  0xE1 //read  address (0x70<<1)|0x01


HAL_StatusTypeDef SHTC3_wakeup(HAL_StatusTypeDef, I2C_HandleTypeDef *);
HAL_StatusTypeDef SHTC3_measure(HAL_StatusTypeDef, I2C_HandleTypeDef *, uint16_t *, uint16_t *);
HAL_StatusTypeDef SHTC3_sleep(HAL_StatusTypeDef, I2C_HandleTypeDef *);
HAL_StatusTypeDef SHTC3_reset(HAL_StatusTypeDef, I2C_HandleTypeDef *);

float SHTC3_CalcHumidity(uint16_t);
float SHTC3_CalcTemperature(uint16_t);


/* USER CODE END  */

#ifdef __cplusplus
}
#endif

#endif /* __SHTC3_H */
