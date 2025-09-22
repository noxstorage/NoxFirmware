/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ms5637_H
#define __ms5637_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"


/* USER CODE BEGIN */
//MS5637
#define MS5637_ADDRESS_W  0xEC //write address 11101100
#define MS5637_ADDRESS_R  0xED //read  address 11101101

HAL_StatusTypeDef MS5637_reset(HAL_StatusTypeDef, I2C_HandleTypeDef *);
HAL_StatusTypeDef MS5637_read_c1(HAL_StatusTypeDef, I2C_HandleTypeDef * , int64_t *);
HAL_StatusTypeDef MS5637_read_c2(HAL_StatusTypeDef, I2C_HandleTypeDef * , int64_t *);
HAL_StatusTypeDef MS5637_read_c3(HAL_StatusTypeDef, I2C_HandleTypeDef *, int64_t *);
HAL_StatusTypeDef MS5637_read_c4(HAL_StatusTypeDef, I2C_HandleTypeDef *, int64_t *);
HAL_StatusTypeDef MS5637_read_c5(HAL_StatusTypeDef, I2C_HandleTypeDef *, int64_t *);
HAL_StatusTypeDef MS5637_read_c6(HAL_StatusTypeDef, I2C_HandleTypeDef *, int64_t *);
HAL_StatusTypeDef MS5637_read_d1(HAL_StatusTypeDef, I2C_HandleTypeDef *, int64_t *);
HAL_StatusTypeDef MS5637_read_d2(HAL_StatusTypeDef, I2C_HandleTypeDef *, int64_t *);

float MS5637_CalcTemperature(int64_t, int64_t);
float MS5637_CalcPressure(int64_t, int64_t, int64_t);

/* USER CODE END  */

#ifdef __cplusplus
}
#endif

#endif /* __ms5637_H */
