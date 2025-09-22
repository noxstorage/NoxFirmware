/*
 * stc31.h
 *
 *  Created on: 28 oct. 2021
 *      Author: llabrotrho
 */

#ifndef STC31_H_
#define STC31_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"


/* USER CODE BEGIN */
#define CRC_POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

#define STC3X_DEFAULT_ADDRESS 0x29
#define STC3x_COMMAND_SET_BINARY_GAS                          0x3615
#define STC3X_COMMAND_SET_RELATIVE_HUMIDITY                   0x3624
#define STC3X_COMMAND_SET_TEMPERATURE                         0x361E
#define STC3X_COMMAND_SET_PRESSURE                            0x362F
#define STC3X_COMMAND_MEASURE_GAS_CONCENTRATION               0x3639
#define STC3X_COMMAND_FORCED_RECALIBRATION                    0x3661
#define STC3X_COMMAND_AUTOMATIC_CALIBRATION_ENABLE            0x3FEF
#define STC3X_COMMAND_AUTOMATIC_CALIBRATION_DISABLE           0x3F6E
#define STC3X_COMMAND_PREPARE_READ_STATE                      0x3752
#define STC3X_COMMAND_READ_WRITE_STATE                        0xE133
#define STC3X_COMMAND_APPLY_STATE                             0x3650
#define STC3X_COMMAND_SELF_TEST                               0x365B
#define STC3X_COMMAND_ENTER_SLEEP_MODE                        0x3677
#define STC3X_COMMAND_READ_PRODUCT_IDENTIFIER_1               0x367C
#define STC3X_COMMAND_READ_PRODUCT_IDENTIFIER_2               0xE102

uint8_t STC31_computeCRC8(uint8_t[], uint8_t);
uint16_t STC31_CalcHumidity(float);
uint16_t STC31_CalcTemperature(float);
HAL_StatusTypeDef STC31_wakeup(HAL_StatusTypeDef, I2C_HandleTypeDef *);
HAL_StatusTypeDef STC31_setRHandTandP(HAL_StatusTypeDef, I2C_HandleTypeDef *, float, float, float);
HAL_StatusTypeDef STC31_measure(HAL_StatusTypeDef, I2C_HandleTypeDef *, float *);


#endif /* STC31_H_ */
