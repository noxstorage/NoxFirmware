/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC3635_H
#define __MC3635_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"


/* USER CODE BEGIN */
#define MC3635_ADDRESS_W  0x98 //write address (0x4C<<1)
#define MC3635_ADDRESS_R  0x99 //read  address (0x4C<<1)|0x01


typedef enum
{
    MC3635_MODE_SLEEP 	   = 0b000,
    MC3635_MODE_STANDBY    = 0b001,
    MC3635_MODE_SNIFF      = 0b010,
    MC3635_MODE_CWAKE      = 0b101,
	MC3635_MODE_SWAKE      = 0b110,
    MC3635_MODE_TRIG       = 0b111,
}   MC3635_mode_t;

typedef enum
{
    MC3635_SNIFF_SR_DEFAULT_6Hz = 0b0000,
    MC3635_SNIFF_SR_0p4Hz      = 0b0001,
    MC3635_SNIFF_SR_0p8Hz      = 0b0010,
    MC3635_SNIFF_SR_2Hz        = 0b0011,
    MC3635_SNIFF_SR_6Hz        = 0b0100,
    MC3635_SNIFF_SR_13Hz       = 0b0101,
    MC3635_SNIFF_SR_25Hz       = 0b0110,
    MC3635_SNIFF_SR_50Hz       = 0b0111,
    MC3635_SNIFF_SR_100Hz      = 0b1000,
    MC3635_SNIFF_SR_200Hz      = 0b1001,
    MC3635_SNIFF_SR_400Hz      = 0b1010,// only for OSR = 32
    MC3635_SNIFF_SR_END,
}   MC3635_sniff_sr_t;


typedef enum
{
    MC3635_FIFO_MODE_NORMAL  = 0b00000000,
    MC3635_FIFO_MODE_CONTINUOUS = 0b00100000,
}   MC3635_fifo_mode_t;




HAL_StatusTypeDef MC3635_initilization(HAL_StatusTypeDef, I2C_HandleTypeDef * );
HAL_StatusTypeDef MC3635_SET_sniff_int(HAL_StatusTypeDef, I2C_HandleTypeDef *, MC3635_sniff_sr_t, uint8_t);
HAL_StatusTypeDef MC3635_SET_mode(HAL_StatusTypeDef, I2C_HandleTypeDef *, MC3635_mode_t);
HAL_StatusTypeDef MC3635_READ_ONE(HAL_StatusTypeDef, I2C_HandleTypeDef * , float *);
HAL_StatusTypeDef MC3635_READ_FIFO_sniffTOcwake(HAL_StatusTypeDef, I2C_HandleTypeDef *, uint8_t[]);
HAL_StatusTypeDef MC3635_SET_FIFO(HAL_StatusTypeDef, I2C_HandleTypeDef *, MC3635_fifo_mode_t);
HAL_StatusTypeDef MC3635_SET_8b8g_ODRWake50ULP(HAL_StatusTypeDef, I2C_HandleTypeDef *);
uint8_t MC3635_READ_CLEAR_int(HAL_StatusTypeDef, I2C_HandleTypeDef *);



/* USER CODE END  */

#ifdef __cplusplus
}
#endif

#endif /* __MC3635_H */
