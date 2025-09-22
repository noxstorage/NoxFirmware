/*
 * da14531.h
 *
 * Author: Louis Labrot
 * Purpose: da14531 Driver
 */
#ifndef DA14531_H_
#define DA14531_H_

// Dependancies
#include "usart.h"

// Defines


// UART DEFINE
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))


#define EVENT_DISABLE_SIZE                      (COUNTOF(aTxBuffer21) - 1)






// Function Prototypes
HAL_StatusTypeDef BLE_Init(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble);
HAL_StatusTypeDef BLE_GoToSleep(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble);
HAL_StatusTypeDef BLE_WakeUp(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble);
HAL_StatusTypeDef BLE_ReadMEM2(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef BLE_ReadMEM3(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef BLE_ResetMEM2(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble);
HAL_StatusTypeDef BLE_ResetMEM3(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble);

uint8_t  BLE_Convert_ID(uint8_t my_id);
uint8_t  BLE_Convert_HexaAscii(uint8_t byte);
uint8_t  BLE_ID_figure_letter(uint8_t my_id);
HAL_StatusTypeDef  BLE_ChangeName_Sigfox(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble);
HAL_StatusTypeDef  BLE_ChangeName_Lorawan(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble);

HAL_StatusTypeDef  BLE_WriteData(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble);
HAL_StatusTypeDef  BLE_Convert_Data(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7, uint8_t byte8, uint8_t byte9);

#endif /* DA14531_H_ */


