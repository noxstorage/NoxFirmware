/*
 * M24512.h
 *
 * Author: Louis Labrot
 * Purpose: M24512 EEPROM Driver
 */
#ifndef M24512_H_
#define M24512_H_

// Dependancies
#include "i2c.h"

#define EEPROM_ADDRESS 			0x50  // 0x50, Chip enable address = (000)--->(E2 E1 E0)
#define EEPROM_ADDRESS_W 			0xA0  // 0xA0, Chip enable address = (000)--->(E2 E1 E0 0)
#define EEPROM_ADDRESS_R 			0xA1  // 0xA1, Chip enable address = (000)--->(E2 E1 E0 1)

#define EEPROM_PAGE_SIZE		128   // Page Size = 128 bytes
#define EEPROM_PAGES_COUNT		512	  // Number of pages
#define EEPROM_SIZE				65536 // EEPROM Size 512Kbit (1024*(512/8)) Bytes
#define EEPROM_ADDRESS_SIZE		2 // Address length
#define EEPROM_PAGE_NUM			EEPROM_SIZE/EEPROM_PAGE_SIZE // Page No.

// Links to the I2C driver API's in the HAL
#define I2C_EEPROM_write(data, len)				HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDRESS_W, data, len, 200)
#define I2C_EEPROM_read(data, len)				HAL_I2C_Master_Receive(&hi2c1, EEPROM_ADDRESS_R, data, len, 200)

// Function Prototypes
HAL_StatusTypeDef EEPROM_read(HAL_StatusTypeDef my_ret, uint16_t adr, void *data, uint16_t size);
HAL_StatusTypeDef EEPROM_write(HAL_StatusTypeDef my_ret, uint16_t adr, void *data, uint16_t size);
HAL_StatusTypeDef EEPROM_erase(HAL_StatusTypeDef my_ret);
uint8_t Convert_decimal(char ascii_value);
#endif /* M24512_H_ */


