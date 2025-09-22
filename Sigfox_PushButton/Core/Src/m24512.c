/*
 * @file   M24512.c
 * @brief  M24512 EEPROM Driver
 * @author Louis Labrot
 * Known Bugs: None.
 *
 * Future Features: 
	- Consider making writes non-blocking with recursive callbacks as needed 
	 if practical without complicating applications using this.
	 
 * NOTES:
	- No end of EEPROM sanity checks has been added for writes/reads
	
  Reference: https://os.mbed.com/users/dsMartyn/code/M24512/docs/tip/ and Vinay Divakar
 */

/*
 *====================
 * Includes
 *====================
 */
#include "M24512.h"
#include <string.h>
#include "sys_app.h" // for APP_PPRINTF
#include "stm32wlxx_hal.h" //HAL_STATUS
/****************************************************************
 * Function Name: EEPROM_writePage
 * Description: Writes to the EEPROM
 * Returns: Hal status of the I2C transmission
 * Params	@adr: Address to write 
			@data: write buffer
			@size: No. of bytes to write
 ****************************************************************/
static HAL_StatusTypeDef EEPROM_writePage(HAL_StatusTypeDef my_ret, uint16_t adr, void *data, uint16_t size){
	uint8_t cmd[EEPROM_ADDRESS_SIZE + EEPROM_PAGE_SIZE]; 
	
	cmd[0] = adr >> 8;
	cmd[1] = (uint8_t)adr;
		
	if(size > EEPROM_PAGE_SIZE){
		//printf("[EEPROM_writePage], ERROR: Invalid/Too big data size, LEN: %hu\r\n", size);
		my_ret = HAL_ERROR;
		return my_ret; // Error, data size is too big!
	}
	
	memcpy(cmd + 2, data, size); // prepare payload i.e. memory address + data
	my_ret=I2C_EEPROM_write(cmd, size + 2); // perform write transaction
	if(my_ret !=HAL_OK) // if error, send command again
	{
		my_ret=I2C_EEPROM_write(cmd, size + 2); // perform write transaction
	}
	return my_ret;
}

/****************************************************************
 * Function Name: EEPROM_write_once
 * Description: Writes ONCE to the EEPROM
 * Returns: Hal status of the I2C transmission
 * Params	@adr: Address to write
			@data: write buffer
			@size: No. of bytes to write
 ****************************************************************/
HAL_StatusTypeDef EEPROM_write_once(HAL_StatusTypeDef my_ret, uint16_t adr, void *data, uint16_t size) {
	uint16_t wlen = 0;		// write length
	uint16_t dlen = size;	// data length remaining
	uint16_t addr_ofst = 0;	// address to write

	while(dlen > 0){
		wlen = ((dlen > EEPROM_PAGE_SIZE) ? EEPROM_PAGE_SIZE : dlen);
		// if addr is not at the start of a page then write bytes until we hit the boundary
		if(addr_ofst == 0 && (adr % EEPROM_PAGES_COUNT) != 0){
			wlen = EEPROM_PAGE_SIZE - (adr % EEPROM_PAGE_SIZE);
			if(size < wlen){// check we have enough data to hit end of page if not just write the entire length
				wlen = size;
			}
		}

		//APP_PPRINTF("Write: Addr:%hu Write: %hu Remain: %hu next: %hu\r\n", adr + addr_ofst,  wlen, dlen - wlen , addr_ofst + wlen);

		// write in blocks of %page_size%
		my_ret=EEPROM_writePage(my_ret, adr + addr_ofst, data + addr_ofst, wlen);
		dlen -= wlen;
		addr_ofst += wlen;
	}//while

	return my_ret;
}


/****************************************************************
 * Function Name: EEPROM_write
 * Description: Writes to the EEPROM
 * Returns: Hal status of the I2C transmission
 * Params	@adr: Address to write 
			@data: write buffer
			@size: No. of bytes to write
 ****************************************************************/
HAL_StatusTypeDef EEPROM_write(HAL_StatusTypeDef my_ret, uint16_t adr, void *data, uint16_t size) {
	uint16_t wlen = 0;		// write length
	uint16_t dlen = size;	// data length remaining
	uint16_t addr_ofst = 0;	// address to write
	
	while(dlen > 0){
		wlen = ((dlen > EEPROM_PAGE_SIZE) ? EEPROM_PAGE_SIZE : dlen);
		// if addr is not at the start of a page then write bytes until we hit the boundary
		if(addr_ofst == 0 && (adr % EEPROM_PAGES_COUNT) != 0){
			wlen = EEPROM_PAGE_SIZE - (adr % EEPROM_PAGE_SIZE);
			if(size < wlen){// check we have enough data to hit end of page if not just write the entire length
				wlen = size;	
			}
		}
		
		//APP_PPRINTF("Write: Addr:%hu Write: %hu Remain: %hu next: %hu\r\n", adr + addr_ofst,  wlen, dlen - wlen , addr_ofst + wlen);
		
		// write in blocks of %page_size%
		my_ret=EEPROM_writePage(my_ret, adr + addr_ofst, data + addr_ofst, wlen);
		dlen -= wlen;
		addr_ofst += wlen;
	}//while

	if(my_ret !=HAL_OK) // if error, send command again
	{
		wlen = 0;		// write length
		dlen = size;	// data length remaining
		addr_ofst = 0;	// address to write

		while(dlen > 0){
			wlen = ((dlen > EEPROM_PAGE_SIZE) ? EEPROM_PAGE_SIZE : dlen);
			// if addr is not at the start of a page then write bytes until we hit the boundary
			if(addr_ofst == 0 && (adr % EEPROM_PAGES_COUNT) != 0){
				wlen = EEPROM_PAGE_SIZE - (adr % EEPROM_PAGE_SIZE);
				if(size < wlen){// check we have enough data to hit end of page if not just write the entire length
					wlen = size;
				}
			}

			//APP_PPRINTF("Write: Addr:%hu Write: %hu Remain: %hu next: %hu\r\n", adr + addr_ofst,  wlen, dlen - wlen , addr_ofst + wlen);

			// write in blocks of %page_size%
			my_ret=EEPROM_writePage(my_ret, adr + addr_ofst, data + addr_ofst, wlen);
			dlen -= wlen;
			addr_ofst += wlen;
		}//while
	}

	return my_ret;
}



/****************************************************************
 * Function Name: EEPROM_read
 * Description: Reads from the EEPROM
 * Returns: Hal status of the I2C transmission
 * Params	@adr: Address to read 
			@data: output buffer
			@size: No. of bytes to read
 ****************************************************************/
HAL_StatusTypeDef EEPROM_read(HAL_StatusTypeDef my_ret, uint16_t adr, void *data, uint16_t size) {
	uint8_t cmd[EEPROM_ADDRESS_SIZE] = {((adr >> 8)&0xFF), (uint8_t )(adr&0xFF)};

	my_ret=I2C_EEPROM_write(cmd, 2); // write address
	my_ret=I2C_EEPROM_read(data, size); // read operation
	if(my_ret !=HAL_OK) // if error, send command again
	{
		my_ret=I2C_EEPROM_write(cmd, 2); // write address
		my_ret=I2C_EEPROM_read(data, size); // read operation
	}
	return my_ret;
}

/****************************************************************
 * Function Name: EEPROM_erase
 * Description: Erase the entire EEPROM
 * Returns: Hal status of the I2C transmission
 * Params: Hal status of the I2C transmission
 ****************************************************************/
HAL_StatusTypeDef EEPROM_erase(HAL_StatusTypeDef my_ret){
	uint8_t zeroArray[EEPROM_PAGE_SIZE] = {0};
    for(uint16_t page = 0; page < EEPROM_PAGE_NUM; page++)
    {
    	my_ret=EEPROM_write_once(my_ret, page*EEPROM_PAGE_SIZE, zeroArray, EEPROM_PAGE_SIZE);
    	if(my_ret != HAL_OK) // if error command again
    	{
    		my_ret=EEPROM_write_once(my_ret, page*EEPROM_PAGE_SIZE, zeroArray, EEPROM_PAGE_SIZE);
    		if(my_ret != HAL_OK)
    		{
    			//return my_ret; // if error again, exit and return error
    		}
    	}
    }
    return my_ret;
}



/****************************************************************
 * Function Name: Convert_decimal
 * Description: Convert ASCII hexadecimal in decimal
 * Returns: Decimal value
 * Params: Hexadeciimal value in ASCII
 ****************************************************************/
uint8_t Convert_decimal(char ascii_value)
{
	uint8_t decimal_value = 0;
	if(ascii_value>47 && ascii_value<58)
	{
		decimal_value =ascii_value-48;
	}
	if(ascii_value>64 && ascii_value<71)
	{
		decimal_value = ascii_value-65+10; //10=A
	}
    return decimal_value;
}
