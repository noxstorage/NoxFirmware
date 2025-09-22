#include "shtc3.h"

uint8_t pSHTC3_WAKEUP[2] = {0x35,0x17}; // wakeup command
uint8_t pSHTC3_MEASURE[2] = {0x58,0xE0}; // measure command
uint8_t pSHTC3_READOUT[6];
uint8_t pSHTC3_SLEEP[2] = {0xB0,0x98}; //sleep command
uint8_t pSHTC3_RESET[2] = {0x80,0x5D}; //reset command


/////////////////////////////////////////////////////////////////////////////////////////
// Wake up
// OUT: HAL Status
HAL_StatusTypeDef SHTC3_wakeup(HAL_StatusTypeDef ret, I2C_HandleTypeDef * pi2c)
{
// Wake-up commande
	ret=HAL_I2C_Master_Transmit(pi2c, SHTC3_ADDRESS_W, pSHTC3_WAKEUP, sizeof(pSHTC3_WAKEUP), 200);
	HAL_Delay(2); //wakeup time
	return ret;
}


/////////////////////////////////////////////////////////////////////////////////////////
// Read raw_temperature and raw_humidity (values are modified because the address is given)
// OUT: HAL Status
HAL_StatusTypeDef SHTC3_measure(HAL_StatusTypeDef ret, I2C_HandleTypeDef * pi2c, uint16_t * praw_temperature, uint16_t * praw_humidity)
{
	if(ret != HAL_OK) //if there is a problem previously, we exit immediately
	{
		return ret;
	}
	// measurement command
	ret=HAL_I2C_Master_Transmit(pi2c, SHTC3_ADDRESS_W, pSHTC3_MEASURE, sizeof(pSHTC3_MEASURE), 200);
	if(ret !=HAL_OK)
	{
		return ret;
	}
	HAL_Delay(12); //measurement time in normal mode

	//readout
	ret=HAL_I2C_Master_Receive(pi2c, SHTC3_ADDRESS_R, pSHTC3_READOUT, sizeof(pSHTC3_READOUT), 200);

	// modification of the values
	*praw_humidity = (pSHTC3_READOUT[0] << 8) | pSHTC3_READOUT[1];
	*praw_temperature = (pSHTC3_READOUT[3] << 8) | pSHTC3_READOUT[4];

	return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Go to sleep mode
// OUT: HAL Status
HAL_StatusTypeDef SHTC3_sleep(HAL_StatusTypeDef ret, I2C_HandleTypeDef * pi2c)
{
		if(ret !=HAL_OK) //if there is a problem previously, we exit immediately
		{
			return ret;
		}
		//sleep commande
		ret=HAL_I2C_Master_Transmit(pi2c, SHTC3_ADDRESS_W, pSHTC3_SLEEP, sizeof(pSHTC3_SLEEP), 200);

		return ret;
}


/////////////////////////////////////////////////////////////////////////////////////////
// Reset command
// OUT: HAL Status
HAL_StatusTypeDef SHTC3_reset(HAL_StatusTypeDef ret, I2C_HandleTypeDef * pi2c)
{
		ret = HAL_OK;
		//reset commande
		ret=HAL_I2C_Master_Transmit(pi2c, SHTC3_ADDRESS_W, pSHTC3_RESET, sizeof(pSHTC3_RESET), 200);

		return ret;
}

//------------------------------------------------------------------------------
float SHTC3_CalcTemperature(uint16_t rawValue){ //static
  // calculate temperature [?C]
  // T = -45 + 175 * rawValue / 2^16
  return 175 * (float)rawValue / 65536.0f - 45.0f;
}

//------------------------------------------------------------------------------
float SHTC3_CalcHumidity(uint16_t rawValue){ //static
  // calculate relative humidity [%RH]
  // RH = rawValue / 2^16 * 100
  return (float)rawValue / 655.36;
}

