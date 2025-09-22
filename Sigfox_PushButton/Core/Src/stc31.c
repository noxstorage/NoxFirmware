#include "stc31.h"

uint8_t pSTC31_GAS_100[3] = {0x00, 0x01, 0xB0}; //command 100% CO2 in air and CRC 0xB0
uint8_t pSTC31_GAS_25[3] = {0x00, 0x03, 0xD2}; //command 25% CO2 in air and CRC 0xD2
uint8_t pSTC31_RH[3]; //set RH to STC31 and CRC
uint8_t pSTC31_T[3]; //set T to STC31 and CRC
uint8_t pSTC31_P[3]; //set P and CRC
uint8_t pSTC31_MEASURE[2] = {0x36,0x39};
uint8_t pSTC31_SLEEP[2] = {0x36,0x77};
uint8_t pSTC31_READOUT[9];

/////////////////////////////////////////////////////////////////////////////////////////
// Wake-up the sensor
// OUT: HAL Status
HAL_StatusTypeDef STC31_wakeup(HAL_StatusTypeDef ret, I2C_HandleTypeDef * pi2c)
{
	//set binary gas
	ret=HAL_I2C_Mem_Write(pi2c,STC3X_DEFAULT_ADDRESS<<1,STC3x_COMMAND_SET_BINARY_GAS,2,pSTC31_GAS_100,3,200);
	return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Set the humidity, temperature and pressure value for the compensation to determine the CO2 concentration
// OUT: HAL Status
HAL_StatusTypeDef STC31_setRHandTandP(HAL_StatusTypeDef ret, I2C_HandleTypeDef * pi2c, float humidity, float temperature, float MS5637_pressure)
{
	if(ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return ret;
	}
	//set humidity
	uint16_t raw_humidity_STC31;
	raw_humidity_STC31 = STC31_CalcHumidity(humidity);
	pSTC31_RH[0]= ((raw_humidity_STC31 & 0xFF00) >> 8);
	pSTC31_RH[1]= (raw_humidity_STC31 & 0x00FF);
	pSTC31_RH[2]= STC31_computeCRC8(pSTC31_RH,2);
	ret=HAL_I2C_Mem_Write(pi2c,STC3X_DEFAULT_ADDRESS<<1,STC3X_COMMAND_SET_RELATIVE_HUMIDITY,2,pSTC31_RH,3,50);
	if(ret !=HAL_OK)
	{
		return ret;
	}
	//set temperature
	if (temperature>-20) //only if we have a value >-20 else it is an error
	{
		uint16_t raw_temperature_STC31;
		raw_temperature_STC31 = STC31_CalcTemperature(temperature);
		pSTC31_T[0]= ((raw_temperature_STC31 & 0xFF00) >> 8);
		pSTC31_T[1]= (raw_temperature_STC31 & 0x00FF);
		pSTC31_T[2]= STC31_computeCRC8(pSTC31_T,2);
		ret=HAL_I2C_Mem_Write(pi2c,STC3X_DEFAULT_ADDRESS<<1,STC3X_COMMAND_SET_TEMPERATURE,2,pSTC31_T,3,200);
	}
	if(ret !=HAL_OK)
	{
		return ret;
	}
	//set pressure
	uint16_t raw_pressure_STC31;
	raw_pressure_STC31=(uint16_t)MS5637_pressure;
	pSTC31_P[0]= ((raw_pressure_STC31 & 0xFF00) >> 8);
	pSTC31_P[1]= (raw_pressure_STC31 & 0x00FF);
	pSTC31_P[2]= STC31_computeCRC8(pSTC31_P,2);
	ret=HAL_I2C_Mem_Write(pi2c,STC3X_DEFAULT_ADDRESS<<1,STC3X_COMMAND_SET_PRESSURE,2,pSTC31_P,3,200);
	return ret;
}
/////////////////////////////////////////////////////////////////////////////////////////
// Measure the co2 concentration (the value is modified in the global variable co2 because the address is given)
// OUT: HAL Status
HAL_StatusTypeDef STC31_measure(HAL_StatusTypeDef ret, I2C_HandleTypeDef * pi2c, float * pco2)
{
	if(ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return ret;
	}
	// command mesaurement
	ret=HAL_I2C_Master_Transmit(pi2c,STC3X_DEFAULT_ADDRESS<<1,pSTC31_MEASURE,2,200);
	if(ret !=HAL_OK)
	{
		return ret;
	}
	HAL_Delay(67); //measurement time
	//readout
	ret=HAL_I2C_Master_Receive(pi2c,(STC3X_DEFAULT_ADDRESS<<1)|0x01,pSTC31_READOUT,9,200);
	if(ret !=HAL_OK)
	{
		return ret;
	}
	uint16_t raw_co2;
	raw_co2 = (pSTC31_READOUT[0] << 8) | pSTC31_READOUT[1];
	if(raw_co2 <16384)
	{
		raw_co2 = 16384;
	}
	*pco2 = ((float)raw_co2 - 16384.0)/327.68;
	// sleep command
	//ret=HAL_I2C_Master_Transmit(&hi2c1,STC3X_DEFAULT_ADDRESS<<1,pSTC31_SLEEP,2,50);
	return ret;
}


/////////////////////////////////////////////////////////////////////////////////////////
// Convert the humidity in a raw value to set the STC31
// IN : humidity value (express in float)
// OUT: raw humidity value
uint16_t STC31_CalcHumidity(float RH){
  // calculate relative humidity [%RH]
  // rawValue = RH * (2^16-1) / 100
  return (uint16_t)(RH*655.35);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Convert the temperature in a raw value to set the STC31
// IN : temperature value (express in float)
// OUT: raw temperature value
uint16_t STC31_CalcTemperature(float temperat){
  // calculate temperature [?C]
  // rawValue = 200*temperat
  return  (uint16_t)(200*temperat);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Convert the humidity in a raw value to set the STC31
//Given an array and a number of bytes, this calculate CRC8 for those bytes
//CRC is only calc'd on the data portion (two bytes) of the four bytes being sent
//From: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
//Tested with: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
//x^8+x^5+x^4+1 = 0x31
//
// IN : data list & size
// OUT: CRC
uint8_t STC31_computeCRC8(uint8_t data[], uint8_t len)
{
  uint8_t crc = 0xFF; //Init with 0xFF

  for (uint8_t x = 0; x < len; x++)
  {
    crc ^= data[x]; // XOR-in the next input byte

    for (uint8_t i = 0; i < 8; i++)
    {
      if ((crc & 0x80) != 0)
        crc = (uint8_t)((crc << 1) ^ 0x31);
      else
        crc <<= 1;
    }
  }

  return crc; //No output reflection
}

