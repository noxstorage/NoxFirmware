#include "ms5637.h"
#include "stm32wlxx_hal.h"
//ICI

//MS5637
uint8_t pMS5637_RESET[1] = {0x1E};   //command 00011110
uint8_t pMS5637_PROMCRC[1] = {0xA0}; //command 1010 000 0
uint8_t pMS5637_PROMC1[1] = {0xA2};  //command 1010 001 0
uint8_t pMS5637_PROMC2[1] = {0xA4};  //command 1010 010 0
uint8_t pMS5637_PROMC3[1] = {0xA6};  //command 1010 011 0
uint8_t pMS5637_PROMC4[1] = {0xA8};  //command 1010 100 0
uint8_t pMS5637_PROMC5[1] = {0xAA};  //command 1010 101 0
uint8_t pMS5637_PROMC6[1] = {0xAC};  //command 1010 110 0
uint8_t  pMS5637_READOUTCRC[2];
uint8_t  pMS5637_READOUTC1[2];
uint8_t  pMS5637_READOUTC2[2];
uint8_t  pMS5637_READOUTC3[2];
uint8_t  pMS5637_READOUTC4[2];
uint8_t  pMS5637_READOUTC5[2];
uint8_t  pMS5637_READOUTC6[2];
uint8_t pMS5637_CONVD1_256[1] = {0x40};  // command to convert D1 with OSR 256
uint8_t pMS5637_CONVD2_256[1] = {0x50};  // command to convert D2 with OSR 256
uint8_t pMS5637_MESURE[1] = {0x00};//00000000
uint8_t  pMS5637_READOUTD1[3];
uint8_t  pMS5637_READOUTD2[3];


/////////////////////////////////////////
// Reset commande
// OUT: HAL Status
HAL_StatusTypeDef MS5637_reset(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c)
{
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_RESET, sizeof(pMS5637_RESET), 200);
	HAL_Delay(3); //Reset time
	if(my_ret !=HAL_OK)
	{
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_RESET, sizeof(pMS5637_RESET), 200);
		HAL_Delay(3); //Reset time
	}
	return my_ret;
}

/////////////////////////////////////////
// Read c1 (c1 value is modified)
// OUT: HAL Status
HAL_StatusTypeDef MS5637_read_c1(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, int64_t * pc1)
{
	if(my_ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return my_ret;
	}
	// read PROM C1 command
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC1, sizeof(pMS5637_PROMC1), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC1, sizeof(pMS5637_PROMC1), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	//readoutC1
	my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC1, sizeof(pMS5637_READOUTC1), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC1, sizeof(pMS5637_READOUTC1), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	*pc1 = (pMS5637_READOUTC1[0] << 8)| pMS5637_READOUTC1[1]; // 46372;
	return my_ret;
}

/////////////////////////////////////////
// Read c2 (c2 value is modified)
// OUT: HAL Status
HAL_StatusTypeDef MS5637_read_c2(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, int64_t * pc2)
{
	if(my_ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return my_ret;
	}
	// read PROM C2 command
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC2, sizeof(pMS5637_PROMC2), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC2, sizeof(pMS5637_PROMC2), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	//readoutC2
	my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC2, sizeof(pMS5637_READOUTC2), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC2, sizeof(pMS5637_READOUTC2), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	*pc2 = (pMS5637_READOUTC2[0] << 8)| pMS5637_READOUTC2[1]; //43981;//
	return my_ret;
}

/////////////////////////////////////////
// Read c3 (c3 value is modified)
// OUT: HAL Status
HAL_StatusTypeDef MS5637_read_c3(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, int64_t * pc3)
{
	if(my_ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return my_ret;
	}
	// read PROM C3 command
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC3, sizeof(pMS5637_PROMC3), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC3, sizeof(pMS5637_PROMC3), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	//readoutC3

	my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC3, sizeof(pMS5637_READOUTC3), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC3, sizeof(pMS5637_READOUTC3), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	*pc3 = (pMS5637_READOUTC3[0] << 8)| pMS5637_READOUTC3[1]; //29059;//
	return my_ret;
}

/////////////////////////////////////////
// Read c4 (c4 value is modified)
// OUT: HAL Status
HAL_StatusTypeDef MS5637_read_c4(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, int64_t * pc4)
{
	if(my_ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return my_ret;
	}
	// read PROM C4 command
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC4, sizeof(pMS5637_PROMC4), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC4, sizeof(pMS5637_PROMC4), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	//readoutC4
	my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC4, sizeof(pMS5637_READOUTC4), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC4, sizeof(pMS5637_READOUTC4), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	*pc4 = (pMS5637_READOUTC4[0] << 8)| pMS5637_READOUTC4[1]; //27842;//
	return my_ret;
}

/////////////////////////////////////////
// Read c5 (c5 value is modified)
// OUT: HAL Status
HAL_StatusTypeDef MS5637_read_c5(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, int64_t * pc5)
{
	if(my_ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return my_ret;
	}
	// read PROM C5 command
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC5, sizeof(pMS5637_PROMC5), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC5, sizeof(pMS5637_PROMC5), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	//readoutC5
	my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC5, sizeof(pMS5637_READOUTC5), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC5, sizeof(pMS5637_READOUTC5), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	*pc5 = (pMS5637_READOUTC5[0] << 8)| pMS5637_READOUTC5[1]; //31553;//
	return my_ret;
}

/////////////////////////////////////////
// Read c6 (c6 value is modified)
// OUT: HAL Status
HAL_StatusTypeDef MS5637_read_c6(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, int64_t * pc6)
{
	if(my_ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return my_ret;
	}
	// read PROM C6 command
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC6, sizeof(pMS5637_PROMC6), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_PROMC6, sizeof(pMS5637_PROMC6), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	//readoutC6
	my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC6, sizeof(pMS5637_READOUTC6), 200);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(5); //small delay
		my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTC6, sizeof(pMS5637_READOUTC6), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	*pc6 = (pMS5637_READOUTC6[0] << 8)| pMS5637_READOUTC6[1]; //28165;//
	return my_ret;
}

/////////////////////////////////////////
// Read d1 (d1 value is modified)
// OUT: HAL Status
HAL_StatusTypeDef MS5637_read_d1(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, int64_t *pd1)
{
	if(my_ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return my_ret;
	}
	// conversion command D1
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_CONVD1_256, sizeof(pMS5637_CONVD1_256), 200);
	if(my_ret !=HAL_OK)
	{
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_CONVD1_256, sizeof(pMS5637_CONVD1_256), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	/////////////////////////////
	HAL_Delay(2); //conversion time
	//////////////////////////////
	// mesure command D1
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_MESURE, sizeof(pMS5637_MESURE), 200);
	if(my_ret !=HAL_OK)
	{
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_MESURE, sizeof(pMS5637_MESURE), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	/////////////////////////////
	HAL_Delay(2); //conversion time
	//////////////////////////////

	//ADC readout D1
	my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTD1, sizeof(pMS5637_READOUTD1), 200);
	if(my_ret !=HAL_OK)
	{
		my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTD1, sizeof(pMS5637_READOUTD1), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}

	*pd1 = (pMS5637_READOUTD1[0] << 16) | (pMS5637_READOUTD1[1] << 8)| pMS5637_READOUTD1[2]; //6465444;//
	return my_ret;
}


/////////////////////////////////////////
// Read d2 (d2 value is modified)
// OUT: HAL Status
HAL_StatusTypeDef MS5637_read_d2(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, int64_t *pd2)
{
	if(my_ret !=HAL_OK) //if there is a problem previously, we exit immediately
	{
		return my_ret;
	}
	// conversion command D2
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_CONVD2_256, sizeof(pMS5637_CONVD2_256), 200);
	if(my_ret !=HAL_OK)
	{
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_CONVD2_256, sizeof(pMS5637_CONVD2_256), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	/////////////////////////////
	HAL_Delay(2); //conversion time
	//////////////////////////////

	// mesure command D2
	my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_MESURE, sizeof(pMS5637_MESURE), 200);
	if(my_ret !=HAL_OK)
	{
		my_ret=HAL_I2C_Master_Transmit(pi2c, MS5637_ADDRESS_W, pMS5637_MESURE, sizeof(pMS5637_MESURE), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	/////////////////////////////
	HAL_Delay(2); //conversion time
	//////////////////////////////
	//ADC readout D2
	my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTD2, sizeof(pMS5637_READOUTD2), 200);
	if(my_ret !=HAL_OK)
	{
		my_ret=HAL_I2C_Master_Receive(pi2c, MS5637_ADDRESS_R, pMS5637_READOUTD2, sizeof(pMS5637_READOUTD2), 200);
		if(my_ret !=HAL_OK)
		{
			return my_ret;
		}
	}
	*pd2 = (pMS5637_READOUTD2[0] << 16) | (pMS5637_READOUTD2[1] << 8)| pMS5637_READOUTD2[2]; //8077636;//
	return my_ret;
}





//------------------------------------------------------------------------------
float MS5637_CalcTemperature(int64_t dT,int64_t c6){ //static
  // calculate temperature
  // temperature = 2000 +dT*c6/(2**23);
  return 0.01*(float)(2000+dT*c6/8388608);
}

//------------------------------------------------------------------------------
float MS5637_CalcPressure(int64_t sens, int64_t off, int64_t d1){ //static
  // calculate pressure
  // P = (d1 * sens/(2**21) - off)*(2**15)
  return 0.01*(float)((d1 * sens/2097152 - off)/32768);
}







