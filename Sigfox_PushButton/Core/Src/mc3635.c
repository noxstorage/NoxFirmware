#include "mc3635.h"
#include <math.h> // to use sqrt()


// NB: Set STANDBY MODE (p.24/83) to be able to send a command
  // To go in standby mode, we have to write a byte in the register 0x10
  // The first bytes desactivate or activate axis (x/y/z)
  // In our application, we want an easy communication and always the three axis
  // Thus, we can write 0b00000+mode
  // Otherwise it would be necessary to read the register to know the order previously passed
//


uint8_t pValueSniffC_odr[1] = {0b00000000}; // Sniff control : modified according to parameter
//uint8_t pValueSniffC_50[1] = {0b00000111}; // Sniff control : 50Hz
//uint8_t pValueSniffC_190[1] = {0b00001001}; // Sniff control : 190Hz
uint8_t pValueINTRC[1] = {0b10000111}; // Interrupt control : active high

// Lists for readout
uint8_t pREADOUT[6];
uint8_t pREADOUT_I2C[1];
uint8_t pREADOUT_INT[1];
uint8_t pREADOUT_21[1];

// Acceleration value after readout
int8_t MC3635_x=0;
int8_t MC3635_y=0;
int8_t MC3635_z=0;
float XAxis_g;
float YAxis_g;
float ZAxis_g;
float Norm_g=0;


/////////////////////////////////////////
// Initialisation sequence (according to p.22/83 datasheet)
// OUT: HAL Status
HAL_StatusTypeDef MC3635_initilization(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c)
{

  if(my_ret !=HAL_OK)
  {
	  return my_ret;
  }
  uint8_t pValue01[1] = {0x01};
  uint8_t pValue40[1] = {0x40};
  uint8_t pValue42[1] = {0x42};
  uint8_t pValue80[1] = {0x80};
  uint8_t pValue00[1] = {0x00};
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x10,1,pValue01,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x24,1,pValue40,1,200);
  HAL_Delay(10); //reset time
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x0D,1,pValue40,1,200); // Enable I2C
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x0F,1,pValue42,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x20,1,pValue01,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x21,1,pValue80,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x28,1,pValue00,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x1A,1,pValue00,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50); //small delay
	  // command again
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x10,1,pValue01,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x24,1,pValue40,1,200);
	  HAL_Delay(10); //reset time
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x0D,1,pValue40,1,200); // Enable I2C
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x0F,1,pValue42,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x20,1,pValue01,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x21,1,pValue80,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x28,1,pValue00,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x1A,1,pValue00,1,200);
  }
  return my_ret;
}


/////////////////////////////////////////
// Set sniff and interrupt
// OUT: HAL Status
HAL_StatusTypeDef MC3635_SET_sniff_int(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, MC3635_sniff_sr_t sniff_odr, uint8_t sensibility)
{
  if(my_ret !=HAL_OK)
  {
	return my_ret;
  }
  uint8_t pValueSniffCFRESET[1] = {0b10001000}; // Sniff configuration : SNIFF RESET
  uint8_t pValueSniffCF001[1] = {0b00001001}; // Sniff configuration : SNIFF Threshold X-axis
  uint8_t pValueSniffCF010[1] = {0b00001010}; // Sniff configuration : SNIFF Threshold Y-axis
  uint8_t pValueSniffCF011[1] = {0b00001011}; // Sniff configuration : SNIFF Threshold Z-axis
  uint8_t pValueSniffCF101[1] = {0b00001101}; // Sniff configuration : SNIFF Detection Count X-axis
  uint8_t pValueSniffCF110[1] = {0b00001110}; // Sniff configuration : SNIFF Detection Count Y-axis
  uint8_t pValueSniffCF111[1] = {0b00001111}; // Sniff configuration : SNIFF Detection Count Z-axis

  uint8_t pValueSniffTH_T[1]  = {0b10000110}; // Sniff threshold control : C2B Mode / OR + SNIFF Threshold X/Y/Z-axis : threshold m/(s**2) -> n*0.61. Here : n = 0b110 = 6
  uint8_t pValueSniffTH_DC[1] = {0b10000000}; // Sniff threshold control : C2B Mode / OR + SNIFF Detection Count X/Y/Z-axis : n+1 event(s). Here : 0+1=1

  // param_sens.
  // When the parameter is 1 : n=6 // 1 : default value // (1 : highly sensitive)
  // When the parameter is 6 : n=11 // (6 : not sensitive)
  if(sensibility>0 && sensibility<7)
  {
	  pValueSniffTH_T[0]= 0b10000101 + (uint8_t)sensibility;
  }


  //RESET block sniff
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCFRESET,1,200);

  // SET SNIFF : X threshold (P.55+58/83 datasheet MC3635)
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF001,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_T,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  //RESET block sniff
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCFRESET,1,200);
	  // SET SNIFF : X threshold (P.55+58/83 datasheet MC3635)
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF001,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_T,1,200);
	  if(my_ret !=HAL_OK)
	  {
		return my_ret;
	  }
  }

  // SET SNIFF : Y threshold (P.55+58/83 datasheet MC3635)
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF010,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_T,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  // SET SNIFF : Y threshold (P.55+58/83 datasheet MC3635)
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF010,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_T,1,200);
	  if(my_ret !=HAL_OK)
	  {
		return my_ret;
	  }
  }

  // SET SNIFF : Z threshold (P.55+58/83 datahseet MC3635)
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF011,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_T,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  // SET SNIFF : Z threshold (P.55+58/83 datahseet MC3635)
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF011,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_T,1,200);
	  if(my_ret !=HAL_OK)
	  {
		return my_ret;
	  }
  }

  // SET SNIFF : x count (P.55+58/83 datahseet MC3635)
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF101,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_DC,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  // SET SNIFF : x count (P.55+58/83 datahseet MC3635)
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF101,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_DC,1,200);
	  if(my_ret !=HAL_OK)
	  {
		return my_ret;
	  }
  }

  // SET SNIFF : y count (P.55+58/83 datahseet MC3635)
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF110,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_DC,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  // SET SNIFF : y count (P.55+58/83 datahseet MC3635)
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF110,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_DC,1,200);
	  if(my_ret !=HAL_OK)
	  {
		return my_ret;
	  }
  }

  // SET SNIFF : y count (P.55+58/83 datasheet MC3635)
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF111,1,200);
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_DC,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  // SET SNIFF : y count (P.55+58/83 datasheet MC3635)
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x14,1,pValueSniffCF111,1,200);
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x13,1,pValueSniffTH_DC,1,200);
	  if(my_ret !=HAL_OK)
	  {
		return my_ret;
	  }
  }
  pValueSniffC_odr[0]=sniff_odr;
  // SET Sniff control
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x12,1,pValueSniffC_odr,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  // SET Sniff control
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x12,1,pValueSniffC_odr,1,200);
	  if(my_ret !=HAL_OK)
	  {
		return my_ret;
	  }
  }
  // SET Interrupt
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x17,1,pValueINTRC,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  // SET Interrupt
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x17,1,pValueINTRC,1,200);
  }

  return my_ret;

}


/////////////////////////////////////////
// Set mode
// OUT: HAL Status
HAL_StatusTypeDef MC3635_SET_mode(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, MC3635_mode_t mode)
{
  if(my_ret !=HAL_OK)
  {
	  return my_ret;
  }
  uint8_t pMODE[1] = {0b00000000};
  pMODE[0]=mode;
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x10,1,pMODE,1,200);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x10,1,pMODE,1,200);
  }
  return my_ret;
}


/////////////////////////////////////////
// Read one acceleration (value passed thanks to pNorm_g pointer)
// OUT: HAL Status
HAL_StatusTypeDef MC3635_READ_ONE(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, float * pNorm_g)
{
  if(my_ret !=HAL_OK)
  {
	return my_ret;
  }
  my_ret=HAL_I2C_Mem_Read(pi2c,MC3635_ADDRESS_R,0x02,1,pREADOUT,6,200);
  if(my_ret !=HAL_OK)
  {
	HAL_Delay(50);//small delay
	// Then command again
	my_ret=HAL_I2C_Mem_Read(pi2c,MC3635_ADDRESS_R,0x02,1,pREADOUT,6,200);
  }


  MC3635_x = pREADOUT[0];//((((uint_16)pREADOUT[1]) << 8) | (uint_16)pREADOUT[0]);
  MC3635_y = pREADOUT[2];//((((uint_16)pREADOUT[3]) << 8) | (uint_16)pREADOUT[2]);
  MC3635_z = pREADOUT[4];//((((uint_16)pREADOUT[5]) << 8) | (uint_16)pREADOUT[4]);

  XAxis_g = ((float) (MC3635_x) / 128.0f)*78.456f;
  YAxis_g = ((float) (MC3635_y) / 128.0f)*78.456f;
  ZAxis_g = ((float) (MC3635_z) / 128.0f)*78.456f;
  *pNorm_g = sqrt(XAxis_g*XAxis_g + YAxis_g*YAxis_g + ZAxis_g*ZAxis_g);

  return my_ret;
}

/////////////////////////////////////////
// Read FIFO after a transition between SNIFF to CWAKE.
// After a transition FIFO is freeze (by using the NORMAL mode when FIFO is enabled)
// Data (acceleration norm) in pREAD_FIFO. Multiply by (78.456f/128.0f) to have the norm in USI
// OUT: HAL Status
HAL_StatusTypeDef MC3635_READ_FIFO_sniffTOcwake(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, uint8_t pREAD_FIFO[])
{
  if(my_ret !=HAL_OK)
  {
	  return my_ret;
  }
  int i = 0;
  for (i=0;i<32;i++)
  {
	  my_ret=HAL_I2C_Mem_Read(pi2c,MC3635_ADDRESS_R,0x02,1,pREADOUT,6,200);
	  if(my_ret !=HAL_OK)
	  { // command again
		  my_ret=HAL_I2C_Mem_Read(pi2c,MC3635_ADDRESS_R,0x02,1,pREADOUT,6,200);
	  }
	  MC3635_x = pREADOUT[0];//((((uint_16)pREADOUT[1]) << 8) | (uint_16)pREADOUT[0]);
	  MC3635_y = pREADOUT[2];//((((uint_16)pREADOUT[3]) << 8) | (uint_16)pREADOUT[2]);
	  MC3635_z = pREADOUT[4];//((((uint_16)pREADOUT[5]) << 8) | (uint_16)pREADOUT[4]);
	  XAxis_g = ((float) (MC3635_x) / 128.0f)*78.456f;
	  YAxis_g = ((float) (MC3635_y) / 128.0f)*78.456f;
	  ZAxis_g = ((float) (MC3635_z) / 128.0f)*78.456f;
	  Norm_g = sqrt(XAxis_g*XAxis_g + YAxis_g*YAxis_g + ZAxis_g*ZAxis_g)/(78.456f/128.0f);
	  pREAD_FIFO[i]=(int8_t)Norm_g;
  }
  return my_ret;
}

/////////////////////////////////////////
// Set FIFO with FIFO status in z-axis and without the FIFO burst
// OUT: HAL Status
// fifo_stream : MC3635_FIFO_MODE_NORMAL or MC3635_FIFO_MODE_CONTINUOUS
HAL_StatusTypeDef MC3635_SET_FIFO(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c, MC3635_fifo_mode_t fifo_stream)
{
  if(my_ret !=HAL_OK)
  {
	  return my_ret;
  }
  uint8_t pFREG_2[1] = {0b00001000};
  pFREG_2[0]= pFREG_2[0]|fifo_stream ; // FIFO status is enabled // with the FIFO burst
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x0E,1,pFREG_2,1,200);
  if(my_ret !=HAL_OK)
  { // command again
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x0E,1,pFREG_2,1,200);
  }
  // (p.62/83) FIFO control register
  uint8_t pFIFO_C[1] = {0b01011111}; // threshold 32 //FIFO enabled
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x16,1,pFIFO_C,1,200);
  if(my_ret !=HAL_OK)
  {
	  // command again
	  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x16,1,pFIFO_C,1,200);
  }
  return my_ret;
}

/////////////////////////////////////////
// SET
// Range -/+ 8g
// Resolution 8bits
// Ultra Low power
// ODR : 50Hz
// OUT: HAL Status
// fifo_stream : MC3635_FIFO_MODE_NORMAL or MC3635_FIFO_MODE_CONTINUOUS
HAL_StatusTypeDef MC3635_SET_8b8g_ODRWake50ULP_once(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c)
{
  if(my_ret !=HAL_OK)
  {
	  return my_ret;
  }
  uint8_t pValue8b8g[1] = {0b00100010}; // 8bit // 8g
  uint8_t pValue50[1] = {0b00000111}; // Ultra low power with 50Hz
  uint8_t pValueULP[1] = {0b00110011}; // ULP
  // uint8_t pValuePRECIS[1] = {0b01000100}; // Precis

  // SET : Resolution and range
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x15,1,pValue8b8g,1,200);
  if(my_ret !=HAL_OK)
  {
	return my_ret;
  }

  // SET : ODR (P.50/83 datasheet MC3635)
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x11,1,pValue50,1,200);
  if(my_ret !=HAL_OK)
  {
	return my_ret;
  }
  // SET : ULP (P.50/83 datasheet MC3635)
  my_ret=HAL_I2C_Mem_Write(pi2c,MC3635_ADDRESS_W,0x1C,1,pValueULP,1,200);
  return my_ret;
}
HAL_StatusTypeDef MC3635_SET_8b8g_ODRWake50ULP(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c)
{
  if(my_ret !=HAL_OK)
  {
	  return my_ret;
  }
  my_ret=MC3635_SET_8b8g_ODRWake50ULP_once(my_ret, pi2c);
  if(my_ret !=HAL_OK)
  {
	  HAL_Delay(50);//small delay
	  // Then command again
	  my_ret=MC3635_SET_8b8g_ODRWake50ULP_once(my_ret, pi2c);
  }

  return my_ret;
}



/////////////////////////////////////////
// Read and clear the interrupt
// OUT: the content of the interrupt register (0x09) (ex: 0b00000100 for SNIFF to CWAKE transition)
// NB : I cannot send the command again, if there is a problem because after a read the register is cleared

uint8_t MC3635_READ_CLEAR_int(HAL_StatusTypeDef my_ret, I2C_HandleTypeDef * pi2c)
{
  // read register 0x09 to reset
  uint8_t pREADOUT_INT[1];
  my_ret=HAL_I2C_Mem_Read(pi2c,MC3635_ADDRESS_R,0x09,1,pREADOUT_INT,1,200); //read and clear interrupt register
  return pREADOUT_INT[0];
}




