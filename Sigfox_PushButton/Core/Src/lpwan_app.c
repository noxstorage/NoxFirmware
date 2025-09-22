/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lpwan_app.c
  * @author  Louis LABROT
  * @brief   provides code for the application of the LPWAN Middleware
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include "stm32_timer.h" //edit
#include "usart.h" //edit
#include "i2c.h" //edit
#include <math.h> //edit
#include "m24512.h" // eeprom
#include "ms5637.h" //pressure
#include "stc31.h" //co2
#include "shtc3.h" //RH&T
#include "mc3635.h" //accelerometer
#include "da14531.h" // ble function

#include "sgfx_app.h"
#include "lpwan_app.h"




void LPWAN_init1(void)
{
  nb_wakeup = 1; //init the variable nb_wakeup
  // Activation Signal
  Init_LED3();
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); /* LED_RED */
  HAL_Delay(1000);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  // BLE INITILISATION//////////////////////////////////////////
  HAL_StatusTypeDef my_ret_ble = HAL_OK;
  #if defined(BLE_WIRED)
  MX_LPUART1_UART_Init(); // car low power
  HAL_Delay(4000); //wakeup BLE
  BLE_Init(&hlpuart1, my_ret_ble);
  #endif
  /////////////////////////////////////////////////////////////
}


float LPWAN_pressure(void)
{
  HAL_StatusTypeDef my_ret = HAL_OK;
  float MS5637_pressure_lpwan;
  #if defined(P_WIRED)
  //MS5637 variable
  int64_t MS5637_c1;
  int64_t MS5637_c2;
  int64_t MS5637_c3;
  int64_t MS5637_c4;
  int64_t MS5637_c5;
  int64_t MS5637_c6;
  int64_t MS5637_d1;
  int64_t MS5637_d2;
  int64_t MS5637_dT;
  int64_t MS5637_sens;
  int64_t MS5637_off;
  //float MS5637_temperature; //not used because we use only the SHTC3 temperature

  // Measure pressure thanks to MS5637 sensor
  my_ret = MS5637_reset(my_ret,&hi2c1);
  my_ret = MS5637_read_c1(my_ret, &hi2c1, &MS5637_c1);
  my_ret = MS5637_read_c2(my_ret, &hi2c1, &MS5637_c2);
  my_ret = MS5637_read_c3(my_ret, &hi2c1, &MS5637_c3);
  my_ret = MS5637_read_c4(my_ret, &hi2c1, &MS5637_c4);
  my_ret = MS5637_read_c5(my_ret, &hi2c1, &MS5637_c5);
  my_ret = MS5637_read_c6(my_ret, &hi2c1, &MS5637_c6);
  my_ret = MS5637_read_d1(my_ret, &hi2c1, &MS5637_d1);
  my_ret = MS5637_read_d2(my_ret, &hi2c1, &MS5637_d2);
  if(my_ret !=HAL_OK)
  {
	  return (float)0xFFFE; // error
  }
  // intermediate variable for pressure and temperature calculation for MS5637
  MS5637_dT = MS5637_d2 - MS5637_c5*256;
  MS5637_off =  (MS5637_c2*131072)+((MS5637_c4*MS5637_dT)>>8); // c2*(2**17)+(c4*dT)/(2**8)
  MS5637_sens = (MS5637_c1<<16)+((MS5637_c3*MS5637_dT)>>7); // c1*(2**16)+(c3*dT)/(2**7)
  //calculation for temperature and pressure (MS5637)
  //MS5637_temperature = 0.01*(float)(2000+MS5637_dT*MS5637_c6/8388608); //not used (because we use a separate temperature sensor)
  MS5637_pressure_lpwan = 0.01*(float)((MS5637_d1 * MS5637_sens/2097152 - MS5637_off)/32768);
  #endif
  return MS5637_pressure_lpwan;
}


void LPWAN_Init_Acc(void)
{
  #if defined(SENSOR_ACC_WIRED)
  HAL_StatusTypeDef my_ret = HAL_OK;
  //MC3635 or MC2630
  // sequence d'initialisation I2C du MC3635 (p.22/83 datasheet)
  my_ret=MC3635_initilization(my_ret,&hi2c1);
  if(my_ret !=HAL_OK)
  {
	  MC3635_initilization(my_ret,&hi2c1);
  }

  // SET STANDBY MODE (p.24/83)
  // to be able to send a command
  // To go in standby mode, we have to write a byte in the register 0x10
  // The first bytes desactivate or activate axis (x/y/z)
  // In our application, we want an easy communication and always the three axis
  // Thus, we can write 0b00000+mode
  // Otherwise it would be necessary to read the register to know the order previously passed
  my_ret=MC3635_SET_mode(my_ret, &hi2c1, MC3635_MODE_STANDBY);

  // Configuration
  my_ret= MC3635_SET_8b8g_ODRWake50ULP(my_ret, &hi2c1);

  //SET FIFO
  my_ret=MC3635_SET_FIFO(my_ret, &hi2c1, MC3635_FIFO_MODE_NORMAL);

  // SET SNIFF	and interrupt
  my_ret=MC3635_SET_sniff_int(my_ret, &hi2c1, MC3635_SNIFF_SR_200Hz, MC3635_sensibility);

  // Sortir du mode STANDBY
  my_ret=MC3635_SET_mode(my_ret, &hi2c1, MC3635_MODE_SNIFF);
  if(my_ret !=HAL_OK)
  {
  	//My_Error_Handler(); // not blocking
  }
  HAL_Delay(10); //STANDBY delay
  #endif
}

uint8_t LPWAN_CO2(float SHTC3_humidity, float SHTC3_temperature, float MS5637_pressure)
{
  //STC31 variable
  float co2=1;
  #if defined(CO2_WIRED)
  HAL_StatusTypeDef my_ret = HAL_OK;
  //STC31////////////////////////////////////////////////
  my_ret = STC31_wakeup(my_ret, &hi2c1);
  my_ret = STC31_setRHandTandP(my_ret, &hi2c1, SHTC3_humidity, SHTC3_temperature, MS5637_pressure);
  my_ret = STC31_measure(my_ret, &hi2c1, &co2);
  if(my_ret !=HAL_OK)
  {
  	HAL_Delay(500); // to avoid to many measurements command in one second
  	my_ret = STC31_wakeup(my_ret, &hi2c1);
  	my_ret = STC31_setRHandTandP(my_ret, &hi2c1, SHTC3_humidity, SHTC3_temperature, MS5637_pressure);
  	my_ret = STC31_measure(my_ret, &hi2c1, &co2);
  	if(my_ret !=HAL_OK)
  	{
  		return 0xFE; // error
  	}
  }
  #endif
  return (uint8_t)(co2*(float)2);
}

void BLElink(void)
{
  uint8_t pcommand_epprom[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  HAL_StatusTypeDef my_ret_ble = HAL_OK;

  #if defined(TEST_MODE) && (TEST_MODE == 1)
  APP_PPRINTF("BLE");
  #endif /* TEST_MODE */

  //Init_LED3();
  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); /* LED_RED */
  //HAL_Delay(200);
  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

#if defined(BLE_WIRED)
  MX_LPUART1_UART_Init(); // car low power
  HAL_Delay(50);
  BLE_WakeUp(&hlpuart1, my_ret_ble);
  HAL_Delay(100);
  MX_LPUART1_UART_Init(); // car low power
  BLE_ReadMEM3(&hlpuart1, my_ret_ble, aRxBuffer1, RXBUFFERSIZE1);
  if(my_ret_ble != HAL_OK)
  {
  	//Error_Handler(); // no blocking state
  }
  uint8_t test_secret_code = 0xFF;
  if (aRxBuffer1[2]==(SECRET_CODE_1+48) && aRxBuffer1[3]==(SECRET_CODE_2+48) && aRxBuffer1[4]==(SECRET_CODE_3+48) && aRxBuffer1[5]==(SECRET_CODE_4+48))
  { // 1 5 2 3
	test_secret_code = 0;
	  Init_LED3();
	  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); /* LED_RED */
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  }
  #if defined(TEST_MODE) && (TEST_MODE == 1)
  else
  {
	//APP_PPRINTF("\r\n\n\r");
	//APP_PPRINTF("secret code NOK\r\n\n\r");

  }
  #endif /* TEST_MODE */


  if (aRxBuffer1[6]==67 && aRxBuffer1[7]==66 && aRxBuffer1[16]==67 && aRxBuffer1[17]==66 && test_secret_code == 0)
  {
  	pcommand_epprom[2] = 0xCB;
  	pcommand_epprom[3] = Convert_decimal(aRxBuffer1[8])*16+Convert_decimal(aRxBuffer1[9]); //48 === 0 in ascii)
  	pcommand_epprom[4] = Convert_decimal(aRxBuffer1[10])*16+Convert_decimal(aRxBuffer1[11]); //48 === 0 in ascii)
  	pcommand_epprom[5] = Convert_decimal(aRxBuffer1[12])*16+Convert_decimal(aRxBuffer1[13]); //48 === 0 in ascii)
  	pcommand_epprom[6] = Convert_decimal(aRxBuffer1[14])*16+Convert_decimal(aRxBuffer1[15]);
  	pcommand_epprom[7] = 0xCB;

	#if defined(TEST_MODE) && (TEST_MODE == 1)
  	APP_PPRINTF("\r\n\n\r");
  	APP_PPRINTF("vvvvCBxxyyzzwwCB\r\n\n\r");
  	char my_string[2];
  	itoa(pcommand_epprom[3],my_string,2);
  	APP_PPRINTF(my_string);
  	APP_PPRINTF("\n\r");
  	itoa(pcommand_epprom[4],my_string,2);
  	APP_PPRINTF(my_string);
  	APP_PPRINTF("\n\r");
  	itoa(pcommand_epprom[5],my_string,2);
  	APP_PPRINTF(my_string);
  	APP_PPRINTF("\n\r");
  	itoa(pcommand_epprom[6],my_string,2);
  	APP_PPRINTF(my_string);
	#endif /* TEST_MODE */

#if defined(MEM_EEPROM_WIRED)
    /////////////////////////////////////////////
  	// reinitilisation of I2C1 and pin I2C
  	MX_I2C1_Init();
  	Init_Switch();
  	Switch_Set();
  	HAL_StatusTypeDef my_ret_eeprom = HAL_OK;
  	if(EEPROM_write(my_ret_eeprom, 0, pcommand_epprom, sizeof(pcommand_epprom)/sizeof(pcommand_epprom[0])) !=HAL_OK)
  	{
  		//My_Error_Handler_EEPROM();
  	}
  	Switch_Reset();
#endif
  }

  #if defined(BLE_WIRED)
  MX_LPUART1_UART_Init();
  BLE_ReadMEM2(&hlpuart1, my_ret_ble, aRxBuffer2, RXBUFFERSIZE2);
  #endif

  if (aRxBuffer2[2]==67 && aRxBuffer2[3]==65)// If CA(00)
  {
	  BLE_ResetMEM2(&hlpuart1, my_ret_ble);
	  #if defined(TEST_MODE) && (TEST_MODE == 1)
      APP_PPRINTF("\rResetMEM2\n\r");
      #endif /* TEST_MODE */
  }
  if (aRxBuffer1[6]==67 && aRxBuffer1[7]==66) //if CB
  {
	  BLE_ResetMEM3(&hlpuart1, my_ret_ble);
	  aRxBuffer1[0]=0;
	  aRxBuffer1[1]=0;
	  aRxBuffer1[2]=0;
	  aRxBuffer1[3]=0;
	  aRxBuffer1[4]=0;
	  aRxBuffer1[5]=0;
	  aRxBuffer1[6]=0;
	  aRxBuffer1[7]=0;
	  aRxBuffer1[8]=0;
	  aRxBuffer1[9]=0;
	  aRxBuffer1[10]=0;
	  aRxBuffer1[11]=0;
	  aRxBuffer1[12]=0;
	  aRxBuffer1[13]=0;
	  aRxBuffer1[14]=0;
	  aRxBuffer1[15]=0;
	  aRxBuffer1[16]=0;
  }



  if (aRxBuffer2[2]==67 && aRxBuffer2[3]==65 && aRxBuffer2[4]==49 && aRxBuffer2[5]==49) // If CA11
  {
	  #if defined(TEST_MODE) && (TEST_MODE == 1)
	  APP_PPRINTF("\rCA11\r\n\n\r");
	  #endif /* TEST_MODE */

	  //Ecrire dans la mémoire 1
	  send_ble_data = 1;
      #if defined(SIGFOX_DOWNLINK)
	  Sigfox_Downlink = SFX_FALSE;
	  SendSigfox(); // Measurement and send data
	  Sigfox_Downlink = SIGFOX_DOWNLINK;
      #elif defined(LORA_DOWNLINK)
	  LPWAN_Downlink = 0;
	  SendTxData(); // Measurement and send data
	  LPWAN_Downlink = LORA_DOWNLINK;
      #endif
	  send_ble_data = 0;
  }

  if (aRxBuffer2[2]==67 && aRxBuffer2[3]==65 && aRxBuffer2[4]==48 && aRxBuffer2[5]==48) // If CA00
  {
	  #if defined(TEST_MODE) && (TEST_MODE == 1)
	  APP_PPRINTF("\rCA00\r\n\n\r");
	  #endif /* TEST_MODE */

	  send_ble_data = 2;
      #if defined(SIGFOX_DOWNLINK)
      Sigfox_Downlink = SFX_FALSE;
      SendSigfox(); // Measurement and send data
      Sigfox_Downlink = SIGFOX_DOWNLINK;
      #elif defined(LORA_DOWNLINK)
      LPWAN_Downlink = 0;
      SendTxData(); // Measurement and send data
      LPWAN_Downlink = LORA_DOWNLINK;
      #endif
      send_ble_data = 0;
  }

  //HAL_Delay(300); // to avoid to many measurement in one second


  MX_I2C1_Init();
  Init_Switch();
  Switch_Set();
  Switch_Reset();
  // reinit the I2C pins in PullDown
  HAL_I2C_DeInit(&hi2c1);
  PullDown_I2C_A10(); //EDIT
  PullDown_I2C_A9();  //EDIT

  // go to sleep mode
  MX_LPUART1_UART_Init(); // car low power
  HAL_Delay(50);
  BLE_WakeUp(&hlpuart1, my_ret_ble);
  HAL_Delay(100);
  if(BLE_GoToSleep(&hlpuart1, my_ret_ble) != HAL_OK)
  {
  	//Error_Handler();
  }


#endif



}

/****************************************************************
 * Function Name: Store_Uplink
 * Description: Store uplink message in the EEPROM
 * Returns: Hal status of the I2C transmission
 * Params
			@my_ret: Hal status of the I2C transmission
			@ul_msg: string (uplink message) to store in EEPROM
			@ul_size: size of the uplink message
 ****************************************************************/
HAL_StatusTypeDef Store_Uplink (HAL_StatusTypeDef my_ret_eeprom, uint8_t * ul_msg, uint8_t ul_size)
{

	  //store the message
	  uint8_t read_current_address[2];
	  read_current_address[0]=0;
	  read_current_address[1]=0;
	  my_ret_eeprom = HAL_OK;
	  my_ret_eeprom = EEPROM_read(my_ret_eeprom, 0x0C, read_current_address, sizeof(read_current_address)/sizeof(read_current_address[0]));
	  uint16_t current_address = (read_current_address[1]<<8) + read_current_address[0];
	  if(my_ret_eeprom != HAL_OK)
	  {
		  //My_Error_Handler_EEPROM();
	  }

	  #if defined(TEST_MODE) && (TEST_MODE == 1)
	  // TEST ONLY///////////////////////////////
	  char my_string[2];
	  itoa(current_address,my_string,2);
	  APP_PPRINTF("current adress");
	  APP_PPRINTF(my_string);
	  APP_PPRINTF("\r\n\n\r");
	  #endif /* TEST_MODE */

	  if (current_address<0x0F)
	  {
		  current_address = 0x0F; //0x0F
	  }

	  if(EEPROM_write(my_ret_eeprom, current_address, ul_msg, ul_size) != HAL_OK)
	  {
		  if(EEPROM_write(my_ret_eeprom, current_address, ul_msg, ul_size) != HAL_OK)
		  { // reinit I2C and wait
			  Switch_Reset();
			  MX_I2C1_Init();
			  Init_Switch();
			  Switch_Set();
			  HAL_Delay(50);
			  if(EEPROM_write(my_ret_eeprom, current_address, ul_msg, ul_size) != HAL_OK)
			  {
				  // reinit I2C and wait
				  Switch_Reset();
				  MX_I2C1_Init();
				  Init_Switch();
				  Switch_Set();
				  HAL_Delay(50);
				  #if defined(TEST_MODE) && (TEST_MODE == 1)
				  APP_PPRINTF("error current address: address+9");
				  APP_PPRINTF("\r\n\n\r");
				  #endif /* TEST_MODE */

				  if(EEPROM_write(my_ret_eeprom, current_address+9, ul_msg, ul_size) != HAL_OK)
				  {
					  //My_Error_Handler_EEPROM();
				  }
			  }

		  }
	  }

	  Switch_Reset();
	  MX_I2C1_Init();
	  Init_Switch();
	  Switch_Set();

	  uint8_t write_next_address[2];
	  uint16_t next_address = current_address + ul_size; //9 = size of ul_msg
	  write_next_address[0]= next_address&0x00FF;
	  write_next_address[1]= ((next_address >> 8) & 0x00FF);

	  #if defined(TEST_MODE) && (TEST_MODE == 1)
	  // TEST ONLY/////
	  itoa(next_address,my_string,2);
	  APP_PPRINTF("next adress");
	  APP_PPRINTF(my_string);
	  APP_PPRINTF("\r\n\n\r");
	  #endif /* TEST_MODE */

	  if(EEPROM_write(my_ret_eeprom, 0x0C, write_next_address, sizeof(write_next_address)/sizeof(write_next_address[0])) != HAL_OK)
	  {
		  if(EEPROM_write(my_ret_eeprom, 0x0C, write_next_address, sizeof(write_next_address)/sizeof(write_next_address[0])) != HAL_OK)
		  {
			  //My_Error_Handler_EEPROM();
		  }
	  }

	  return my_ret_eeprom;
}



