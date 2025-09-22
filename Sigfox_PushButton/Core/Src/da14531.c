/*
 * @file   da14531.c
 * @brief  da14531 BLE module Driver
 * @author Louis Labrot

	
  Reference: 
 */

/*
 *====================
 * Includes
 *====================
 */
#include "da14531.h"
#include <string.h>
#include "stm32wlxx_hal.h" //HAL_STATUS
#include "sys_app.h" // for APP_PPRINTF
#include "main.h"


#define RAW_TO_INT4A(a,b,c,d) 0x##d, 0x##c, 0x##b, 0x##a
#define FORMAT_ID(...) RAW_TO_INT4A(__VA_ARGS__)

#if defined(SIGFOX_DOWNLINK)
#include "sigfox_data.h"
uint8_t my_sigfox_ID[] = { FORMAT_ID(SIGFOX_ID) };
#elif defined(LORA_DOWNLINK)
#include "se-identity.h"
uint8_t my_Lora_ID[]= LORAWAN_DEVICE_EUI;
#endif /* Sigfox_ID */


uint8_t aATMEM1[]="AT+MEM=1,112233445566778899\r";

/****************************************************************
 * Function Name: BLE_init
 * Description: Init the BLE module
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
 ****************************************************************/
HAL_StatusTypeDef  BLE_Init(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble)
{
	#if defined(PIN_BLE) && (PIN_BLE == 11)
	uint8_t aATIOCFG[]="AT+IOCFG=11,4\r"; // pin BLE : 11
	uint8_t aATHNDL[]="AT+HNDL=2,AT+IO=11,1; AT+IO=11,0\r"; // pin BLE : 11
	uint8_t aATCMDSTORE[]="AT+CMDSTORE=0,AT+MEM=2,CA11;AT+IO=11,1;AT+IO=11,0\r";
	#elif defined(PIN_BLE) && (PIN_BLE == 9)
	uint8_t aATIOCFG[]="AT+IOCFG=9,4\r"; // pin BLE : 9
	uint8_t aATHNDL[]="AT+HNDL=2,AT+IO=9,1; AT+IO=9,0;AT+SLEEP=1\r"; // pin BLE : 9
	uint8_t aATCMDSTORE[]="AT+CMDSTORE=0,AT+MEM=2,CA11;AT+IO=9,1;AT+IO=9,0\r";
	#elif !defined (PIN_BLE)
	#error PIN_BLE defined not correctly
	#endif /* DA14531_PIN */
	//uint8_t aATCMDSTORE2[]="AT+CMDSTORE=1,AT+SLEEP=1\r";

	uint8_t aATE[]="ATE=0\r"; //local echo turn off
	uint8_t aTxBuffer21[]="AT+EVENT=1,0\r";
	uint8_t aTxBuffer22[]="AT+EVENT=2,0\r";
	uint8_t aTxBuffer24[]="AT+EVENT=4,0\r";
	//uint8_t aATHNDL4[]="AT+HNDL=4,AT+IOCFG=9,4;AT+IO=9,1\r";
	uint8_t aATSLEEP[]="AT+SLEEP=1\r";
	uint8_t aATSEC[]="AT+SEC=0\r";
	// 0 = LE secure connections pairing: In this case cryptography will be used along with the Diffie - Hellman public key exchange mechanism.
	// The passkey entry pairing method will be used for MITM protection.
	// The LTK will be stored along with other parameters in the bonding database if the bonding database is available.
	// The device will print the six-digit pin (by default 000000) which can also be set in advance using the AT+PIN command.
	uint8_t aATPIN[]="AT+PIN=654321\r";

	uint8_t aATF[]="ATF\r"; // Turns error reporting on/off

	uint8_t aATPWRLVL[]="AT+PWRLVL=1\r"; // 9 by default : 0dBm // 1 -> -19.5dBm

	uint8_t aATCONPAR[]="AT+CONPAR=12,0,2500,1\r"; //12 by default

	uint8_t aATR[]="ATR\r"; // Triggers a platform reset

	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATR, (COUNTOF(aATR) - 1),1000);

	HAL_Delay(300);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATR, (COUNTOF(aATR) - 1),1000);

	HAL_Delay(300);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATIOCFG, (COUNTOF(aATIOCFG) - 1),1000); // first command twice

	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATE, (COUNTOF(aATE) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATE, (COUNTOF(aATE) - 1),1000);
	}
	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATE, (COUNTOF(aATF) - 1),1000);
	HAL_Delay(100);

	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer21, EVENT_DISABLE_SIZE,1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer22, EVENT_DISABLE_SIZE,1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer24, EVENT_DISABLE_SIZE,1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATPIN, (COUNTOF(aATPIN) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATSEC, (COUNTOF(aATSEC) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATPWRLVL, (COUNTOF(aATPWRLVL) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATCONPAR, (COUNTOF(aATCONPAR) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);

	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATCMDSTORE, (COUNTOF(aATCMDSTORE) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATHNDL, (COUNTOF(aATHNDL) - 1),1000);

	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);

    #if defined(SIGFOX_ID)
	// Change Name Sigfox
	my_ret_ble = BLE_ChangeName_Sigfox(puart, my_ret_ble);
	#elif defined(LORAWAN_DEVICE_EUI)
	my_ret_ble = BLE_ChangeName_Lorawan(puart, my_ret_ble);
	#endif /* Sigfox_ID */
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(100);

	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATSLEEP, (COUNTOF(aATSLEEP) - 1),1000);

	return my_ret_ble;

}



/****************************************************************
 * Function Name: BLE_GoToSleep
 * Description: Send AT+SLEEP=1 to the BLE module
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
			@pble_pin: string of the pin number used for interrupt (on the DA14531)
 ****************************************************************/
HAL_StatusTypeDef  BLE_GoToSleep(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble)
{
	uint8_t aTxBuffer8[]="AT+SLEEP=1\r"; // go to sleep
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer8, (COUNTOF(aTxBuffer8) - 1),1000);

	return my_ret_ble;
}

/****************************************************************
 * Function Name: BLE_WakeUp
 * Description: Send AT+SLEEP=0 to the BLE module
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
 ****************************************************************/
HAL_StatusTypeDef  BLE_WakeUp(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble)
{
	uint8_t aTxBuffer7[]="AT+SLEEP=0\r"; //disable sleep

	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer7, (COUNTOF(aTxBuffer7) - 1),1000);
	HAL_Delay(10);
	return my_ret_ble;
}


/****************************************************************
 * Function Name: BLE_ReadMEM2
 * Description: Read MEM2 to the BLE module
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
			@pData: string to store the values read
			@Size: Size of pData
 ****************************************************************/
HAL_StatusTypeDef  BLE_ReadMEM2(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble, uint8_t *pData, uint16_t Size)
{
	  uint8_t aTxBuffer9[]="AT+MEM=2\r";
	  // Access MEM2
	  // receive MEM2
	  pData[2]=0;
	  pData[3]=0;
	  pData[4]=0;
	  pData[5]=0;

	  my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer9, (COUNTOF(aTxBuffer9) - 1),1000);
	  if(my_ret_ble != HAL_OK)
	  {
	      return my_ret_ble;
	  }

	  my_ret_ble = HAL_UART_Receive(puart, pData, Size, 1000);

	return my_ret_ble;
}


/****************************************************************
 * Function Name: BLE_ReadMEM3
 * Description: Read MEM3 to the BLE module
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
			@pData: string to store the values read
			@Size: Size of pData
 ****************************************************************/
HAL_StatusTypeDef  BLE_ReadMEM3(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble, uint8_t *pData, uint16_t Size)
{
	uint8_t aTxBuffer1[]="AT+MEM=3\r";

	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer1, (COUNTOF(aTxBuffer1) - 1),1000);

	pData[2]=0;
	pData[3]=0;
	pData[4]=0;
	pData[5]=0;
	pData[6]=0;
	pData[7]=0;
	pData[8]=0;
	pData[9]=0;
	pData[9]=0;
	pData[10]=0;
	pData[11]=0;
	pData[12]=0;
	pData[13]=0;
	pData[14]=0;
	pData[15]=0;
	pData[16]=0;

	// xxxxCBxxxxxxxxCB
	my_ret_ble = HAL_UART_Receive(puart, pData, Size, 1000);

	return my_ret_ble;
}


/****************************************************************
 * Function Name: BLE_ResetMEM2
 * Description: Reset MEM2 (BLE module)
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
 ****************************************************************/
HAL_StatusTypeDef  BLE_ResetMEM2(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble)
{
	uint8_t aTxBuffer10[]="AT+MEM=2,0000\r";
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer10, (COUNTOF(aTxBuffer10) - 1),1000);

	return my_ret_ble;
}


/****************************************************************
 * Function Name: BLE_ResetMEM3
 * Description: Reset MEM3 (BLE module)
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
 ****************************************************************/
HAL_StatusTypeDef  BLE_ResetMEM3(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble)
{
	uint8_t aTxBuffer10[]="AT+MEM=3,\r";
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aTxBuffer10, (COUNTOF(aTxBuffer10) - 1),1000);

	return my_ret_ble;
}

/****************************************************************
 * Function Name: BLE_ConvertID
 * Description: Convert hexa in ASCII
 * Params	@my_id: hexa id to convert
 ****************************************************************/
uint8_t  BLE_Convert_ID(uint8_t my_id)
{
	uint8_t result_ascii = 0;
	if (my_id>9) // A or B or C or D
	{
		result_ascii = my_id-10+0x31;
	}
	else
	{
		result_ascii = my_id+0x30;
	}

	return result_ascii;
}

/****************************************************************
 * Function Name: BLE_ID_figure_letter
 * Description: return 0x33 if the id if a fugre (0 or 1... or 9) and 0x34 if it's a letter (A or B, or C...)
 * Params	@my_id: hexa id to convert
 ****************************************************************/
uint8_t  BLE_ID_figure_letter(uint8_t my_id)
{
	uint8_t result_ascii = 0;
	if (my_id>9)
	{
		result_ascii = 0x34;
	}
	else
	{
		result_ascii = 0x33;
	}

	return result_ascii;
}

/****************************************************************
 * Function Name: Convert_HexaAscii
 * Description: Convert hexa in ASCII
 * Params	@my_id: hexa id to convert
 ****************************************************************/
uint8_t  BLE_Convert_HexaAscii(uint8_t byte)
{
	uint8_t result_ascii = 0;
	if ((byte>9) && (byte<16)) // A or B or C or D or E or F
	{
		result_ascii = byte+0x41-10;
	}
	else
	{
		result_ascii = byte+0x30;
	}

	return result_ascii;
}


#if defined(SIGFOX_ID)


/****************************************************************
 * Function Name: BLE_ChangeName_Sigfox
 * Description: Change name (Adverti. Data and resp) of the BLE module with Sigfox ID
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
 ****************************************************************/
HAL_StatusTypeDef  BLE_ChangeName_Sigfox(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble)
{
	uint8_t aATADVRESP[]="AT+ADVRESP=0C:09:4E:4F:58:30:30:30:30:30:30:30:30\r"; // 00000000 //+9caractères
	uint8_t aATADVDATA[]="AT+ADVDATA=11:07:B7:5C:49:D2:04:A3:40:71:A0:B5:35:85:3E:B0:83:07:03:02:F5:FE:05:09:41:41:41:41\r"; // name = AAAA

	uint8_t aATADVSTOP[]="AT+ADVSTOP\r";

	uint8_t aATADVSTART[]="AT+ADVSTART\r";

	// modification of the ADVDATA
	aATADVDATA[83]=BLE_ID_figure_letter((my_sigfox_ID[1]&0xF0)>>4); // first character
	aATADVDATA[84]=BLE_Convert_ID((my_sigfox_ID[1]&0xF0)>>4);//

	aATADVDATA[86]=BLE_ID_figure_letter(my_sigfox_ID[1]&0x0F); // 0x34 = 4
	aATADVDATA[87]=BLE_Convert_ID(my_sigfox_ID[1]&0x0F); // 0x31 = 1

	aATADVDATA[89]=BLE_ID_figure_letter((my_sigfox_ID[0]&0xF0)>>4); // 0x33 = 3
	aATADVDATA[90]=BLE_Convert_ID((my_sigfox_ID[0]&0xF0)>>4); // 0x37 = 7

	aATADVDATA[92]=BLE_ID_figure_letter(my_sigfox_ID[0]&0x0F); // 0x33 = 3
	aATADVDATA[93]=BLE_Convert_ID(my_sigfox_ID[0]&0x0F); // 93 last character // 0x32 = 2


	// modification of the ADVRESP

	aATADVRESP[26]=BLE_ID_figure_letter((my_sigfox_ID[3]&0xF0)>>4);
	aATADVRESP[27]=BLE_Convert_ID((my_sigfox_ID[3]&0xF0)>>4);

	aATADVRESP[29]=BLE_ID_figure_letter(my_sigfox_ID[3]&0x0F);
	aATADVRESP[30]=BLE_Convert_ID(my_sigfox_ID[3]&0x0F);

	aATADVRESP[32]=BLE_ID_figure_letter((my_sigfox_ID[2]&0xF0)>>4);
	aATADVRESP[33]=BLE_Convert_ID((my_sigfox_ID[2]&0xF0)>>4);

	aATADVRESP[35]=BLE_ID_figure_letter(my_sigfox_ID[2]&0x0F);
	aATADVRESP[36]=BLE_Convert_ID(my_sigfox_ID[2]&0x0F);

	aATADVRESP[38]=BLE_ID_figure_letter((my_sigfox_ID[1]&0xF0)>>4);
	aATADVRESP[39]=BLE_Convert_ID((my_sigfox_ID[1]&0xF0)>>4);

	aATADVRESP[41]=BLE_ID_figure_letter(my_sigfox_ID[1]&0x0F);
	aATADVRESP[42]=BLE_Convert_ID(my_sigfox_ID[1]&0x0F);

	aATADVRESP[44]=BLE_ID_figure_letter((my_sigfox_ID[0]&0xF0)>>4);
	aATADVRESP[45]=BLE_Convert_ID((my_sigfox_ID[0]&0xF0)>>4);

	aATADVRESP[47]=BLE_ID_figure_letter(my_sigfox_ID[0]&0x0F);
	aATADVRESP[48]=BLE_Convert_ID(my_sigfox_ID[0]&0x0F);


	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATADVRESP, (COUNTOF(aATADVRESP) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATADVDATA, (COUNTOF(aATADVDATA) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATADVSTOP, (COUNTOF(aATADVSTOP) - 1),1000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATADVSTART, (COUNTOF(aATADVSTART) - 1),1000);


	return my_ret_ble;
	}

#elif !defined (SIGFOX_ID)
/****************************************************************
 * Function Name: BLE_ChangeName_Lorawan
 * Description: Change name (Adverti. Data and resp) of the BLE module with Dev_EUI
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
 ****************************************************************/
HAL_StatusTypeDef  BLE_ChangeName_Lorawan(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble)
{
	uint8_t aATADVRESP[]="AT+ADVRESP=14:09:4E:4F:58:30:30:30:30:30:30:30:30:30:30:30:30:30:30:30:30\r"; // 0000000000000000 //+9caractères
	uint8_t aATADVDATA[]="AT+ADVDATA=11:07:B7:5C:49:D2:04:A3:40:71:A0:B5:35:85:3E:B0:83:07:03:02:F5:FE:05:09:41:41:41:41\r"; // name = AAAA

	uint8_t aATADVSTOP[]="AT+ADVSTOP\r";

	uint8_t aATADVSTART[]="AT+ADVSTART\r";

	// modification of the ADVDATA
	aATADVDATA[83]=BLE_ID_figure_letter((my_Lora_ID[6]&0xF0)>>4); // first character
	aATADVDATA[84]=BLE_Convert_ID((my_Lora_ID[6]&0xF0)>>4);//

	aATADVDATA[86]=BLE_ID_figure_letter(my_Lora_ID[6]&0x0F); // 0x34 = 4
	aATADVDATA[87]=BLE_Convert_ID(my_Lora_ID[6]&0x0F); // 0x31 = 1

	aATADVDATA[89]=BLE_ID_figure_letter((my_Lora_ID[7]&0xF0)>>4); // 0x33 = 3
	aATADVDATA[90]=BLE_Convert_ID((my_Lora_ID[7]&0xF0)>>4); // 0x37 = 7

	aATADVDATA[92]=BLE_ID_figure_letter(my_Lora_ID[7]&0x0F); // 0x33 = 3
	aATADVDATA[93]=BLE_Convert_ID(my_Lora_ID[7]&0x0F); // 93 last character // 0x32 = 2


	// modification of the ADVRESP

	aATADVRESP[26]=BLE_ID_figure_letter((my_Lora_ID[0]&0xF0)>>4);
	aATADVRESP[27]=BLE_Convert_ID((my_Lora_ID[0]&0xF0)>>4);

	aATADVRESP[29]=BLE_ID_figure_letter(my_Lora_ID[0]&0x0F);
	aATADVRESP[30]=BLE_Convert_ID(my_Lora_ID[0]&0x0F);

	aATADVRESP[32]=BLE_ID_figure_letter((my_Lora_ID[1]&0xF0)>>4);
	aATADVRESP[33]=BLE_Convert_ID((my_Lora_ID[1]&0xF0)>>4);

	aATADVRESP[35]=BLE_ID_figure_letter(my_Lora_ID[1]&0x0F);
	aATADVRESP[36]=BLE_Convert_ID(my_Lora_ID[1]&0x0F);

	aATADVRESP[38]=BLE_ID_figure_letter((my_Lora_ID[2]&0xF0)>>4);
	aATADVRESP[39]=BLE_Convert_ID((my_Lora_ID[2]&0xF0)>>4);

	aATADVRESP[41]=BLE_ID_figure_letter(my_Lora_ID[2]&0x0F);
	aATADVRESP[42]=BLE_Convert_ID(my_Lora_ID[2]&0x0F);

	aATADVRESP[44]=BLE_ID_figure_letter((my_Lora_ID[3]&0xF0)>>4);
	aATADVRESP[45]=BLE_Convert_ID((my_Lora_ID[3]&0xF0)>>4);

	aATADVRESP[47]=BLE_ID_figure_letter(my_Lora_ID[3]&0x0F);
	aATADVRESP[48]=BLE_Convert_ID(my_Lora_ID[3]&0x0F);

	aATADVRESP[50]=BLE_ID_figure_letter((my_Lora_ID[4]&0xF0)>>4);
	aATADVRESP[51]=BLE_Convert_ID((my_Lora_ID[4]&0xF0)>>4);

	aATADVRESP[53]=BLE_ID_figure_letter(my_Lora_ID[4]&0x0F);
	aATADVRESP[54]=BLE_Convert_ID(my_Lora_ID[4]&0x0F);

	aATADVRESP[56]=BLE_ID_figure_letter((my_Lora_ID[5]&0xF0)>>4);
	aATADVRESP[57]=BLE_Convert_ID((my_Lora_ID[5]&0xF0)>>4);

	aATADVRESP[59]=BLE_ID_figure_letter(my_Lora_ID[5]&0x0F);
	aATADVRESP[60]=BLE_Convert_ID(my_Lora_ID[5]&0x0F);

	aATADVRESP[62]=BLE_ID_figure_letter((my_Lora_ID[6]&0xF0)>>4);
	aATADVRESP[63]=BLE_Convert_ID((my_Lora_ID[6]&0xF0)>>4);

	aATADVRESP[65]=BLE_ID_figure_letter(my_Lora_ID[6]&0x0F);
	aATADVRESP[66]=BLE_Convert_ID(my_Lora_ID[6]&0x0F);

	aATADVRESP[68]=BLE_ID_figure_letter((my_Lora_ID[7]&0xF0)>>4);
	aATADVRESP[69]=BLE_Convert_ID((my_Lora_ID[7]&0xF0)>>4);

	aATADVRESP[71]=BLE_ID_figure_letter(my_Lora_ID[7]&0x0F);
	aATADVRESP[72]=BLE_Convert_ID(my_Lora_ID[7]&0x0F);

	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATADVRESP, (COUNTOF(aATADVRESP) - 1),5000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(500);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATADVDATA, (COUNTOF(aATADVDATA) - 1),5000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(500);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATADVSTOP, (COUNTOF(aATADVSTOP) - 1),5000);
	if(my_ret_ble != HAL_OK)
	{
		return my_ret_ble;
	}
	HAL_Delay(500);
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATADVSTART, (COUNTOF(aATADVSTART) - 1),5000);
	HAL_Delay(500);

	return my_ret_ble;
	}

#endif /* Sigfox_ID */

/****************************************************************
 * Function Name: BLE_WriteData
 * Description: Write Data in MEM1
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
 ****************************************************************/
HAL_StatusTypeDef  BLE_WriteData(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble)
{
	my_ret_ble = HAL_UART_Transmit(puart,(uint8_t*)aATMEM1, (COUNTOF(aATMEM1) - 1),1000);

	return my_ret_ble;
}

/****************************************************************
 * Function Name: BLE_Convert_Data
 * Description: Convert uint8 data measurement in ASCII format
 * Returns: Hal status of the UART transmission
 * Params	@puart: pointer uart
			@my_ret_ble: Hal status of the UART transmission
			@byte1: first byte
			@byte2: second byte
			@byte3: third byte
			@byte4: fourth byte
			@byte5: fifth byte
			@byte6: ... byte
			@byte7: ... byte
			@byte8: ... byte
			@byte9: last byte
 ****************************************************************/
HAL_StatusTypeDef  BLE_Convert_Data(UART_HandleTypeDef * puart, HAL_StatusTypeDef my_ret_ble, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7, uint8_t byte8, uint8_t byte9)
{
	// AT+MEM
	aATMEM1[9]=BLE_Convert_HexaAscii((byte1&0xF0)>>4);
	aATMEM1[10]=BLE_Convert_HexaAscii(byte1&0x0F);
	aATMEM1[11]=BLE_Convert_HexaAscii((byte2&0xF0)>>4);
	aATMEM1[12]=BLE_Convert_HexaAscii(byte2&0x0F);
	aATMEM1[13]=BLE_Convert_HexaAscii((byte3&0xF0)>>4);
	aATMEM1[14]=BLE_Convert_HexaAscii(byte3&0x0F);
	aATMEM1[15]=BLE_Convert_HexaAscii((byte4&0xF0)>>4);
	aATMEM1[16]=BLE_Convert_HexaAscii(byte4&0x0F);
	aATMEM1[17]=BLE_Convert_HexaAscii((byte5&0xF0)>>4);
	aATMEM1[18]=BLE_Convert_HexaAscii(byte5&0x0F);
	aATMEM1[19]=BLE_Convert_HexaAscii((byte6&0xF0)>>4);
	aATMEM1[20]=BLE_Convert_HexaAscii(byte6&0x0F);
	aATMEM1[21]=BLE_Convert_HexaAscii((byte7&0xF0)>>4);
	aATMEM1[22]=BLE_Convert_HexaAscii(byte7&0x0F);
	aATMEM1[23]=BLE_Convert_HexaAscii((byte8&0xF0)>>4);
	aATMEM1[24]=BLE_Convert_HexaAscii(byte8&0x0F);
	aATMEM1[25]=BLE_Convert_HexaAscii((byte9&0xF0)>>4);
	aATMEM1[26]=BLE_Convert_HexaAscii(byte9&0x0F);
	return my_ret_ble;
	}
