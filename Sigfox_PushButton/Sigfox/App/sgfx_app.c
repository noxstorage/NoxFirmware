/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sgfx_app.c
  * @author  MCD Application Team
  * @brief   provides code for the application of the sigfox Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "st_sigfox_api.h"
#include "sgfx_app.h"
#include "sgfx_app_version.h"
#include "sigfox_version.h"
#include "subghz_phy_version.h"
#include "radio.h"
#include "sys_conf.h"
#include "sgfx_eeprom_if.h"
#include "sys_app.h"
#include "stm32_lpm.h"
#include "stm32_seq.h"
#include "utilities_def.h"

/* USER CODE BEGIN Includes */
#include "sys_sensors.h"
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

#include "lpwan_app.h"
#include "main.h"

/* USER CODE END Includes */

/* Exported variables ---------------------------------------------------------*/

/* USER CODE BEGIN ExpV */
extern uint32_t nb_wakeup;
/* USER CODE END ExpV */

/* External variables ---------------------------------------------------------*/
extern RadioEvents_t RfApiRadioEvents;
/* USER CODE BEGIN EV */

#if defined(SIGFOX_DOWNLINK)


////////////////////////////////////////////////////////////////
// EEPROM and parameters
uint64_t delay_timer_acc = DELAYACC_DEFAULT;
uint64_t delay_timer = DELAYSENSOR_DEFAULT;
uint8_t preceive_command[8];
GPIO_InitTypeDef GPIO_InitStruct; // for GPIO reinitilization

//float Norm_g=0;
uint8_t pREAD_FIFO[32]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t MC3635_sensibility = SENSIBILITY_DEFAULT;
uint8_t MC3635_sensibility_msg = SENSIBILITY_DEFAULT; //bump_threshold_2

// buffer uart
uint8_t aRxBuffer1[20]; //16+4
uint8_t aRxBuffer2[8];

// BLE data
uint8_t send_ble_data = 0;
#endif

#if defined(SIGFOX_DOWNLINK)
//Sigfox downlink
sfx_bool Sigfox_Downlink = SIGFOX_DOWNLINK;
sfx_bool Sigfox_Downlink_Init = SIGFOX_DOWNLINK;
#elif defined(LORA_DOWNLINK)
sfx_bool Sigfox_Downlink = SFX_FALSE;
#endif

/////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UTIL_TIMER_Object_t TxTimer; //edit
static UTIL_TIMER_Object_t TxTimer_Acc; //edit
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  It calls SIGFOX_API_open()
  * @param  Config
  * @retval Status
  */
static sfx_error_t st_sigfox_open(sfx_rc_enum_t sfx_rc);

/**
  * @brief  The example sends T, P and H data. To be changed by the user.
  * @param  None
  * @retval None
  */
void SendSigfox(void);

/* USER CODE BEGIN PFP */
static void SendSigfoxAcc(void); //edit

static void OnTxTimerEvent(void *context); //edit

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

void Sigfox_Init(void)
{
  sfx_rc_enum_t sfx_rc = SFX_RC1;
  sfx_error_t error = 0;
  /* USER CODE BEGIN Sigfox_Init_1 */

  /* Get Sigfox APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Get MW Sigfox info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_SIGFOX_VERSION:   V%X.%X.%X\r\n",
          (uint8_t)(SIGFOX_VERSION_MAIN),
          (uint8_t)(SIGFOX_VERSION_SUB1),
          (uint8_t)(SIGFOX_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* USER CODE END Sigfox_Init_1 */

  E2P_Write_Rc(DEFAULT_RC);
  sfx_rc = E2P_Read_Rc();

  /*Open Sigfox library */
  error = st_sigfox_open(sfx_rc);

  Radio.Init(&RfApiRadioEvents);
  /* USER CODE BEGIN Sigfox_Init_ErrorCheck */
  if (1U == E2P_Read_AtEcho())
  {
    if (error == SFX_ERR_NONE)
    {
      APP_PPRINTF("\r\n\n\rSIGFOX APPLICATION READY\n\r\n\r");
    }
    else
    {
      APP_PPRINTF("\r\n\n\rSIGFOX APPLICATION ERROR: 0x%04X\n\r\n\r", error);
    }
  }
  /* USER CODE END Sigfox_Init_ErrorCheck */
  (void)error;

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Pb), UTIL_SEQ_RFU, SendSigfox);

  /* USER CODE BEGIN Sigfox_Init_2 */
  /* USER CODE BEGIN Sigfox_Init_2 */
     LPWAN_init1();

     UTIL_TIMER_Create(&TxTimer_Acc,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL); //edit
     UTIL_TIMER_SetPeriod(&TxTimer_Acc, DELAYACC_FIRST); //edit
     UTIL_TIMER_Create(&TxTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL); //edit
     UTIL_TIMER_SetPeriod(&TxTimer, DELAYSENSOR_FIRST); //edit
     UTIL_TIMER_Start(&TxTimer); //edit
     UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SendOnData), UTIL_SEQ_RFU, SendSigfox); //edit
     UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SendOnTxAcceleration), UTIL_SEQ_RFU, SendSigfoxAcc); //edit
     UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_BLE), UTIL_SEQ_RFU, BLElink); //edit

   #if defined(MEM_EEPROM_WIRED)
     //clean one time
     MX_I2C1_Init();
     Init_Switch();
     Switch_Set();
       ///
     HAL_StatusTypeDef my_ret_eeprom = HAL_OK;
     my_ret_eeprom = EEPROM_erase(my_ret_eeprom);
     if(my_ret_eeprom != HAL_OK)
     {
   	  //My_Error_Handler_EEPROM();
     }
     HAL_Delay(500);
     Switch_Reset();

     // reinit the I2C pins in PullDown
     HAL_I2C_DeInit(&hi2c1);
     PullDown_I2C_A10(); //EDIT
     PullDown_I2C_A9();  //EDIT

   #endif
  /* USER CODE END Sigfox_Init_2 */
}
/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
static sfx_error_t st_sigfox_open(sfx_rc_enum_t sfx_rc)
{
  /* USER CODE BEGIN st_sigfox_open_1 */

  /* USER CODE END st_sigfox_open_1 */
  sfx_u32 config_words[3] = {0};

  E2P_Read_ConfigWords(sfx_rc, config_words);

  sfx_error_t error = SFX_ERR_NONE;

  /*record RCZ*/
  switch (sfx_rc)
  {
    case SFX_RC1:
    {
      sfx_rc_t SgfxRc = RC1;
      error = SIGFOX_API_open(&SgfxRc);

      break;
    }
    case SFX_RC2:
    {
      sfx_rc_t SgfxRc = RC2;

      error = SIGFOX_API_open(&SgfxRc);

      if (error == SFX_ERR_NONE)
      {
        error = SIGFOX_API_set_std_config(config_words, RC2_SET_STD_TIMER_ENABLE);
      }

      break;
    }
    case SFX_RC3A:
    {
      sfx_rc_t SgfxRc = RC3A;

      error = SIGFOX_API_open(&SgfxRc);

      if (error == SFX_ERR_NONE)
      {
        error = SIGFOX_API_set_std_config(config_words, NA);
      }
      break;
    }
    case SFX_RC3C:
    {
      sfx_rc_t SgfxRc = RC3C;

      error = SIGFOX_API_open(&SgfxRc);

      if (error == SFX_ERR_NONE)
      {
        error = SIGFOX_API_set_std_config(config_words, NA);
      }
      break;
    }
    case SFX_RC4:
    {
      sfx_rc_t SgfxRc = RC4;

      error = SIGFOX_API_open(&SgfxRc);

      if (error == SFX_ERR_NONE)
      {
        error = SIGFOX_API_set_std_config(config_words, RC4_SET_STD_TIMER_ENABLE);
      }
      break;
    }
    case SFX_RC5:
    {
      sfx_rc_t SgfxRc = RC5;

      error = SIGFOX_API_open(&SgfxRc);

      if (error == SFX_ERR_NONE)
      {
        error = SIGFOX_API_set_std_config(config_words, NA);
      }
      break;
    }
    case SFX_RC6:
    {
      sfx_rc_t SgfxRc = RC6;
      error = SIGFOX_API_open(&SgfxRc);
      break;
    }
    case SFX_RC7:
    {
      sfx_rc_t SgfxRc = RC7;
      error = SIGFOX_API_open(&SgfxRc);
      break;
    }
    /* USER CODE BEGIN st_sigfox_open_case */

    /* USER CODE END st_sigfox_open_case */
    default:
    {
      error = SFX_ERR_API_OPEN;
      break;
    }
  }

  return error;
  /* USER CODE BEGIN st_sigfox_open_2 */

  /* USER CODE END st_sigfox_open_2 */
}

/* USER CODE BEGIN PB_Callbacks */

/* USER CODE END PB_Callbacks */

void SendSigfox(void)
{
  /* USER CODE BEGIN SendSigfox */
// AFTER EACH REGENERATION REMOVE the static

		  uint8_t ul_msg[12] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11};
		  uint8_t dl_msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		  uint32_t  ul_size = 0;
		  float batteryLevel = 0;
		  uint32_t nbTxRepeatFlag = 1;

		  #if defined(TEST_MODE) && (TEST_MODE == 1)
		  //ONLY TEST ////////////////////////////////////////
		  APP_PPRINTF("\r\n\n\r");
		  APP_PPRINTF("SENSORS MEASUREMENT \r\n\n\r");
		  #endif /* TEST_MODE */


		  MX_I2C1_Init();
		  Init_Switch();
		  Switch_Set(); //EDIT

		//////////////////////////////////////////////
		///// SENSORS FUNCTIONS //////////////////////
		//////////////////////////////////////////////

		  //SHTC3 variable
		  float SHTC3_temperature=-100;
		  float SHTC3_humidity=0;
		  uint16_t raw_temperature_SHTC3=0;
		  uint16_t raw_humidity_SHTC3=0;
		  //MS3637 variable
		  float MS5637_pressure=1000;

		  //raw values for msg
		  int raw_pressure=0;
		  int raw_co2=0;
		  int raw_temperature=0;
		  int raw_humidity=0;

		  // I2C status
		  HAL_StatusTypeDef my_ret_shtc3 = HAL_OK;
		  // EEPROM Status
		  HAL_StatusTypeDef my_ret_eeprom = HAL_OK;

		  // Activate Vdd sensor and I2C(PIN 3 CN10)
		  Switch_Set();
		  MS5637_pressure = LPWAN_pressure();
		  raw_pressure= (uint16_t)(MS5637_pressure);


          #if defined(RH_T_WIRED)
          //SHTC3/////////////////////////////////////////
          my_ret_shtc3 = SHTC3_wakeup(my_ret_shtc3, &hi2c1);
          my_ret_shtc3 = SHTC3_measure(my_ret_shtc3, &hi2c1, &raw_temperature_SHTC3, &raw_humidity_SHTC3);
          my_ret_shtc3 = SHTC3_sleep(my_ret_shtc3, &hi2c1);
          if(my_ret_shtc3 !=HAL_OK)
          {
        	 my_ret_shtc3 = SHTC3_reset(my_ret_shtc3, &hi2c1);
  	         HAL_Delay(500); // to avoid to many measurements command in one second
  	         my_ret_shtc3 = SHTC3_wakeup(my_ret_shtc3, &hi2c1);
  	         my_ret_shtc3 = SHTC3_measure(my_ret_shtc3, &hi2c1, &raw_temperature_SHTC3, &raw_humidity_SHTC3);
  	         my_ret_shtc3 = SHTC3_sleep(my_ret_shtc3, &hi2c1);
  	         if(my_ret_shtc3 !=HAL_OK)
  	         {
  	        	 my_ret_shtc3 = SHTC3_reset(my_ret_shtc3, &hi2c1);
  		         //My_Error_Handler(); //not blocking
  	  	         HAL_Delay(500); // to avoid to many measurements command in one second
  	  	         my_ret_shtc3 = SHTC3_wakeup(my_ret_shtc3, &hi2c1);
  	  	         my_ret_shtc3 = SHTC3_measure(my_ret_shtc3, &hi2c1, &raw_temperature_SHTC3, &raw_humidity_SHTC3);
  	  	         my_ret_shtc3 = SHTC3_sleep(my_ret_shtc3, &hi2c1);
  	         }
          }
          if(my_ret_shtc3 == HAL_OK)
          {
        	 SHTC3_humidity = SHTC3_CalcHumidity(raw_humidity_SHTC3);
        	 SHTC3_temperature = SHTC3_CalcTemperature(raw_temperature_SHTC3);
          }
          else
          {
        	  SHTC3_temperature=-100;
          }

          #endif

		  raw_co2 = LPWAN_CO2(SHTC3_humidity, SHTC3_temperature, MS5637_pressure);

		#if defined(MEM_EEPROM_WIRED)
		  // Read sensibility: CD
		   EEPROM_read(my_ret_eeprom, 0, preceive_command, sizeof(preceive_command)/sizeof(preceive_command[0]));
		   if(preceive_command[2]==203)// CB 2 and 7
		   {   // third parameter
			   MC3635_sensibility = ((preceive_command[5])&0xF0)>>8; // wake-up
			   MC3635_sensibility_msg = (preceive_command[5])&0x0F;  // message
		   }
		   #if defined(TEST_MODE) && (TEST_MODE == 1)
		   char my_string[2];
		   itoa(MC3635_sensibility,my_string,2);
		   APP_PPRINTF("SENSIBILITY ACCELEROMETER \r\n\n\r");
		   APP_PPRINTF(my_string);
		   APP_PPRINTF("\r\n\n\r");
		   #endif /* TEST_MODE */
		#endif

		  LPWAN_Init_Acc();

		  // 6V Sense
		  batteryLevel = ((float)SYS_Get6VLevel())/2800*255; //voltage divider : 150kOhm and 100kOhm

		  #if !defined(O2_SENSOR) || (O2_SENSOR == 0)
		  if (my_ret_shtc3 == HAL_OK)
		  {
			  raw_temperature = (int)((SHTC3_temperature+20)*100); // +20 to have negative temperature up to -20°C
			  raw_humidity = (int)(SHTC3_humidity*100);
		  }
		  else
		  {
			  raw_temperature = 0xFFFE;
			  raw_humidity = 0xFFFE;
		  }

		  ul_msg[ul_size++] = 0;
		  ul_msg[ul_size++] = (uint8_t)((raw_temperature >> 8) & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)(raw_temperature & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)((raw_humidity >> 8) & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)(raw_humidity & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)((raw_pressure >> 8) & 0xFF);//(uint8_t)(temperature & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)(raw_pressure & 0xFF);//(uint8_t)((humidity >> 8) & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)(raw_co2);//(uint8_t)(humidity & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)(batteryLevel);

		  #elif defined(O2_SENSOR) && (O2_SENSOR == 1)
		  uint8_t raw_o2 = 240;
		  // 40 : 0.025 resolution
		  // add here the O2 measurement (with switch command)
		  if (my_ret_shtc3 == HAL_OK)
		  {
			  raw_temperature = (int)((SHTC3_temperature+20)*40); // +20 to have negative temperature up to -20°C
			  raw_humidity = (int)(SHTC3_humidity*40);
		  }
		  else
		  {
			  raw_temperature = 0xFFE;
			  raw_humidity = 0xFFE;

		  }

		  ul_msg[ul_size++] = 2;

		  ul_msg[ul_size++] = (uint8_t)((raw_temperature >> 4) & 0xFF);
		  ul_msg[ul_size++] = (uint8_t) (     ((raw_temperature & 0x0F)<<4)    |     ((raw_humidity >> 8) & 0x0F)    );
		  ul_msg[ul_size++] = (uint8_t)(raw_humidity & 0xFF);

		  ul_msg[ul_size++] = (uint8_t)(raw_o2);
		  ul_msg[ul_size++] = (uint8_t)((raw_pressure >> 8) & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)(raw_pressure & 0xFF);
		  ul_msg[ul_size++] = (uint8_t)(raw_co2);
		  ul_msg[ul_size++] = (uint8_t)(batteryLevel);
		  #endif /* O2_SENSOR */

		  Switch_Reset();
		#if defined(BLE_WIRED)
		  if((send_ble_data == 1) ||(send_ble_data == 2)) // there was a CA11 or CA00
		  {
		    HAL_StatusTypeDef my_ret_ble = HAL_OK;
		    MX_LPUART1_UART_Init(); // car low power
		    HAL_Delay(50);
		    BLE_WakeUp(&hlpuart1, my_ret_ble);
			if (send_ble_data == 1) //CA11
			{
				BLE_Convert_Data(&hlpuart1, my_ret_ble, ul_msg[0], ul_msg[1], ul_msg[2], ul_msg[3], ul_msg[4], ul_msg[5], ul_msg[6], ul_msg[7], ul_msg[8]);
				BLE_WriteData(&hlpuart1, my_ret_ble);
			}
			send_ble_data = 0;
			BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
		  }/*
		  else
		  {
			  // A ENLEVER A TERME
			  HAL_StatusTypeDef my_ret_ble = HAL_OK;
			  MX_LPUART1_UART_Init(); // car low power
			  HAL_Delay(50);
			  BLE_WakeUp(&hlpuart1, my_ret_ble);// A ENLEVER A TERME
			  HAL_Delay(100);
			  BLE_Init(&hlpuart1, my_ret_ble); // A ENLEVER A TERME
			  HAL_Delay(100);
			  //BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
		  }*/
		#endif

		  Init_LED1();
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		  SIGFOX_API_send_frame(ul_msg, ul_size, dl_msg, nbTxRepeatFlag, Sigfox_Downlink);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

		  if((dl_msg[0]==203)&&(dl_msg[5]==203)) //CB xx xx xx xx CB 00 00
		  {

			  dl_msg[7]=dl_msg[5];
			  dl_msg[6]=dl_msg[4];
			  dl_msg[5]=dl_msg[3];
			  dl_msg[4]=dl_msg[2];
			  dl_msg[3]=dl_msg[1];
			  dl_msg[2]=dl_msg[0];
			  dl_msg[1]=0;
			  dl_msg[0]=0;

			  // reinitilisation of I2C1 and pin I2C
			  MX_I2C1_Init();
			  Init_Switch();

			  #if defined(MEM_EEPROM_WIRED)
			  /// PIN 3 CN10 : wakeup I2C
			  Switch_Set();
			  HAL_Delay(100);
			  my_ret_eeprom = HAL_OK;
			  my_ret_eeprom = EEPROM_write(my_ret_eeprom, 0, dl_msg, sizeof(dl_msg)/sizeof(dl_msg[0]));
			  if(my_ret_eeprom !=HAL_OK)
			  {
			  	//My_Error_Handler_EEPROM();
			  }
			  Switch_Reset();
			  #endif
		      #if defined(TEST_MODE) && (TEST_MODE == 1)
			  // TEST ONLY///////////////////////////////
			  APP_PPRINTF("GOOD DOWNLINK");
			  APP_PPRINTF("\r\n\n\r");
			  #endif /* TEST_MODE */

		  }
		  else
		  {
			  if(Sigfox_Downlink==SFX_TRUE)
			  {
				#if defined(TEST_MODE) && (TEST_MODE == 1)
				// TEST ONLY///////////////////////////////
				APP_PPRINTF("BAD DOWNLINK");
				APP_PPRINTF("\r\n\n\r");
				   char my_string[2];
				   itoa((dl_msg[0]),my_string,2);
				   APP_PPRINTF(my_string);
				   APP_PPRINTF("\r\n\n\r");
				   itoa((dl_msg[1]),my_string,2);
				   APP_PPRINTF(my_string);
				   APP_PPRINTF("\r\n\n\r");
				   itoa((dl_msg[2]),my_string,2);
				   APP_PPRINTF(my_string);
				   APP_PPRINTF("\r\n\n\r");
				   itoa((dl_msg[3]),my_string,2);
				   APP_PPRINTF(my_string);
				   APP_PPRINTF("\r\n\n\r");
				   itoa((dl_msg[4]),my_string,2);
				   APP_PPRINTF(my_string);
				   APP_PPRINTF("\r\n\n\r");
				   itoa((dl_msg[5]),my_string,2);
				   APP_PPRINTF(my_string);
				   APP_PPRINTF("\r\n\n\r");
				   itoa((dl_msg[6]),my_string,2);
				   APP_PPRINTF(my_string);
				   APP_PPRINTF("\r\n\n\r");
				   itoa((dl_msg[7]),my_string,2);
				   APP_PPRINTF(my_string);
				   APP_PPRINTF("\r\n\n\r");
				#endif /* TEST_MODE */
			  }
			  else
			  {
				  //test ajout ces commandes BLE
				  //HAL_StatusTypeDef my_ret_ble = HAL_OK;
				  //MX_LPUART1_UART_Init(); // car low power
				  //HAL_Delay(50);
				  //BLE_WakeUp(&hlpuart1, my_ret_ble);
				  //HAL_Delay(100);
				  //BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
			  }
		  }

		  MX_I2C1_Init();
		  Init_Switch();
		  Switch_Set();
		  //store the message

		#if defined(MEM_EEPROM_WIRED)
		  Store_Uplink (my_ret_eeprom, ul_msg, 9);

		  #endif
		  Switch_Reset();

		  // reinit the I2C pins in PullDown
		  HAL_I2C_DeInit(&hi2c1);
		  PullDown_I2C_A10(); //EDIT
		  PullDown_I2C_A9();  //EDIT

		  //test remove ces commandes BLE : quand il y a un downlink après cela n'est pas utile
		  //HAL_StatusTypeDef my_ret_ble = HAL_OK;
		  //MX_LPUART1_UART_Init(); // car low power
		  //HAL_Delay(50);
		  //BLE_WakeUp(&hlpuart1, my_ret_ble);
		  //HAL_Delay(100);
		  //BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
  /* USER CODE END SendSigfox */
}

/* USER CODE BEGIN PrFD */


static void SendSigfoxAcc(void) //edit
{
  uint8_t ul_msg[12] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11};
  uint8_t dl_msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint32_t  ul_size = 0;
  float batteryLevel = 0;
  uint32_t nbTxRepeatFlag = 1;

  // EEPROM Status
  HAL_StatusTypeDef my_ret_eeprom = HAL_OK;

  // reinitilisation of I2C1 and pin I2C
  MX_I2C1_Init();
  Init_Switch();
  Switch_Set();

  // Read sensibility:
   EEPROM_read(my_ret_eeprom, 0, preceive_command, sizeof(preceive_command)/sizeof(preceive_command[0]));
   if(my_ret_eeprom != HAL_OK)
   { // on a a écrit dans la mémoire deja, il n'y a pas d'erreur normalement
	   HAL_Delay(100);
	   EEPROM_read(my_ret_eeprom, 0, preceive_command, sizeof(preceive_command)/sizeof(preceive_command[0]));
	   //Error_Handler();
   }
   if(preceive_command[2]==203)// CB 2 and 7
   {   // third parameter
	   MC3635_sensibility = ((preceive_command[5])&0xF0)>>8; // wake-up
	   MC3635_sensibility_msg = (preceive_command[5])&0x0F;  // message
   }
   else
   {
	   //Error_Handler();
   }
   //if(MC3635_sensibility>MC3635_sensibility_msg)
   //{ //wake-up threshold must be lower than message threshold
	   //My_Error_Handler(); //not blocking
   //}
   #if defined(TEST_MODE) && (TEST_MODE == 1)
   char my_string[2];
   itoa(MC3635_sensibility,my_string,2);
   APP_PPRINTF("SENSIBILITY ACCELEROMETER \r\n\n\r");
   APP_PPRINTF(my_string);
   APP_PPRINTF("\r\n\n\r");
   #endif /* TEST_MODE */

#if defined(SENSOR_ACC_WIRED)
  //// ACC FUNCTIONS///////////////
  HAL_StatusTypeDef my_ret = HAL_OK;
  //MC3635 accelerometer
  if ((MC3635_READ_CLEAR_int(my_ret, &hi2c1)&0b00000100)==0b00000100) // interrupt : SNIFF to CWAKE
  {
  	HAL_Delay(650); // wait 0.64s
  	my_ret=MC3635_READ_FIFO_sniffTOcwake(my_ret, &hi2c1, pREAD_FIFO);
  }
  // set standby mode to be able to send a command
  my_ret=MC3635_SET_mode(my_ret, &hi2c1, MC3635_MODE_STANDBY);
  my_ret=MC3635_SET_FIFO(my_ret, &hi2c1, MC3635_FIFO_MODE_NORMAL);
  //set sniff and interrupt
  my_ret=MC3635_SET_sniff_int(my_ret, &hi2c1, MC3635_SNIFF_SR_200Hz, MC3635_sensibility);
  // Sortir du mode STANDBY
  my_ret=MC3635_SET_mode(my_ret, &hi2c1, MC3635_MODE_SNIFF);
  if(my_ret !=HAL_OK)
  {
	HAL_Delay(100);
	my_ret = HAL_OK;
	my_ret=MC3635_SET_mode(my_ret, &hi2c1, MC3635_MODE_SNIFF);
	if(my_ret !=HAL_OK)
	{
		HAL_Delay(100);
		my_ret = HAL_OK;
		my_ret=MC3635_SET_mode(my_ret, &hi2c1, MC3635_MODE_SNIFF);
		if(my_ret !=HAL_OK)
		{
			//My_Error_Handler();
		}
	 }
  }
#endif
  ///////////////////////////////////
  uint8_t max_acc = 0;
  uint8_t min_acc = 255;
  uint8_t cpt_acc = 0;
  uint32_t sum_acc = 0;
  uint32_t j = 0;
  uint8_t cpt_threshold = 17; // 1.06g
  for(j=0;j<32;j++)
  {
	  if(pREAD_FIFO[j]<min_acc)
	  {
		  min_acc=pREAD_FIFO[j];
	  }
	  if(pREAD_FIFO[j]>max_acc)
	  {
		  max_acc=pREAD_FIFO[j];
	  }
	  if ((pREAD_FIFO[j]-16)>0)
	  {
		  sum_acc = sum_acc + (pREAD_FIFO[j]-16); //because 9.81/(78.456/128)=16.0048945
	  }
	  else
	  {
		  sum_acc = sum_acc + (16-pREAD_FIFO[j]); //because 9.81/(78.456/128)=16.0048945
	  }

	  if(pREAD_FIFO[j]>cpt_threshold)
	  {
		  cpt_acc=j;
	  }
  }


  batteryLevel = ((float)SYS_Get6VLevel())/2800*255; //voltage divider : 150kOhm and 100kOhm


  ul_msg[ul_size++] = 1; //message type: acceleration
  ul_msg[ul_size++] = max_acc;
  ul_msg[ul_size++] = min_acc;
  ul_msg[ul_size++] = cpt_acc;
  ul_msg[ul_size++] = (uint8_t)((sum_acc >> 24) & 0xFF);
  ul_msg[ul_size++] = (uint8_t)((sum_acc >> 16) & 0xFF);
  ul_msg[ul_size++] = (uint8_t)((sum_acc >> 8) & 0xFF);
  ul_msg[ul_size++] = (uint8_t)((sum_acc) & 0xFF);
  ul_msg[ul_size++] = (uint8_t)(batteryLevel);

#if defined(MEM_EEPROM_WIRED)
  //store the message
  Store_Uplink (my_ret_eeprom, ul_msg, 9);
  Switch_Reset();
#endif

  if((max_acc-min_acc)>(MC3635_sensibility_msg-1)) //msg sent only if the threshold is reached.
  {                                    // threshold between 0.6m/s2 and 3.6m/s2
	    Init_LED1();
	    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	    SIGFOX_API_send_frame(ul_msg, ul_size, dl_msg, nbTxRepeatFlag, SFX_FALSE);
	    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  }


  // reinit the I2C pins in PullDown
  HAL_I2C_DeInit(&hi2c1);
  PullDown_I2C_A10(); //EDIT
  PullDown_I2C_A9();  //EDIT

  //test comment
  //HAL_StatusTypeDef my_ret_ble = HAL_OK;
  //MX_LPUART1_UART_Init(); // car low power
  //HAL_Delay(50);
  //BLE_WakeUp(&hlpuart1, my_ret_ble);
  //HAL_Delay(100);
  //BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
}

static void OnTxTimerEvent(void *context) //edit
{

// reinitilisation of I2C1 and pin I2C
  MX_I2C1_Init();
  Init_Switch();

  // READ parameter
  Switch_Set();
  HAL_StatusTypeDef my_ret_eeprom = HAL_OK;
  EEPROM_read(my_ret_eeprom, 0, preceive_command, sizeof(preceive_command)/sizeof(preceive_command[0]));
  if(nb_wakeup==1)
  {
	  if (preceive_command[2]==203)// CB
	    {
	        delay_timer = (preceive_command[3])*DELAYSENSOR; //CC then X times DELAYSENSOR
	    }
	    else
	    {
	  	  delay_timer = DELAYSENSOR_DEFAULT;
	  #if defined(TEST_MODE) && (TEST_MODE == 1)
	  	  APP_PPRINTF("delay timer NOK");
	  #endif /* TEST_MODE */
	    }
	  #if defined(TEST_MODE) && (TEST_MODE == 1)
	  char my_string[2];
	  itoa(preceive_command[3],my_string,2);
	  APP_PPRINTF("DELAY SENSOR \r\n\n\r");
	  APP_PPRINTF(my_string);
	  APP_PPRINTF("\r\n\n\r");
	  #endif /* TEST_MODE */
	    UTIL_TIMER_SetPeriod(&TxTimer, delay_timer); //edit

	    Switch_Reset();

	    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SendOnData), CFG_SEQ_Prio_0);
	    /*Wait for next tx slot*/
	    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
	  nb_wakeup = nb_wakeup - 1 ;
	  if (preceive_command[2]==203) //test CB
	  {
	  	delay_timer_acc = (preceive_command[4])*DELAYACC; //CC then X times 40seconds
	  }
	  else
	  {
	  	delay_timer_acc = DELAYACC_DEFAULT; // defaut value
	  }

	  UTIL_TIMER_SetPeriod(&TxTimer_Acc, delay_timer_acc); //edit

	  Switch_Reset();

	  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SendOnData), CFG_SEQ_Prio_0);
	  UTIL_TIMER_Start(&TxTimer_Acc); //edit
  }
}




void Sigfox_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	#if defined(TEST_MODE) && (TEST_MODE == 1)
	APP_PPRINTF("start callback");
	APP_PPRINTF("\r\n\n\r");
 	 #elif !defined (TEST_MODE)
 	 #error TEST_MODE defined not correctly
 	 #endif /* TEST_MODE */

	// reinitilisation of I2C1 and pin I2C
	MX_I2C1_Init();
	Init_Switch();
	Switch_Set();
	#if defined(MEM_EEPROM_WIRED)

	HAL_StatusTypeDef my_ret_eeprom = HAL_OK;
	my_ret_eeprom = EEPROM_read(my_ret_eeprom, 0, preceive_command, sizeof(preceive_command)/sizeof(preceive_command[0]));
	if(my_ret_eeprom != HAL_OK)
	{
		//My_Error_Handler_EEPROM(); // without error to avoid blocking state
	}
	#endif


	#if defined(TEST_MODE) && (TEST_MODE == 1) && defined(MEM_EEPROM_WIRED)
	//ONLY TEST ////////////////////////////////////////
	uint8_t preceive_mydata[30]={0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
	my_ret_eeprom = EEPROM_read(my_ret_eeprom, 0, preceive_mydata, sizeof(preceive_mydata));
	if(my_ret_eeprom != HAL_OK)
	{
		//My_Error_Handler_EEPROM();
	}

	char my_string[2];
	APP_PPRINTF("DATA");
	APP_PPRINTF("\r\n\n\r");
	int i=0;
	for(i=0 ; i<26; i++)
	{
		itoa(preceive_mydata[i],my_string,2);
		APP_PPRINTF(my_string);
		APP_PPRINTF("\r\n\n\r");
	} /////////////////////////////////////////////////////
	#elif !defined (TEST_MODE)
	#error TEST_MODE defined not correctly
	#endif /* TEST_MODE */

	switch (GPIO_Pin)
	{
	case  1: //acceleration wakeup
		//read parameter to set the period of the timer and the nb of wakeup
			  	if (preceive_command[2]==203) //test CB
			  	{
			  		delay_timer_acc = (preceive_command[4])*DELAYACC; //CC then X times 40seconds
			  		nb_wakeup = preceive_command[6]&0b000011;
			  	}
			  	else
			  	{
			  		delay_timer_acc = DELAYACC_DEFAULT; // defaut value
			  		nb_wakeup = 1 ; // defaut value
			  	}
				#if defined(TEST_MODE) && (TEST_MODE == 1)
			  	char my_string2[2];
			  	itoa(preceive_command[4],my_string2,2);
			  	APP_PPRINTF("DELAY ACC ");
			  	APP_PPRINTF(my_string2);
			  	APP_PPRINTF("\r\n\n\r");
				#endif /* TEST_MODE */
			  	UTIL_TIMER_SetPeriod(&TxTimer_Acc, delay_timer_acc); //edit
			    ///
		   	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SendOnTxAcceleration), CFG_SEQ_Prio_0); // edit
		   	UTIL_TIMER_Start(&TxTimer_Acc); //edit
	  		break;
    	case  2:
    		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_BLE), CFG_SEQ_Prio_0);
    		break;

    	default:
    		break;
	}
}
/* USER CODE END PB_Callbacks */
/* USER CODE END PrFD */
