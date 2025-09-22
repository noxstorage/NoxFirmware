/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
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
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

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
#include "sys_conf.h"
/* USER CODE END Includes */


/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
#if !defined(SIGFOX_DOWNLINK)
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

//Downlink
uint8_t LPWAN_Downlink = LORA_DOWNLINK;

// BLE data
uint8_t send_ble_data = 0;
#endif
/////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * LEDs period value of the timer in ms
  */
#define LED_PERIOD_TIME 2000

/**
  * Join switch period value of the timer in ms
  */
#define JOIN_TIME 2000

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((uint32_t)0x0803F000UL)

/* USER CODE BEGIN PD */
static const char *slotStrings[] = { "1", "2", "C", "C_MC", "P", "P_MC" };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static void SendTxAcc(void);
static UTIL_TIMER_Object_t TxTimer_Acc; //edit
extern uint32_t nb_wakeup; //edit
/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief callback when LoRaWAN application has sent a frame
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRaWAN application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
  * @brief callback when LoRaWAN Beacon status is updated
  * @param params status of Last Beacon
  */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
  * @brief callback when LoRaWAN application Class is changed
  * @param deviceClass new class
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief  LoRa store context in Non Volatile Memory
  */
static void StoreContext(void);

/**
  * @brief  stop current LoRa execution to switch into non default Activation mode
  */
static void StopJoin(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
static void OnStopJoinTimerEvent(void *context);

/**
  * @brief  Notifies the upper layer that the NVM context has changed
  * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
  */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
  * @brief  Store the NVM Data context to the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were stored
  */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * @brief  Restore the NVM Data context from the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were restored
  */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * Will be called each time a Radio IRQ is handled by the MAC layer
  *
  */
static void OnMacProcessNotify(void);

/**
  * @brief Change the periodicity of the uplink frames
  * @param periodicity uplink frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
  * @brief Change the confirmation control of the uplink frames
  * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
  * @note Compliance test protocol callbacks
  */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
  * @brief Change the periodicity of the ping slot frames
  * @param pingSlotPeriodicity ping slot frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
  * @brief Will be called to reset the system
  * @note Compliance test protocol callbacks
  */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */

/**
  * @brief  LED Tx timer callback function
  * @param  context ptr of LED context
  */
static void OnTxTimerLedEvent(void *context);

/**
  * @brief  LED Rx timer callback function
  * @param  context ptr of LED context
  */
static void OnRxTimerLedEvent(void *context);

/**
  * @brief  LED Join timer callback function
  * @param  context ptr of LED context
  */
static void OnJoinTimerLedEvent(void *context);

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperature =               GetTemperatureLevel,
  .GetUniqueId =                  GetUniqueId,
  .GetDevAddr =                   GetDevAddr,
  .OnRestoreContextRequest =      OnRestoreContextRequest,
  .OnStoreContextRequest =        OnStoreContextRequest,
  .OnMacProcess =                 OnMacProcessNotify,
  .OnNvmDataChange =              OnNvmDataChange,
  .OnJoinRequest =                OnJoinRequest,
  .OnTxData =                     OnTxData,
  .OnRxData =                     OnRxData,
  .OnBeaconStatusChange =         OnBeaconStatusChange,
  .OnClassChange =                OnClassChange,
  .OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
  .OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
  .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
  .OnSystemReset =                OnSystemReset,
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
  .RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};

/**
  * @brief Type of Event to generate application Tx
  */
//static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief Tx Timer period
  */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
  * @brief Join Timer period
  */
static UTIL_TIMER_Object_t StopJoinTimer;

/* USER CODE BEGIN PV */
/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/**
  * @brief Specifies the state of the application LED
  */
//static uint8_t AppLedStateOn = RESET;

/**
  * @brief Timer to handle the application Tx Led to toggle
  */
static UTIL_TIMER_Object_t TxLedTimer;

/**
  * @brief Timer to handle the application Rx Led to toggle
  */
static UTIL_TIMER_Object_t RxLedTimer;

/**
  * @brief Timer to handle the application Join Led to toggle
  */
static UTIL_TIMER_Object_t JoinLedTimer;

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_LV */
  uint32_t feature_version = 0UL;
  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* Get LoRaWAN APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Get MW LoRaWAN info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION:  V%X.%X.%X\r\n",
          (uint8_t)(LORAWAN_VERSION_MAIN),
          (uint8_t)(LORAWAN_VERSION_SUB1),
          (uint8_t)(LORAWAN_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* Get LoRaWAN Link Layer info */
  LmHandlerGetVersion(LORAMAC_HANDLER_L2_VERSION, &feature_version);
  //APP_LOG(TS_OFF, VLEVEL_M, "L2_SPEC_VERSION:     V%X.%X.%X\r\n",(uint8_t)(feature_version >> 24),(uint8_t)(feature_version >> 16),(uint8_t)(feature_version >> 8));

  /* Get LoRaWAN Regional Parameters info */
  LmHandlerGetVersion(LORAMAC_HANDLER_REGION_VERSION, &feature_version);
  APP_LOG(TS_OFF, VLEVEL_M, "RP_SPEC_VERSION:     V%X-%X.%X.%X\r\n",
          (uint8_t)(feature_version >> 24),
          (uint8_t)(feature_version >> 16),
          (uint8_t)(feature_version >> 8),
          (uint8_t)(feature_version));

  UTIL_TIMER_Create(&TxLedTimer, LED_PERIOD_TIME, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&RxLedTimer, LED_PERIOD_TIME, UTIL_TIMER_ONESHOT, OnRxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&JoinLedTimer, LED_PERIOD_TIME, UTIL_TIMER_PERIODIC, OnJoinTimerLedEvent, NULL);

  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  UTIL_TIMER_Start(&JoinLedTimer);
  LmHandlerJoin(ActivationType, ForceRejoin);
#if defined(NOEXIST)
  /* USER CODE END LoRaWAN_Init_2 */

  LmHandlerJoin(ActivationType, ForceRejoin);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */
#endif

  	 LPWAN_init1();

     UTIL_TIMER_Create(&TxTimer_Acc,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL); //edit
     UTIL_TIMER_SetPeriod(&TxTimer_Acc, DELAYACC_FIRST); //edit
     UTIL_TIMER_Create(&TxTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL); //edit
     UTIL_TIMER_SetPeriod(&TxTimer, DELAYSENSOR_FIRST); //edit
     UTIL_TIMER_Start(&TxTimer); //edit
     UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SendOnData), UTIL_SEQ_RFU, SendTxData); //edit
     UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SendOnTxAcceleration), UTIL_SEQ_RFU, SendTxAcc); //edit
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

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

void Lora_GPIO_EXTI_Callback(int GPIO_Pin)
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
		//My_Error_Handler_EEPROM();// without error to avoid blocking state
	}

	delay_timer_acc = (preceive_command[5])*40000;
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
		   	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SendOnTxAcceleration), CFG_SEQ_Prio_0); // edit : ne bouge pas
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

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  uint8_t RxPort = 0;

  if (params != NULL)
  {
    //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); /* LED_BLUE */

    //UTIL_TIMER_Start(&RxLedTimer);

    if (params->IsMcpsIndication)
    {
      if (appData != NULL)
      {
        RxPort = appData->Port;
        if (appData->Buffer != NULL)
        {
            switch (appData->Port)
            {
              case LORAWAN_USER_APP_PORT:
                if (appData->BufferSize == 8)
                {
                  uint8_t dl_msg[8]={0,0,0,0,0,0,0,0};
                  dl_msg[0]=appData->Buffer[0];
                  dl_msg[1]=appData->Buffer[1];
                  dl_msg[2]=appData->Buffer[2];
                  dl_msg[3]=appData->Buffer[3];
                  dl_msg[4]=appData->Buffer[4];
                  dl_msg[5]=appData->Buffer[5];
                  dl_msg[6]=appData->Buffer[6];
                  dl_msg[7]=appData->Buffer[7];

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
                	  HAL_StatusTypeDef my_ret_eeprom = HAL_OK;
                	  my_ret_eeprom = EEPROM_write(my_ret_eeprom, 0, dl_msg, sizeof(dl_msg)/sizeof(dl_msg[0]));
                	  if(my_ret_eeprom !=HAL_OK)
                	  {
                	  	//My_Error_Handler_EEPROM();
                	  }

                	  #endif

        			  #if defined(TEST_MODE) && (TEST_MODE == 1)
                	  // TEST ONLY///////////////////////////////
                	  APP_PPRINTF("GOOD DOWNLINK");
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
                	  if(LPWAN_Downlink==1)
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
                  }
                }
                else
                {
        			#if defined(TEST_MODE) && (TEST_MODE == 1)
                	if(LPWAN_Downlink==1)
                    {
                		// TEST ONLY///////////////////////////////
                        APP_PPRINTF("BAD DOWNLINK : bad size");
                        APP_PPRINTF("\r\n\n\r");
                        char my_string[2];
                        itoa(appData->BufferSize,my_string,2);
                        APP_PPRINTF(my_string);
                        APP_PPRINTF("\r\n\n\r");
                    }
        			#endif /* TEST_MODE */
                }
                break;

              default:

                break;
          }
        }
      }
    }
    if (params->RxSlot < RX_SLOT_NONE)
    {
      //APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | PORT:%d | DR:%d | SLOT:%s | RSSI:%d | SNR:%d\r\n",
     //         params->DownlinkCounter, RxPort, params->Datarate, slotStrings[params->RxSlot], params->Rssi, params->Snr);
    }
  }
  Init_Switch();
  Switch_Reset();

  // reinit the I2C pins in PullDown
  HAL_I2C_DeInit(&hi2c1);
  PullDown_I2C_A10(); //EDIT
  PullDown_I2C_A9();  //EDIT


  ////////////////////////
  /* USER CODE END OnRxData_1 */
}

void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
  UTIL_TIMER_Time_t nextTxIn = 0;
  AppData.Port = LORAWAN_USER_APP_PORT;

  uint32_t  ul_size = 0;
  float batteryLevel = 0;


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
  if(my_ret_shtc3 ==HAL_OK)
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

  AppData.Buffer[ul_size++] = 0;
  AppData.Buffer[ul_size++] = (uint8_t)((raw_temperature >> 8) & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)(raw_temperature & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)((raw_humidity >> 8) & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)(raw_humidity & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)((raw_pressure >> 8) & 0xFF);//(uint8_t)(temperature & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)(raw_pressure & 0xFF);//(uint8_t)((humidity >> 8) & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)(raw_co2);//(uint8_t)(humidity & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)(batteryLevel);
  AppData.BufferSize = ul_size;

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

  AppData.Buffer[ul_size++] = 2;

  AppData.Buffer[ul_size++] = (uint8_t)((raw_temperature >> 4) & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t) (     ((raw_temperature & 0x0F)<<4)    |     ((raw_humidity >> 8) & 0x0F)    );
  AppData.Buffer[ul_size++] = (uint8_t)(raw_humidity & 0xFF);

  AppData.Buffer[ul_size++] = (uint8_t)(raw_o2);
  AppData.Buffer[ul_size++] = (uint8_t)((raw_pressure >> 8) & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)(raw_pressure & 0xFF);
  AppData.Buffer[ul_size++] = (uint8_t)(raw_co2);
  AppData.Buffer[ul_size++] = (uint8_t)(batteryLevel);
  AppData.BufferSize = ul_size;
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
		BLE_Convert_Data(&hlpuart1, my_ret_ble, AppData.Buffer[0], AppData.Buffer[1], AppData.Buffer[2], AppData.Buffer[3], AppData.Buffer[4], AppData.Buffer[5], AppData.Buffer[6], AppData.Buffer[7], AppData.Buffer[8]); //edit special Lora
		BLE_WriteData(&hlpuart1, my_ret_ble);
	}
	send_ble_data = 0;
	BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
  }
  else
  {
	  // A ENLEVER A TERME
	  //HAL_StatusTypeDef my_ret_ble = HAL_OK;
	  //MX_LPUART1_UART_Init(); // car low power
	  //HAL_Delay(50);
	  //BLE_WakeUp(&hlpuart1, my_ret_ble);// A ENLEVER A TERME
	  //BLE_Init(&hlpuart1, my_ret_ble); // A ENLEVER A TERME
	  //HAL_Delay(100);
	  //BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
  }
#endif



  /// SEND IN LORA//////////////////////////////////////////////////////////////

  Init_LED3(); //edit

  if ((JoinLedTimer.IsRunning) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    UTIL_TIMER_Stop(&JoinLedTimer);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
  }

  status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
  //if (LORAMAC_HANDLER_SUCCESS == status)
  //{
    //APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
  //}
  //else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
  //{
  //  nextTxIn = LmHandlerGetDutyCycleWaitTime();
  //  if (nextTxIn > 0)
  //  {
      //APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
  //  }
  //}
/////////////////////////////END SEND IN LORA///////////////////////////////////////////

  ///////////////////////////////////////
  MX_I2C1_Init();
  Init_Switch();
  Switch_Set();
  //store the message

#if defined(MEM_EEPROM_WIRED)
  Store_Uplink (my_ret_eeprom, AppData.Buffer, 9); // edit Lora Wan

#endif
  Switch_Reset();

  // reinit the I2C pins in PullDown
  HAL_I2C_DeInit(&hi2c1);
  PullDown_I2C_A10(); //EDIT
  PullDown_I2C_A9();  //EDIT

  //test remove ces commandes BLE
  //HAL_StatusTypeDef my_ret_ble = HAL_OK;
  //MX_LPUART1_UART_Init(); // car low power
  //HAL_Delay(50);
  //BLE_WakeUp(&hlpuart1, my_ret_ble);
  //BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
  ////////////////////////

  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */
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

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */
	  }
	  else {
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
  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */
static void OnTxTimerLedEvent(void *context)
{
  //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_GREEN */
}

static void OnRxTimerLedEvent(void *context)
{
  //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); /* LED_BLUE */
}

static void OnJoinTimerLedEvent(void *context)
{
  //HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); // NO TOGGLE
}


static void SendTxAcc(void)
{
	  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
	  UTIL_TIMER_Time_t nextTxIn = 0;
	  AppData.Port = LORAWAN_USER_APP_PORT;

	  uint32_t  ul_size = 0;
	  float batteryLevel = 0;


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
		   //Error_Handler(); // a commenter
	   }
	   //if(MC3635_sensibility>MC3635_sensibility_msg)
	   //{ //wake-up threshold must be lower than message threshold
		//   Error_Handler();
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
		  	my_ret=MC3635_SET_mode(my_ret, &hi2c1, MC3635_MODE_SNIFF);
		  	if(my_ret !=HAL_OK)
		  	{
		  		HAL_Delay(100);
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


	  AppData.Buffer[ul_size++] = 1; //message type: acceleration
	  AppData.Buffer[ul_size++] = max_acc;
	  AppData.Buffer[ul_size++] = min_acc;
	  AppData.Buffer[ul_size++] = cpt_acc;
	  AppData.Buffer[ul_size++] = (uint8_t)((sum_acc >> 24) & 0xFF);
	  AppData.Buffer[ul_size++] = (uint8_t)((sum_acc >> 16) & 0xFF);
	  AppData.Buffer[ul_size++] = (uint8_t)((sum_acc >> 8) & 0xFF);
	  AppData.Buffer[ul_size++] = (uint8_t)((sum_acc) & 0xFF);
	  AppData.Buffer[ul_size++] = (uint8_t)(batteryLevel);
	  AppData.BufferSize = ul_size;


	#if defined(MEM_EEPROM_WIRED)
	  //store the message
	  Store_Uplink (my_ret_eeprom, AppData.Buffer, 9); //edit Lorawan
	  Switch_Reset();
	#endif

	  if((max_acc-min_acc)>(6-1)) //msg sent only if the threshold is reached.
	  {                                    // threshold between 0.6m/s2 and 3.6m/s2
		  /// SEND IN LORA//////////////////////////////////////////////////////////////

		  //Init_LED3(); //edit

		  if ((JoinLedTimer.IsRunning) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
		  {
			  UTIL_TIMER_Stop(&JoinLedTimer);
			  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
		  }

		  status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
		  //if (LORAMAC_HANDLER_SUCCESS == status)
		  //{
		  //APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
		  //}
		  // else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
		  //{
		  //   nextTxIn = LmHandlerGetDutyCycleWaitTime();
		  //  if (nextTxIn > 0)
		  //  {
	      //APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
		  // }
		  // }
		  /////////////////////////////END SEND IN LORA///////////////////////////////////////////

	  	 }


	  // reinit the I2C pins in PullDown
	  HAL_I2C_DeInit(&hi2c1);
	  PullDown_I2C_A10(); //EDIT
	  PullDown_I2C_A9();  //EDIT

	  //useless now
	  //HAL_StatusTypeDef my_ret_ble = HAL_OK;
	  //MX_LPUART1_UART_Init(); // car low power
	  //HAL_Delay(50);
	  //BLE_WakeUp(&hlpuart1, my_ret_ble);
	  //BLE_GoToSleep(&hlpuart1, my_ret_ble); //each time command BLE : GoToSleep
}



/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  if ((params != NULL))
  {
    /* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
    if (params->IsMcpsConfirm != 0)
    {
      //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); /* LED_GREEN */
      //UTIL_TIMER_Start(&TxLedTimer);

      //APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
      //APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
      //        params->AppData.Port, params->Datarate, params->TxPower);

      //APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
      if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
      {
        //APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
      }
      else
      {
        //APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
      }
    }
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  if (joinParams != NULL)
  {
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {
      UTIL_TIMER_Stop(&JoinLedTimer);
      //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */

      //APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
        //APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
      }
      else
      {
        //APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");


      }
    }
    else
    {
      //APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    }
  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN OnBeaconStatusChange_1 */
  if (params != NULL)
  {
    switch (params->State)
    {
      default:
      case LORAMAC_HANDLER_BEACON_LOST:
      {
        //APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON LOST\r\n");
        break;
      }
      case LORAMAC_HANDLER_BEACON_RX:
      {
        //APP_LOG(TS_OFF, VLEVEL_M,
        //        "\r\n###### BEACON RECEIVED | DR:%d | RSSI:%d | SNR:%d | FQ:%d | TIME:%d | DESC:%d | "
        //        "INFO:02X%02X%02X %02X%02X%02X\r\n",
        //        params->Info.Datarate, params->Info.Rssi, params->Info.Snr, params->Info.Frequency,
        //        params->Info.Time.Seconds, params->Info.GwSpecific.InfoDesc,
        //        params->Info.GwSpecific.Info[0], params->Info.GwSpecific.Info[1],
        //        params->Info.GwSpecific.Info[2], params->Info.GwSpecific.Info[3],
        //        params->Info.GwSpecific.Info[4], params->Info.GwSpecific.Info[5]);
        break;
      }
      case LORAMAC_HANDLER_BEACON_NRX:
      {
        //APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON NOT RECEIVED\r\n");
        break;
      }
    }
  }
  /* USER CODE END OnBeaconStatusChange_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN OnClassChange_1 */
  //APP_LOG(TS_OFF, VLEVEL_M, "Switch to Class %c done\r\n", "ABC"[deviceClass]);
  /* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
  /* USER CODE BEGIN OnTxPeriodicityChanged_1 */

  /* USER CODE END OnTxPeriodicityChanged_1 */
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0)
  {
    /* Revert to application default periodicity */
    TxPeriodicity = APP_TX_DUTYCYCLE;
  }

  /* Update timer periodicity */
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxPeriodicityChanged_2 */

  /* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
  /* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

  /* USER CODE END OnTxFrameCtrlChanged_1 */
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
  /* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

  /* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

  /* USER CODE END OnPingSlotPeriodicityChanged_1 */
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

  /* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void)
{
  /* USER CODE BEGIN OnSystemReset_1 */

  /* USER CODE END OnSystemReset_1 */
  if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    NVIC_SystemReset();
  }
  /* USER CODE BEGIN OnSystemReset_Last */

  /* USER CODE END OnSystemReset_Last */
}

static void StopJoin(void)
{
  /* USER CODE BEGIN StopJoin_1 */
  //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); /* LED_BLUE */
  //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); /* LED_GREEN */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); /* LED_RED */
  /* USER CODE END StopJoin_1 */

  UTIL_TIMER_Stop(&TxTimer);

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
  {
    //APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
  }
  else
  {
    //APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
    if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
    {
      ActivationType = ACTIVATION_TYPE_OTAA;
      //APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
    }
    else
    {
      ActivationType = ACTIVATION_TYPE_ABP;
      //APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
    }
    LmHandlerConfigure(&LmHandlerParams);
    LmHandlerJoin(ActivationType, true);
    UTIL_TIMER_Start(&TxTimer);
  }
  UTIL_TIMER_Start(&StopJoinTimer);
  /* USER CODE BEGIN StopJoin_Last */

  /* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context)
{
  /* USER CODE BEGIN OnStopJoinTimerEvent_1 */

  /* USER CODE END OnStopJoinTimerEvent_1 */
  if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
  {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
  }
  /* USER CODE BEGIN OnStopJoinTimerEvent_Last */
  //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); /* LED_BLUE */
  //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_GREEN */
  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
  /* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    //APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    //APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
  }
  /* USER CODE BEGIN StoreContext_Last */

  /* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* USER CODE BEGIN OnNvmDataChange_1 */

  /* USER CODE END OnNvmDataChange_1 */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    //APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    //APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
  /* USER CODE BEGIN OnNvmDataChange_Last */

  /* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  /* store nvm in flash */
  if (HAL_FLASH_Unlock() == HAL_OK)
  {
    if (FLASH_IF_EraseByPages(PAGE(LORAWAN_NVM_BASE_ADDRESS), 1, 0U) == FLASH_OK)
    {
      FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (uint8_t *)nvm, nvm_size, NULL);
    }
    HAL_FLASH_Lock();
  }
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */

  /* USER CODE END OnRestoreContextRequest_1 */
  UTIL_MEM_cpy_8(nvm, (void *)LORAWAN_NVM_BASE_ADDRESS, nvm_size);
  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}

