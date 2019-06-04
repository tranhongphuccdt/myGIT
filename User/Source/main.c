

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "board.h"
#include "stm32f1xx_hal_flash_ex.h"

//Define version for Release:
#define RELEASE 1


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

FLASH_OBProgramInitTypeDef saFlashOB;
USBD_HandleTypeDef  hUSBDDevice;

PAYLOAD_TYPE  asPayloadTx;
PAYLOAD_TYPE  asPayloadRx;

DATA_HW_INFOR asDataHwInfor;
DATA_RUNING_INFOR asRuningInforCurrent,asRuningInforSet,asRuningInforOlder;
volatile STATE_MECHINE asStateMechine;
DATA_MOTOR_INFOR asMotorInfor;
DATA_MOTOR_CURR_INFOR asMotorCurrInfor;

volatile uint8_t aaUsbBufferOut[64];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
int8_t APP_LoadConfig(void);
int8_t CMD_SaveAllInForToFlash(void);

extern void COMMAND_FuntionAssign(void);
extern int8_t CMD_GetHwInfor(void);
extern int8_t UPDATE_RunningInfor(void);
extern void STRING_Process(int8_t *pData);
extern uint8_t vituarlComTransmit(int8_t*pData, uint32_t size);



volatile int cError,cSuccess;
extern USBD_DescriptorsTypeDef VCP_Desc;
uint32_t aTimeSendData=0,aTimeIndicateErr=0;
uint32_t aTimeLedProcess=0;
uint32_t aTimeUpdateInfor;
int8_t ret;
extern volatile int8_t usbVituarCom_ReceiverFlag;
int8_t vituarlComBuffer[50];
extern int32_t timeUpdateToHost ;

uint8_t buffer[60];
int8_t usbVirtualComFlag =0;
int flag=0;
int main(void) 
{
  HAL_Init();
	/* Configure the system clock to 72 MHz */
  SystemClock_Config();
	
//#if defined(RELEASE)

//	HAL_FLASHEx_OBGetConfig(&saFlashOB);
//	if(	saFlashOB.RDPLevel == 0){
//		HAL_FLASH_Unlock();
//	  HAL_FLASH_OB_Unlock();
//		saFlashOB.OptionType = OPTIONBYTE_RDP;
//		saFlashOB.RDPLevel =   OB_RDP_LEVEL_1;
//		
//		HAL_FLASHEx_OBProgram(&saFlashOB);
//		HAL_FLASH_OB_Launch();             // write OB to Flash and reset
//		HAL_FLASH_OB_Lock();                  // Lock Optionbytes
//    HAL_FLASH_Lock();                     // lock Flash  
//	}
//		
//#endif
	
	IO_Config();
	//Init the hardware:
  LED_StatusConfig();
  LED_StatusDislay(LED_RED_ON);
	
	HAL_Delay(100);
//Config USB:
	USBD_Init(&hUSBDDevice,&SPENCIFIC_Desc,0); 
	USBD_RegisterClass(&hUSBDDevice,USBD_SPENCIFIC_CLASS);
	USBD_Start(&hUSBDDevice);
//		USBD_Init(&hUSBDDevice, &VCP_Desc, 0);
//		USBD_RegisterClass(&hUSBDDevice, &USBD_CDC);
//		USBD_CDC_RegisterInterface(&hUSBDDevice, &USBD_CDC_fops);
//		USBD_Start(&hUSBDDevice);
	
	ret = APP_LoadConfig();
  if(ret){    //Error loading config;
    LED_StatusDislay(LED_YELLOW);
		asRuningInforCurrent.aListError = SYSTEM_FAIL;
		//Indicate Error:
  } 

	ADC_DMA_Config();
  ENCODER_Init(asDataHwInfor.aEncoderFilter);
  DCM_Init();
  COMMAND_FuntionAssign();
	switch(asStateMechine.asCmType){ //Choose method controler
		case CM_RS485: 
			RS485_Init(asDataHwInfor.aBaudrate,asDataHwInfor.aAdd);
			break;
		case CM_CAN: 
			break;
		case CM_PULSE: 
		case CM_ANALOG:
			ExternalCounterInit();
			break;
		default:
			break;
	}
		
	HAL_Delay(1000);
	
	
	
	LoopEnable();
	//Enable Control
  DC_ENABLE();
	
//	while(1){
//		HAL_Delay(50);
//		DCM_UpdateDuty(1000,1);
//		HAL_Delay(50);
//		DCM_UpdateDuty(1000,0);
//	}
	
	asStateMechine.asLoopControlStatus =CLOOP_STARTING;
	
  LED_StatusDislay(LED_GREEN_ON);      //Finish Init: 
  LED_StatusDislay(LED_RED_OFF);      //Finish Init: 
	while (1){
							//Process a command from Host:
									if(((USBD_SPENCIFIC_HandleTypeDef *)(hUSBDDevice.pClassData))->stateReceiver == SPENCIFIC_DATA_RECEIVER){// New data is comming:
											
											((USBD_SPENCIFIC_HandleTypeDef *)(hUSBDDevice.pClassData))->stateReceiver = SPENCIFIC_DATA_NONE_RECEIVER;        
											PAYLOAD_Process(aaUsbBufferOut);
							 			
									}
							//Network Processing:     
									if(asStateMechine.asCmType==CM_RS485){ 
											RS485_Polling();
									}
							//Update Running infor to hot:
									if(hUSBDDevice.dev_state == USBD_STATE_CONFIGURED){//USB conected:   so send data to hot every TIME_UPDATE 
										asRuningInforCurrent.aMsTick = HAL_GetTick();
										if(aTimeSendData < asRuningInforCurrent.aMsTick){         
											 UPDATE_RunningInfor();
											 aTimeSendData = asRuningInforCurrent.aMsTick + TIME_UPDATE;
											
										}
									} 
							//Auto Tuning update Process:		
									if(asStateMechine.asRunningMode == MODE_TURNING && asStateMechine.asLoopControlStatus == CLOOP_FINISH 	){ //wait for processing Turning finish: the turning will ran in INTERRUP:
										TURNING_Respont();
										asStateMechine.asLoopControlStatus = CLOOP_STOOP;
											 
									}  
							//Indicate led message:   
								 if(aTimeLedProcess<HAL_GetTick()){//indicate Led message:
										 aTimeLedProcess =  HAL_GetTick()+ 200;
										 PROCESS_Indicate();
									}
									
									if(aTimeIndicateErr<HAL_GetTick()){		//ERROR Indicate
											 aTimeIndicateErr = HAL_GetTick()+ 200;
											 ERROR_Indicate();	  
									}
									
									//Updata infor to Host by Uart: timeUpdateToHost
									if(timeUpdateToHost!=0 && aTimeUpdateInfor<HAL_GetTick()){
										aTimeUpdateInfor = HAL_GetTick()+ timeUpdateToHost;
										sprintf(buffer,"A%d P%d \n",asDataHwInfor.aAdd, (int32_t)(1000*asRuningInforCurrent.aPosition));
										UartSendData(buffer,strlen(buffer));
									}
	}
}

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  RCC_PeriphCLKInitTypeDef rccperiphclkinit = {0};
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
    
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Start Conversation Error */ 
  }
  
  /* USB clock selection */
  rccperiphclkinit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  rccperiphclkinit.UsbClockSelection = RCC_USBPLLCLK_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&rccperiphclkinit);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Start Conversation Error */
  }
	
	__HAL_RCC_AFIO_CLK_ENABLE();
}
#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
  }
}
#endif



