// Header:
// File Name: 
// Author:        	LinhTran
// Mail:		linhtran.ccs@gmail.com
// Company:	Cc-Smart.net
// Date:		17/1/2014


#include "rs485.h"

#include "command.h"
#include "product_infor.h"
#include "data_struct.h"
#include "stm32f1xx_hal_flash.h"
#include "crc.h"
#include "delay.h"

extern int64_t aExternCounterValue;
extern PAYLOAD_TYPE  asPayloadTx;
extern PAYLOAD_TYPE  asPayloadRx;
extern uint8_t aHaveNewCommand;

extern STATE_MECHINE asStateMechine;
extern DATA_HW_INFOR asDataHwInfor;
extern DATA_RUNING_INFOR asRuningInforCurrent,asRuningInforSet;
extern DATA_MOTOR_INFOR asMotorInfor;
//struct avariable:
UART_HandleTypeDef   UartHandleRs485;
TIM_HandleTypeDef    TimHandleRs485;
volatile int8_t aaRs485Buffer[RS485_MAX_LEN];
RS485_STATE Rs485StateRx ;
RS485_DATA_STRUCT asRS485Rx;
USER_DATA_STRUCT asUserData;
volatile int16_t aRxCount=0;
volatile int8_t aAckNack =0;
volatile uint8_t aRs485SoureAdd =0;
volatile uint32_t aCountPlayload =0;

 

__weak void RS485_ProcessNewData(RS485_DATA_STRUCT *psRs485Data){
	
}

__weak void RS485_ProcessNewDataRealTime(int16_t data,int16_t cmd){
	
}


int8_t RS485_TimeInit(){
	
	 /* -----------------------------------------------------------------------
		TIM2 Configuration: 
    
    In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM2CLK = 2 * PCLK1 = 2*18 =36Mhz  
       
			To get TIM2 counter clock at 36MHz , the prescaler is computed as follows: 
			Prescaler = (TIM2CLK / TIM2 counter clock) - 1 = 36 000 000 / 36000000   -1= 0 ;
	*/
	
	TimHandleRs485.Instance = RS485_TIME_RESOURT;
  TimHandleRs485.Init.Prescaler = 0;
  TimHandleRs485.Init.CounterMode = TIM_COUNTERMODE_UP;
	
  TimHandleRs485.Init.Period = 36000;									//Set time over when t = 1 ms   //(fixed 3.5 charater at baudrate:115200)
  TimHandleRs485.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&TimHandleRs485);
	
	 /* System interrupt init*/
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);	
	
	__HAL_TIM_CLEAR_IT(&TimHandleRs485, TIM_IT_UPDATE);
	HAL_TIM_Base_Stop_IT(&TimHandleRs485);
}
int8_t RS485_Init(uint32_t aBaudrate,uint8_t aSourdAdd){
		GPIO_InitTypeDef GPIO_InitStructure;
	
	
		RS485_CK_ENABLE();
	
	  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStructure.Pin =  RS485_DIR_PIN;
		HAL_GPIO_Init(RS485_DIR_PORT, &GPIO_InitStructure);
	
	
	//Uart TX,RX:
		GPIO_InitStructure.Pin       = RS485_TX_PIN;
		GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull      = GPIO_PULLUP;

	
	  HAL_GPIO_Init(RS485_TX_PORT, &GPIO_InitStructure);
	
		GPIO_InitStructure.Pin = RS485_RX_PIN;
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(RS485_RX_PORT, &GPIO_InitStructure);
		
		/*##-1- Configure the UART peripheral ######################################*/
		/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
		/* UART1 configured as follow:
				- Word Length = 8 Bits
				- Stop Bit    = One Stop bit
				- Parity      = ODD parity
				- BaudRate    = x baud
				- Hardware flow control disabled (RTS and CTS signals) */
		UartHandleRs485.Instance        = RS485_UARTx;

		UartHandleRs485.Init.BaudRate   = aBaudrate;
		UartHandleRs485.Init.WordLength = UART_WORDLENGTH_8B;
		UartHandleRs485.Init.StopBits   = UART_STOPBITS_1;
		UartHandleRs485.Init.Parity     = UART_PARITY_ODD;
		UartHandleRs485.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
		UartHandleRs485.Init.Mode       = UART_MODE_RX;
		
		HAL_UART_DeInit(&UartHandleRs485);
		
		if (HAL_UART_Init(&UartHandleRs485) != HAL_OK)
		{
			/* Initialization Error */
			//Error_Handler();
		}
		  /* Enable the UART Parity Error Interrupt */
    //__HAL_UART_ENABLE_IT(&UartHandleRs485, UART_IT_PE);    
    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    //__HAL_UART_ENABLE_IT(&UartHandleRs485, UART_IT_ERR);
    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&UartHandleRs485, UART_IT_RXNE);
			 /* System interrupt init*/
		HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);	
		
		Rs485StateRx = STATE_IDLE;
		aRs485SoureAdd = aSourdAdd;
		
		RS485_TimeInit();
}
int8_t RS485_Crc8(RS485_DATA_STRUCT *psPayload){
	int8_t crc=0;
	uint8_t i;
	
	crc ^= psPayload->aOptCode;
	crc ^= psPayload->aAddDest;
	crc ^=psPayload->aAddSource;
	crc ^= psPayload->aLen;
	for(i=0;i<psPayload->aLen;i++){
		crc ^= psPayload->aData[i];
	}
	
	return crc^0xff;
}
int8_t Crc8(int8_t* pData,uint8_t size){
	int8_t crc=0;
	uint8_t i;
	for(i=0;i<size++;i++){
		crc ^= *pData;
		pData++;
	}
	return crc^0xff;
}
void RS485_RxDataIRQ(void){	//Add this fution to UARx_IRQ:
	static int8_t aData;

	if(__HAL_UART_GET_IT_SOURCE(&UartHandleRs485,UART_IT_RXNE)){
				aData  =  UartHandleRs485.Instance->DR;
			
				//RESET time when have new data;
				RS485_TIME_RESOURT->CNT =0;		
				HAL_TIM_Base_Start_IT(&TimHandleRs485);
				
				if(aRxCount>=RS485_MAX_LEN){
					aRxCount =0;
				}
				aaRs485Buffer[aRxCount++] = aData;
				aaRs485Buffer[aRxCount] =0;
				
					
		
	}
	else{ //Error UART
		aData =0;
	}
	__HAL_UART_CLEAR_IDLEFLAG(&UartHandleRs485);
}

void Rs485_NewPayloadIRQ(void){	//Add this fution to Rs485_TimIRQ
	__HAL_TIM_CLEAR_IT(&TimHandleRs485, TIM_IT_UPDATE);
	HAL_TIM_Base_Stop_IT(&TimHandleRs485);
	STRING_Process(aaRs485Buffer);
	aRxCount=0;
//	
//	if(aRxCount ==RS485_MAX_LEN){ //good frame:
//		
//		RS485_ProcessNewDataRealTime(*(int16_t*)(aaRs485Buffer+2*aRs485SoureAdd+2), *(int16_t*)(aaRs485Buffer));	//2 byte dau de lam command:
//		aCountPlayload++;
//	}
//	aRxCount =0;
	
}

int8_t UartSendData(uint8_t *pData,uint16_t len){
	HAL_UART_Transmit(&UartHandleRs485, pData, len, 0xFFFF);
}

int8_t RS485_SendAckNACK(uint8_t cmd){
	RS485_OUT();
	HAL_UART_Transmit(&UartHandleRs485, &cmd, 1, 0xFFFF);
	RS485_IN();
}

int8_t RS485_Polling(void){
	int i,j;
	
	if(Rs485StateRx == STATE_RCVED){
		Rs485StateRx = STATE_IDLE;
		asRS485Rx = *(RS485_DATA_STRUCT*)aaRs485Buffer;	
					RS485_OUT();
						if(asRS485Rx.aCrc8 == RS485_Crc8(&asRS485Rx)){		//good payload:
							  Rs485StateRx = STATE_IDLE;
								//Send ACK if I is requested:
								if(asRS485Rx.aOptCode & ACK_MASK){
									RS485_SendAckNACK(ACK);
								}
								//Call a funtion process data:
								RS485_ProcessNewData(&asRS485Rx);
						}
						else{	// payload wrong:
								Rs485StateRx = STATE_ERROR;
								//Send NCK if I is requested:
								if(asRS485Rx.aOptCode & ACK_MASK){
									RS485_SendAckNACK(NACK);
								}
						}
		aRxCount =0;
	}
	
}
int8_t RS485_Transmit(uint8_t optCode,int8_t addDest,uint8_t *pData,uint16_t dataLen,uint32_t aTimeOut){
	int8_t ret;
	int i;
	RS485_DATA_STRUCT asRs485Data;
	
	RS485_OUT();
	
	asRs485Data.aOptCode = optCode;
	asRs485Data.aAddDest = addDest;
	asRs485Data.aAddSource = aRs485SoureAdd;
	asRs485Data.aLen = 		 dataLen;
	
	for(i=0;i<dataLen;i++){
		asRs485Data.aData[i] = *(pData+i);
	}
	//Create checksum:
	asRs485Data.aCrc8 = RS485_Crc8(&asRs485Data);
	//Send payload: // Cho nay co ve chua duoc on vi thoi gian truyen con giai , co the thay DMA de truyen.
	ret =  HAL_UART_Transmit(&UartHandleRs485, (uint8_t *)&asRs485Data, asRs485Data.aLen+5, 0xFFFF);
	
	if(asRs485Data.aOptCode & ACK_MASK || ret==0){// waitting ACK:
		RS485_IN();
		Rs485StateRx = STATE_WAITTING_ACK;
		while(Rs485StateRx == STATE_WAITTING_ACK && aTimeOut--){
			
		}
		if(aTimeOut>0 && aAckNack == ACK){
			ret =0;
		}
		else ret = 1;
	}
	//Finished:
	RS485_OUT();
	return ret;	
}