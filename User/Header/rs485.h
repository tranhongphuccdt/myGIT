


#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"

	
	#define RS485_ADDRESS 0x20
	#define ACK_MASK	0x10
	#define ACK	0xAE
	#define NACK 0xBE
	
  #define RS485_RX_PIN       GPIO_PIN_3   // Data receive pin
  #define RS485_RX_PORT	    GPIOA

  #define RS485_TX_PIN       GPIO_PIN_2 // Data transmit pin
  #define RS485_TX_PORT	  	GPIOA

  #define RS485_DIR_PIN   		GPIO_PIN_1   // Controls DE pin.  RX low, TX high.
  #define RS485_DIR_PORT  		GPIOB

  #define RS485_UARTx		   		 USART2
	#define RS485_TIME_RESOURT	 TIM2
	 
	#define RS485_CK_ENABLE()	__GPIOB_CLK_ENABLE();__USART2_CLK_ENABLE() ; __TIM2_CLK_ENABLE();
	
	#define RS485_MAX_LEN		122
	 
	#define RS485_IN()					HAL_GPIO_WritePin(RS485_DIR_PORT,RS485_DIR_PIN,0)
	#define RS485_OUT()				  HAL_GPIO_WritePin(RS485_DIR_PORT,RS485_DIR_PIN,1)
typedef enum{
    STATE_INIT,             
    STATE_IDLE,              
    STATE_RCVING,
		STATE_RCVED,
		STATE_TRANSMITING,
		STATE_TRANSMITED,
		STATE_WAITTING_ACK,
    STATE_ERROR              
} RS485_STATE;

typedef struct{
	int8_t aOptCode;
	uint8_t aAddDest;
	uint8_t aAddSource;
	int8_t aLen;
	int8_t aCrc8;
	int8_t aData[5];
}RS485_DATA_STRUCT;

typedef struct{
	uint8_t aAdress;
	uint8_t aCmd;
	int32_t aData;
	int8_t aCheckSum;
}USER_DATA_STRUCT;
/*
Command: 
	<P>: Position
	<V>: Velocity
	<A>: Accel
	<R>: Reset
	<G>: Read position.
	<K>: ACK
	<E>: Enable
	<D>: Disable
	<S>: Stop
	<C>: Clear Error.
*/
typedef enum{
	RS485_CMD_NULL,
	RS485_CMD_POSITION,
	RS485_CMD_VELOCITY,
	RS485_CMD_TIME_UPDATE,
	RS485_CMD_ACCEL,
	RS485_CMD_RESET,
	RS485_CMD_READ,
	RS485_CMD_ENABLE,
	RS485_CMD_DISABLE,
	RS485_CMD_STOP,
	RS485_CMD_CLEAR_ERR
}RS485_CMD;



int8_t RS485_Crc8(RS485_DATA_STRUCT *psPayload);
int8_t Crc8(int8_t* pData,uint8_t size);
int8_t RS485_Init(uint32_t aBaudrate,uint8_t aSourdAdd);
int8_t RS485_Transmit(uint8_t optCode,int8_t addDest,uint8_t *pData,uint16_t dataLen,uint32_t aTimeOut);
int8_t RS485_Polling(void);
int8_t UartSendData(uint8_t *pData,uint16_t len);

void Rs485_NewPayloadIRQ(void);	//Insert to IRQ Time Resourt.
void RS485_RxDataIRQ(void);//Insert UART RX IRQ
extern void  ADC_Read(void);
