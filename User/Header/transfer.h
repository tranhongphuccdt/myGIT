// Header:
// File Name: playload.c
// Author:		LinhTran
// Date:			2/11/14

#include <stdint.h>
#include "usbd_def.h"

#define PAYLOAD_START_CODE	0xFAAF
#define PAYLOAD_LEN_MAX	    58
extern USBD_HandleTypeDef hUSBDDevice;
#pragma pack(1)
typedef struct{
  uint16_t 	startCode ;
	uint16_t 	payloadLen;	// packageLen is only include pData len.
	uint8_t 	optCode;		//bit 7= 1 Input, 0 Output	; 6:0 Command;
	uint8_t  	crc8;
	uint8_t		data[PAYLOAD_LEN_MAX];
}PAYLOAD_TYPE;

void PAYLOAD_Create(uint16_t payloadLen,uint8_t optCode,uint8_t	*pData,PAYLOAD_TYPE *payload_type);
uint8_t CRC8_Generate(unsigned crc, unsigned char *data, uint32_t len);
uint8_t PAYLOAD_Crc8( PAYLOAD_TYPE *payload_type);
int8_t PAYLOAD_Process(uint8_t* pData);
int8_t PAYLOAD_Transmit(uint8_t optCode,uint8_t *pData,uint16_t dataLen,uint32_t aTimeOutUS);
void PAYLOAD_DataIncommingProcess(uint8_t aRxData);
void PAYLOAD_DatasCommingProcess(uint8_t* pData,uint16_t len);