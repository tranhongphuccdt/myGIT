// Header:
// File Name: 
// Author:		LinhTran
// Date:			2/11/14

#include "transfer.h"
#include "crc.h"
#include "usbd_specific.h"
#include "delay.h"

extern int8_t COMMAND_Process(PAYLOAD_TYPE *psPayload);



uint8_t PAYLOAD_Crc8( PAYLOAD_TYPE *payload_type){
	uint8_t crc;
	
	if(payload_type->payloadLen<64){
	
		crc = CRC8_Generate(0,(uint8_t*)(&payload_type->payloadLen),2);
		crc = CRC8_Generate(crc,&payload_type->optCode,1);
		crc =  CRC8_Generate(crc,payload_type->data,payload_type->payloadLen);
  }
  return crc;
}
//a block per only one package
void PAYLOAD_Create(uint16_t payloadLen,uint8_t optCode,uint8_t	*pData,PAYLOAD_TYPE *payload_type){
	uint16_t nextI;
	
	payload_type->startCode = PAYLOAD_START_CODE;
	payload_type->payloadLen = payloadLen;
	payload_type->optCode = optCode;
	
	for(nextI=0;nextI<payloadLen;nextI++){
		payload_type->data[nextI] = *(pData+nextI);
	}

	payload_type->crc8 =  PAYLOAD_Crc8(payload_type);
}
//int8_t PAYLOAD_Transmit(PAYLOAD_TYPE *psPayload,uint32_t aTimeOutUS){ //Call low driver funtion to transmit data:
//  int8_t ret = 0; 
//  while(USBD_PreparaSendData(&USBD_Device,0x81,(uint8_t*)psPayload,(psPayload->payloadLen )+6) !=USBD_OK){
//    aTimeOutUS--;
//    if(aTimeOutUS==0){
//      ret =-1;
//    }
//  }
//  return ret;
//}
int8_t PAYLOAD_Transmit(uint8_t optCode,uint8_t *pData,uint16_t dataLen,uint32_t aTimeOutUS){
   PAYLOAD_TYPE asPayload;
   int8_t ret =0;
		//uint8_t term[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
	
   PAYLOAD_Create(dataLen,optCode,pData,&asPayload);
   while(USBD_PreparaSendData(&hUSBDDevice,0x81,(uint8_t*)&asPayload,(asPayload.payloadLen+6)) !=USBD_OK)
	 {
  // while(USBD_PreparaSendData(&hUSBDDevice,0x81,(uint8_t*)&term,16) !=USBD_OK){
				if(aTimeOutUS>0){
					aTimeOutUS--;
				}
				else{
					ret =-1;
					break;
				}
		}
  return ret;  
}
int8_t PAYLOAD_Process(uint8_t* pData){
  PAYLOAD_TYPE asPayload;
  int8_t ret=0;
  asPayload = *(PAYLOAD_TYPE*)pData;
  
  if(PAYLOAD_Crc8(&asPayload)== asPayload.crc8){  //good payload
		COMMAND_Process(&asPayload);         
  }
  else{ //Send a Command not supported.
    ret =-1;
  }	
  return ret;
}
//Use for long package with sigal byte per a transmit (many block per a package):   /////////////////////////////////
void PAYLOAD_DataIncommingProcess(uint8_t aRxData){ //Use for long package
  static uint8_t aRxState =0;
  static uint8_t aRxDataOld =0;
  static uint16_t aRxIndexCount =0;
  static uint16_t aLen ;
  static uint8_t aaRxBuffer[PAYLOAD_LEN_MAX+6];
  static PAYLOAD_TYPE asPayload;
  
  switch(aRxState){
    case 0:
      if(aRxDataOld == (uint8_t)(PAYLOAD_START_CODE>>8)  && aRxData == (uint8_t)PAYLOAD_START_CODE){
        aRxState =1;
        aaRxBuffer[0] = (uint8_t)(PAYLOAD_START_CODE>>8);
        aaRxBuffer[1] = (uint8_t)PAYLOAD_START_CODE;
        aRxIndexCount =1;
        aLen = 0xffff;
      }
      aRxDataOld = aRxData;
      break;
    case 1:
      if(aRxIndexCount ==3){  //Find length of payload:
        aRxState =255;
        aLen = *(uint16_t*)&aaRxBuffer[2];
      }
      aaRxBuffer[aRxIndexCount] = aRxData;
      break;
    default:
      aaRxBuffer[aRxIndexCount] = aRxData;
      aLen--;
      if(aRxIndexCount> (PAYLOAD_LEN_MAX +6)){
        aRxIndexCount =0;
      }
      break;    
  }
   aRxIndexCount++;;
  if(aLen==0){  //Finish of payload:
    aRxState =0;
    aRxDataOld =0;
    aRxIndexCount =0;
      
    asPayload = *(PAYLOAD_TYPE*)aaRxBuffer;
    if(PAYLOAD_Crc8(&asPayload)== asPayload.crc8){    //payload is good:
      COMMAND_Process(&asPayload);                    //Call a h leval funiton. 
    }
    else{   //Send NACK : Frame Error
      
    }
  }
}
 
void PAYLOAD_DatasCommingProcess(uint8_t* pData,uint16_t len){ //Use for long package
  uint16_t i;
  for(i=0;i<len;i++){
    PAYLOAD_DataIncommingProcess(pData[i]);
  }  
}

