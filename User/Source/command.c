// Header:
// File Name: 
// Author:		LinhTran
// Date:

#include "command.h"
#include "product_infor.h"
#include "data_struct.h"
#include "stm32f1xx_hal_flash.h"
#include "crc.h"
#include "delay.h"
#include "rs485.h"
#include "board.h"

#define TRANSFER_TIME_OUT  50000

extern int64_t aExternCounterValue;
extern PAYLOAD_TYPE  asPayloadTx;
extern PAYLOAD_TYPE  asPayloadRx;

extern STATE_MECHINE asStateMechine;
extern DATA_HW_INFOR asDataHwInfor;
extern DATA_RUNING_INFOR asRuningInforCurrent,asRuningInforSet;
extern DATA_MOTOR_INFOR asMotorInfor;

extern DATA_MOTOR_CURR_INFOR asMotorCurrInfor;
	
extern void   FLASH_ClearErrorCode(void);
extern void ExternalCounterInit(void);


static int8_t  (*pfCommandProccess[CMD_END_OF_COMMAND])(void);

int8_t CMD_StringMessage(void);
int8_t CMD_ResetMcu(void);
int8_t CMD_ReadHwfwprVersion(void);
int8_t CMD_WriteFwprVersion(void);
int8_t CMD_SetHwInfor(void);
int8_t CMD_GetHwInfor(void);
int8_t CMD_SetControlLoopMode(void);
int8_t CMD_GetControlLoopMode(void);
int8_t CMD_SetRunningInfor(void);
int8_t CMD_GetRunningInforSet(void);
int8_t CMD_SetMotorInfor(void);
int8_t CMD_SetMotorCurrInfor(void);
int8_t CMD_GetMotorInfor(void);
int8_t CMD_TurnOnLoopControl(void);
int8_t CMD_TurnOffLoopControl(void);
int8_t CMD_SaveAllInForToFlash(void);
int8_t CMD_SetCommunicaType(void);
int8_t CMD_GetCommunicaType(void);
int8_t CMD_BootLoaderJum(void);
int8_t CMD_AutoTuning(void);
int8_t UPDATE_RunningInfor(void);
int8_t CMD_GetOnOfLoopControl(void);
int8_t CMD_ClearError(void);
int8_t CMD_GetSourceVoltage(void);
int8_t CMD_GetSmartInfor(void);

//Funtion Communication with Host:
void COMMAND_FuntionAssign(void){ // linking a funtion to array funtion:
	pfCommandProccess[CMD_STRING_MESSAGE] = 								CMD_StringMessage;
	pfCommandProccess[CMD_RESET_MCU] = 										  CMD_ResetMcu;
	pfCommandProccess[CMD_READ_HWFWPR_VERSION] = 						CMD_ReadHwfwprVersion;
  pfCommandProccess[CMD_WRITE_FWPR_VERSION] =             CMD_WriteFwprVersion;
	pfCommandProccess[CMD_SET_HW_INFOR] = 									CMD_SetHwInfor;
	pfCommandProccess[CMD_GET_HW_INFOR] = 									CMD_GetHwInfor;
	pfCommandProccess[CMD_SET_CONTROL_LOOP_MODE] = 					CMD_SetControlLoopMode;
	pfCommandProccess[CMD_GET_CONTROL_LOOP_MODE] = 					CMD_GetControlLoopMode;
	pfCommandProccess[CMD_SET_RUNNING_INFOR] = 							CMD_SetRunningInfor;
	pfCommandProccess[CMD_GET_RUNNING_INFOR_SET] = 					CMD_GetRunningInforSet;
	pfCommandProccess[CMD_SET_MOTOR_INFOR] = 								CMD_SetMotorInfor;
	pfCommandProccess[CMD_GET_MOTOR_INFOR] = 								CMD_GetMotorInfor;
	pfCommandProccess[CMD_TURN_ON_LOOP_CONTROL] = 					CMD_TurnOnLoopControl;
	pfCommandProccess[CMD_TURN_OFF_LOOP_CONTROL] = 					CMD_TurnOffLoopControl;
	pfCommandProccess[CMD_GET_ON_OFF_LOOP_CONTROL] = 				CMD_GetOnOfLoopControl;
	pfCommandProccess[CMD_SAVE_ALL_INFOR_TO_FLASH] = 				CMD_SaveAllInForToFlash;
	pfCommandProccess[CMD_SET_COMUNICA_TYPE] = 							CMD_SetCommunicaType;
	pfCommandProccess[CMD_GET_COMUNICA_TYPE] = 							CMD_GetCommunicaType;
	pfCommandProccess[CMD_AUTO_TUNING] = 										CMD_AutoTuning;
	pfCommandProccess[CMD_BOOT_LOADER_JUM] = 								CMD_BootLoaderJum;
	pfCommandProccess[CMD_CLEAR_ERROR]  = 									CMD_ClearError;
	pfCommandProccess[CMD_GET_SOURCE_VOLTAGE]	= 					  CMD_GetSourceVoltage;
	pfCommandProccess[CMD_SET_MOTOR_CURR_INFOR]	= 					CMD_SetMotorCurrInfor;
	pfCommandProccess[CMD_GET_SMART_INFOR]	= 					    CMD_GetSmartInfor;

}


int8_t COMMAND_Process(PAYLOAD_TYPE *psPayload){ 
  int8_t ret=0;
		
    if((psPayload->optCode) < CMD_END_OF_COMMAND){
			asPayloadRx = *psPayload;
      (*pfCommandProccess[psPayload->optCode])();
    }
    else{
      ret =1;
    }
  return ret;
}
int8_t CMD_StringMessage(void){
	
  
}
int8_t CMD_ResetMcu(void){
  PAYLOAD_Transmit(CMD_RESET_MCU|CMD_RESPONT_MASK,NULL,0,TRANSFER_TIME_OUT);
	HAL_NVIC_SystemReset(); 
}
int8_t CMD_ReadHwfwprVersion(void){
  void *pData;
  int8_t ret=0;
  uint32_t aAddress;
  uint32_t aSize= sizeof(PRODUCT_INFOR);
  pData = malloc(aSize+5);
  ret = HAL_FLASH_Unlock();
  
  //Status of Command:
  *(uint8_t*)pData = ret;
  
  aAddress = PRODUCT_INFOR_FLASH_ADDR;
  while (aAddress < (aSize+PRODUCT_INFOR_FLASH_ADDR))
  {
      *((uint32_t*)((uint8_t*)pData+1) + (aAddress-PRODUCT_INFOR_FLASH_ADDR)/4) = *(__IO uint32_t *)aAddress;
      aAddress = aAddress + 4;
  }

  if(pData!=NULL){
  
    PAYLOAD_Transmit(CMD_READ_HWFWPR_VERSION,(uint8_t*)pData,sizeof(PRODUCT_INFOR)+1,TRANSFER_TIME_OUT);

  }
  else{
     ret =-1;
  }
  HAL_FLASH_Lock();
  return ret;
}
int8_t CMD_WriteFwprVersion(void){
  
}
int8_t CMD_SetHwInfor(void){
  int8_t ret=0;
  asDataHwInfor = *((DATA_HW_INFOR *)&asPayloadRx.data);
  ret = PAYLOAD_Transmit(CMD_SET_HW_INFOR|CMD_RESPONT_MASK,(uint8_t*)&asDataHwInfor,sizeof(DATA_HW_INFOR),TRANSFER_TIME_OUT);
  return ret;
}
int8_t CMD_GetHwInfor(void){
  int8_t ret=0;
  ret = PAYLOAD_Transmit(CMD_GET_HW_INFOR|CMD_RESPONT_MASK,(uint8_t*)&asDataHwInfor,sizeof(DATA_HW_INFOR),TRANSFER_TIME_OUT);
  return ret;
}
int8_t TURNING_Respont(void){
	int8_t ret =0;
	ret = PAYLOAD_Transmit(CMD_AUTO_TUNING|CMD_RESPONT_MASK,(uint8_t*)&asMotorInfor,sizeof(asMotorInfor),TRANSFER_TIME_OUT);
}
int8_t CMD_GetSmartInfor(void){
	int8_t ret =0;
	ret = PAYLOAD_Transmit(CMD_GET_SMART_INFOR|CMD_RESPONT_MASK,(uint8_t*)&asMotorCurrInfor,sizeof(asMotorCurrInfor),TRANSFER_TIME_OUT);
}
int8_t CMD_AutoTuning(void){
	int8_t ret =0;
	int timeOut =3000;	//3000(ms)
	asStateMechine.asRunningMode = MODE_TURNING;	
	asStateMechine.asLoopControlStatus = CLOOP_STARTING;
	//reponst a payload when turning finished in main funtion:
	return ret; 
}
int8_t CMD_SetControlLoopMode(void){
	int8_t ret=0;
  uint8_t aCmd = CMD_SET_CONTROL_LOOP_MODE;
  
  asStateMechine.asRunningMode = asPayloadRx.data[0];
  ret = PAYLOAD_Transmit(CMD_SET_CONTROL_LOOP_MODE|CMD_RESPONT_MASK,(uint8_t*)&aCmd,1,TRANSFER_TIME_OUT);
  return ret;
}

int8_t CMD_GetControlLoopMode(void){
  int8_t ret=0;
  
  ret = PAYLOAD_Transmit(CMD_GET_CONTROL_LOOP_MODE|CMD_RESPONT_MASK,(uint8_t*)&asStateMechine.asRunningMode,1,TRANSFER_TIME_OUT);
  return ret;
}

int8_t CMD_SetRunningInfor(void){
  int8_t ret =1;
	int64_t termPulseSet;
	
	if(asStateMechine.asLoopControlStatus != CLOOP_PROCESS || asStateMechine.asCmType == CM_PULSE){
		  asRuningInforSet =*(DATA_RUNING_INFOR*)asPayloadRx.data;
	
			termPulseSet = (asRuningInforSet.aPosition*asDataHwInfor.aEncoderLine)/1.570796327;
			asRuningInforSet.aPosition =    (termPulseSet*1.570796327)/(asDataHwInfor.aEncoderLine);
		
			asStateMechine.asLoopControlStatus =CLOOP_STARTING;
		  ret = 0;
	}
  
  ret = PAYLOAD_Transmit(CMD_SET_RUNNING_INFOR|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_GetRunningInforSet(void){
  int8_t ret =0;
	
  ret = PAYLOAD_Transmit(CMD_GET_RUNNING_INFOR_SET|CMD_RESPONT_MASK,(uint8_t*)&asRuningInforSet,sizeof(asRuningInforSet),TRANSFER_TIME_OUT);

  return ret; 
}
int8_t UPDATE_RunningInfor(void){
	int8_t ret =0;
		
	ret = PAYLOAD_Transmit(CMD_GET_RUNNING_INFOR_SET,(uint8_t*)&asRuningInforCurrent,sizeof(DATA_RUNING_INFOR),TRANSFER_TIME_OUT);
  
  return ret; 
}
int8_t CMD_SetMotorInfor(void){
  int8_t ret =0;
  
  asMotorInfor =*(DATA_MOTOR_INFOR*)asPayloadRx.data;
  ret = PAYLOAD_Transmit(CMD_SET_MOTOR_INFOR|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_SetMotorCurrInfor(void){
	int8_t ret =0;
	asMotorCurrInfor =*(DATA_MOTOR_CURR_INFOR*)asPayloadRx.data;
  ret = PAYLOAD_Transmit(CMD_SET_MOTOR_CURR_INFOR|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}

int8_t CMD_GetMotorInfor(void){
  int8_t ret =0;
  ret = PAYLOAD_Transmit(CMD_GET_MOTOR_INFOR|CMD_RESPONT_MASK,(uint8_t*)&asMotorInfor,sizeof(DATA_MOTOR_INFOR),TRANSFER_TIME_OUT);
  return ret; 
}

int8_t CMD_GetSourceVoltage(void){
	
}
int8_t CMD_ClearError(void){
	int8_t ret =0;
	asRuningInforCurrent.aListError =0;
	if(asStateMechine.aStatus == STATUS_RUN){
		DC_ENABLE();
	}
	if(asStateMechine.asCmType == CM_PULSE){
			HAL_NVIC_SystemReset();	
	}
  ret = PAYLOAD_Transmit(CMD_CLEAR_ERROR|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_GetOnOfLoopControl(void){
	int8_t ret=0;

  PAYLOAD_Transmit(CMD_GET_ON_OFF_LOOP_CONTROL|CMD_RESPONT_MASK,(uint8_t*)&asStateMechine.aStatus,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_TurnOnLoopControl(void){
  int8_t ret=0;
  asStateMechine.aStatus = STATUS_RUN;
	DC_ENABLE();
  PAYLOAD_Transmit(CMD_TURN_ON_LOOP_CONTROL|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret;  
}
int8_t CMD_TurnOffLoopControl(void){
  int8_t ret=0;
  asStateMechine.aStatus = STATUS_STOOP;
	DC_DISABLE();
  ret = PAYLOAD_Transmit(CMD_TURN_OFF_LOOP_CONTROL|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t Flash_WriteData(uint8_t *pData,uint32_t size,uint32_t *aAddress){
  uint32_t aIndex=0,i=0;
  
  while(i<size+2){
		i+=2;
		
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD ,*aAddress,*(__IO uint16_t *)(pData+aIndex))!=HAL_OK){
      return 1;
      break;
    }
    else{ //Check data again;
      if( *(__IO uint16_t *)(*aAddress) != *(__IO uint16_t *)(pData+aIndex)){   //data wrong:
        return 1;
      }
      else{
        (*aAddress) +=2;
        aIndex+=2;
      }
    }   
  }
  return 0;
}
int8_t CMD_SaveAllInForToFlash(void){
	static FLASH_EraseInitTypeDef EraseInitStruct;
	static uint32_t SectorError;
	uint8_t byte[500],term;
	uint32_t i,j;
  int8_t ret=0;
  uint8_t aCrc=0;
  
  uint32_t aAddCurrent = USER_DATA_BASE_FLASH_ADDR;
	//Erase a flash area:
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = USER_DATA_BASE_FLASH_ADDR;
  EraseInitStruct.NbPages = 1;
	
	/* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
		ret = 0xff;
	}
	term = sizeof(DATA_HW_INFOR);
	for(i=0;i<term;i++){
		byte[i] = *((uint8_t*)&asDataHwInfor+i);
	}
	term += sizeof(DATA_RUNING_INFOR);
	j=0;
	for(;i<term;i++){
		byte[i] = *((uint8_t*)&asRuningInforSet+j++);
	}
	term +=sizeof(DATA_MOTOR_INFOR);
	j=0;
	for(;i<term;i++){
		byte[i] = *((uint8_t*)&asMotorInfor+j++);
	}
	term +=sizeof(STATE_MECHINE);
	j=0;
	for(;i<term;i++){
		byte[i] = *((uint8_t*)&asStateMechine+j++);
	}
	
	term +=sizeof(DATA_MOTOR_CURR_INFOR);
	j=0;
	for(;i<term;i++){
		byte[i] = *((uint8_t*)&asMotorCurrInfor+j++);
	}
  //Save user data Struct:
  //ret += Flash_WriteData((uint8_t*)&asDataHwInfor,sizeof(DATA_HW_INFOR),&aAddCurrent);
  //ret += Flash_WriteData((uint8_t*)&asRuningInforSet,sizeof(DATA_RUNING_INFOR),&aAddCurrent);
  //ret += Flash_WriteData((uint8_t*)&asMotorInfor,sizeof(DATA_MOTOR_INFOR),&aAddCurrent);
  //ret += Flash_WriteData((uint8_t*)&asStateMechine,sizeof(STATE_MECHINE),&aAddCurrent);
	
  //Create checksum
  aCrc = CRC8_Generate(0,(uint8_t*)&asDataHwInfor,sizeof(DATA_HW_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asRuningInforSet,sizeof(DATA_RUNING_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asMotorInfor,sizeof(DATA_MOTOR_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asStateMechine,sizeof(STATE_MECHINE));
	aCrc = CRC8_Generate(aCrc,(uint8_t*)&asMotorCurrInfor,sizeof(DATA_MOTOR_CURR_INFOR));
  //Save CRC:
	byte[i] = aCrc;
	ret = Flash_WriteData(byte,i,&aAddCurrent);
  //ret += Flash_WriteData((uint8_t*)&aCrc,1,&aAddCurrent);
  HAL_FLASH_Lock();
  //responst:
  PAYLOAD_Transmit(CMD_SAVE_ALL_INFOR_TO_FLASH|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret;
}
int8_t CMD_SetCommunicaType(void){
	int8_t ret =0;
	if( asPayloadRx.data[0] == CM_PULSE && asStateMechine.asCmType!=CM_PULSE){
		ExternalCounterInit();

		aExternCounterValue = (asRuningInforCurrent.aEstimatesValue*asDataHwInfor.aBaudrate)/6.283185307;
	}
	else if(asPayloadRx.data[0] == CM_RS485 && asStateMechine.asCmType!=CM_RS485){	//Init for network:
		asStateMechine.asLoopControlStatus = CLOOP_STOOP;
	}
	
	asStateMechine.asCmType = asPayloadRx.data[0];
  ret = PAYLOAD_Transmit(CMD_SET_COMUNICA_TYPE|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_GetCommunicaType(void){
	int8_t ret=0;
	ret =PAYLOAD_Transmit(CMD_GET_COMUNICA_TYPE|CMD_RESPONT_MASK,(uint8_t*)&asStateMechine.asCmType,1,TRANSFER_TIME_OUT);
	return ret;
}
int8_t CMD_BootLoaderJum(void){
  
}

/*
typedef enum{
	RS485_CMD_NULL,
	RS485_CMD_POSITION,
	RS485_CMD_VELOCITY,
	RS485_CMD_ACCEL,
	RS485_CMD_RESET,
	RS485_CMD_READ,
	RS485_CMD_ENABLE,
	RS485_CMD_DISABLE,
	RS485_CMD_STOP,
	RS485_CMD_CLEAR_ERR
}RS485_CMD;
*/
RS485_ProcessNewData(RS485_DATA_STRUCT *psRs485Data){
	switch(psRs485Data->aOptCode){
		case RS485_CMD_POSITION:
			asRuningInforSet.aPosition = (double)(*(float*)psRs485Data->aData);
			break;
		case RS485_CMD_VELOCITY:
			asRuningInforSet.aVelocity = (double)(*(float*)psRs485Data->aData);
			break;
		case RS485_CMD_ACCEL:
			asRuningInforSet.aAccel = (double)(*(float*)psRs485Data->aData);
			break;
		case RS485_CMD_RESET:
			HAL_NVIC_SystemReset();
			break;
		case RS485_CMD_READ:
		{
			float position = (float)asRuningInforCurrent.aPosition;
			RS485_Transmit(psRs485Data->aOptCode,psRs485Data->aAddSource,(uint8_t*)&position,4,0xffffff);
			break;
		}
		case RS485_CMD_ENABLE:
		{
			asStateMechine.aStatus = STATUS_RUN;
			ENCODER_Reset();
			asRuningInforSet.aPosition =0;
			asRuningInforCurrent.aEstimatesValue =0;
			break;
		}
		case RS485_CMD_DISABLE:
			asStateMechine.aStatus = STATUS_STOOP;
			break;
		case RS485_CMD_STOP:
			asRuningInforSet.aPosition = asRuningInforCurrent.aEstimatesValue;
			break;
			case RS485_CMD_CLEAR_ERR:
				asRuningInforCurrent.aListError =0;
			break;
		default:
			break;
		
	}
}