
#include "stdio.h"
#include "string.h"
#include "stdint.h"

#include "command.h"
#include "product_infor.h"
#include "data_struct.h"
#include "stm32f1xx_hal_flash.h"
#include "crc.h"
#include "delay.h"

//extern uint8_t vituarlComTransmit(int8_t*pData, uint32_t size);
//extern void CDC_GetNewPack(void);
/*
Command format: <command1> + [data1] + <command2> + [data2] + ... + <#>

Command: 
	<P>: Position
	<V>: Velocity
	<A>: Accel
	<R>: Reset Mcu
	<r>: Reset potition
	<G>: Read position.
	<K>: ACK
	<E>: Enable
	<D>: Disable
	<S>: Stop
	<C>: Clear Error.
*/
extern int64_t aExternCounterValue;
extern PAYLOAD_TYPE  asPayloadTx;
extern PAYLOAD_TYPE  asPayloadRx;

extern STATE_MECHINE asStateMechine;
extern DATA_HW_INFOR asDataHwInfor;
extern DATA_RUNING_INFOR asRuningInforCurrent,asRuningInforSet;
extern DATA_MOTOR_INFOR asMotorInfor;
extern void ENCODER_Reset(void);
int32_t timeUpdateToHost =0;

void STRING_Process(int8_t *pData){ //data format: #_P/V/A ; vd: #adressP200V400A500 adress: adress of board, 200 position, V: velocity, .
	uint8_t *pTerm,*pTerm1;
	uint8_t failCmd =0;	double aTerm; uint8_t len;
	len = strlen(pData);
	
	pTerm = memchr(pData,'#',len);
	if(pTerm!=NULL){//adress:
		if(asDataHwInfor.aAdd != atoi(pTerm+1)){
			return;
		}
	}
	else{
		return;
	}

	//Good CMD format :
	if(pTerm!=NULL){
			//P,V,A:	
				if(asStateMechine.asLoopControlStatus != CLOOP_PROCESS)
					{
						int8_t rememberF =0;
								pTerm = memchr(pData,'P',len);
								if(pTerm!=NULL){
									pTerm+=1;
									asRuningInforSet.aPosition = strtod(pTerm,&pTerm1);
									rememberF =1;
								}
								/////////
								pTerm = memchr(pData,'V',len);
								if(pTerm!=NULL){
									pTerm+=1;
									asRuningInforSet.aVelocity = strtod(pTerm,&pTerm1);
									rememberF =1;
								}
								////////////
								pTerm = memchr(pData,'A',len);
								if(pTerm!=NULL){
									pTerm+=1;
									asRuningInforSet.aAccel = strtod(pTerm,&pTerm1);
									rememberF =1;
								}
								
								if(rememberF){
										asStateMechine.asLoopControlStatus =CLOOP_STARTING;
								}
				}

				pTerm = memchr(pData,'G',64);
				if(pTerm!=NULL){
					pTerm+=1;
					timeUpdateToHost = atoi(pTerm);
					
				}
				if(memchr(pData,'E',len)!=NULL){//Enable command:
					asStateMechine.aStatus = STATUS_RUN;
					ENCODER_Reset();
					asRuningInforSet.aPosition =0;
					asRuningInforCurrent.aEstimatesValue =0;
				}
				if(memchr(pData,'D',len)!=NULL){//Disable command:
					asStateMechine.aStatus = STATUS_STOOP;
				}
				if(memchr(pData,'S',len)!=NULL){//Stop command:
					asRuningInforSet.aPosition = asRuningInforCurrent.aEstimatesValue;
				}
				if(memchr(pData,'C',len)!=NULL){//Clear error command:
					asRuningInforCurrent.aListError =0;
				}
				if(memchr(pData,'R',len)!=NULL){//Reset command:
					
					HAL_NVIC_SystemReset(); 
				}
				if(memchr(pData,'r',len)!=NULL){//Reset potition:
					asRuningInforCurrent.aPosition =0;
				 
				}
	}
	//CDC_GetNewPack();
}