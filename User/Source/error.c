// Header:
// File Name: 	error.c
// Author:        	LinhTran
// Mail:		linhtran.ccs@gmail.com
// Company:	Cc-Smart.net
// Date:		/2014


#include "data_struct.h"
#include <stdint.h>
#include <math.h>
#include "CloseLoopControl.h"
#include "board.h"
#include "EncoderIncremental.h"
#include "transfer.h"

extern int64_t aExternCounterValue;
extern PAYLOAD_TYPE  asPayloadTx;
extern PAYLOAD_TYPE  asPayloadRx;

extern STATE_MECHINE asStateMechine;
extern DATA_HW_INFOR asDataHwInfor;
extern DATA_RUNING_INFOR asRuningInforCurrent,asRuningInforSet;
extern DATA_MOTOR_INFOR asMotorInfor;

int8_t ERROR_PhaseCheck(void){


}
