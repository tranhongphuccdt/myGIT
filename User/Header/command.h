// Header:
// File Name: 
// Author:		LinhTran
// Date:			2/11/14
#include <stdint.h>
#include "transfer.h"


#define CMD_RESPONT_MASK							0x80
//difine cmd:
#define CMD_STRING_MESSAGE 						0x00
#define CMD_RESET_MCU 								0x01
#define	CMD_READ_HWFWPR_VERSION 			0x02
#define CMD_WRITE_FWPR_VERSION        0x03
#define CMD_SET_HW_INFOR							0x04
#define CMD_GET_HW_INFOR							0x05
#define CMD_SET_CONTROL_LOOP_MODE					0x06
#define CMD_GET_CONTROL_LOOP_MODE					   0x07
#define CMD_SET_RUNNING_INFOR					   0x08
#define CMD_GET_RUNNING_INFOR_SET					   0x09
#define CMD_SET_MOTOR_INFOR						0x0A
#define	CMD_GET_MOTOR_INFOR						0x0B
#define CMD_TURN_ON_LOOP_CONTROL			      0x0C
#define CMD_TURN_OFF_LOOP_CONTROL			   0x0E
#define CMD_SAVE_ALL_INFOR_TO_FLASH		      0x0F
#define CMD_SET_COMUNICA_TYPE							0x10
#define CMD_GET_COMUNICA_TYPE						0x11
#define CMD_BOOT_LOADER_JUM						0x12

#define CMD_READ_DATA_IN_NETWORK             0x13
#define CMD_WRITE_DATA_OUT_NETWORK           0x14
#define CMD_AUTO_TUNING								0x15
#define CMD_GET_ON_OFF_LOOP_CONTROL			   0x16
#define CMD_CLEAR_ERROR										0x17
#define CMD_GET_SOURCE_VOLTAGE										0x18
#define CMD_SET_MOTOR_CURR_INFOR					0x19
#define CMD_GET_SMART_INFOR								0x20

#define CMD_END_OF_COMMAND                   0x21


void COMMAND_FuntionAssign(void);
int8_t COMMAND_Process(PAYLOAD_TYPE *psPayload);
int8_t TURNING_Respont(void);



