// Header:    
// File Name: CloseLoopControl.h
// Author:		LinhTran
// Date:      3/12/2014

#include "stdint.h"


// Phien
#define POS_MODE							1
#define VEL_MODE							0

void DRIVER_Control(void);
void LOOP_AutoTunning(void);
void PI_ControlVelocity(void);
void PI_ControlVelocityByADC(void);
void PID_PositionControl(void);
void PV_eControl(void);
void StateFeedback_PositionControl(void);
double wFilter(double wfilter);
void LoopEnable();
int8_t ERROR_MotorCheck(void);

//Phien
float PI_Velocity_Current_Control(unsigned char mode);
float PID_Position_Current_Control(void);
void Current_Control(unsigned char mode);
void PidPiPi_PositionControl(void);