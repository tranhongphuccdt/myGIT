#include <stdint.h>

#define FOLLOW_ERR						0x01
#define PHASE_ENCODER_REVERSE	0x02
#define ENCODER_ERR						0x04
#define PHASE_ERR							0x08
#define CURRENT_ERR						0x10
#define OVER_VOLTAGE_ERR 			0x20
#define SYSTEM_FAIL						0x40

#pragma pack(1)
typedef struct{
	uint8_t aAdd;
	uint16_t aEncoderLine;
	uint32_t aBaudrate;
	uint8_t aEncoderFilter;
	uint8_t aWFilter;
	uint8_t aAcelFilter;
	uint16_t aMaxCurrent;
  float aFollowError;
}DATA_HW_INFOR;

typedef struct{
	double aPosition;       //8 
	double aVelocity;       //16
	double aAccel;          //32
  double aEstimatesValue; //40
	int16_t aCurrent;       //42
	uint32_t aMsTick;       //46
	uint8_t aListError;			//47
	uint8_t aSourceVoltage;	//48
	uint8_t aTermperature;	//49
}DATA_RUNING_INFOR;
typedef struct{
   int8_t aTermperater;
   int8_t aMcuUse;
   
}HEALTH_OF_MCU;

typedef struct{
	float aK1;
	float aK2;
	float aK3;
	float aE;
	float aJ;
	float aB;
  float aWn;
}DATA_MOTOR_INFOR;

typedef enum{
  MODE_TURNING,
  MODE_SF_POSITION,
  MODE_PID_POSITION,
  MODE_PI_VELOCITY,
	MODE_SMART_POSITION,
  MODE_PI_VELOCITY_BY_ADC	
}RUNING_MODE;
typedef enum{
  STAUTS_IDLE,
  STATUS_STOOP,
  STATUS_RUN,
  STATUS_ERR,
	STATUS_ERR_CURRENT,
	STATUS_ERR_SYSTEM
}STATUS;
typedef enum{
  CM_PULSE,
  CM_RS485,
  CM_CAN,
	CM_ANALOG
}COMMUNICA_TYPE;
typedef enum{
    CLOOP_STOOP,
    CLOOP_STARTING,
    CLOOP_PROCESS,
    CLOOP_FINISH,
}CONTROL_LOOP_STATE;
typedef struct{
  RUNING_MODE asRunningMode;
  COMMUNICA_TYPE asCmType;
  STATUS aStatus;
  CONTROL_LOOP_STATE asLoopControlStatus;
}STATE_MECHINE;
// Phien
typedef struct
{
	float aR;
	float aL;
//	float aK;
	float aJ;
	float aB;
	float aWn;
	float aKp_I;
	float aKi_I;
	float aKd_I;
	float aKp_V;
	float aKi_V;
	float aKd_V;
	float aKp_P;
	float aKi_P;
	float aKd_P;
}DATA_MOTOR_CURR_INFOR;

void DATA_Insert(void);

