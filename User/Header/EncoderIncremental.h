//////////////////////////// EncoderIncremental.h //////////////////////////////
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"

#define ENCODER_PIN_CHA				  GPIO_PIN_6
#define ENCODER_PIN_CHB				  GPIO_PIN_7
#define ENCODER_PORT					  GPIOB

#define ENCODER_CONTROL				  TIM4

#define ENCODER_TIM_CLK_ENABLE()				__TIM4_CLK_ENABLE()	
#define ENCODER_GPIO_CLK_ENABLE()				__GPIOB_CLK_ENABLE()

//Global vavariable:
extern volatile int64_t EncoderCounterValue;


//funtion:
int8_t ENCODER_Init(uint8_t filterEncoder);
void ENCODER_Reset(void);
void ENCODER_Read(void);