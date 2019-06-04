///////////////////// Encoder Incremental /////////////////////////


#include "EncoderIncremental.h"
#include "status.h"


//Global vavariable:
volatile int64_t EncoderCounterValue;


int8_t ENCODER_Init(uint8_t filterEncoder){
		/*  Declaration */
	TIM_HandleTypeDef    TimHandle;
	GPIO_InitTypeDef 		 GPIO_InitStruct;
	TIM_Encoder_InitTypeDef TIM_EncoderStruct;
	
	//Config clk for module:
	ENCODER_GPIO_CLK_ENABLE();
	ENCODER_TIM_CLK_ENABLE();
	//Config I/O driver:
	GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = ENCODER_PIN_CHA | ENCODER_PIN_CHB;
  HAL_GPIO_Init(ENCODER_PORT, &GPIO_InitStruct);
	//CONFIG TIMER FOR EXTERN ENCODER:	
	TimHandle.Instance = ENCODER_CONTROL;
  TimHandle.Init.Prescaler = 0;
  TimHandle.Init.Period = 0xffff;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	//TimHandle.Init.
	HAL_TIM_Base_Init(&TimHandle);

	HAL_TIM_Encoder_DeInit(&TimHandle);
	TIM_EncoderStruct.EncoderMode = TIM_ENCODERMODE_TI12;
	TIM_EncoderStruct.IC1Filter = 15-filterEncoder;
	TIM_EncoderStruct.IC2Filter = 15-filterEncoder;
	TIM_EncoderStruct.IC1Prescaler =0;
	TIM_EncoderStruct.IC2Prescaler =0;
	TIM_EncoderStruct.IC1Polarity = TIM_ICPOLARITY_RISING;
	TIM_EncoderStruct.IC2Polarity = TIM_ICPOLARITY_RISING;
	ENCODER_Reset();
	
	HAL_TIM_Encoder_Init(&TimHandle,&TIM_EncoderStruct);
	HAL_TIM_Encoder_Start(&TimHandle,TIM_CHANNEL_ALL);
	
	ENCODER_Reset();
	
}
void ENCODER_Reset(void){
	__disable_irq();

	ENCODER_CONTROL->CNT =0;
	ENCODER_Read();
	EncoderCounterValue =0;
	__enable_irq();

}
void ENCODER_Read(void){
   static int16_t leftEncoder,oldLeftEncoder;
   int16_t leftCount;
    
   oldLeftEncoder = leftEncoder;
   leftEncoder   = ENCODER_CONTROL->CNT;
   leftCount = leftEncoder - oldLeftEncoder;
   EncoderCounterValue+=	 leftCount;
}