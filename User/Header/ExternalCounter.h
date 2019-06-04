
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"
#include <stdio.h>




#define EXT_DIR_PIN	 		  GPIO_PIN_5
#define EXT_DIR_PORT	 		GPIOB

#define EXT_COUNTER_PIN	 		GPIO_PIN_4
#define EXT_COUNTER_PORT	 	GPIOB

#define EXT_COUNTER_TIM			TIM3

//NC
#define EXT_ENABLE_PIN		GPIO_PIN_5
#define EXT_ENABLE_PORT		GPIOA

#define EXT_EN_PIN        GPIO_PIN_6
#define EXT_EN_PORT       GPIOA

#define EXT_COUNTER_CLK()			__GPIOA_CLK_ENABLE();__GPIOB_CLK_ENABLE();  __TIM3_CLK_ENABLE();	
#define EXT_DIR_STATE() HAL_GPIO_ReadPin(EXT_DIR_PORT,EXT_DIR_PIN) 
extern volatile int64_t aExternCounterValue;

void ExternalCounterInit( void);
void ProcessDirCounter(void);
void ReadExternalCounter(void);
void ExternalCounterDeinit(void);
void ProcessExtEnable(void);