

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"



#define DC_TIM	                  	TIM1      

#define HIN1_GPIO_PIN             	GPIO_PIN_8
#define HIN1_GPIO_PORT							GPIOA

#define HIN2_GPIO_PIN             	GPIO_PIN_9
#define HIN2_GPIO_PORT							GPIOA

#define LIN1_GPIO_PIN             	GPIO_PIN_13
#define LIN1_GPIO_PORT							GPIOB

#define LIN2_GPIO_PIN             	GPIO_PIN_14
#define LIN2_GPIO_PORT							GPIOB


#define DC_EN_GPIO_PIN                GPIO_PIN_8
#define DC_EN_GPIO_PORT								GPIOB 
#define DC_ENABLE()										HAL_GPIO_WritePin(DC_EN_GPIO_PORT, DC_EN_GPIO_PIN,1)
#define DC_DISABLE()									HAL_GPIO_WritePin(DC_EN_GPIO_PORT, DC_EN_GPIO_PIN,0)

#define DC_CLK_ENABLE()									__TIM1_CLK_ENABLE(); __GPIOB_CLK_ENABLE();

#define IO_CONFIG1_PIN				GPIO_PIN_5
#define IO_CONFIG1_PORT				GPIOA

//#define USB_DISCONNECT_PIN				GPIO_PIN_12
//#define USB_DISCONNECT_PORT				GPIOB


#define IO_CONFIG2_PIN				GPIO_PIN_5
#define IO_CONFIG2_PORT				GPIOA

#define IO_COFIG_CLK_ENABLE()		__GPIOA_CLK_ENABLE()

#define LED_GPIO_CLK_ENABLE()							__GPIOB_CLK_ENABLE()

#define LED_YELLOW_GPIO_PIN								GPIO_PIN_9
#define LED_RED_GPIO_PIN									GPIO_PIN_11
#define LED_GREEN_GPIO_PIN							  GPIO_PIN_10
#define LED_GPIO_PORT											GPIOB


#define ADC_SOURCE_PIN						GPIO_PIN_0
#define ADC_SOURCE_PORT						GPIOB
#define ADC_SOURCE_CHANNEL			ADC_CHANNEL_8

#define ADC_VELOCITY_PIN						GPIO_PIN_7
#define ADC_VELOCITY_PORT						GPIOA
#define ADC_VELOCITY_CHANNEL				ADC_CHANNEL_7

#define ADC_CURRENT_PIN							GPIO_PIN_4
#define ADC_CURRENT_PORT						GPIOA
#define ADC_CURRENT_CHANNEL					ADC_CHANNEL_4

#define ADC_DMA_CHANNEL             DMA_CHANNEL_0
#define ADC_DMA_STREAM              DMA2_Stream0
#define ADC_RCC_ENABLE()			__ADC1_CLK_ENABLE(); __GPIOA_CLK_ENABLE();__GPIOC_CLK_ENABLE(); __DMA1_CLK_ENABLE();


typedef enum __LED_STATUS {LED_RED_ON,LED_RED_OFF,LED_GREEN_ON,LED_GREEN_OFF,LED_YELLOW,LED_OFF} LED_STATUS; 

int8_t DCM_Init(void);
void DCM_UpdateDuty(uint16_t duty, uint8_t dir);
void LOOP_Start(void);
int8_t LED_StatusConfig(void);
int8_t LED_StatusDislay(int8_t ledStatus);
void ADC_DMA_Config(void);
int8_t ERROR_CurrentCheck();
float READ_VelocitySetByADC(void);
void ADC_Read(void);
void ERROR_Indicate(void);
void PROCESS_Indicate(void);
float SOURCE_Read(void);
void IO_Config(void);
int8_t IO_Read(void);
void Flash_Init(void);