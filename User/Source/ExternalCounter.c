#include "ExternalCounter.h" 	
extern volatile uint8_t aHaveNewCommand;
volatile int64_t aExternCounterValue;
TIM_HandleTypeDef  TimHandleEXT;

__IO uint32_t            uwIC2Value = 0;
/* Duty Cycle Value */
__IO uint32_t            uwDutyCycle = 0;
/* Frequency Value */
__IO uint32_t            uwFrequency = 0;
GPIO_PinState aOldStateDir;

void ExternalCounterDeinit(void){
	
}
void ExternalCounterInit( void){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ClockConfigTypeDef sClockConfigType;

  EXT_COUNTER_CLK();
  
//Thiet lap I/O Dir
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin =  EXT_DIR_PIN;
  HAL_GPIO_Init(EXT_DIR_PORT, &GPIO_InitStructure);
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

 //Config I/O driver:
	GPIO_InitStructure.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin = EXT_COUNTER_PIN ;
  HAL_GPIO_Init(EXT_COUNTER_PORT, &GPIO_InitStructure);
  
	//CONFIG TIMER FOR EXTERN COUNTER:	
  TimHandleEXT.Instance = EXT_COUNTER_TIM;
  TimHandleEXT.Init.Prescaler = 0;
  TimHandleEXT.Init.Period = 0xffff;
  TimHandleEXT.Init.ClockDivision = 0;
  TimHandleEXT.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&TimHandleEXT);
	
	__HAL_AFIO_REMAP_TIM3_PARTIAL();
	
	sClockConfigType.ClockFilter =15;
	sClockConfigType.ClockPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sClockConfigType.ClockPrescaler = TIM_ICPSC_DIV1;
	sClockConfigType.ClockSource = TIM_CLOCKSOURCE_TI1;
	
	HAL_TIM_ConfigClockSource(&TimHandleEXT,&sClockConfigType);
	HAL_TIM_Base_Start(&TimHandleEXT);
	
	ProcessExtEnable();
	//Reset encoder value:
	aOldStateDir = HAL_GPIO_ReadPin(EXT_DIR_PORT,EXT_DIR_PIN);
	ReadExternalCounter();
	aExternCounterValue=0;
}
void ExternalMeasureFrequencyInit(void){
			/* Timer Input Capture Configuration Structure declaration */
			TIM_IC_InitTypeDef       sConfig;
			/* Slave configuration structure */
			TIM_SlaveConfigTypeDef   sSlaveConfig;
			/* Captured Value */
			GPIO_InitTypeDef GPIO_InitStructure;
			TIM_ClockConfigTypeDef sClockConfigType;

			EXT_COUNTER_CLK();
			
		//Thiet lap I/O Dir
			GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
			GPIO_InitStructure.Pin =   EXT_DIR_PIN;
			HAL_GPIO_Init(EXT_DIR_PORT, &GPIO_InitStructure);
			/* Enable and set EXTI Line9 Interrupt to the lowest priority */
			HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			
//			//EXT_ENABLE_PIN
//			GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
//			GPIO_InitStructure.Pull = GPIO_NOPULL;
//			GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
//			GPIO_InitStructure.Pin =  EXT_ENABLE_PIN;
//			HAL_GPIO_Init(EXT_ENABLE_PORT, &GPIO_InitStructure);
//			/* Enable and set EXTI Line12 Interrupt to the lowest priority */
//			HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
//			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			
			
		 //Config I/O driver:
			GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
			GPIO_InitStructure.Pin = EXT_COUNTER_PIN ;
			HAL_GPIO_Init(EXT_COUNTER_PORT, &GPIO_InitStructure);
			/*##-1- Configure the TIM peripheral #######################################*/
			/* --------------------------------------------------------------------------- 
			TIM2 configuration: PWM Input mode

				In this example TIM2 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
				since APB1 prescaler is different from 1.   
					TIM4CLK = 2 * PCLK1  
					PCLK1 = HCLK / 4 
					=> TIM4CLK = HCLK / 2 = SystemCoreClock /2

				External Signal Frequency = TIM4 counter clock / TIM4_CCR2 in Hz. 

				External Signal DutyCycle = (TIM4_CCR1*100)/(TIM4_CCR2) in %.

			--------------------------------------------------------------------------- */
			
			/* Set TIMx instance */
			TimHandleEXT.Instance = EXT_COUNTER_TIM;
		 
			/* Initialize TIMx peripheral as follow:
					 + Period = 0xFFFF
					 + Prescaler = 0
					 + ClockDivision = 0
					 + Counter direction = Up
			*/
			TimHandleEXT.Init.Period = 0xFFFFFFFF;
			TimHandleEXT.Init.Prescaler = 0;
			TimHandleEXT.Init.ClockDivision = 0;
			TimHandleEXT.Init.CounterMode = TIM_COUNTERMODE_UP;  
			if(HAL_TIM_IC_Init(&TimHandleEXT) != HAL_OK)
			{
				/* Initialization Error */
				//Error_Handler();
			}
			
			/*##-2- Configure the Input Capture channels ###############################*/ 
			/* Common configuration */
			sConfig.ICPrescaler = TIM_ICPSC_DIV1;
			sConfig.ICFilter = 0;  
			
			/* Configure the Input Capture of channel 1 */
			sConfig.ICPolarity = TIM_ICPOLARITY_FALLING;
			sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;    
			if(HAL_TIM_IC_ConfigChannel(&TimHandleEXT, &sConfig, TIM_CHANNEL_1) != HAL_OK)
			{
				/* Configuration Error */
				//Error_Handler();
			}
			
			/* Configure the Input Capture of channel 2 */
			sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
			sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
			if(HAL_TIM_IC_ConfigChannel(&TimHandleEXT, &sConfig, TIM_CHANNEL_2) != HAL_OK)
			{
				/* Configuration Error */
				//Error_Handler();
			}
			/*##-3- Configure the slave mode ###########################################*/
			/* Select the slave Mode: Reset Mode  */
			sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_RESET;
			sSlaveConfig.InputTrigger  = TIM_TS_TI2FP2;
			if(HAL_TIM_SlaveConfigSynchronization(&TimHandleEXT, &sSlaveConfig) != HAL_OK)
			{
				/* Configuration Error */
				//Error_Handler();
			}
			
			/*##-4- Start the Input Capture in interrupt mode ##########################*/
			if(HAL_TIM_IC_Start_IT(&TimHandleEXT, TIM_CHANNEL_2) != HAL_OK)
			{
				/* Starting Error */
				//Error_Handler();
			}
			
			/*##-5- Start the Input Capture in interrupt mode ##########################*/
			if(HAL_TIM_IC_Start_IT(&TimHandleEXT, TIM_CHANNEL_1) != HAL_OK)
			{
				/* Starting Error */
				//Error_Handler();
			}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* Get the Input Capture value */
    uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    
    if (uwIC2Value != 0)
    {
      /* Duty cycle computation */
      uwDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;
      
      /* uwFrequency computation
      TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */      
      uwFrequency = (HAL_RCC_GetHCLKFreq())/2 / uwIC2Value;
    }
    else
    {
      uwDutyCycle = 0;
      uwFrequency = 0;
    }
  }
}
void ProcessExtEnable(void){
	__HAL_GPIO_EXTI_CLEAR_IT(EXT_ENABLE_PIN);

		if(HAL_GPIO_ReadPin(EXT_ENABLE_PORT,EXT_ENABLE_PIN)){
			HAL_TIM_Base_Start(&TimHandleEXT);
		}
		else{
			HAL_TIM_Base_Stop(&TimHandleEXT);
		}
}
void ReadExternalFrequency(void){
	static uint64_t aCountRead=0;
	//as
}
void ReadExternalCounter(void){
	static int16_t leftExtCounter,oldExtCounter;
  int16_t leftCount ;

	oldExtCounter =  leftExtCounter;
	leftExtCounter = EXT_COUNTER_TIM->CNT;
	leftCount =	(leftExtCounter - oldExtCounter) ;
	
	if(leftCount!=0){
		aHaveNewCommand = 1;
	}
	
  if(aOldStateDir){
      aExternCounterValue+= leftCount;
   }
   else{
			aExternCounterValue-= leftCount;
  }	 
}


void ProcessDirCounter(void){
 
/* EXTI line interrupt detected */
	ReadExternalCounter();

  __HAL_GPIO_EXTI_CLEAR_IT(EXT_DIR_PIN);   
	aOldStateDir = HAL_GPIO_ReadPin(EXT_DIR_PORT,EXT_DIR_PIN);
}