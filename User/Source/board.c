
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"
#include "board.h"
#include "data_struct.h"
#include <stdint.h>
#include "product_infor.h"
#include "crc.h"

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef htimLoop;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;
volatile int16_t aCurrentMotor;
//volatile int16_t aVelocityAdcSet;

volatile uint8_t aHaveNewCommand=0;

volatile float adcTerm;

__IO uint16_t aaADC_Buffer[20];
ADC_HandleTypeDef    		AdcHandle;
	
extern DATA_HW_INFOR asDataHwInfor;
extern DATA_RUNING_INFOR asRuningInforCurrent,asRuningInforSet;
extern STATE_MECHINE asStateMechine;
extern DATA_MOTOR_INFOR asMotorInfor;
extern DATA_MOTOR_CURR_INFOR asMotorCurrInfor;

void SORT_Tab(uint16_t tab[], uint8_t lenght);

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

//  if(htim_base->Instance==TIM10)
//  {
//  /* USER CODE BEGIN TIM10_MspInit 0 */

//  /* USER CODE END TIM10_MspInit 0 */
//    /* Peripheral clock enable */
//    __TIM10_CLK_ENABLE();
//  /* System interrupt init*/
//    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 3, 0);
//    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
//  /* USER CODE BEGIN TIM10_MspInit 1 */

//  /* USER CODE END TIM10_MspInit 1 */
//  }

}
void IO_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	IO_COFIG_CLK_ENABLE();
	
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin =  IO_CONFIG1_PIN|IO_CONFIG2_PIN;
  HAL_GPIO_Init(IO_CONFIG1_PORT, &GPIO_InitStructure);
	
}
int8_t IO_Read(void){
	int8_t ret;
	ret = HAL_GPIO_ReadPin(IO_CONFIG1_PORT,IO_CONFIG1_PIN);
	ret |= (HAL_GPIO_ReadPin(IO_CONFIG2_PORT,IO_CONFIG2_PIN)<<1);
	
	return ret;
}
/* TIM10 init function */
void LOOP_Start(void)
{//Config timer generate with f=5Khz
	
//  htimLoop.Instance = TIM10;
//  htimLoop.Init.Prescaler = 0;
//  htimLoop.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htimLoop.Init.Period = 12000;
//  htimLoop.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  HAL_TIM_Base_Init(&htimLoop);
//	HAL_TIM_Base_Start_IT(&htimLoop);
}
//Pwm control Dc Motor:
int8_t DCM_Init(void){
	
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	//Config GPIO: 
	GPIO_InitTypeDef   GPIO_InitStruct;
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	
	DC_CLK_ENABLE();	
  /* Common configuration for all channels */
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	//BLDC HIN1,2,3:
	GPIO_InitStruct.Pin = HIN1_GPIO_PIN;
  HAL_GPIO_Init(HIN1_GPIO_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = HIN2_GPIO_PIN;
  HAL_GPIO_Init(HIN2_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LIN1_GPIO_PIN;
  HAL_GPIO_Init(LIN1_GPIO_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = LIN2_GPIO_PIN;
  HAL_GPIO_Init(LIN2_GPIO_PORT, &GPIO_InitStruct);

	
//  //REMAP IO:
	__HAL_AFIO_REMAP_TIM1_DISABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

	//DC_EN:
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	
	GPIO_InitStruct.Pin = DC_EN_GPIO_PIN;
  HAL_GPIO_Init(DC_EN_GPIO_PORT, &GPIO_InitStruct);

	
  /* -----------------------------------------------------------------------
  TIM3 Configuration: generate 1 PWM signals at channel 3.
    
    In this example TIM1 input clock (TIM3CLK) is set to 2APB1 clock (PCLK2), 
    since APB1 prescaler is different from 1.   
      TIM1CLK = PCLK2  
    
      => TIM1CLK =72Mhz.
       
			To get TIM1 counter clock at 36MHz , the prescaler is computed as follows: 
			Prescaler = (TIM1CLK / TIM1 counter clock) - 1 = 72 000 000 / 36 000 000 -1= 1 ;

    To get TIM1 output clock at 36 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM1 counter clock / TIM1 output clock) - 1
           =  36 000 000 / 36 000 - 1 = 999;             
   
    TIM1 Channelx duty cycle = (TIM1_CCRx/ TIM1_ARR + 1)* 100 = 25% 
  ----------------------------------------------------------------------- */ 
  
  /* Initialize TIMx peripheral as follow:
       + Prescaler = (SystemCoreClock/2)/21000000
       + Period = 999
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Instance = DC_TIM;
  
  TimHandle.Init.Prescaler = 1;
  TimHandle.Init.Period = 999;
  TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);
	
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&TimHandle, &sClockSourceConfig);
	
	if(HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
		return 0;
  }
	
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);
	
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = 20;  //18
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&TimHandle, &sBreakDeadTimeConfig);
	
  
  /*##-2- Configure the PWM channels #########################################*/ 
  /* Common configuration for all channels */
	
	sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
  /* Set the pulse value for channel 3 */
  sConfig.Pulse = 0;
	
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2);

	
  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_2);

  
	return 1;
}

void DCM_UpdateDuty(uint16_t duty, uint8_t dir){
	static uint8_t dirRemember;

	if(dir){
		
		//Force Hide side ch2 Off:
			TIM_CCxChannelCmd(DC_TIM, TIM_CHANNEL_2, TIM_CCx_DISABLE);
		// Low side FET CH2: Forced ON  
			HAL_TIM_SelectOCxM(DC_TIM,TIM_CHANNEL_2,TIM_OCMODE_FORCED_ACTIVE);	
			TIM_CCxNChannelCmd(DC_TIM, TIM_CHANNEL_2, TIM_CCxN_ENABLE);
		//CH1:
			HAL_TIM_SelectOCxM(DC_TIM,TIM_CHANNEL_1,TIM_OCMODE_PWM1);
			TIM_CCxChannelCmd(DC_TIM, TIM_CHANNEL_1, TIM_CCx_ENABLE);
			TIM_CCxNChannelCmd(DC_TIM, TIM_CHANNEL_1, TIM_CCxN_DISABLE);

		
	}
	else{
		//Force Hide side ch1 Off:
			TIM_CCxChannelCmd(DC_TIM, TIM_CHANNEL_1, TIM_CCx_DISABLE);
		// Low side FET CH1: Forced ON  
			HAL_TIM_SelectOCxM(DC_TIM,TIM_CHANNEL_1,TIM_OCMODE_FORCED_ACTIVE);	
			TIM_CCxNChannelCmd(DC_TIM, TIM_CHANNEL_1, TIM_CCxN_ENABLE);
		//CH2:
			HAL_TIM_SelectOCxM(DC_TIM,TIM_CHANNEL_2,TIM_OCMODE_PWM1);
			TIM_CCxChannelCmd(DC_TIM, TIM_CHANNEL_2, TIM_CCx_ENABLE);
			TIM_CCxNChannelCmd(DC_TIM, TIM_CHANNEL_2, TIM_CCxN_DISABLE);
	}
	
	DC_TIM->CCR1 =  duty;
	DC_TIM->CCR2 =  duty;
}

//Led Status:
int8_t LED_StatusConfig(void){
	
	//Config GPIO: 
	GPIO_InitTypeDef   GPIO_InitStruct;
	
	LED_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Pin = LED_RED_GPIO_PIN | LED_GREEN_GPIO_PIN | LED_YELLOW_GPIO_PIN;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
  
	LED_StatusDislay(0xff);
	return 1;
}

int8_t LED_StatusDislay(int8_t ledStatus){
	
	
	switch(ledStatus){
		case LED_GREEN_ON:
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_GREEN_GPIO_PIN,0);
				break;
		case LED_RED_ON:
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_RED_GPIO_PIN,0);
				break;
		case LED_GREEN_OFF:
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_GREEN_GPIO_PIN,1);
				break;
		case LED_RED_OFF:
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_RED_GPIO_PIN,1);
				break;
		case LED_YELLOW:
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_GREEN_GPIO_PIN,1);
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_RED_GPIO_PIN,1);
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_YELLOW_GPIO_PIN,0);
				break;
		
		default:
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_GREEN_GPIO_PIN,1);
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_RED_GPIO_PIN,1);
				HAL_GPIO_WritePin(LED_GPIO_PORT,LED_YELLOW_GPIO_PIN,1);
				break;	
	}
}
int8_t APP_LoadConfig(void){
  int8_t ret=0;
  uint8_t aCrc =0;
  uint32_t aAddCurrent =USER_DATA_BASE_FLASH_ADDR;
  
  asDataHwInfor = *(DATA_HW_INFOR*)aAddCurrent;
  aAddCurrent +=sizeof(DATA_HW_INFOR);
  asRuningInforSet = *(DATA_RUNING_INFOR*)(aAddCurrent);
  aAddCurrent +=sizeof(DATA_RUNING_INFOR);
  asMotorInfor = *(DATA_MOTOR_INFOR*)(aAddCurrent);
  aAddCurrent +=sizeof(DATA_MOTOR_INFOR);
  asStateMechine = *(STATE_MECHINE*)(aAddCurrent);
  aAddCurrent +=(sizeof(STATE_MECHINE));
	
	asMotorCurrInfor = *(DATA_MOTOR_CURR_INFOR*)(aAddCurrent);
	aAddCurrent +=(sizeof(DATA_MOTOR_CURR_INFOR));
  //load CRC:
  aCrc = CRC8_Generate(0,(uint8_t*)&asDataHwInfor,sizeof(DATA_HW_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asRuningInforSet,sizeof(DATA_RUNING_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asMotorInfor,sizeof(DATA_MOTOR_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asStateMechine,sizeof(STATE_MECHINE));
	aCrc = CRC8_Generate(aCrc,(uint8_t*)&asMotorCurrInfor,sizeof(DATA_MOTOR_CURR_INFOR));
  
  if(aCrc ==  *(uint8_t*)(aAddCurrent)){
  //reset some state.
    asRuningInforSet.aMsTick =0;
    asRuningInforSet.aPosition =0.0; 
		if(asStateMechine.asCmType == CM_PULSE){
				asStateMechine.asLoopControlStatus = CLOOP_STARTING;
		}
  }
  else{
    ret =1;
  }
  return ret;
}
extern int8_t MOTOR_ControlDir(void){

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc){
	GPIO_InitTypeDef          GPIO_InitStruct;
  static DMA_HandleTypeDef  hdma_adc;
	RCC_PeriphCLKInitTypeDef  PeriphClkInit;

	
	
  /* Configure ADCx clock prescaler */
  /* Caution: On STM32F1, ADC clock frequency max is 14MHz (refer to device   */
  /*          datasheet).                                                     */
  /*          Therefore, ADC clock prescaler must be configured in function   */
  /*          of ADC clock source frequency to remain below this maximum      */
  /*          frequency.                                                      */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /*##-2- Configure peripheral GPIO ##########################################*/
	
  GPIO_InitStruct.Pin = ADC_SOURCE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_SOURCE_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = ADC_VELOCITY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_VELOCITY_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = ADC_CURRENT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_CURRENT_PORT, &GPIO_InitStruct);

  /*##-3- Configure the DMA streams ##########################################*/
	hdma_adc.Instance = DMA1_Channel1;

  hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc.Init.MemDataAlignment = 		DMA_MDATAALIGN_HALFWORD;
  hdma_adc.Init.Mode = DMA_CIRCULAR;
  hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;

	HAL_DMA_DeInit(&hdma_adc);
  HAL_DMA_Init(&hdma_adc);

  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt */
  //HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
 // HAL_NVIC_EnableIRQ(ADC_IRQn);
 }
void Error_Handler(){
	
}
void ADC_DMA_Config(void){
	int i;
  ADC_ChannelConfTypeDef 	sConfig;

	ADC_RCC_ENABLE();
	
	AdcHandle.Instance = ADC1;	

	AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;
  AdcHandle.Init.ContinuousConvMode = DISABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion = 0;

  AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion = 16;

	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }
	/*##-2- Configure ADC2 regular channel #####################################*/
  sConfig.Channel = ADC_VELOCITY_CHANNEL;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
//  sConfig.Offset = 0; 
	for(i=1;i<=5;i++){
		sConfig.Rank = i;
		if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK  )
		{
			Error_Handler();
		}
	}
	/*##-2- Configure ADC2 regular channel #####################################*/
  sConfig.Channel = ADC_CURRENT_CHANNEL;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  for(i=6;i<=10;i++){
		sConfig.Rank = i;
		if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK  )
		{
			Error_Handler();
		}
	}
	sConfig.Channel = ADC_SOURCE_CHANNEL;
  for(i=11;i<=15;i++){
		sConfig.Rank = i;
		if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK  )
		{
			Error_Handler();
		}
	}
	//Termperater:
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 16;
	if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK  )
	{
		Error_Handler();
	}
	
//	 
//  /* Run the ADC calibration */  
//  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
//  {
//    /* Calibration Error */
//    Error_Handler();
//  }
	
	HAL_ADC_Start_DMA(&AdcHandle,(uint32_t *)aaADC_Buffer,16);
	
}
float SOURCE_Read(void){
	static uint16_t flagRememberCurrent=0 ;
	static uint8_t aFlagFirstCall=0;
	static float adcF=0,termperF=0;
	int16_t adcValue;
	
	//SORT_Tab(&aaADC_Buffer[10],5);
	adcValue = (aaADC_Buffer[11]+aaADC_Buffer[12]  +aaADC_Buffer[13])/3;	
	
	if(aFlagFirstCall==0){
		aFlagFirstCall=1;
		adcF =  adcValue;
		termperF = aaADC_Buffer[15];
	}
	else{
		//Low Pass filter: //wF = wF + freFilter*dt*(wCurrent-wF); 
		adcF += 0.0005*((float)adcValue - adcF );		  //f filter =0.1Hz. dt=0.005
		termperF+= 0.0005*((float)aaADC_Buffer[15]-termperF);
	}
	
	asRuningInforCurrent.aSourceVoltage = adcF*0.014234+1;		//(adcF/4096)*3.3*53/3;
	asRuningInforCurrent.aTermperature  = termperF;
}
void  ADC_Read(void){
	HAL_ADC_Start_DMA(&AdcHandle,(uint32_t *)aaADC_Buffer,16);	
}
 /* @brief Sort the N ADC samples
* @param ADC samples to be sorted
* @param Numbre of ADC samples to be sorted
* @retval None
*/
void SORT_Tab(uint16_t tab[], uint8_t lenght){
	uint8_t l=0x00, exchange =0x01;
	uint16_t tmp=0x00;
	/* Sort tab */
	while(exchange==1)
	{
			exchange=0;
			for(l=0; l<lenght-1; l++)
			{
					if( tab[l] > tab[l+1] )
					{
						tmp = tab[l];
						tab[l] = tab[l+1];
						tab[l+1] = tmp;
						exchange=1;
					}
			}
	}
}
float READ_VelocitySetByADC(void){
	static float adcF=0;
	uint16_t adcValue;
	
	//SORT_Tab(&aaADC_Buffer[0],5);
	adcValue =  (aaADC_Buffer[2]+aaADC_Buffer[3]+aaADC_Buffer[4])/3;
	
	//Low Pass filter: //wF = wF + freFilter*dt*(wCurrent-wF); 
	adcF += 0.01*(adcValue - adcF );		  //f filter =10Hz. dt=0.001
	return adcF;
}

void PROCESS_Indicate(void){
	static uint16_t aRememberIndicate=0;
	static int8_t aOn,aOff;
	
	if(asRuningInforCurrent.aListError!=0){
		LED_StatusDislay(LED_GREEN_OFF);
		return;
	}
	
	if(asStateMechine.asCmType == CM_PULSE){
			if(aHaveNewCommand ==1 && aOn!=1 && aOff!=1 && asRuningInforCurrent.aListError==0){
				aOn=1;
				aOff=1;
				aHaveNewCommand =0;
			}
			else if(asRuningInforCurrent.aListError==0){
				LED_StatusDislay(LED_GREEN_ON);
			}
			else{
				LED_StatusDislay(LED_GREEN_OFF);
			}
			
			if(aOn || aOff){
				if(aRememberIndicate++%2 ==0){
					LED_StatusDislay(LED_GREEN_ON);
					aOn =0;
				}
				else{
					LED_StatusDislay(LED_GREEN_OFF);
					aOff =0;
				}
			}
	}
	else{
		if(asStateMechine.asLoopControlStatus == CLOOP_PROCESS){
			if(aRememberIndicate++%2 ==0){
					LED_StatusDislay(LED_GREEN_ON);
			}
			else{
					LED_StatusDislay(LED_GREEN_OFF);
			}
		}
		else if(asRuningInforCurrent.aListError==0){
			LED_StatusDislay(LED_GREEN_ON);
		}
		else{
			LED_StatusDislay(LED_GREEN_OFF);
		}
	}
}
void ERROR_Indicate(void){
	static int aSline=0;
	static int8_t aTatalCount= 15;
	
		switch(asRuningInforCurrent.aListError){
				case FOLLOW_ERR:
					if(aSline<2){
						if(aSline%2==0){
								LED_StatusDislay(LED_RED_ON); 
						}
						else{
								LED_StatusDislay(LED_RED_OFF); 
						}
					}
					else{
						LED_StatusDislay(LED_RED_OFF); 
					}
					break;
				case PHASE_ENCODER_REVERSE:
					if(aSline<4){
						if(aSline%2==0){
								LED_StatusDislay(LED_RED_ON); 
						}
						else{
								LED_StatusDislay(LED_RED_OFF); 
						}
					}
					else{
						LED_StatusDislay(LED_RED_OFF); 
					}
					break;
				case ENCODER_ERR:
					if(aSline<6){
						if(aSline%2==0){
								LED_StatusDislay(LED_RED_ON); 
						}
						else{
								LED_StatusDislay(LED_RED_OFF); 
						}
					}
					else{
						LED_StatusDislay(LED_RED_OFF); 
					}
					break;
				case PHASE_ERR:
					if(aSline<8){
						if(aSline%2==0){
								LED_StatusDislay(LED_RED_ON); 
						}
						else{
								LED_StatusDislay(LED_RED_OFF); 
						}
					}
					else{
						LED_StatusDislay(LED_RED_OFF); 
					}
					break;
				case CURRENT_ERR:
					if(aSline<10){
						if(aSline%2==0){
								LED_StatusDislay(LED_RED_ON); 
						}
						else{
								LED_StatusDislay(LED_RED_OFF); 
						}
					}
					else{
						LED_StatusDislay(LED_RED_OFF); 
					}
					break;
				case OVER_VOLTAGE_ERR:
					if(aSline<12){
						if(aSline%2==0){
								LED_StatusDislay(LED_RED_ON); 
						}
						else{
								LED_StatusDislay(LED_RED_OFF); 
						}
					}
					else{
						LED_StatusDislay(LED_RED_OFF); 
					}
					break;
				default:
					break;
	}
	
	if(aSline++>aTatalCount){
		aSline=0;
	}
}
void Flash_Init(void)
{
  FLASH_OBProgramInitTypeDef Optbyte;   
  
  HAL_FLASHEx_OBGetConfig(&Optbyte);   // read out RDPLvL
  
	//if(Optbyte.RDPLevel
	
  if(Optbyte.RDPLevel ==RESET){        // Lvl 0 = 0, Lvl 1,2 =1               
    Optbyte.OptionType=OPTIONBYTE_RDP; // select RDP optionbyte
    Optbyte.RDPLevel=OB_RDP_LEVEL_0;   // select RDP level 1
    HAL_FLASH_Unlock();                // unlock Flash        
    HAL_FLASH_OB_Unlock();             // unlock Optionbytes
    HAL_FLASHEx_OBProgram(&Optbyte);   // set RDP=1
    HAL_FLASH_OB_Launch();             // write OB to Flash and reset
  }
  HAL_FLASH_OB_Lock();                  // Lock Optionbytes
  HAL_FLASH_Lock();                     // lock Flash  
} 

int8_t ERROR_CurrentCheck(){
	static uint16_t flagRememberCurrent=0 ;
	static uint16_t aCountCall=0;
	static float adcF=0;
	static int16_t aCurrentOffset=0;
	uint16_t adcValue;
	
	SORT_Tab(&aaADC_Buffer[5],5);
	adcValue = (aaADC_Buffer[6]+aaADC_Buffer[7]  +aaADC_Buffer[8])/3;	
	
	if(aCountCall==0){
		aCountCall=1;
		adcF =  adcValue;
	}
	else{
		//Low Pass filter: //wF = wF + freFilter*dt*(wCurrent-wF); 
		adcF += 20*0.001*((float)adcValue - adcF );		  //f filter =20Hz. dt=0.001
	}
//	//Offset sensor
//	if(aCountCall<10000){
//		aCountCall+=1;
//	}
//	else if(aCountCall==10000){
//		aCountCall+=1;
//		aCurrentOffset = (int16_t)((2500 -adcF*0.805664 )*10) ;//mA
//	}
//	adcTerm = (adcValue/4096.0)*3314;
	asRuningInforCurrent.aCurrent = (((adcF/4096.0)*3.313)/16/0.1)*1000 -200 ; //  (int16_t)((2500 -adcF*0.805664 )*10)   ;//- aCurrentOffset;//mA
	 
	if(asRuningInforCurrent.aCurrent  > asDataHwInfor.aMaxCurrent){
		flagRememberCurrent++;
		if(flagRememberCurrent>50){ //5mS
			flagRememberCurrent =0 ;
			asRuningInforCurrent.aListError |= CURRENT_ERR;
			return 1;
		}
	}
	else{
		flagRememberCurrent =0;
		return 0;
	}

}

