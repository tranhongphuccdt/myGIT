/**
  ******************************************************************************
  * @file    usbd_SPENCIFIC.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    13-June-2014
  * @brief   header file for the USBD_SPENCIFIC.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/

#ifndef __USB_SPENCIFIC_CORE_H_
#define __USB_SPENCIFIC_CORE_H_

#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_SPENCIFIC
  * @brief This file is the Header file for USBD_msc.c
  * @{
  */ 


/** @defgroup USBD_SPENCIFIC_Exported_Defines
  * @{
  */ 
#define SPENCIFIC_EPIN1_ADDR                 0x81
#define SPENCIFIC_EPIN2_ADDR                 0x82

#define SPENCIFIC_EPOUT1_ADDR                 0x01
#define SPENCIFIC_EPOUT1_MAX_SIZE							64

#define SPENCIFIC_EPOUT2_ADDR                 0x02
#define SPENCIFIC_EPOUT2_MAX_SIZE							64

#define SPENCIFIC_EPIN1_SIZE                 64
#define SPENCIFIC_EPIN2_SIZE                 64
#define SPENCIFIC_EPOUT1_SIZE                64
#define SPENCIFIC_EPOUT2_SIZE                64



#define USB_SPENCIFIC_CONFIG_DESC_SIZ        39

/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
  SPENCIFIC_IDLE = 0,
  SPENCIFIC_BUSY,
}SPENCIFIC_StateTypeDef;
typedef enum
{
	SPENCIFIC_DATA_NONE_RECEIVER =0,
	SPENCIFIC_DATA_RECEIVER,
}SPENCIFIC_RECEIVER_StateTypeDef;

typedef struct
{
  uint32_t             Protocol;   
  uint32_t             IdleState;  
  uint32_t             AltSetting;
	SPENCIFIC_StateTypeDef     state;
  SPENCIFIC_RECEIVER_StateTypeDef stateReceiver;	
}
USBD_SPENCIFIC_HandleTypeDef;

typedef struct _USBD_SPECIFIC_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  

}USBD_SPECIFIC_ItfTypeDef;
/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_SPENCIFIC_ClassDriver;


#define USBD_SPENCIFIC_CLASS    &USBD_SPENCIFIC_ClassDriver
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
	uint8_t USBD_PreparaSendData(USBD_HandleTypeDef  *pdev,uint8_t en_addr, uint8_t *report,uint16_t len);
/**
  * @}
  */ 

#endif  // __USB_SPENCIFIC_CORE_H_
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
