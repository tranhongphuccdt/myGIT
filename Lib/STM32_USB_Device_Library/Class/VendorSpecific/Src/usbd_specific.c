/**
  ******************************************************************************
  * @file    usbd_SPENCIFIC.c
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    13-June-2014
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                SPENCIFIC Class  Description
  *          ===================================================================
  *          
  *
  *
  *
  *           
  *      
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *           
  *      
  *  @endverbatim
  *
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
#include "usbd_specific.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

#include "usbd_cdc.h"

extern volatile uint8_t aaUsbBufferOut[64];

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_SPENCIFIC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_SPENCIFIC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_SPENCIFIC_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_SPENCIFIC_Private_Macros
  * @{
  */ 
                                         
/**
  * @}
  */ 




/** @defgroup USBD_SPENCIFIC_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_SPENCIFIC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_SPENCIFIC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_SPENCIFIC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_SPENCIFIC_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_SPENCIFIC_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_SPENCIFIC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_SPENCIFIC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_SPENCIFIC_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_SPENCIFIC_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_SPENCIFIC_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_SPENCIFIC_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_SPENCIFIC_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

/**
  * @}
  */ 

/** @defgroup USBD_SPENCIFIC_Private_Variables
  * @{
  */ 

USBD_ClassTypeDef  USBD_SPENCIFIC_ClassDriver = 
{
  USBD_SPENCIFIC_Init,
  USBD_SPENCIFIC_DeInit,
  NULL,                    //USBD_SPENCIFIC_Setup,
  NULL,             //USBD_SPENCIFIC_EP0_TxReady,  
  NULL,             //USBD_SPENCIFIC_EP0_RxReady,
  USBD_SPENCIFIC_DataIn,
  USBD_SPENCIFIC_DataOut,
  NULL,                      //USBD_SPENCIFIC_SOF,
  NULL,         //USBD_SPENCIFIC_IsoINIncomplete,
  NULL,        //USBD_SPENCIFIC_IsoOutIncomplete,      
  USBD_SPENCIFIC_GetCfgDesc,
  USBD_SPENCIFIC_GetCfgDesc, 
  USBD_SPENCIFIC_GetCfgDesc,
  USBD_SPENCIFIC_GetDeviceQualifierDesc,
};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
/* USB SPENCIFIC device Configuration Descriptor */
static uint8_t USBD_SPENCIFIC_CfgDesc[USB_SPENCIFIC_CONFIG_DESC_SIZ] =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_SPENCIFIC_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supprts Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  
  /**********  Descriptor of SPENCIFIC interface 0 Alternate setting 0 **************/  
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x03,         /*bNumEndpoints*/
  0xFF,         /*bInterfaceClass: Vender Spencific*/
  0x00,         /*bInterfaceSubClass : None*/
  0x00,         /*nInterfaceProtocol : None*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of Spencific endpoint ********************/
  /* 18 */
  0x07,          /*bLength: Endpoint1 IN Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
  SPENCIFIC_EPIN1_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  USBD_EP_TYPE_BULK,          /*bmAttributes: Bulk endpoint*/
  LOBYTE(SPENCIFIC_EPIN1_SIZE), /*wMaxPacketSize: x Byte max */
  HIBYTE(SPENCIFIC_EPIN1_SIZE),
  0x01,          /*bInterval: Polling Interval (1 ms)*/
  /* 25 */
  0x07,                       /*bLength: Endpoint1 OUT Descriptor size*/
  USB_DESC_TYPE_ENDPOINT,     /*bDescriptorType:*/
  SPENCIFIC_EPOUT1_ADDR,       /*bEndpointAddress: Endpoint Address Out*/
  USBD_EP_TYPE_BULK,                       /*bmAttributes: Interrupt endpoint*/
  SPENCIFIC_EPIN2_SIZE, /*wMaxPacketSize: 64 Byte max */
  0x00,
  0,          /*bInterval: Polling Interval (0 ms)*/
  /* 32 */
  0x07,                       /*bLength: Endpoint1 OUT Descriptor size*/
  USB_DESC_TYPE_ENDPOINT,     /*bDescriptorType:*/
  SPENCIFIC_EPOUT2_ADDR,       /*bEndpointAddress: Endpoint Address (IN)*/
  USBD_EP_TYPE_BULK,                       /*bmAttributes: Bulk endpoint*/
  SPENCIFIC_EPOUT1_SIZE,      /*wMaxPacketSize: 64 Byte max */
  0x00,
  0,          /*bInterval: Polling Interval (0 ms)*/
  /* 39 */
 
};
  
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
/* USB Standard Device Descriptor */
static uint8_t USBD_SPENCIFIC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_SPENCIFIC_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_SPENCIFIC_Init
  *         Initialize the SPENCIFIC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
	
	//Open Endpoint:
  /* Open EP1,2 OUT */
    USBD_LL_OpenEP(pdev,
                   SPENCIFIC_EPOUT1_ADDR,
                   USBD_EP_TYPE_BULK,
									 SPENCIFIC_EPOUT1_SIZE);
	 //EP2 OUT: Use to receiver data:
    USBD_LL_OpenEP(pdev,
                   SPENCIFIC_EPOUT2_ADDR,
                   USBD_EP_TYPE_BULK,
                   SPENCIFIC_EPOUT2_SIZE);
    
    /* Open EP1 IN 1 with Type */
    USBD_LL_OpenEP(pdev,
                   SPENCIFIC_EPIN1_ADDR,
                   USBD_EP_TYPE_BULK,
                   SPENCIFIC_EPIN1_SIZE);  
	
  pdev->pClassData = USBD_malloc(sizeof (USBD_SPENCIFIC_HandleTypeDef));
													 
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
	else
  {   
		
		/* Prepare Out endpoint to receive 1st packet */ 
    USBD_LL_PrepareReceive(pdev,
                           SPENCIFIC_EPOUT1_ADDR,
                           aaUsbBufferOut,                        
                           SPENCIFIC_EPOUT1_MAX_SIZE); 
		USBD_LL_PrepareReceive(pdev,
                           SPENCIFIC_EPOUT2_ADDR,
                           aaUsbBufferOut,                        
                           SPENCIFIC_EPOUT1_MAX_SIZE);  
		
    ((USBD_SPENCIFIC_HandleTypeDef *)pdev->pClassData)->state = SPENCIFIC_IDLE;		//USB bus Free.
  }

  return ret;
}

/**
  * @brief  USBD_SPENCIFIC_Init
  *         DeInitialize the SPENCIFIC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
     /* Close HID EPs */
  USBD_LL_CloseEP(pdev,
                  SPENCIFIC_EPIN1_ADDR);
  USBD_LL_CloseEP(pdev,
                  SPENCIFIC_EPOUT1_ADDR);
  USBD_LL_CloseEP(pdev,
                  SPENCIFIC_EPOUT2_ADDR);
//  
  /* FRee allocated memory */
  if(pdev->pClassData != NULL)
  {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  } 

  return USBD_OK;
}

/**
  * @brief  USBD_SPENCIFIC_Setup
  *         Handle the SPENCIFIC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
 
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :  
    switch (req->bRequest)
    {
      
    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL; 
    }
    break;
    
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    
    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;     
    }
  }
  return USBD_OK;
}


/**
  * @brief  USBD_SPENCIFIC_GetCfgDesc 
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_SPENCIFIC_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_SPENCIFIC_CfgDesc);
  return USBD_SPENCIFIC_CfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_SPENCIFIC_DeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_SPENCIFIC_DeviceQualifierDesc);
  return USBD_SPENCIFIC_DeviceQualifierDesc;
}


/**
  * @brief  USBD_SPENCIFIC_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
	/* Ensure that the FIFO is empty before a new transfer, this condition could 
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_SPENCIFIC_HandleTypeDef *)pdev->pClassData)->state = SPENCIFIC_IDLE;		//USB Transmited package.
  return USBD_OK;
}

/**
  * @brief  USBD_SPENCIFIC_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{

  return USBD_OK;
}
/**
  * @brief  USBD_SPENCIFIC_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_EP0_TxReady (USBD_HandleTypeDef *pdev)
{

  return USBD_OK;
}
/**
  * @brief  USBD_SPENCIFIC_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_SOF (USBD_HandleTypeDef *pdev)
{

  return USBD_OK;
}
/**
  * @brief  USBD_SPENCIFIC_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_SPENCIFIC_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_SPENCIFIC_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_SPENCIFIC_DataOut(USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
	USBD_SPENCIFIC_HandleTypeDef *hspf;
	hspf = pdev->pClassData;
	if( hspf->stateReceiver == SPENCIFIC_DATA_RECEIVER)   // the older package
      return USBD_BUSY;
//      
	hspf->stateReceiver = SPENCIFIC_DATA_RECEIVER;			//USB notify new package.
	/* Prepare Out endpoint to receive new packet */ 
		switch(epnum){
			case SPENCIFIC_EPOUT1_ADDR:		//Prepare to Receive New Data frome endpoint1
				USBD_LL_PrepareReceive(pdev,SPENCIFIC_EPOUT1_ADDR,aaUsbBufferOut,SPENCIFIC_EPOUT1_MAX_SIZE); 
				break;
			case SPENCIFIC_EPOUT2_ADDR:		//Prepare to Receive New Data frome endpoint2
				USBD_LL_PrepareReceive(pdev,SPENCIFIC_EPOUT2_ADDR,aaUsbBufferOut,SPENCIFIC_EPOUT2_MAX_SIZE); 
				break;
			default:
				break;
		}
  return USBD_OK;
}

uint8_t USBD_PreparaSendData(USBD_HandleTypeDef  *pdev,uint8_t en_addr, uint8_t *report,uint16_t len)
{
	
  USBD_SPENCIFIC_HandleTypeDef     *hspf = pdev->pClassData;
  
  if (pdev->dev_state == USBD_STATE_CONFIGURED )
  {
    if(hspf->state == SPENCIFIC_IDLE)
    {
      hspf->state = SPENCIFIC_BUSY;
      USBD_LL_Transmit (pdev, en_addr, report,len);
    }
		else{
			return USBD_BUSY;
		}
  }
  return USBD_OK;
}


/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_SPENCIFIC_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_SPENCIFIC_DeviceQualifierDesc);
  return USBD_SPENCIFIC_DeviceQualifierDesc;
}

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
