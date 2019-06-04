// Header:
// File Name: 
// Author:		LinhTran
// Date:

#include <stdint.h>


#define ADDR_FLASH_SECTOR_55     ((uint32_t)0x08019000)


#define USER_DATA_BASE_FLASH_ADDR       ADDR_FLASH_SECTOR_55              //
#define PRODUCT_INFOR_FLASH_ADDR        USER_DATA_BASE_FLASH_ADDR            // 2KByte for product_infor
#define APP_USER_DATA_FLASH_ADDR        (PRODUCT_INFOR_FLASH_ADDR + 0x800)   // 2KByte for user data

#define USER_DATA_BASE_SECTOR	5

#define PRODUCT_INFOR_FLASH_SIZE  2046

#pragma pack(1)
typedef struct{
    uint8_t  aaProductName[24];
    uint32_t aDateRelease;               // fomated: dd/mm/yyyy.
    uint32_t aAppAddress;
    uint32_t aAppSize;
    uint16_t aFirmwareVersion;           // Example:  1.0.1.1
    uint16_t aHardwareVersion;           // Example:  1.0.1.1
    uint16_t aBootLoaderVersion;         // Example:  1.0.1.1
    uint32_t aTimeUse;					         // hour/value
		uint8_t aaSerialNumber[12];
    uint8_t  aStatus;                    //  status ==0 Run boot mode |  status ==1 Run application code | status==1 Error.    
		uint8_t  aCrc8;
}PRODUCT_INFOR;