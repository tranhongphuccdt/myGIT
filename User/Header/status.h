#include <stdint.h>


typedef struct __DRIVER_STATUS{
	int8_t status;
	int16_t error;
	
		
}DRIVER_STATUS;

//delag:
extern volatile DRIVER_STATUS Dc_ServoStatus;