// Header:
// File Name: 
// Author:		LinhTran
// Date:

#ifndef _DELAY_H_
#define _DELAY_H_
#include <stdint.h>
#define SYSTEM_CLOCK 120000000


void delay_nop(uint32_t nopNumber);
void delay_us(uint32_t time);
void delay_ms(uint32_t time);

#endif
