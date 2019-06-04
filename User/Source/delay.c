#include "delay.h"

#define T_DENTA_COUNT (11*SYSTEM_CLOCK)/72000000

void delay_nop(uint32_t nopNumber){
    while(nopNumber--);
}
void delay_us(uint32_t time){
    uint32_t i;
    while(time--){            //2 chu ky          //6.5us
        i= T_DENTA_COUNT;                         //1 chu ky
        while(i--);             //2 chu ky         //72/5 ~15
    }
}

void delay_ms(uint32_t time){
   while(time--){
        delay_us(1000);
   }
}
