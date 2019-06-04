/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REPRINTF_H
#define __REPRINTF_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f10x.h"
#include "typedef.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#ifdef __cplusplus
}
#endif


#endif /* __REPRINTF_H */


