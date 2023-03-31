#include <stdint.h>
#include <stdio.h>
#include "stm32f103xe.h"

//重定向printf到USART1
int fputc(int byte, FILE *f)
{
	while((USART1->SR&0x40) == 0){};
	USART1->DR = (unsigned char)byte;
	return byte;
}
