/*
 * delay.c
 */
#include "delay.h"
#include "cmsis_os.h"


void delay_us(uint16_t nus)
{
	uint32_t Buf,Buf2;
	Buf = osKernelGetSysTimerCount();
	Buf += nus*72;// 72MHz，systemcount=1/72us
	do {
		Buf2 = osKernelGetSysTimerCount();
	} while(Buf > Buf2);
}


