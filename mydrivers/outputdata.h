


#ifndef _outputdata_H
#define _outputdata_H
#include "usart.h"
#include "stm32f4xx_hal.h"

extern int OutData[4];
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);

void OutPut_Data(void);

void Out_My_Data(float d1,float d2,float d3,float d4);
#endif 
