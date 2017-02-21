
#ifndef _BSP_ADC_H
#define _BSP_ADC_H
//#include "stm32f2xx.h"


void bsp_InitADC(void);

int8_t getCurent_IntTempValue(void);
int8_t getCurent_ExtTempValue(void);
uint8_t getLightVLuxValue(void);
uint16_t getMicAmp_dBValue(void);


#endif

