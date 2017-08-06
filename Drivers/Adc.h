#ifndef __ADC_H__
#define __ADC_H__

void Adc_Init(void);
u16 ReadBatteryVoltage(void);
int8_t getCurent_IntTempValue(void);
int8_t getCurent_ExtTempValue(void);
uint8_t getLightVLuxValue(void);
uint16_t getMicAmp_dBValue(void);


#endif

