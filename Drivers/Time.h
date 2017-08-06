#ifndef _TIME_H_
#define _TIME_H_

void Tim3_Init(void);
void Tim3_PWM(u8 value);
void Tim2_Init(void);
void Tim4_Init(void);
void Tim5_Init(void);
void Tim2_Set(u16 Val,u16 uS_Base,bool AutoReload);
void Tim4_Set(u16 Val,u16 uS_Base,bool AutoReload);
void Tim5_Set(u16 Val,u16 uS_Base,bool AutoReload);
void bsp_SetTIMOutPWM_N(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,
        TIM_TypeDef* TIMx, uint8_t _ucChannel,	 uint32_t _ulFreq, uint32_t _ulDutyCycle);


#endif

