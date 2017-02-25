#line 1 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
































 

#line 1 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 36 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
#line 1 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"










































 



 



 
    






  


 
  


 







 





#line 88 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"







            







 










 
#line 123 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
                                             


 



 



 








 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,        
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,          
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  ETH_IRQn                    = 61,      
  ETH_WKUP_IRQn               = 62,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,       
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  OTG_HS_EP1_OUT_IRQn         = 74,      
  OTG_HS_EP1_IN_IRQn          = 75,      
  OTG_HS_WKUP_IRQn            = 76,      
  OTG_HS_IRQn                 = 77,      
  DCMI_IRQn                   = 78,      
  CRYP_IRQn                   = 79,      
  HASH_RNG_IRQn               = 80       
} IRQn_Type;



 

#line 1 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"
 




 

























 











#line 1 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"

















 




 



 

 













#line 120 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"



 







#line 162 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"

#line 1 "..\\Libraries\\CMSIS\\Include\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\Libraries\\CMSIS\\Include\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}




#line 297 "..\\Libraries\\CMSIS\\Include\\cmsis_armcc.h"



 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 "..\\Libraries\\CMSIS\\Include\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 

#line 731 "..\\Libraries\\CMSIS\\Include\\cmsis_armcc.h"
 


#line 54 "..\\Libraries\\CMSIS\\Include\\core_cmInstr.h"

 
#line 84 "..\\Libraries\\CMSIS\\Include\\core_cmInstr.h"

   

#line 164 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"
#line 1 "..\\Libraries\\CMSIS\\Include\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\Libraries\\CMSIS\\Include\\core_cmFunc.h"

 
#line 84 "..\\Libraries\\CMSIS\\Include\\core_cmFunc.h"

 

#line 165 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"
















 
#line 203 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"

 






 
#line 219 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"

 




 












 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:27;               
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 


















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 



























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 




#line 500 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"

 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    

  volatile uint32_t ACTLR;                   



} SCnSCB_Type;

 



 










 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;

 









 









 



 









 






























 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1372 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"

#line 1381 "..\\Libraries\\CMSIS\\Include\\core_cm3.h"






 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 244 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
#line 1 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\system_stm32f2xx.h"

























 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           




 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 245 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
#line 246 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"



   
 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;



 



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;           
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
  uint32_t      RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
  uint32_t      RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
  uint32_t      RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
  uint32_t      RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;   
  uint32_t      RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;   
  uint32_t      RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
} FLASH_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];     
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;        
  volatile uint32_t SR2;         
  volatile uint32_t PMEM2;       
  volatile uint32_t PATT2;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR2;       
} FSMC_Bank2_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR3;        
  volatile uint32_t SR3;         
  volatile uint32_t PMEM3;       
  volatile uint32_t PATT3;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR3;       
} FSMC_Bank3_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR4;        
  volatile uint32_t SR4;         
  volatile uint32_t PMEM4;       
  volatile uint32_t PATT4;       
  volatile uint32_t PIO4;        
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint16_t BSRRL;     
  volatile uint16_t BSRRH;     
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED[2];    
  volatile uint32_t CMPCR;         
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;         
  uint16_t      RESERVED0;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED1;   
  volatile uint16_t OAR1;        
  uint16_t      RESERVED2;   
  volatile uint16_t OAR2;        
  uint16_t      RESERVED3;   
  volatile uint16_t DR;          
  uint16_t      RESERVED4;   
  volatile uint16_t SR1;         
  uint16_t      RESERVED5;   
  volatile uint16_t SR2;         
  uint16_t      RESERVED6;   
  volatile uint16_t CCR;         
  uint16_t      RESERVED7;   
  volatile uint16_t TRISE;       
  uint16_t      RESERVED8;   
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  uint32_t RESERVED1;     
  uint32_t RESERVED2;     
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  uint32_t RESERVED3;     
  uint32_t RESERVED4;     
  volatile uint32_t TAFCR;    
  uint32_t RESERVED5;     
  uint32_t RESERVED6;     
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;           
  volatile uint32_t CLKCR;           
  volatile uint32_t ARG;             
  volatile uint32_t CMD;             
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;          
  volatile uint32_t DLEN;            
  volatile uint32_t DCTRL;           
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;             
  volatile uint32_t MASK;            
  uint32_t      RESERVED0[2];    
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];   
  volatile uint32_t FIFO;            
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;         
  uint16_t      RESERVED0;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED1;   
  volatile uint16_t SR;          
  uint16_t      RESERVED2;   
  volatile uint16_t DR;          
  uint16_t      RESERVED3;   
  volatile uint16_t CRCPR;       
  uint16_t      RESERVED4;   
  volatile uint16_t RXCRCR;      
  uint16_t      RESERVED5;   
  volatile uint16_t TXCRCR;      
  uint16_t      RESERVED6;   
  volatile uint16_t I2SCFGR;     
  uint16_t      RESERVED7;   
  volatile uint16_t I2SPR;       
  uint16_t      RESERVED8;   
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;          
  uint16_t      RESERVED0;    
  volatile uint16_t CR2;          
  uint16_t      RESERVED1;    
  volatile uint16_t SMCR;         
  uint16_t      RESERVED2;    
  volatile uint16_t DIER;         
  uint16_t      RESERVED3;    
  volatile uint16_t SR;           
  uint16_t      RESERVED4;    
  volatile uint16_t EGR;          
  uint16_t      RESERVED5;    
  volatile uint16_t CCMR1;        
  uint16_t      RESERVED6;    
  volatile uint16_t CCMR2;        
  uint16_t      RESERVED7;    
  volatile uint16_t CCER;         
  uint16_t      RESERVED8;    
  volatile uint32_t CNT;          
  volatile uint16_t PSC;          
  uint16_t      RESERVED9;    
  volatile uint32_t ARR;          
  volatile uint16_t RCR;          
  uint16_t      RESERVED10;   
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint16_t BDTR;         
  uint16_t      RESERVED11;   
  volatile uint16_t DCR;          
  uint16_t      RESERVED12;   
  volatile uint16_t DMAR;         
  uint16_t      RESERVED13;   
  volatile uint16_t OR;           
  uint16_t      RESERVED14;   
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;          
  uint16_t      RESERVED0;   
  volatile uint16_t DR;          
  uint16_t      RESERVED1;   
  volatile uint16_t BRR;         
  uint16_t      RESERVED2;   
  volatile uint16_t CR1;         
  uint16_t      RESERVED3;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED4;   
  volatile uint16_t CR3;         
  uint16_t      RESERVED5;   
  volatile uint16_t GTPR;        
  uint16_t      RESERVED6;   
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t SR;      
  volatile uint32_t DR;      
  volatile uint32_t DOUT;    
  volatile uint32_t DMACR;   
  volatile uint32_t IMSCR;   
  volatile uint32_t RISR;    
  volatile uint32_t MISR;    
  volatile uint32_t K0LR;    
  volatile uint32_t K0RR;    
  volatile uint32_t K1LR;    
  volatile uint32_t K1RR;    
  volatile uint32_t K2LR;    
  volatile uint32_t K2RR;    
  volatile uint32_t K3LR;    
  volatile uint32_t K3RR;    
  volatile uint32_t IV0LR;   
  volatile uint32_t IV0RR;   
  volatile uint32_t IV1LR;   
  volatile uint32_t IV1RR;   
} CRYP_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;         
  volatile uint32_t DIN;        
  volatile uint32_t STR;        
  volatile uint32_t HR[5];      
  volatile uint32_t IMR;        
  volatile uint32_t SR;         
  uint32_t  RESERVED[52];   
  volatile uint32_t CSR[51];      
} HASH_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 
  


 










 





 
#line 1048 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1065 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1103 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 





 






 




 
  


   
#line 1208 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 
 
#line 1237 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1263 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
  
 
#line 1289 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1327 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1369 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 


 


 


 


 
#line 1418 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1456 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1494 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1523 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 


 


 



 
#line 1559 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1581 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 



 
 
 
 
 
 
 
#line 1602 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1613 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1631 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"











 





 





 
#line 1669 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 












 
#line 1690 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 1830 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1847 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1864 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1881 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1915 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1949 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 1983 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2017 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2051 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2085 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2119 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2153 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2187 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2221 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2255 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2289 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2323 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2357 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2391 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2425 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2459 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2493 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2527 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2561 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2595 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2629 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2663 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2697 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2731 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2765 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2799 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 2833 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
 
 
 
 
 



 



 


 
 
 
 
 
 


#line 2870 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 2879 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
 





 


 


 


 



 
 
 
 
 
 












































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 

 
 
 
 
 
 
#line 3018 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 




 






 






 






 






 
 
 
 
 
  
#line 3093 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3112 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

  
#line 3123 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

  
#line 3145 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

  
#line 3167 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

  
#line 3189 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

  
#line 3211 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
 
 
 
 
 
#line 3238 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3260 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3282 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3304 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3326 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3348 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
 
 
 
 
 
#line 3364 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 3372 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3381 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3395 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3425 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
 
 
 
 
 











#line 3453 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 











#line 3476 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 











#line 3499 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 











#line 3522 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 












#line 3545 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"























 












#line 3590 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"























 












#line 3635 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"























 












#line 3680 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"























 












#line 3725 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

















 












#line 3764 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

















 












#line 3803 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

















 












#line 3842 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

















 



























 



























 



























 
#line 3951 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3960 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3969 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 3980 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 3990 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4000 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4010 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4021 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4031 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4041 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4051 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4062 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4072 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4082 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4092 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4103 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4113 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4123 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4133 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4144 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4154 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4164 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4174 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4185 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4195 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4205 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4215 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4226 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4236 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4246 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4256 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 
 
 
 
 
 
































































 
#line 4350 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
































































 
































































 
#line 4498 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
 
#line 4515 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4533 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
 
#line 4550 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


 
#line 4585 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
 
 
 
 
 
#line 4606 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4615 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 



 





 
 
 
 
 
 
#line 4646 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4655 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"







 



#line 4676 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"



 



 


 
#line 4701 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4711 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 




 


 
 
 
 
 
 


 





 


 



 
 
 
 
 
 











 
#line 4767 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"





 
#line 4779 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
 
 
 
 
 



#line 4795 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4805 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4814 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4823 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 4834 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"















 
 








 








 






#line 4884 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 











 











 
#line 4916 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 




















 
#line 4959 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 4975 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 



  




 


 
#line 5012 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5025 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
 


 
#line 5048 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 






 


 
#line 5083 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5098 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5122 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 






 


 
#line 5157 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5172 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 











 
#line 5196 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 





 



 
 
 
 
 
 



 






 
 
 
 
 
 
#line 5256 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5286 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5312 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5327 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 



 


 



 
#line 5380 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5422 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 
#line 5454 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5474 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5482 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 
 
 
 
 
 




 












 


 






#line 5583 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5653 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5668 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5694 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 
 
 
 
 
 









#line 5726 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5734 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 5744 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 


 


 





















 




 
 
 
 
 
   




 

 


 






  
#line 5816 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5828 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5840 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5852 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 






  
#line 5870 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5882 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5894 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5906 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 




           


  
#line 5925 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5937 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5949 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5961 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 






  
#line 5978 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 5989 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 6000 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


  
#line 6011 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

   



 
 
 
 
 
 
















 









#line 6056 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 

























 
#line 6099 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 6113 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 6123 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 




























 





















 




























 





















 
#line 6242 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 


 


 


 


 


 


 
#line 6277 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"





#line 6288 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 6296 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 6303 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 
#line 6314 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"


 
 
 
 
 
 
#line 6332 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 



 
#line 6356 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 6365 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"







 
#line 6385 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 6396 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"



 
 
 
 
 
 
#line 6413 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"



 
#line 6425 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"







 



 
 
 
 
 
 



 









 
#line 6473 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
 


 






 
 
 
 
 
 
#line 6517 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 6533 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 
#line 6550 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
  
 


 
#line 6566 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 



  


 








 

  
#line 6593 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 






 



 


 


 
#line 6622 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 
#line 6637 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 
#line 6652 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 
 
 

 
#line 6667 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 




 




 




 




 


 


 


 


 


 


 
 
 

 
#line 6720 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

#line 6727 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 



 


 



 


 


 


 



 
 
 

 
#line 6802 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 


 


 


 


 




   
#line 6853 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 6879 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 
#line 6896 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"

 





 


 


 


 




 

 

  

#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"

























  

 



 
 
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


























 

 







 
#line 1 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"










































 



 



 
    
#line 6954 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"



 

  

 

 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"



 



  

 



  
typedef struct
{
  uint32_t ADC_Resolution;                
                                    
  FunctionalState ADC_ScanConvMode;       


  
  FunctionalState ADC_ContinuousConvMode; 

 
  uint32_t ADC_ExternalTrigConvEdge;      


 
  uint32_t ADC_ExternalTrigConv;          


 
  uint32_t ADC_DataAlign;                 

 
  uint8_t  ADC_NbrOfConversion;           


 
}ADC_InitTypeDef;
  


  
typedef struct 
{
  uint32_t ADC_Mode;                      

                                               
  uint32_t ADC_Prescaler;                 

 
  uint32_t ADC_DMAAccessMode;             


 
  uint32_t ADC_TwoSamplingDelay;          

 
  
}ADC_CommonInitTypeDef;


 



  






  
#line 141 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 157 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 173 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"
                                     


  




  
#line 214 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"
                                     


  




  
#line 231 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"
                                      


  




  
#line 248 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 288 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  






  




  
#line 327 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"





#line 351 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 375 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 391 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"
                                            


  




  
#line 432 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 448 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 470 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 484 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  
#line 498 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"
  
#line 506 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_adc.h"


  




  



  




  



  




  



  




  



  




  



  




  



  




  



  




  

 
   

   
void ADC_DeInit(void);

 
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_CommonInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_CommonStructInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);

 
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
void ADC_VBATCmd(FunctionalState NewState);

 
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_EOCOnEachRegularChannelCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ContinuousModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetMultiModeConversionValue(void);

 
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMARequestAfterLastTransferCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_MultiModeDMARequestAfterLastTransferCmd(FunctionalState NewState);

 
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvEdgeConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConvEdge);
void ADC_SoftwareStartInjectedConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);

 
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









  



  

 
#line 35 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"



 



 

 






 
typedef struct
{
  uint16_t CAN_Prescaler;   
 
  
  uint8_t CAN_Mode;         
 

  uint8_t CAN_SJW;          


 

  uint8_t CAN_BS1;          

 

  uint8_t CAN_BS2;          
 
  
  FunctionalState CAN_TTCM; 
 
  
  FunctionalState CAN_ABOM;  
 

  FunctionalState CAN_AWUM;  
 

  FunctionalState CAN_NART;  
 

  FunctionalState CAN_RFLM;  
 

  FunctionalState CAN_TXFP;  
 
} CAN_InitTypeDef;



 
typedef struct
{
  uint16_t CAN_FilterIdHigh;         

 

  uint16_t CAN_FilterIdLow;          

 

  uint16_t CAN_FilterMaskIdHigh;     


 

  uint16_t CAN_FilterMaskIdLow;      


 

  uint16_t CAN_FilterFIFOAssignment; 
 
  
  uint8_t CAN_FilterNumber;           

  uint8_t CAN_FilterMode;            
 

  uint8_t CAN_FilterScale;           
 

  FunctionalState CAN_FilterActivation; 
 
} CAN_FilterInitTypeDef;



 
typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     

 

  uint8_t Data[8]; 
 
} CanTxMsg;



 
typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     
 

  uint8_t Data[8]; 
 

  uint8_t FMI;     

 
} CanRxMsg;

 



 



 





 




 



 












 


 


   










 
  



   





 



 









 



 
#line 289 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"




 



 
#line 306 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"




 



 



 



 



 



 







 



 







 



 





 




 



 



 



 






 



 





 




 



 




 




 



 





 	






 



 






 



 



 	




 



 



 




 




                                                          
#line 481 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"


 



 

 

 

 




 
#line 505 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"

 



 

 





#line 526 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"








 

  


  


 
#line 549 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"

 



 






 





#line 574 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"

#line 581 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_can.h"


 



 

 
   

  
void CAN_DeInit(CAN_TypeDef* CANx);

  
uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
void CAN_SlaveStartBank(uint8_t CAN_BankNumber); 
void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState);
void CAN_TTComModeCmd(CAN_TypeDef* CANx, FunctionalState NewState);

 
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox);
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint8_t Mailbox);

 
void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void CAN_FIFORelease(CAN_TypeDef* CANx, uint8_t FIFONumber);
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);

 
uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode);
uint8_t CAN_Sleep(CAN_TypeDef* CANx);
uint8_t CAN_WakeUp(CAN_TypeDef* CANx);

 
uint8_t CAN_GetLastErrorCode(CAN_TypeDef* CANx);
uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef* CANx);
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef* CANx);

 
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
void CAN_ClearFlag(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT);
void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT);









 



 

 
#line 36 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_crc.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_crc.h"



 



 

 
 



 



 

 
   

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);









 



 

 
#line 37 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_cryp.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_cryp.h"



 



  

 



  
typedef struct
{
  uint16_t CRYP_AlgoDir;   
 
  uint16_t CRYP_AlgoMode;  

 
  uint16_t CRYP_DataType;  
  
  uint16_t CRYP_KeySize;   

 
}CRYP_InitTypeDef;



  
typedef struct
{
  uint32_t CRYP_Key0Left;   
  uint32_t CRYP_Key0Right;  
  uint32_t CRYP_Key1Left;   
  uint32_t CRYP_Key1Right;  
  uint32_t CRYP_Key2Left;   
  uint32_t CRYP_Key2Right;  
  uint32_t CRYP_Key3Left;   
  uint32_t CRYP_Key3Right;  
}CRYP_KeyInitTypeDef;


  
typedef struct
{
  uint32_t CRYP_IV0Left;   
  uint32_t CRYP_IV0Right;  
  uint32_t CRYP_IV1Left;   
  uint32_t CRYP_IV1Right;  
}CRYP_IVInitTypeDef;



  
typedef struct
{
   
  uint32_t CR_bits9to2;
   
  uint32_t CRYP_IV0LR;
  uint32_t CRYP_IV0RR;
  uint32_t CRYP_IV1LR;
  uint32_t CRYP_IV1RR;
   
  uint32_t CRYP_K0LR;
  uint32_t CRYP_K0RR;
  uint32_t CRYP_K1LR;
  uint32_t CRYP_K1RR;
  uint32_t CRYP_K2LR;
  uint32_t CRYP_K2RR;
  uint32_t CRYP_K3LR;
  uint32_t CRYP_K3RR;
}CRYP_Context;


 



 



 







  
 


 

 



 



 





#line 160 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_cryp.h"


  
 


 
#line 175 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_cryp.h"


 
                                     


 
#line 188 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_cryp.h"


 



 
#line 207 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_cryp.h"

#line 215 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_cryp.h"


 



 







 



 





 



 





  



  

 
 

 
void CRYP_DeInit(void);

 
void CRYP_Init(CRYP_InitTypeDef* CRYP_InitStruct);
void CRYP_StructInit(CRYP_InitTypeDef* CRYP_InitStruct);
void CRYP_KeyInit(CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_KeyStructInit(CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_IVInit(CRYP_IVInitTypeDef* CRYP_IVInitStruct);
void CRYP_IVStructInit(CRYP_IVInitTypeDef* CRYP_IVInitStruct);
void CRYP_Cmd(FunctionalState NewState);

 
void CRYP_DataIn(uint32_t Data);
uint32_t CRYP_DataOut(void);
void CRYP_FIFOFlush(void);

 
ErrorStatus CRYP_SaveContext(CRYP_Context* CRYP_ContextSave,
                             CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_RestoreContext(CRYP_Context* CRYP_ContextRestore);

 
void CRYP_DMACmd(uint8_t CRYP_DMAReq, FunctionalState NewState);

 
void CRYP_ITConfig(uint8_t CRYP_IT, FunctionalState NewState);
ITStatus CRYP_GetITStatus(uint8_t CRYP_IT);
FlagStatus CRYP_GetFlagStatus(uint8_t CRYP_FLAG);

 
ErrorStatus CRYP_AES_ECB(uint8_t Mode,
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_CBC(uint8_t Mode,
                         uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_CTR(uint8_t Mode,
                         uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

 
ErrorStatus CRYP_TDES_ECB(uint8_t Mode,
                           uint8_t Key[24], 
                           uint8_t *Input, uint32_t Ilength,
                           uint8_t *Output);

ErrorStatus CRYP_TDES_CBC(uint8_t Mode,
                          uint8_t Key[24],
                          uint8_t InitVectors[8],
                          uint8_t *Input, uint32_t Ilength,
                          uint8_t *Output);

 
ErrorStatus CRYP_DES_ECB(uint8_t Mode,
                         uint8_t Key[8],
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_DES_CBC(uint8_t Mode,
                         uint8_t Key[8],
                         uint8_t InitVectors[8],
                         uint8_t *Input,uint32_t Ilength,
                         uint8_t *Output);









 



  

 
#line 38 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dac.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dac.h"



 



 

 



 

typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;

 



 



 

#line 89 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dac.h"




#line 102 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dac.h"



 



 

#line 117 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dac.h"


 



 

#line 149 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dac.h"

#line 174 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dac.h"


 



 







 



 







 



 

#line 212 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dac.h"


 



 







 



 




 
  


    





  



  
  





 



 

 
   

   
void DAC_DeInit(void);

 
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);

 
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);

 
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState);
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG);
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG);
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT);
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT);









 



 

 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dbgmcu.h"

























 

 







 
#line 38 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dbgmcu.h"



 



  

 
 



  





#line 76 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dbgmcu.h"

#line 83 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dbgmcu.h"


  

 
  
uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB2PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);









  



  

 
#line 40 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dcmi.h"

























 

 







 
#line 38 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dcmi.h"



 



  

 


  
typedef struct
{
  uint16_t DCMI_CaptureMode;      
 

  uint16_t DCMI_SynchroMode;      
 

  uint16_t DCMI_PCKPolarity;      
 

  uint16_t DCMI_VSPolarity;       
 

  uint16_t DCMI_HSPolarity;       
 

  uint16_t DCMI_CaptureRate;      
 

  uint16_t DCMI_ExtendedDataMode; 
 
} DCMI_InitTypeDef;



  
typedef struct
{
  uint16_t DCMI_VerticalStartLine;      
 

  uint16_t DCMI_HorizontalOffsetCount;  
 

  uint16_t DCMI_VerticalLineCount;      
 

  uint16_t DCMI_CaptureCount;           

 
} DCMI_CROPInitTypeDef;



  
typedef struct
{
  uint8_t DCMI_FrameStartCode;  
  uint8_t DCMI_LineStartCode;   
  uint8_t DCMI_LineEndCode;     
  uint8_t DCMI_FrameEndCode;    
} DCMI_CodesInitTypeDef;

 



 



  
#line 120 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dcmi.h"


  




  
#line 134 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dcmi.h"


  




  






  




  






  




  






  




  
#line 184 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dcmi.h"


  




  
#line 200 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dcmi.h"


  




  
#line 219 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dcmi.h"


  




  


  





  







  
#line 262 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dcmi.h"
                                



  



  

 
  

  
void DCMI_DeInit(void);

 
void DCMI_Init(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_StructInit(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_CROPConfig(DCMI_CROPInitTypeDef* DCMI_CROPInitStruct);
void DCMI_CROPCmd(FunctionalState NewState);
void DCMI_SetEmbeddedSynchroCodes(DCMI_CodesInitTypeDef* DCMI_CodesInitStruct);
void DCMI_JPEGCmd(FunctionalState NewState);

 
void DCMI_Cmd(FunctionalState NewState);
void DCMI_CaptureCmd(FunctionalState NewState);
uint32_t DCMI_ReadData(void);

 
void DCMI_ITConfig(uint16_t DCMI_IT, FunctionalState NewState);
FlagStatus DCMI_GetFlagStatus(uint16_t DCMI_FLAG);
void DCMI_ClearFlag(uint16_t DCMI_FLAG);
ITStatus DCMI_GetITStatus(uint16_t DCMI_IT);
void DCMI_ClearITPendingBit(uint16_t DCMI_IT);









  



  

 
#line 41 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"


























  

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"



 



 

 



 

typedef struct
{
  uint32_t DMA_Channel;            
 
 
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_Memory0BaseAddr;    

 

  uint32_t DMA_DIR;                

 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_FIFOMode;          


 

  uint32_t DMA_FIFOThreshold;      
 

  uint32_t DMA_MemoryBurst;        


 

  uint32_t DMA_PeripheralBurst;    


   
}DMA_InitTypeDef;

 



 

#line 134 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"






  
#line 149 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"

#line 158 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"


  




  









  




  



  




  







  




  







  




  









  




  









  




  







  




  











  




  







  




  











  




  











  




  











  




 
#line 346 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"

#line 353 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"


  



 
#line 400 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"




#line 424 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"


  




  









  




  
#line 487 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"





#line 512 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_dma.h"


  




  







  




  







  




  






  



  

 
  

  
void DMA_DeInit(DMA_Stream_TypeDef* DMAy_Streamx);

 
void DMA_Init(DMA_Stream_TypeDef* DMAy_Streamx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);

 
void DMA_PeriphIncOffsetSizeConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_Pincos);
void DMA_FlowControllerConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FlowCtrl);

 
void DMA_SetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx, uint16_t Counter);
uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx);

 
void DMA_DoubleBufferModeConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t Memory1BaseAddr,
                                uint32_t DMA_CurrentMemory);
void DMA_DoubleBufferModeCmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);
void DMA_MemoryTargetConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t MemoryBaseAddr,
                            uint32_t DMA_MemoryTarget);
uint32_t DMA_GetCurrentMemoryTarget(DMA_Stream_TypeDef* DMAy_Streamx);

 
FunctionalState DMA_GetCmdStatus(DMA_Stream_TypeDef* DMAy_Streamx);
uint32_t DMA_GetFIFOStatus(DMA_Stream_TypeDef* DMAy_Streamx);
FlagStatus DMA_GetFlagStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ClearFlag(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ITConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT, FunctionalState NewState);
ITStatus DMA_GetITStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);
void DMA_ClearITPendingBit(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);









 



 


 
#line 42 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_exti.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_exti.h"



 



 

 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 
   
  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
  
}EXTI_InitTypeDef;

 



 



 

#line 128 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_exti.h"
                                          


#line 143 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_exti.h"
                    


 



 

 
 

 
void EXTI_DeInit(void);

 
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);

 
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);









 



 

 
#line 43 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_flash.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_flash.h"



 



  

 


  
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PGS,
  FLASH_ERROR_PGP,
  FLASH_ERROR_PGA,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_ERROR_OPERATION,
  FLASH_COMPLETE
}FLASH_Status;

 



   



  
#line 81 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_flash.h"

#line 90 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_flash.h"


  



  











  



  
#line 133 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_flash.h"


  



  
#line 153 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_flash.h"




 



 


  
 





  



  





  



  





  




  





 
  


   
#line 213 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_flash.h"


 



  





  



  
#line 242 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_flash.h"


 



 







  



  







  



  



  






  

 
  
 
 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_PrefetchBufferCmd(FunctionalState NewState);
void FLASH_InstructionCacheCmd(FunctionalState NewState);
void FLASH_DataCacheCmd(FunctionalState NewState);
void FLASH_InstructionCacheReset(void);
void FLASH_DataCacheReset(void);

    
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllSectors(uint8_t VoltageRange);
FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data);

  
void FLASH_OB_Unlock(void);
void FLASH_OB_Lock(void);
void FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
void FLASH_OB_RDPConfig(uint8_t OB_RDP);
void FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
void FLASH_OB_BORConfig(uint8_t OB_BOR);
FLASH_Status FLASH_OB_Launch(void);
uint8_t FLASH_OB_GetUser(void);
uint16_t FLASH_OB_GetWRP(void);
FlagStatus FLASH_OB_GetRDP(void);
uint8_t FLASH_OB_GetBOR(void);

 
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(void);









  



  

 
#line 44 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_fsmc.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_fsmc.h"



 



 

 



 
typedef struct
{
  uint32_t FSMC_AddressSetupTime;       


 

  uint32_t FSMC_AddressHoldTime;        


 

  uint32_t FSMC_DataSetupTime;          


 

  uint32_t FSMC_BusTurnAroundDuration;  


 

  uint32_t FSMC_CLKDivision;            

 

  uint32_t FSMC_DataLatency;            





 

  uint32_t FSMC_AccessMode;             
 
}FSMC_NORSRAMTimingInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_Bank;                
 

  uint32_t FSMC_DataAddressMux;      

 

  uint32_t FSMC_MemoryType;          

 

  uint32_t FSMC_MemoryDataWidth;     
 

  uint32_t FSMC_BurstAccessMode;     

 

  uint32_t FSMC_AsynchronousWait;     

                                           

  uint32_t FSMC_WaitSignalPolarity;  

 

  uint32_t FSMC_WrapMode;            

 

  uint32_t FSMC_WaitSignalActive;    


 

  uint32_t FSMC_WriteOperation;      
 

  uint32_t FSMC_WaitSignal;          

 

  uint32_t FSMC_ExtendedMode;        
 

  uint32_t FSMC_WriteBurst;          
  

  FSMC_NORSRAMTimingInitTypeDef* FSMC_ReadWriteTimingStruct;    

  FSMC_NORSRAMTimingInitTypeDef* FSMC_WriteTimingStruct;            
}FSMC_NORSRAMInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_SetupTime;      



 

  uint32_t FSMC_WaitSetupTime;  



 

  uint32_t FSMC_HoldSetupTime;  




 

  uint32_t FSMC_HiZSetupTime;   



 
}FSMC_NAND_PCCARDTimingInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_Bank;              
 

  uint32_t FSMC_Waitfeature;      
 

  uint32_t FSMC_MemoryDataWidth;  
 

  uint32_t FSMC_ECC;              
 

  uint32_t FSMC_ECCPageSize;      
 

  uint32_t FSMC_TCLRSetupTime;    

 

  uint32_t FSMC_TARSetupTime;     

  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;     

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;  
}FSMC_NANDInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Waitfeature;    
 

  uint32_t FSMC_TCLRSetupTime;  

 

  uint32_t FSMC_TARSetupTime;   

  

  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;    
  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_IOSpaceTimingStruct;    
}FSMC_PCCARDInitTypeDef;

 



 



 






 



   




 



     



 



















 



 







 



 

#line 314 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_fsmc.h"


 



 







 



 







 
    


 






 



 






 



 






 



 






 



 






 



 






 



 







 



 







 



 



 



 



 



 



 



 



 



 



 



 



 



 
#line 491 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_fsmc.h"


 



 
  


 



 






 




 






 



 
#line 541 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_fsmc.h"


 



 



 



 



 



 



 



 



 



 



 



 



 



 
#line 603 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_fsmc.h"


 



 
#line 618 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_fsmc.h"




 



 



 

 
  

 
void FSMC_NORSRAMDeInit(uint32_t FSMC_Bank);
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NORSRAMCmd(uint32_t FSMC_Bank, FunctionalState NewState);

 
void FSMC_NANDDeInit(uint32_t FSMC_Bank);
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_NANDCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_NANDECCCmd(uint32_t FSMC_Bank, FunctionalState NewState);
uint32_t FSMC_GetECC(uint32_t FSMC_Bank);

 
void FSMC_PCCARDDeInit(void);
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_PCCARDCmd(FunctionalState NewState);

 
void FSMC_ITConfig(uint32_t FSMC_Bank, uint32_t FSMC_IT, FunctionalState NewState);
FlagStatus FSMC_GetFlagStatus(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
void FSMC_ClearFlag(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
ITStatus FSMC_GetITStatus(uint32_t FSMC_Bank, uint32_t FSMC_IT);
void FSMC_ClearITPendingBit(uint32_t FSMC_Bank, uint32_t FSMC_IT);








 



  

 
#line 45 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_hash.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_hash.h"



 



  

 



  
typedef struct
{
  uint32_t HASH_AlgoSelection; 
 
  uint32_t HASH_AlgoMode;      
 
  uint32_t HASH_DataType;      

 
  uint32_t HASH_HMACKeyType;   
 
}HASH_InitTypeDef;



  
typedef struct
{
  uint32_t Data[5];      
 
} HASH_MsgDigest; 



  
typedef struct
{
  uint32_t HASH_IMR; 
  uint32_t HASH_STR;      
  uint32_t HASH_CR;     
  uint32_t HASH_CSR[51];       
}HASH_Context;

 



  



  







 



  







 



   











 



  







 



   




 



   





				   


 



   

















  



  

 
  
  
 
void HASH_DeInit(void);

 
void HASH_Init(HASH_InitTypeDef* HASH_InitStruct);
void HASH_StructInit(HASH_InitTypeDef* HASH_InitStruct);
void HASH_Reset(void);

 
void HASH_DataIn(uint32_t Data);
uint8_t HASH_GetInFIFOWordsNbr(void);
void HASH_SetLastWordValidBitsNbr(uint16_t ValidNumber);
void HASH_StartDigest(void);
void HASH_GetDigest(HASH_MsgDigest* HASH_MessageDigest);

 
void HASH_SaveContext(HASH_Context* HASH_ContextSave);
void HASH_RestoreContext(HASH_Context* HASH_ContextRestore);

 
void HASH_DMACmd(FunctionalState NewState);

 
void HASH_ITConfig(uint8_t HASH_IT, FunctionalState NewState);
FlagStatus HASH_GetFlagStatus(uint16_t HASH_FLAG);
void HASH_ClearFlag(uint16_t HASH_FLAG);
ITStatus HASH_GetITStatus(uint8_t HASH_IT);
void HASH_ClearITPendingBit(uint8_t HASH_IT);

 
ErrorStatus HASH_SHA1(uint8_t *Input, uint32_t Ilen, uint8_t Output[20]);
ErrorStatus HMAC_SHA1(uint8_t *Key, uint32_t Keylen,
                      uint8_t *Input, uint32_t Ilen,
                      uint8_t Output[20]);

 
ErrorStatus HASH_MD5(uint8_t *Input, uint32_t Ilen, uint8_t Output[16]);
ErrorStatus HMAC_MD5(uint8_t *Key, uint32_t Keylen,
                     uint8_t *Input, uint32_t Ilen,
                     uint8_t Output[16]);









  



  

 
#line 46 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_gpio.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_gpio.h"



 



  

 

#line 59 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_gpio.h"
                                                                


    
typedef enum
{ 
  GPIO_Mode_IN   = 0x00,  
  GPIO_Mode_OUT  = 0x01,  
  GPIO_Mode_AF   = 0x02,  
  GPIO_Mode_AN   = 0x03   
}GPIOMode_TypeDef;





   
typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;





   
typedef enum
{ 
  GPIO_Speed_2MHz   = 0x00,  
  GPIO_Speed_25MHz  = 0x01,  
  GPIO_Speed_50MHz  = 0x02,  
  GPIO_Speed_100MHz = 0x03   
}GPIOSpeed_TypeDef;





  
typedef enum
{ 
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;





  
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;





  
typedef struct
{
  uint32_t GPIO_Pin;              
 

  GPIOMode_TypeDef GPIO_Mode;     
 

  GPIOSpeed_TypeDef GPIO_Speed;   
 

  GPIOOType_TypeDef GPIO_OType;   
 

  GPIOPuPd_TypeDef GPIO_PuPd;     
 
}GPIO_InitTypeDef;

 



  



  
#line 167 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_gpio.h"

#line 185 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_gpio.h"


  




  
#line 209 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_gpio.h"

#line 226 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_gpio.h"


  



  


  








  





  






  







  






  





  




  






  






  








  





  




  






  




  


#line 350 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_gpio.h"


  



 
    








 



 

 
  

 
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

 
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);









  



  

 
#line 47 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"



 



 

 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;

 




 






 

#line 95 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"


 



 







  



 







 



 







 



 







  



 

#line 169 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"


 



 







  



 







 



 







  



 







  



 

#line 239 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"



#line 249 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"


 



 



 

#line 268 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"



 

#line 287 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"



#line 301 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"


 



 





 








 
 

























 

 


 





























 

  
 


 
 

 







 

























 

    
 



 



 



























 

  
 

 


 
 


 






 

#line 507 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_i2c.h"


 



 




 



 




 



 

 
  

 
void I2C_DeInit(I2C_TypeDef* I2Cx);

 
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DutyCycle);
void I2C_NACKPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_NACKPosition);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, uint16_t I2C_SMBusAlert);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

  
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);

  
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);

 
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

 
uint16_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState);




















































































 





 
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);




 
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);




 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);


void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);









  



  

 
#line 48 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_iwdg.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_iwdg.h"



 



 

 
 



 
  


 






 



 
#line 83 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_iwdg.h"


 



 






 



 

 
 

 
void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);

 
void IWDG_Enable(void);

 
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);









 



 

 
#line 49 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_pwr.h"


























  

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_pwr.h"



 



  

 
 



  



  

#line 67 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_pwr.h"







 

  


 







 



 




 


 



 












 



 

 
  

  
void PWR_DeInit(void);

  
void PWR_BackupAccessCmd(FunctionalState NewState);

  
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_PVDCmd(FunctionalState NewState);

  
void PWR_WakeUpPinCmd(FunctionalState NewState);

  
void PWR_BackupRegulatorCmd(FunctionalState NewState);

  
void PWR_FlashPowerDownCmd(FunctionalState NewState);

  
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);

  
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);









 



 

 
#line 50 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"

























 

 







 
#line 38 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"



 



  

 
typedef struct
{
  uint32_t SYSCLK_Frequency;  
  uint32_t HCLK_Frequency;    
  uint32_t PCLK1_Frequency;   
  uint32_t PCLK2_Frequency;   
}RCC_ClocksTypeDef;

 



 
  


 







  
  


 
#line 85 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"
 




  
  


 
#line 101 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


 
#line 122 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


 
#line 137 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


 
#line 157 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


 







  
  


 
#line 240 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


 






  
  


  
#line 283 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


   
#line 296 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


  




  
  


  
#line 336 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


  
#line 359 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  
  


 
#line 377 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"
                                   





  
  


 
#line 399 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"
                                   





  
  


 
#line 431 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rcc.h"


  



  

 
  

 
void RCC_DeInit(void);

 
void RCC_HSEConfig(uint8_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);

void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_PLLI2SConfig(uint32_t PLLI2SN, uint32_t PLLI2SR);
void RCC_PLLI2SCmd(FunctionalState NewState);

void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCO1Config(uint32_t RCC_MCO1Source, uint32_t RCC_MCO1Div);
void RCC_MCO2Config(uint32_t RCC_MCO2Source, uint32_t RCC_MCO2Div);

 
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

 
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_I2SCLKConfig(uint32_t RCC_I2SCLKSource); 

void RCC_AHB1PeriphClockCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphClockCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphClockCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

void RCC_AHB1PeriphResetCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphResetCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphResetCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

void RCC_AHB1PeriphClockLPModeCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphClockLPModeCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphClockLPModeCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphClockLPModeCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphClockLPModeCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

 
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);









  



  

 
#line 51 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rng.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rng.h"



 



  

 
  



 
  


  











  



   







  



  

 
  

  
void RNG_DeInit(void);

 
void RNG_Cmd(FunctionalState NewState);

 
uint32_t RNG_GetRandomNumber(void);

 
void RNG_ITConfig(FunctionalState NewState);
FlagStatus RNG_GetFlagStatus(uint8_t RNG_FLAG);
void RNG_ClearFlag(uint8_t RNG_FLAG);
ITStatus RNG_GetITStatus(uint8_t RNG_IT);
void RNG_ClearITPendingBit(uint8_t RNG_IT);









  



  

 
#line 52 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"



 



  

 



  
typedef struct
{
  uint32_t RTC_HourFormat;   
 
  
  uint32_t RTC_AsynchPrediv; 
 
  
  uint32_t RTC_SynchPrediv;  
 
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t RTC_Hours;    


 

  uint8_t RTC_Minutes;  
 
  
  uint8_t RTC_Seconds;  
 

  uint8_t RTC_H12;      
 
}RTC_TimeTypeDef; 



 
typedef struct
{
  uint8_t RTC_WeekDay; 
 
  
  uint8_t RTC_Month;   
 

  uint8_t RTC_Date;     
 
  
  uint8_t RTC_Year;     
 
}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef RTC_AlarmTime;      

  uint32_t RTC_AlarmMask;            
 

  uint32_t RTC_AlarmDateWeekDaySel;  
 
  
  uint8_t RTC_AlarmDateWeekDay;      



 
}RTC_AlarmTypeDef;

 



  




  






  



  

 


  




  




  



  







  



  






  



  




  



  

 
#line 211 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"



  



  
  
#line 234 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"


  




  
#line 250 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"



  




  








  




  
#line 280 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"



  



  







  



  
#line 313 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"


  



  






  



  




 







  



  






  




  








  




  











  



  







  



  





 



  






  



  






  



  







  



 

#line 487 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"


  



  






  



  
#line 524 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"



  



  
#line 538 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_rtc.h"









  



  





  



  

 
  

 
ErrorStatus RTC_DeInit(void);

 
ErrorStatus RTC_Init(RTC_InitTypeDef* RTC_InitStruct);
void RTC_StructInit(RTC_InitTypeDef* RTC_InitStruct);
void RTC_WriteProtectionCmd(FunctionalState NewState);
ErrorStatus RTC_EnterInitMode(void);
void RTC_ExitInitMode(void);
ErrorStatus RTC_WaitForSynchro(void);
ErrorStatus RTC_RefClockCmd(FunctionalState NewState);

 
ErrorStatus RTC_SetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_TimeStructInit(RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_GetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
ErrorStatus RTC_SetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);
void RTC_DateStructInit(RTC_DateTypeDef* RTC_DateStruct);
void RTC_GetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);

 
void RTC_SetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_AlarmStructInit(RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_GetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
ErrorStatus RTC_AlarmCmd(uint32_t RTC_Alarm, FunctionalState NewState);

 
void RTC_WakeUpClockConfig(uint32_t RTC_WakeUpClock);
void RTC_SetWakeUpCounter(uint32_t RTC_WakeUpCounter);
uint32_t RTC_GetWakeUpCounter(void);
ErrorStatus RTC_WakeUpCmd(FunctionalState NewState);

 
void RTC_DayLightSavingConfig(uint32_t RTC_DayLightSaving, uint32_t RTC_StoreOperation);
uint32_t RTC_GetStoreOperation(void);

 
void RTC_OutputConfig(uint32_t RTC_Output, uint32_t RTC_OutputPolarity);

 
ErrorStatus RTC_CoarseCalibConfig(uint32_t RTC_CalibSign, uint32_t Value);
ErrorStatus RTC_CoarseCalibCmd(FunctionalState NewState);
void RTC_CalibOutputCmd(FunctionalState NewState);

 
void RTC_TimeStampCmd(uint32_t RTC_TimeStampEdge, FunctionalState NewState);
void RTC_GetTimeStamp(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_StampTimeStruct,
                                      RTC_DateTypeDef* RTC_StampDateStruct);                                  

 
void RTC_TamperTriggerConfig(uint32_t RTC_Tamper, uint32_t RTC_TamperTrigger);
void RTC_TamperCmd(uint32_t RTC_Tamper, FunctionalState NewState);

 
void RTC_WriteBackupRegister(uint32_t RTC_BKP_DR, uint32_t Data);
uint32_t RTC_ReadBackupRegister(uint32_t RTC_BKP_DR);


 
void RTC_TamperPinSelection(uint32_t RTC_TamperPin);
void RTC_TimeStampPinSelection(uint32_t RTC_TimeStampPin);
void RTC_OutputTypeConfig(uint32_t RTC_OutputType);

 
void RTC_ITConfig(uint32_t RTC_IT, FunctionalState NewState);
FlagStatus RTC_GetFlagStatus(uint32_t RTC_FLAG);
void RTC_ClearFlag(uint32_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint32_t RTC_IT);
void RTC_ClearITPendingBit(uint32_t RTC_IT);









  



  

 
#line 53 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_sdio.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_sdio.h"



 



 

 

typedef struct
{
  uint32_t SDIO_ClockEdge;            
 

  uint32_t SDIO_ClockBypass;          

 

  uint32_t SDIO_ClockPowerSave;       

 

  uint32_t SDIO_BusWide;              
 

  uint32_t SDIO_HardwareFlowControl;  
 

  uint8_t SDIO_ClockDiv;              
 
                                           
} SDIO_InitTypeDef;

typedef struct
{
  uint32_t SDIO_Argument;  


 

  uint32_t SDIO_CmdIndex;   

  uint32_t SDIO_Response;  
 

  uint32_t SDIO_Wait;      
 

  uint32_t SDIO_CPSM;      

 
} SDIO_CmdInitTypeDef;

typedef struct
{
  uint32_t SDIO_DataTimeOut;     

  uint32_t SDIO_DataLength;      
 
  uint32_t SDIO_DataBlockSize;  
 
 
  uint32_t SDIO_TransferDir;    

 
 
  uint32_t SDIO_TransferMode;   
 
 
  uint32_t SDIO_DPSM;           

 
} SDIO_DataInitTypeDef;


 



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 225 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_sdio.h"


  



 




 



 

#line 248 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_sdio.h"


 



 








 



 






  



 

#line 286 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_sdio.h"


 



 




 



 

#line 333 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_sdio.h"


 



 







 



 







 



 






 



 

#line 424 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_sdio.h"



#line 451 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_sdio.h"





 



 







 



 

 
 
 
void SDIO_DeInit(void);

 
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);

 
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint8_t SDIO_GetCommandResponse(void);
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP);

 
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t SDIO_GetDataCounter(void);
uint32_t SDIO_ReadData(void);
void SDIO_WriteData(uint32_t Data);
uint32_t SDIO_GetFIFOCount(void);

 
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);

 
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);

 
void SDIO_DMACmd(FunctionalState NewState);

 
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);









 



 

 
#line 54 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"


























  

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"



 



  

 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 

  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;



 

typedef struct
{

  uint16_t I2S_Mode;         
 

  uint16_t I2S_Standard;     
 

  uint16_t I2S_DataFormat;   
 

  uint16_t I2S_MCLKOutput;   
 

  uint32_t I2S_AudioFreq;    
 

  uint16_t I2S_CPOL;         
 
}I2S_InitTypeDef;

 



 










 
  
#line 138 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 222 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"


  



 







 



 

#line 250 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"


 
  



 

#line 269 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"


 
  


 

#line 285 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"


 



 







 



 

#line 315 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"






 
            


 







 



 






 



 







 



 






 



 







 



 























 



 

#line 422 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"

#line 429 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"


 



 




 



 

#line 465 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_spi.h"


 
  


 

 
  

  
void SPI_I2S_DeInit(SPI_TypeDef* SPIx);

 
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TIModeCmd(SPI_TypeDef* SPIx, FunctionalState NewState);

  
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);

 
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);

 
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);

 
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);









 



 

 
#line 55 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_syscfg.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_syscfg.h"



 



  

 
 
  


  



  
#line 67 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_syscfg.h"
                                      
#line 77 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_syscfg.h"


  




  
#line 117 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_syscfg.h"


  




  




   






  




  







  



  

 
  
 
void SYSCFG_DeInit(void);
void SYSCFG_MemoryRemapConfig(uint8_t SYSCFG_MemoryRemap);
void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);
void SYSCFG_ETH_MediaInterfaceConfig(uint32_t SYSCFG_ETH_MediaInterface); 
void SYSCFG_CompensationCellCmd(FunctionalState NewState); 
FlagStatus SYSCFG_GetCompensationCellStatus(void);









  



  

 
#line 56 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"



 



  

 




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint32_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef; 



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint32_t TIM_Pulse;         
 

  uint16_t TIM_OCPolarity;    
 

  uint16_t TIM_OCNPolarity;   

 

  uint16_t TIM_OCIdleState;   

 

  uint16_t TIM_OCNIdleState;  

 
} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;




 

typedef struct
{

  uint16_t TIM_OSSRState;        
 

  uint16_t TIM_OSSIState;        
 

  uint16_t TIM_LOCKLevel;        
  

  uint16_t TIM_DeadTime;         

 

  uint16_t TIM_Break;            
 

  uint16_t TIM_BreakPolarity;    
 

  uint16_t TIM_AutomaticOutput;  
 
} TIM_BDTRInitTypeDef;

 



 

#line 189 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"
                                          
#line 202 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"
                                     
 
#line 212 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"
 
#line 219 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"
 


 
#line 231 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"
                                






 

#line 260 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


 



 







  



 





                                 




                                 







  



 

#line 309 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


 



 

#line 327 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 451 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 







 



 







  



 







  



 







  



 

#line 513 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 

#line 529 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 

#line 545 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 

#line 562 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"

#line 571 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 

#line 619 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 

#line 663 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 

#line 679 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"



  



 

#line 696 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 

#line 724 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 







  



  






 



 







  



 







  



 

#line 785 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  




 

#line 803 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"
  


  



 

#line 818 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 879 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 

#line 895 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


  



 







  


 














#line 937 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"



  


 

#line 969 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"



  



 




  



 




  



 

#line 1014 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_tim.h"


 



 

 
  

 
void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint32_t Autoreload);
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint32_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint32_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint32_t Compare4);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);

 
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
uint32_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture4(TIM_TypeDef* TIMx);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);

 
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);

 
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);

    
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_RemapConfig(TIM_TypeDef* TIMx, uint16_t TIM_Remap);









  



 

 
#line 57 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"


























  

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"



 



  

  



  
  
typedef struct
{
  uint32_t USART_BaudRate;            



 

  uint16_t USART_WordLength;          
 

  uint16_t USART_StopBits;            
 

  uint16_t USART_Parity;              




 
 
  uint16_t USART_Mode;                
 

  uint16_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  
  
typedef struct
{

  uint16_t USART_Clock;   
 

  uint16_t USART_CPOL;    
 

  uint16_t USART_CPHA;    
 

  uint16_t USART_LastBit; 

 
} USART_ClockInitTypeDef;

 



  
  
#line 116 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"








  
  


                                    




  



  
  
#line 147 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"


  



  
  
#line 161 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"


  



  
  





  



  
#line 188 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 255 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"



 



 

#line 276 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 348 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_usart.h"
                              








  



  

 
   

  
void USART_DeInit(USART_TypeDef* USARTx);

 
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);

  
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

 
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendBreak(USART_TypeDef* USARTx);

 
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);

 
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);

 
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);









  



  

 
#line 58 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_wwdg.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_wwdg.h"



 



  

 
 



  
  


 
  
#line 69 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_wwdg.h"



  



  

 
 
  
   
void WWDG_DeInit(void);

 
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);

 
void WWDG_Enable(uint8_t Counter);

 
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  

 
#line 59 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"
#line 1 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\misc.h"


























 

 







 
#line 39 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\misc.h"



 



 

 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  


 

  uint8_t NVIC_IRQChannelSubPriority;         


 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 
 



 



 







 



 

#line 104 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\misc.h"


 



 

#line 122 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\misc.h"















 



 







 



 

 
 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 

 
#line 60 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"

 
 



 
   



 
 

 
#line 91 "..\\Libraries\\STM32F2xx_StdPeriph_Driver\\inc\\stm32f2xx_conf.h"



 
#line 6925 "..\\Libraries\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"




 

















 









 

  

 

 
#line 37 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
#line 1 "..\\STemWin\\inc\\LCD_Private.h"
































 


















 
  



#line 1 "..\\STemWin\\Config\\LCDConf.h"
































 






 
#line 58 "..\\STemWin\\inc\\LCD_Private.h"
#line 1 "..\\STemWin\\inc\\LCD_Protected.h"
































 


















 
  



#line 1 "..\\STemWin\\inc\\LCD.h"
































 


















 
  



#line 1 "..\\STemWin\\inc\\GUI_ConfDefaults.h"




































 


















 
  



#line 1 "..\\STemWin\\Config\\GUIConf.h"
































 


















 
 






 





 









 







 





 





#line 62 "..\\STemWin\\inc\\GUI_ConfDefaults.h"

#line 70 "..\\STemWin\\inc\\GUI_ConfDefaults.h"



#line 79 "..\\STemWin\\inc\\GUI_ConfDefaults.h"






 







 







 







 
















































 
#line 166 "..\\STemWin\\inc\\GUI_ConfDefaults.h"

 





 











 




 






 
#line 58 "..\\STemWin\\inc\\LCD.h"
#line 1 "..\\STemWin\\inc\\Global.h"
































 


















 
  








 
#line 81 "..\\STemWin\\inc\\Global.h"



 
#line 59 "..\\STemWin\\inc\\LCD.h"















 












 








 







 








 
typedef int LCD_DRAWMODE;
typedef unsigned long LCD_COLOR;




 
typedef struct { signed short x,y; } GUI_POINT;
typedef struct { signed short x0,y0,x1,y1; } LCD_RECT;

typedef struct {
  int              NumEntries;
  char             HasTrans;
  const LCD_COLOR * pPalEntries;
} LCD_LOGPALETTE;

 
typedef struct {
  int x,y;
  unsigned char KeyStat;
} LCD_tMouseState;

typedef struct {
  int               NumEntries;
  const LCD_COLOR * pPalEntries;
} LCD_PHYSPALETTE;




 
typedef LCD_COLOR      tLCDDEV_Index2Color  (unsigned Index);
typedef unsigned int   tLCDDEV_Color2Index  (LCD_COLOR Color);
typedef unsigned int   tLCDDEV_GetIndexMask (void);

typedef void tLCDDEV_Index2ColorBulk(void * pIndex, LCD_COLOR * pColor, unsigned long NumItems, unsigned char SizeOfIndex);
typedef void tLCDDEV_Color2IndexBulk(LCD_COLOR * pColor, void * pIndex, unsigned long NumItems, unsigned char SizeOfIndex);




 
typedef struct {
  tLCDDEV_Color2Index  * pfColor2Index;
  tLCDDEV_Index2Color  * pfIndex2Color;
  tLCDDEV_GetIndexMask * pfGetIndexMask;
  int NoAlpha;
  tLCDDEV_Color2IndexBulk * pfColor2IndexBulk;
  tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk;
} LCD_API_COLOR_CONV;

extern const LCD_API_COLOR_CONV LCD_API_ColorConv_0;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_2;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_4;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_5;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_8;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_16;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_24;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_2;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_4;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_5;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_6;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_8;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_16;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1616I;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_111;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_222;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_233;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_323;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_332;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_444_12;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_444_12_1;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_444_16;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_555;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_565;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_556;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_655;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_666;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_666_9;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_822216;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_84444;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_8666;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_8666_1;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_88666I;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_888;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_8888;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M111;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M1555I;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M222;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M233;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M323;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M332;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M4444I;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M444_12;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M444_12_1;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M444_16;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M555;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M565;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M556;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M655;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M666;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M666_9;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M8565;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M888;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M8888;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M8888I;

#line 278 "..\\STemWin\\inc\\LCD.h"

void GUICC_M1555I_SetCustColorConv(tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);
void GUICC_M565_SetCustColorConv  (tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);
void GUICC_M4444I_SetCustColorConv(tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);
void GUICC_M888_SetCustColorConv  (tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);
void GUICC_M8888I_SetCustColorConv(tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);




 
#line 297 "..\\STemWin\\inc\\LCD.h"












 
typedef void         tLCDDEV_DrawPixel    (int x, int y);
typedef void         tLCDDEV_DrawHLine    (int x0, int y0,  int x1);
typedef void         tLCDDEV_DrawVLine    (int x , int y0,  int y1);
typedef void         tLCDDEV_FillRect     (int x0, int y0, int x1, int y1);
typedef unsigned int tLCDDEV_GetPixelIndex(int x, int y);
typedef void         tLCDDEV_SetPixelIndex(int x, int y, int ColorIndex);
typedef void         tLCDDEV_XorPixel     (int x, int y);
typedef void         tLCDDEV_FillPolygon  (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
typedef void         tLCDDEV_FillPolygonAA(const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
typedef void         tLCDDEV_GetRect      (LCD_RECT * pRect);
typedef int          tLCDDEV_Init         (void);
typedef void         tLCDDEV_On           (void);
typedef void         tLCDDEV_Off          (void);
typedef void         tLCDDEV_SetLUTEntry  (unsigned char Pos, LCD_COLOR color);
typedef void *       tLCDDEV_GetDevFunc   (int Index);
typedef signed long          tLCDDEV_GetDevProp   (int Index);
typedef void         tLCDDEV_SetOrg       (int x, int y);




 
typedef struct GUI_DEVICE     GUI_DEVICE;
typedef struct GUI_DEVICE_API GUI_DEVICE_API;

typedef void tLCDDEV_DrawBitmap   (int x0, int y0, int xsize, int ysize,
                       int BitsPerPixel, int BytesPerLine,
                       const unsigned char * pData, int Diff,
                       const void * pTrans);    


















 

#line 374 "..\\STemWin\\inc\\LCD.h"

int LCD_GetXSizeMax(void);
int LCD_GetYSizeMax(void);
int LCD_GetVXSizeMax(void);
int LCD_GetVYSizeMax(void);
int LCD_GetBitsPerPixelMax(void);
void LCD_SetDisplaySize(int xSizeDisplay, int ySizeDisplay);
int LCD_GetXSizeDisplay(void);
int LCD_GetYSizeDisplay(void);

int LCD_GetXSizeEx          (int LayerIndex);
int LCD_GetYSizeEx          (int LayerIndex);
int LCD_GetVXSizeEx         (int LayerIndex);
int LCD_GetVYSizeEx         (int LayerIndex);
int LCD_GetBitsPerPixelEx   (int LayerIndex);
unsigned long LCD_GetNumColorsEx      (int LayerIndex);
int LCD_GetXMagEx           (int LayerIndex);
int LCD_GetYMagEx           (int LayerIndex);
int LCD_GetMirrorXEx        (int LayerIndex);
int LCD_GetMirrorYEx        (int LayerIndex);
int LCD_GetSwapXYEx         (int LayerIndex);
int LCD_GetReversLUTEx      (int LayerIndex);
int LCD_GetPhysColorsInRAMEx(int LayerIndex);

int LCD_GetXSize            (void);
int LCD_GetYSize            (void);
int LCD_GetVXSize           (void);
int LCD_GetVYSize           (void);
int LCD_GetBitsPerPixel     (void);
unsigned long LCD_GetNumColors        (void);
int LCD_GetXMag             (void);
int LCD_GetYMag             (void);
int LCD_GetMirrorX          (void);
int LCD_GetMirrorY          (void);
int LCD_GetSwapXY           (void);
int LCD_GetReversLUT        (void);
int LCD_GetPhysColorsInRAM  (void);

signed long LCD__GetBPP      (unsigned long IndexMask);
signed long LCD__GetBPPDevice(unsigned long IndexMask);

tLCDDEV_Index2Color * LCD_GetpfIndex2ColorEx(int LayerIndex);
tLCDDEV_Color2Index * LCD_GetpfColor2IndexEx(int LayerIndex);

tLCDDEV_Color2Index * LCD_GetpfColor2Index(void);

int LCD_GetNumLayers(void);

LCD_COLOR * LCD_GetPalette  (void);
LCD_COLOR * LCD_GetPaletteEx(int LayerIndex);

void (* LCD_GetDevFunc(int LayerIndex, int Item))(void);




 
                                        
#line 456 "..\\STemWin\\inc\\LCD.h"
                                        
#line 467 "..\\STemWin\\inc\\LCD.h"




 
                                           





 
                                        






 
typedef struct {
  void * pVRAM;
} LCD_X_SETVRAMADDR_INFO;

typedef struct {
  int xPos, yPos;
} LCD_X_SETORG_INFO;

typedef struct {
  LCD_COLOR Color;
  unsigned char Pos;
} LCD_X_SETLUTENTRY_INFO;

typedef struct {
  int xSize, ySize;
} LCD_X_SETSIZE_INFO;

typedef struct {
  int xPos, yPos;
  int xLen, yLen;
  int BytesPerPixel;
  unsigned long Off;
} LCD_X_SETPOS_INFO;

typedef struct {
  int Alpha;
} LCD_X_SETALPHA_INFO;

typedef struct {
  int OnOff;
} LCD_X_SETVIS_INFO;

typedef struct {
  int AlphaMode;
} LCD_X_SETALPHAMODE_INFO;

typedef struct {
  int ChromaMode;
} LCD_X_SETCHROMAMODE_INFO;

typedef struct {
  LCD_COLOR ChromaMin;
  LCD_COLOR ChromaMax;
} LCD_X_SETCHROMA_INFO;

typedef struct {
  int Index;
} LCD_X_SHOWBUFFER_INFO;




 
#line 554 "..\\STemWin\\inc\\LCD.h"

int  LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData);
void LCD_X_Config(void);




 
int LCD_SetAlphaEx     (int LayerIndex, int Alpha);
int LCD_SetPosEx       (int LayerIndex, int xPos, int yPos);
int LCD_SetSizeEx      (int LayerIndex, int xSize, int ySize);
int LCD_SetVisEx       (int LayerIndex, int OnOff);
int LCD_SetVRAMAddrEx  (int LayerIndex, void * pVRAM);
int LCD_SetVSizeEx     (int LayerIndex, int xSize, int ySize);
int LCD_SetAlphaModeEx (int LayerIndex, int AlphaMode);
int LCD_SetChromaModeEx(int LayerIndex, int ChromaMode);
int LCD_SetChromaEx    (int LayerIndex, LCD_COLOR ChromaMin, LCD_COLOR ChromaMax);

int LCD_SetAlpha     (int Alpha);
int LCD_SetVRAMAddr  (void * pVRAM);
int LCD_SetVSize     (int xSize, int ySize);
int LCD_SetSize      (int xSize, int ySize);
int LCD_SetVis       (int OnOff);
int LCD_SetPos       (int xPos, int yPos);
int LCD_SetAlphaMode (int AlphaMode);
int LCD_SetChromaMode(int ChromaMode);
int LCD_SetChroma    (LCD_COLOR ChromaMin, LCD_COLOR ChromaMax);
int LCD_SetLUTEntry  (unsigned char Pos, LCD_COLOR Color);
int LCD_SetDevFunc   (int LayerIndex, int IdFunc, void (* pDriverFunc)(void));




 
int LCD_GetPosEx(int LayerIndex, int * pxPos, int * pyPos);

int LCD_GetPos  (int * pxPos, int * pyPos);




 
int LCD_Refresh  (void);
int LCD_RefreshEx(int LayerIndex);




 
typedef struct {
  int  (* pfStart)   (int x0, int y0, int x1, int y1);
  void (* pfSetPixel)(int PixelIndex);
  void (* pfNextLine)(void);
  void (* pfEnd)     (void);
} LCD_API_NEXT_PIXEL;

LCD_API_NEXT_PIXEL * LCD_GetNextPixelAPI(void);




 
typedef void tLCD_HL_DrawHLine    (int x0, int y0,  int x1);
typedef void tLCD_HL_DrawPixel    (int x0, int y0);

typedef struct {
  tLCD_HL_DrawHLine * pfDrawHLine;
  tLCD_HL_DrawPixel * pfDrawPixel;
} tLCD_HL_APIList;

void LCD_DrawHLine(int x0, int y0,  int x1);
void LCD_DrawPixel(int x0, int y0);
void LCD_DrawVLine(int x,  int y0,  int y1);





 
void LCD_SetClipRectEx(const LCD_RECT * pRect);
void LCD_SetClipRectMax(void);

 
signed long  LCD_GetDevCap  (int Index);
signed long  LCD_GetDevCapEx(int LayerIndex, int Index);

 
int emWin_LCD_Init(void);
int LCD_InitColors(void);

void LCD_SetBkColor   (LCD_COLOR Color);  
void LCD_SetColor     (LCD_COLOR Color);  
void LCD_SetPixelIndex(int x, int y, int ColorIndex);

 
void LCD_InitLUT(void);
int  LCD_SetLUTEntryEx(int LayerIndex, unsigned char Pos, LCD_COLOR Color);
void LCD_SetLUTEx(int LayerIndex, const LCD_PHYSPALETTE * pPalette);
void LCD_SetLUT  (const LCD_PHYSPALETTE * pPalette);

LCD_DRAWMODE LCD_SetDrawMode  (LCD_DRAWMODE dm);
void LCD_SetColorIndex(unsigned PixelIndex);
void LCD_SetBkColorIndex(unsigned PixelIndex);
void LCD_FillRect(int x0, int y0, int x1, int y1);
typedef void tLCD_SetPixelAA(int x, int y, unsigned char Intens);

void LCD_SetPixelAA4_Trans  (int x, int y, unsigned char Intens);
void LCD_SetPixelAA4_NoTrans(int x, int y, unsigned char Intens);

void LCD_SetPixelAA8_Trans  (int x, int y, unsigned char Intens);
void LCD_SetPixelAA8_NoTrans(int x, int y, unsigned char Intens);

LCD_COLOR    LCD_AA_MixColors16 (LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens);
LCD_COLOR    LCD_AA_MixColors256(LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens);
LCD_COLOR    LCD_MixColors256   (LCD_COLOR Color, LCD_COLOR BkColor, unsigned Intens);
LCD_COLOR    LCD_GetPixelColor(int x, int y);      
unsigned int LCD_GetPixelIndex(int x, int y);
int          LCD_GetBkColorIndex (void);
int          LCD_GetColorIndex (void);



unsigned long          LCD_AA_SetAndMask(unsigned long AndMask);


 
int LCD_SetMaxNumColors(unsigned MaxNumColors);




 


typedef void tLCD_DrawBitmap(int x0, int y0, int xsize, int ysize,
                             int xMul, int yMul, int BitsPerPixel, int BytesPerLine,
                             const unsigned char * pPixel, const void * pTrans);
typedef void tRect2TextRect (LCD_RECT * pRect);

struct tLCD_APIList_struct {
  tLCD_DrawBitmap   * pfDrawBitmap;
  tRect2TextRect    * pfRect2TextRect;
  tRect2TextRect    * pfTransformRect;
};

typedef struct tLCD_APIList_struct tLCD_APIList;

extern tLCD_APIList LCD_APIListCCW;
extern tLCD_APIList LCD_APIListCW;
extern tLCD_APIList LCD_APIList180;







tLCD_SetPixelAA * LCD__GetPfSetPixel(int BitsPerPixel);






 
void LCD__SetPhysColor(unsigned char Pos, LCD_COLOR Color);




 




int LCD_ControlCache  (int Cmd);
int LCD_ControlCacheEx(int LayerIndex, int Cmd);




 
unsigned         LCD_Color2Index     (LCD_COLOR Color);
LCD_COLOR        LCD_Index2Color     (int Index);
LCD_COLOR        LCD_Index2ColorEx   (int i, unsigned LayerIndex);




 
unsigned char LCD_X_Read00(void);
unsigned char LCD_X_Read01(void);
void LCD_X_Write00 (unsigned char c);
void LCD_X_Write01 (unsigned char c);
void LCD_X_WriteM01(unsigned char * pData, int NumBytes);







 
#line 58 "..\\STemWin\\inc\\LCD_Protected.h"








 
typedef struct {
  LCD_COLOR * paColor;
  signed short         NumEntries;
} LCD_LUT_INFO;

typedef struct {
  tLCDDEV_DrawPixel  * pfDrawPixel;
  tLCDDEV_DrawHLine  * pfDrawHLine;
  tLCDDEV_DrawVLine  * pfDrawVLine;
  tLCDDEV_FillRect   * pfFillRect;
  tLCDDEV_DrawBitmap * pfDrawBitmap;
} LCD_API_LIST;




 
extern const unsigned char LCD_aMirror[256];
extern unsigned long * LCD__aConvTable;




 
void LCD_UpdateColorIndices   (void);
int  LCD_PassingBitmapsAllowed(void);
void LCD_EnableCursor         (int OnOff);
void LCD_SelectLCD            (void);

void LCD_DrawBitmap(int x0,    int y0,
                    int xsize, int ysize,
                    int xMul,  int yMul,
                    int BitsPerPixel,
                    int BytesPerLine,
                    const unsigned char * pPixel,
                    const unsigned long * pTrans);

void LCD__DrawBitmap_1bpp(int x0,    int y0,
                          int xsize, int ysize,
                          int xMul,  int yMul,
                          int BitsPerPixel,
                          int BytesPerLine,
                          const unsigned char * pPixel,
                          const unsigned long * pTrans,
                          int OffData);




 
tLCDDEV_Index2Color LCD_Index2Color_444_12;
tLCDDEV_Index2Color LCD_Index2Color_M444_12;
tLCDDEV_Index2Color LCD_Index2Color_444_12_1;
tLCDDEV_Index2Color LCD_Index2Color_M444_12_1;
tLCDDEV_Index2Color LCD_Index2Color_444_16;
tLCDDEV_Index2Color LCD_Index2Color_M444_16;
tLCDDEV_Index2Color LCD_Index2Color_555;
tLCDDEV_Index2Color LCD_Index2Color_565;
tLCDDEV_Index2Color LCD_Index2Color_8666;
tLCDDEV_Index2Color LCD_Index2Color_888;
tLCDDEV_Index2Color LCD_Index2Color_8888;
tLCDDEV_Index2Color LCD_Index2Color_M8888I;
tLCDDEV_Index2Color LCD_Index2Color_M555;
tLCDDEV_Index2Color LCD_Index2Color_M565;
tLCDDEV_Index2Color LCD_Index2Color_M888;

tLCDDEV_Color2Index LCD_Color2Index_8666;







 

#line 59 "..\\STemWin\\inc\\LCD_Private.h"
#line 1 "..\\STemWin\\inc\\GUI.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\GUI.h"
#line 1 "..\\STemWin\\inc\\GUI_Type.h"




































 


















 
  



#line 62 "..\\STemWin\\inc\\GUI_Type.h"
#line 63 "..\\STemWin\\inc\\GUI_Type.h"




 
typedef const char *  GUI_ConstString;

typedef LCD_COLOR       GUI_COLOR;
typedef LCD_LOGPALETTE  GUI_LOGPALETTE;
typedef LCD_DRAWMODE    GUI_DRAWMODE;
typedef LCD_RECT        GUI_RECT;

typedef struct {
  void      (* pfDraw)  (int x0,
                         int y0,
                         int xsize, 
                         int ysize, 
                         const unsigned char * pPixel, 
                         const LCD_LOGPALETTE * pLogPal, 
                         int xMag, 
                         int yMag);
  GUI_COLOR (* pfIndex2Color)(unsigned Index);
  void      (* pfDrawHW)(int x0,
                         int y0,
                         int xsize, 
                         int ysize, 
                         const unsigned char * pPixel, 
                         const LCD_LOGPALETTE * pLogPal, 
                         int xMag, 
                         int yMag);
  const LCD_API_COLOR_CONV * pColorConvAPI;
} GUI_BITMAP_METHODS;

typedef struct {
  unsigned short XSize;
  unsigned short YSize;
  unsigned short BytesPerLine;
  unsigned short BitsPerPixel;
  const unsigned char * pData;
  const GUI_LOGPALETTE * pPal;
  const GUI_BITMAP_METHODS * pMethods;
} GUI_BITMAP;





 
typedef struct {
  unsigned short ID;
  unsigned short Format;
  unsigned short XSize;
  unsigned short YSize;
  unsigned short BytesPerLine;
  unsigned short BitsPerPixel;
  unsigned short NumColors;
  unsigned short HasTrans;
} GUI_BITMAP_STREAM;

typedef struct {
  int    Cmd;
  unsigned long    v;
  void * p;
} GUI_BITMAPSTREAM_PARAM;

typedef struct {
  int XSize;
  int YSize;
  int BitsPerPixel;
  int NumColors;
  int HasTrans;
} GUI_BITMAPSTREAM_INFO;

typedef void * (* GUI_BITMAPSTREAM_CALLBACK)(GUI_BITMAPSTREAM_PARAM * pParam);

typedef struct {
  int x,y;
  unsigned char  Pressed;
  unsigned char  Layer;
} GUI_PID_STATE;

typedef struct {
  int Key;
  int Pressed;
} GUI_KEY_STATE;

typedef struct {
  int xPos;
  int yPos;
  int xSize;
  int ySize;
  int Delay;
} GUI_GIF_IMAGE_INFO;

typedef struct {
  int xSize;
  int ySize;
  int NumImages;
} GUI_GIF_INFO;

typedef struct GUI_REGISTER_EXIT GUI_REGISTER_EXIT;

struct GUI_REGISTER_EXIT {
  void (* pfVoid)(void);
  GUI_REGISTER_EXIT * pNext;
};

typedef struct {
  void (* cbBegin)(void);
  void (* cbEnd)  (void);
} GUI_MULTIBUF_API;

typedef struct {
  void (* cbBeginEx)(int LayerIndex);
  void (* cbEndEx)  (int LayerIndex);
} GUI_MULTIBUF_API_EX;




 



 
typedef struct {
  signed short c0;
  signed short c1;
} GUI_FONT_TRANSLIST;

typedef struct {
  unsigned short FirstChar;
  unsigned short LastChar;
  const GUI_FONT_TRANSLIST * pList;
} GUI_FONT_TRANSINFO;

typedef struct {
  unsigned char XSize;
  unsigned char XDist;
  unsigned char BytesPerLine;
  const unsigned char * pData;
} GUI_CHARINFO;

typedef struct {
  unsigned char XSize;
  unsigned char YSize;
  signed char XPos;
  signed char YPos;
  unsigned char XDist;
  const unsigned char * pData;
} GUI_CHARINFO_EXT;

typedef struct GUI_FONT_PROP {
  unsigned short First;                                   
  unsigned short Last;                                    
  const GUI_CHARINFO         * paCharInfo;      
  const struct GUI_FONT_PROP * pNext;           
} GUI_FONT_PROP;

typedef struct GUI_FONT_PROP_EXT {
  unsigned short First;                                   
  unsigned short Last;                                    
  const GUI_CHARINFO_EXT         * paCharInfo;  
  const struct GUI_FONT_PROP_EXT * pNext;       
} GUI_FONT_PROP_EXT;

typedef struct {
  const unsigned char      * pData;
  const unsigned char                 * pTransData;
  const GUI_FONT_TRANSINFO * pTrans;
  unsigned short                       FirstChar;
  unsigned short                       LastChar;
  unsigned char                         XSize;
  unsigned char                         XDist;
  unsigned char                         BytesPerLine;
} GUI_FONT_MONO;







 
typedef struct {
  unsigned short Flags;
  unsigned char Baseline;
  unsigned char LHeight;      
  unsigned char CHeight;      
} GUI_FONTINFO;

#line 260 "..\\STemWin\\inc\\GUI_Type.h"




 
typedef unsigned short  tGUI_GetCharCode   (const char * s);
typedef int  tGUI_GetCharSize   (const char * s);
typedef int  tGUI_CalcSizeOfChar(unsigned short Char);
typedef int  tGUI_Encode        (char * s, unsigned short Char);

typedef struct {
  tGUI_GetCharCode    * pfGetCharCode;
  tGUI_GetCharSize    * pfGetCharSize;
  tGUI_CalcSizeOfChar * pfCalcSizeOfChar;
  tGUI_Encode         * pfEncode;
} GUI_UC_ENC_APILIST;




 
typedef int  tGUI_GetLineDistX(const char * s, int Len);
typedef int  tGUI_GetLineLen  (const char * s, int MaxLen);
typedef void tGL_DispLine     (const char * s, int Len);

typedef struct {
  tGUI_GetLineDistX * pfGetLineDistX;
  tGUI_GetLineLen   * pfGetLineLen;
  tGL_DispLine      * pfDispLine;
} tGUI_ENC_APIList;

extern const tGUI_ENC_APIList GUI_ENC_APIList_SJIS;
extern const tGUI_ENC_APIList GUI_ENC_APIList_EXT;




 
typedef struct GUI_FONT GUI_FONT;

typedef void GUI_DISPCHAR    (unsigned short c);
typedef int  GUI_GETCHARDISTX(unsigned short c, int * pSizeX);
typedef void GUI_GETFONTINFO (const GUI_FONT * pFont, GUI_FONTINFO * pfi);
typedef char GUI_ISINFONT    (const GUI_FONT * pFont, unsigned short c);
typedef int  GUI_GETCHARINFO (unsigned short c, GUI_CHARINFO_EXT * pInfo);

#line 312 "..\\STemWin\\inc\\GUI_Type.h"





void GUIMONO_DispChar (unsigned short c); int GUIMONO_GetCharDistX(unsigned short c, int * pSizeX); void GUIMONO_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIMONO_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIMONO_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_DispChar (unsigned short c); int GUIPROP_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_EXT_DispChar (unsigned short c); int GUIPROP_EXT_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_EXT_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_EXT_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_EXT_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_FRM_DispChar (unsigned short c); int GUIPROP_FRM_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_FRM_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_FRM_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_FRM_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROPAA_DispChar (unsigned short c); int GUIPROPAA_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROPAA_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROPAA_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROPAA_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_AA2_DispChar (unsigned short c); int GUIPROP_AA2_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_AA2_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_AA2_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_AA2_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_AA2_EXT_DispChar (unsigned short c); int GUIPROP_AA2_EXT_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_AA2_EXT_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_AA2_EXT_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_AA2_EXT_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_AA4_DispChar (unsigned short c); int GUIPROP_AA4_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_AA4_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_AA4_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_AA4_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_AA4_EXT_DispChar (unsigned short c); int GUIPROP_AA4_EXT_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_AA4_EXT_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_AA4_EXT_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_AA4_EXT_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);

 
#line 335 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 344 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 353 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 362 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 371 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 380 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 389 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 398 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 407 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 416 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 425 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 434 "..\\STemWin\\inc\\GUI_Type.h"





struct GUI_FONT {
  GUI_DISPCHAR     * pfDispChar; 
  GUI_GETCHARDISTX * pfGetCharDistX; 
  GUI_GETFONTINFO  * pfGetFontInfo; 
  GUI_ISINFONT     * pfIsInFont;
  GUI_GETCHARINFO  * pfGetCharInfo;
  const tGUI_ENC_APIList* pafEncode;
  unsigned char YSize;
  unsigned char YDist;
  unsigned char XMag;
  unsigned char YMag;
  union {
    const void              * pFontData;
    const GUI_FONT_MONO     * pMono;
    const GUI_FONT_PROP     * pProp;
    const GUI_FONT_PROP_EXT * pPropExt;
  } p;
  unsigned char Baseline;
  unsigned char LHeight;      
  unsigned char CHeight;      
};




 
typedef void GUI_CALLBACK_VOID_U8_P(unsigned char Data, void * p);




 
typedef struct {
  unsigned long ID;            
  unsigned short YSize;         
  unsigned short YDist;         
  unsigned short Baseline;      
  unsigned short LHeight;       
  unsigned short CHeight;       
  unsigned short NumAreas;      
} GUI_SI_FONT;

typedef struct {
  unsigned short First;         
  unsigned short Last;          
} GUI_SIF_CHAR_AREA;

typedef struct {
  unsigned short XSize;         
  unsigned short XDist;         
  unsigned short BytesPerLine;  
  unsigned short Dummy;
  unsigned long OffData;       
} GUI_SIF_CHARINFO;

typedef struct {
  unsigned short XSize;         
  unsigned short YSize;         
  signed short XOff;          
  signed short YOff;          
  unsigned short XDist;         
  unsigned short Dummy;
  unsigned long OffData;       
} GUI_SIF_CHARINFO_EXT;

typedef struct tGUI_SIF_APIList_struct {
  GUI_DISPCHAR          * pfDispChar;
  GUI_GETCHARDISTX      * pfGetCharDistX;
  GUI_GETFONTINFO       * pfGetFontInfo;
  GUI_ISINFONT          * pfIsInFont;
  GUI_GETCHARINFO       * pfGetCharInfo;
  const tGUI_ENC_APIList* pafEncode;
} tGUI_SIF_APIList;

#line 521 "..\\STemWin\\inc\\GUI_Type.h"




 
typedef int GUI_XBF_GET_DATA_FUNC(unsigned long Off, unsigned short NumBytes, void * pVoid, void * pBuffer);

typedef struct {
  unsigned short First;                          
  unsigned short Last;                           
  void * pVoid;                       
  GUI_XBF_GET_DATA_FUNC * pfGetData;  
} GUI_XBF_DATA;

typedef struct tGUI_XBF_APIList_struct {
  GUI_DISPCHAR          * pfDispChar;
  GUI_GETCHARDISTX      * pfGetCharDistX;
  GUI_GETFONTINFO       * pfGetFontInfo;
  GUI_ISINFONT          * pfIsInFont;
  GUI_GETCHARINFO       * pfGetCharInfo;
  const tGUI_ENC_APIList* pafEncode;
} tGUI_XBF_APIList;

#line 550 "..\\STemWin\\inc\\GUI_Type.h"




 
typedef struct {
  const void * pData;       
  unsigned long NumBytes;             
} GUI_TTF_DATA;

typedef struct {
  GUI_TTF_DATA * pTTF;      
  unsigned long aImageTypeBuffer[4];  
  int PixelHeight;         


 
  int FaceIndex;           

 
} GUI_TTF_CS;




 
typedef void (* GUI_SIGNAL_EVENT_FUNC)    (void);
typedef void (* GUI_WAIT_EVENT_FUNC)      (void);
typedef void (* GUI_WAIT_EVENT_TIMED_FUNC)(int Period);




 




typedef     signed long      GUI_HWIN;
typedef     signed long      GUI_HSPRITE;




 




typedef struct {
  signed long x;
  signed long y;
  unsigned long Id;
  unsigned short Flags;
} GUI_MTOUCH_INPUT;

typedef struct {
  int            LayerIndex;
  unsigned       NumPoints;
  int TimeStamp;
  signed long       hInput;
} GUI_MTOUCH_EVENT;




typedef struct {
  unsigned char  Layer;
  unsigned char  NumPoints;
  signed short ax[5];
  signed short ay[5];
  unsigned short aId[5];
  unsigned char  aFlags[5];
} GUI_MTOUCH_STATE;

typedef void (* T_GUI_MTOUCH_STOREEVENT)(GUI_MTOUCH_EVENT *, GUI_MTOUCH_INPUT * pInput);




 
typedef struct {
  
  
  
  void (* pfWrite8_A0)  (unsigned char Data);
  void (* pfWrite8_A1)  (unsigned char Data);
  void (* pfWriteM8_A0) (unsigned char * pData, int NumItems);
  void (* pfWriteM8_A1) (unsigned char * pData, int NumItems);
  unsigned char   (* pfRead8_A0)   (void);
  unsigned char   (* pfRead8_A1)   (void);
  void (* pfReadM8_A0)  (unsigned char * pData, int NumItems);
  void (* pfReadM8_A1)  (unsigned char * pData, int NumItems);
  
  
  
  void (* pfWrite16_A0) (unsigned short Data);
  void (* pfWrite16_A1) (unsigned short Data);
  void (* pfWriteM16_A0)(unsigned short * pData, int NumItems);
  void (* pfWriteM16_A1)(unsigned short * pData, int NumItems);
  unsigned short  (* pfRead16_A0)  (void);
  unsigned short  (* pfRead16_A1)  (void);
  void (* pfReadM16_A0) (unsigned short * pData, int NumItems);
  void (* pfReadM16_A1) (unsigned short * pData, int NumItems);
  
  
  
  void (* pfWrite32_A0) (unsigned long Data);
  void (* pfWrite32_A1) (unsigned long Data);
  void (* pfWriteM32_A0)(unsigned long * pData, int NumItems);
  void (* pfWriteM32_A1)(unsigned long * pData, int NumItems);
  unsigned long  (* pfRead32_A0)  (void);
  unsigned long  (* pfRead32_A1)  (void);
  void (* pfReadM32_A0) (unsigned long * pData, int NumItems);
  void (* pfReadM32_A1) (unsigned long * pData, int NumItems);
  
  
  
  void (* pfSetCS)      (unsigned char NotActive);
  
  
  
  void (* pfFlushBuffer)(void);
} GUI_PORT_API;




 
typedef int    (* GUI_tSend)  (const unsigned char * pData, int len, void * p);
typedef int    (* GUI_tRecv)  (      unsigned char * pData, int len, void * p);




 
typedef void * (* GUI_tMalloc)(unsigned int);
typedef void   (* GUI_tFree)  (void *);



 
#line 59 "..\\STemWin\\inc\\GUI.h"
#line 1 "..\\STemWin\\inc\\GUI_Version.h"
































 


















 
  







 
#line 60 "..\\STemWin\\inc\\GUI.h"








 









 








 
typedef struct GUI_CONTEXT GUI_CONTEXT;

#line 108 "..\\STemWin\\inc\\GUI.h"




 
struct GUI_DEVICE_API {
  
  
  
  int DeviceClassIndex;
  
  
  
  void     (* pfDrawBitmap   )(GUI_DEVICE *  pDevice,  int x0, int y0, int xsize, int ysize, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, int Diff, const unsigned long * pTrans);
  void     (* pfDrawHLine    )(GUI_DEVICE *  pDevice,  int x0, int y0,  int x1);
  void     (* pfDrawVLine    )(GUI_DEVICE *  pDevice,  int x , int y0,  int y1);
  void     (* pfFillRect     )(GUI_DEVICE *  pDevice,  int x0, int y0, int x1, int y1);
  unsigned (* pfGetPixelIndex)(GUI_DEVICE *  pDevice,  int x, int y);
  void     (* pfSetPixelIndex)(GUI_DEVICE *  pDevice,  int x, int y, int ColorIndex);
  void     (* pfXorPixel     )(GUI_DEVICE *  pDevice,  int x, int y);
  
  
  
  void     (* pfSetOrg       )(GUI_DEVICE *  pDevice,  int x, int y);
  
  
  
  void   (*(* pfGetDevFunc)   (GUI_DEVICE ** ppDevice, int Index))(void);
  signed long      (* pfGetDevProp   )(GUI_DEVICE *  pDevice,  int Index);
  void    *(* pfGetDevData   )(GUI_DEVICE *  pDevice,  int Index);
  void     (* pfGetRect      )(GUI_DEVICE *  pDevice,  LCD_RECT * pRect);
};




 
typedef enum {
  DEVICE_CLASS_DRIVER = 0,
  DEVICE_CLASS_DRIVER_MODIFIER,   
  DEVICE_CLASS_VNC,
  DEVICE_CLASS_SPRITE,
  DEVICE_CLASS_MEMDEV,
  DEVICE_CLASS_ALPHA,
  DEVICE_CLASS_AUTOALPHA,
  DEVICE_CLASS_MEASDEV
} DEVICE_CLASS;






 



extern const GUI_DEVICE_API GUIDRV_Win_API;

extern const GUI_DEVICE_API GUIDRV_Template_API;



















 
struct GUI_DEVICE {
  
  
  
  GUI_DEVICE * pNext;
  GUI_DEVICE * pPrev;
  
  
  
  union {
    signed long hContext; 
    void   * pContext; 
  } u;
  
  
  
  const GUI_DEVICE_API     * pDeviceAPI;
  const LCD_API_COLOR_CONV * pColorConvAPI;
  unsigned short Flags;
  int LayerIndex;
};

extern const GUI_DEVICE_API GUI_MEMDEV_DEVICE_1;
extern const GUI_DEVICE_API GUI_MEMDEV_DEVICE_8;
extern const GUI_DEVICE_API GUI_MEMDEV_DEVICE_16;
extern const GUI_DEVICE_API GUI_MEMDEV_DEVICE_32;







 
typedef union {
  unsigned char  aColorIndex8[2];
  unsigned short aColorIndex16[2];
  unsigned long aColorIndex32[2];
} LCD_COLORINDEX_UNION;

struct GUI_CONTEXT {
  
  
  
  LCD_COLORINDEX_UNION uLCD;
  LCD_RECT       ClipRect;
  unsigned char             DrawMode;
  unsigned char             SelLayer;
  unsigned char             TextStyle;
  
  
  
  GUI_RECT * pClipRect_HL;                 
  unsigned char         PenSize;
  unsigned char         PenShape;
  unsigned char         LineStyle;
  
  
  
  const GUI_FONT * pAFont;
  signed short LBorder;
  signed short DispPosX, DispPosY;
  signed short DrawPosX, DrawPosY;
  signed short TextMode, TextAlign;
  GUI_COLOR Color, BkColor;                
  
  
  
  unsigned long * LCD_pBkColorIndex;
  unsigned long * LCD_pColorIndex;
  unsigned long * LCD_pAColorIndex;
  
  
  

    const GUI_RECT * WM__pUserClipRect;
    GUI_HWIN hAWin;
    int xOff, yOff;
    unsigned char WM_IsActive;

  
  
  
  
  GUI_DEVICE * apDriver[2];
  
  
  
  signed long    hDevData;
  
  
  
  const tLCD_HL_APIList * pLCD_HL;       
  unsigned char AA_Factor;
  unsigned char AA_HiResEnable;
  void (* AA_pfSetPixelAA)(int x, int y, unsigned char Intens); 
};

 









 
GUI_DEVICE * GUI_DEVICE_Create       (const GUI_DEVICE_API * pDeviceAPI, const LCD_API_COLOR_CONV * pColorConvAPI, unsigned short Flags, int LayerIndex);
GUI_DEVICE * GUI_DEVICE_CreateAndLink(const GUI_DEVICE_API * pDeviceAPI, const LCD_API_COLOR_CONV * pColorConvAPI, unsigned short Flags, int LayerIndex);
void         GUI_DEVICE_Delete       (GUI_DEVICE * pDevice);
int          GUI_DEVICE_Link         (GUI_DEVICE * pDevice);
void         GUI_DEVICE_Unlink       (GUI_DEVICE * pDevice);
GUI_DEVICE * GUI_DEVICE__GetpDriver  (int LayerIndex);
GUI_DEVICE * GUI_DEVICE__GetpDevice  (int LayerIndex, int DeviceClass);

GUI_DEVICE * GUI_DEVICE_UnlinkTaskDevices(void);
void         GUI_DEVICE_LinkDevices      (GUI_DEVICE * pDevice);

void _RegisterExit(void);



 
typedef struct {
  void * pData;         
  int    x0, y0;        
  int    xSize, ySize;  
  int    LineOff;       
  int    BytesPerPixel; 
  int    IsDirty;       
} GUI_DIRTYDEVICE_INFO;

int GUI_DIRTYDEVICE_Create      (void);
int GUI_DIRTYDEVICE_CreateEx    (int LayerIndex);
int GUI_DIRTYDEVICE_CreateExInfo(GUI_DIRTYDEVICE_INFO * pInfo, int LayerIndex);
int GUI_DIRTYDEVICE_Delete      (void);
int GUI_DIRTYDEVICE_DeleteEx    (int LayerIndex);
int GUI_DIRTYDEVICE_Fetch       (GUI_DIRTYDEVICE_INFO * pInfo);
int GUI_DIRTYDEVICE_FetchEx     (GUI_DIRTYDEVICE_INFO * pInfo, int LayerIndex);




 
typedef struct {
  int xPos;
  int yPos;
  int xSize;
  int ySize;
  int Visible;
} GUI_SOFTLAYER_CONFIG;

int  GUI_SOFTLAYER_Enable           (GUI_SOFTLAYER_CONFIG * pConfig, int NumLayers, GUI_COLOR CompositeColor);
int  GUI_SOFTLAYER_Refresh          (void);
void GUI_SOFTLAYER_SetCompositeColor(unsigned long Color);
int  GUI_SOFTLAYER_MULTIBUF_Enable  (int OnOff);




 
int          GUI_Init             (void);
int          GUI_IsInitialized    (void);
void         GUI_Exit             (void);
void         GUI_SetDefaultFont   (const GUI_FONT * pFont);
void         GUI_SetDefault       (void);
GUI_DRAWMODE GUI_SetDrawMode      (GUI_DRAWMODE dm);
const char * GUI_GetVersionString (void);
void         GUI_SaveContext_W      (      GUI_CONTEXT * pContext);
void         GUI_RestoreContext   (const GUI_CONTEXT * pContext);
void         GUI_SetScreenSizeX   (int xSize);
void         GUI_SetScreenSizeY   (int ySize);
int          GUI_GetScreenSizeX   (void);
int          GUI_GetScreenSizeY   (void);
const GUI_RECT * GUI_SetClipRect  (const GUI_RECT * pRect);
void         GUI_SetRefreshHook   (void (* pFunc)(void));
void         GUI_SetControlHook   (void (* pFunc)(int LayerIndex, int Cmd));
void         MainTask             (void);




 
int  GUI_RectsIntersect(const GUI_RECT * pr0, const GUI_RECT * pr1);
void GUI_MoveRect       (GUI_RECT * pRect, int x, int y);
void GUI_MergeRect      (GUI_RECT * pDest, const GUI_RECT * pr0, const GUI_RECT * pr1);
int  GUI__IntersectRects(GUI_RECT * pDest, const GUI_RECT * pr0, const GUI_RECT * pr1);
void GUI__IntersectRect (GUI_RECT * pDest, const GUI_RECT * pr0);
void GUI__ReduceRect    (GUI_RECT * pDest, const GUI_RECT * pRect, int Dist);




 
signed long  GUI__ATan2(signed long x, signed long y, signed long * ph);
signed long  GUI__ASinHQ(signed long SinHQ);
int  GUI__CompactPixelIndices  (unsigned long * pBuffer, int NumPixels, int BitsPerPixel);
int  GUI__CompactPixelIndicesEx(unsigned long * pBuffer, int NumPixels, int BitsPerPixel, const LCD_API_COLOR_CONV * pColorConvAPI);
int  GUI__ConvertColor2Index   (unsigned long * pBuffer, int NumPixels, int BitsPerPixel, const LCD_API_COLOR_CONV * pColorConvAPI, void * pResult);
void GUI__Config(void);
signed long  GUI__CosHQ(signed long Ang1000);
int  GUI__DivideRound     (int a, int b);
signed long  GUI__DivideRound32   (signed long a, signed long b);
void GUI__ExpandPixelIndices   (void * pBuffer, int NumPixels, int BitsPerPixel);
void GUI__ExpandPixelIndicesEx (void * pBuffer, int NumPixels, int BitsPerPixel, const LCD_API_COLOR_CONV * pColorConvAPI);
void GUI__memcpy(void * pDest, const void * pSrc, int NumBytes);
int  GUI__SetText(signed long * phText, const char * s);
signed long  GUI__SinHQ(signed long Ang1000);
signed long  GUI__sqrt32(signed long Square);
void GUI__DrawTwinArc2(int xl, int xr, int y0,         int r, GUI_COLOR ColorR0, GUI_COLOR ColorR1, GUI_COLOR ColorFill);
void GUI__DrawTwinArc4(int x0, int y0, int x1, int y1, int r, GUI_COLOR ColorR0, GUI_COLOR ColorR1, GUI_COLOR ColorFill);
void GUI__FillTrippleArc(int x0, int y0, int Size, GUI_COLOR ColorR0, GUI_COLOR ColorR1, GUI_COLOR ColorR2, GUI_COLOR ColorFill);
void GUI__RegisterExit(GUI_REGISTER_EXIT * pRegisterExit);




 
GUI_COLOR GUI_GetBkColor     (void);
int       GUI_GetBkColorIndex(void);
GUI_COLOR GUI_GetColor       (void);
int       GUI_GetColorIndex  (void);
unsigned char        GUI_GetLineStyle   (void);
unsigned char        GUI_GetPenSize     (void);
unsigned char        GUI_GetPenShape    (void);
unsigned  GUI_GetPixelIndex  (int x, int y);

void      GUI_SetBkColor   (GUI_COLOR);
void      GUI_SetColor     (GUI_COLOR);
void      GUI_SetBkColorIndex(int Index);
void      GUI_SetColorIndex(int Index);

unsigned char        GUI_SetPenSize   (unsigned char Size);
unsigned char        GUI_SetPenShape  (unsigned char Shape);
unsigned char        GUI_SetLineStyle (unsigned char Style);

 
char      GUI_GetDecChar(void);
char      GUI_SetDecChar(char c);




 
int       GUI_Color2Index(GUI_COLOR color);
GUI_COLOR GUI_Color2VisColor(GUI_COLOR color);
char      GUI_ColorIsAvailable(GUI_COLOR color);
GUI_COLOR GUI_Index2Color(int Index);
unsigned long       GUI_CalcColorDist (GUI_COLOR Color0, GUI_COLOR  Color1);
unsigned long       GUI_CalcVisColorError(GUI_COLOR color);




 
void GUI_SetOnErrorFunc(void (* pFunc)(const char * s));




 
void GUI_Log      (const char * s);
void GUI_Log1     (const char * s, signed long p0);
void GUI_Log2     (const char * s, signed long p0, signed long p1);
void GUI_Log3     (const char * s, signed long p0, signed long p1, signed long p2);
void GUI_Log4     (const char * s, signed long p0, signed long p1, signed long p2,signed long p3);
void GUI_Warn     (const char * s);
void GUI_Warn1    (const char * s, signed long p0);
void GUI_Warn2    (const char * s, signed long p0, signed long p1);
void GUI_Warn3    (const char * s, signed long p0, signed long p1, signed long p2);
void GUI_Warn4    (const char * s, signed long p0, signed long p1, signed long p2, signed long p3);
void GUI_ErrorOut (const char * s);
void GUI_ErrorOut1(const char * s, signed long p0);
void GUI_ErrorOut2(const char * s, signed long p0, signed long p1);
void GUI_ErrorOut3(const char * s, signed long p0, signed long p1, signed long p2);
void GUI_ErrorOut4(const char * s, signed long p0, signed long p1, signed long p2, signed long p3);




 
void GUI_Clear            (void);
void GUI_ClearRect        (int x0, int y0, int x1, int y1);
void GUI_ClearRectEx      (const GUI_RECT * pRect);
void GUI_CopyRect         (int x0, int y0, int x1, int y1, int dx, int dy);
void GUI_DrawArc          (int x0, int y0, int rx, int ry, int a0, int a1);
void GUI_DrawBitmap       (const GUI_BITMAP * pBM, int x0, int y0);
void GUI_DrawBitmapMag    (const GUI_BITMAP * pBM, int x0, int y0, int XMul, int YMul);
void GUI_DrawBitmapEx     (const GUI_BITMAP * pBM, int x0, int y0, int xCenter, int yCenter, int xMag, int yMag);
void GUI_DrawBitmapExp    (int x0, int y0, int XSize, int YSize, int XMul,  int YMul, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, const GUI_LOGPALETTE * pPal);
void GUI_DrawBitmapHWAlpha(const GUI_BITMAP * pBM, int x0, int y0);
void GUI_DrawCircle       (int x0, int y0, int r);
void GUI_DrawEllipse      (int x0, int y0, int rx, int ry);
void GUI_DrawGradientH    (int x0, int y0, int x1, int y1, GUI_COLOR Color0, GUI_COLOR Color1);
void GUI_DrawGradientV    (int x0, int y0, int x1, int y1, GUI_COLOR Color0, GUI_COLOR Color1);
void GUI_DrawGradientRoundedH(int x0, int y0, int x1, int y1, int rd, GUI_COLOR Color0, GUI_COLOR Color1);
void GUI_DrawGradientRoundedV(int x0, int y0, int x1, int y1, int rd, GUI_COLOR Color0, GUI_COLOR Color1);
void GUI_DrawGraph        (signed short * pay, int NumPoints, int x0, int y0);
void GUI_DrawGraphEx      (signed short * pay, int NumPoints, int x0, int y0, int Numerator, int Denominator, int MirrorX);
void GUI_DrawHLine        (int y0, int x0, int x1);
void GUI_DrawLine         (int x0, int y0, int x1, int y1);
void GUI_DrawLineRel      (int dx, int dy);
void GUI_DrawLineTo       (int x, int y);
void GUI_DrawPie          (int x0, int y0, int r, int a0, int a1, int Type);
void GUI_DrawPixel        (int x, int y);
void GUI_DrawPoint        (int x, int y);
void GUI_DrawPolygon      (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GUI_DrawPolyLine     (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GUI_DrawFocusRect    (const GUI_RECT  * pRect, int Dist);
void GUI_DrawRect         (int x0, int y0, int x1, int y1);
void GUI_DrawRectEx       (const GUI_RECT * pRect);
void GUI_DrawRoundedFrame (int x0, int y0, int x1, int y1, int r, int w);
void GUI_DrawRoundedRect  (int x0, int y0, int x1, int y1, int r);
void GUI_DrawVLine        (int x0, int y0, int y1);
void GUI_FillCircle       (int x0, int y0, int r);
void GUI_FillEllipse      (int x0, int y0, int rx, int ry);
void GUI_FillPolygon      (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GUI_FillRect         (int x0, int y0, int x1, int y1);
void GUI_FillRectEx       (const GUI_RECT * pRect);
void GUI_FillRoundedFrame (int x0, int y0, int x1, int y1, int r, int w);
void GUI_FillRoundedRect  (int x0, int y0, int x1, int y1, int r);
void GUI_FillRoundedRectB (int x0, int y0, int x1, int y1, int r);
void GUI_FillRoundedRectT (int x0, int y0, int x1, int y1, int r);
void GUI_GetClientRect    (GUI_RECT * pRect);
void GUI_InvertRect       (int x0, int y0, int x1, int y1);
void GUI_MoveRel          (int dx, int dy);
void GUI_MoveTo           (int x, int y);
void GUI_SetAlphaMask8888 (unsigned long OrMask, unsigned long AndMask);




 
typedef int GUI_GET_DATA_FUNC(void * p, const unsigned char ** ppData, unsigned NumBytes, unsigned long Off);




 
int GUI_GIF_Draw           (const void * pGIF, unsigned long NumBytes,         int x0, int y0);
int GUI_GIF_DrawEx         (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0);
int GUI_GIF_DrawSub        (const void * pGIF, unsigned long NumBytes,         int x0, int y0, int Index);
int GUI_GIF_DrawSubEx      (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0, int Index);
int GUI_GIF_DrawSubScaled  (const void * pGIF, unsigned long NumBytes,         int x0, int y0, int Index, int Num, int Denom);
int GUI_GIF_DrawSubScaledEx(GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0, int Index, int Num, int Denom);
int GUI_GIF_GetComment     (const void * pGIF, unsigned long NumBytes,         unsigned char * pBuffer, int MaxSize, int Index);
int GUI_GIF_GetCommentEx   (GUI_GET_DATA_FUNC * pfGetData, void * p, unsigned char * pBuffer, int MaxSize, int Index);
int GUI_GIF_GetImageInfo   (const void * pGIF, unsigned long NumBytes,         GUI_GIF_IMAGE_INFO * pInfo, int Index);
int GUI_GIF_GetImageInfoEx (GUI_GET_DATA_FUNC * pfGetData, void * p, GUI_GIF_IMAGE_INFO * pInfo, int Index);
int GUI_GIF_GetInfo        (const void * pGIF, unsigned long NumBytes,         GUI_GIF_INFO * pInfo);
int GUI_GIF_GetInfoEx      (GUI_GET_DATA_FUNC * pfGetData, void * p, GUI_GIF_INFO * pInfo);
int GUI_GIF_GetXSize       (const void * pGIF);
int GUI_GIF_GetXSizeEx     (GUI_GET_DATA_FUNC * pfGetData, void * p);
int GUI_GIF_GetYSize       (const void * pGIF);
int GUI_GIF_GetYSizeEx     (GUI_GET_DATA_FUNC * pfGetData, void * p);
int GUI_GIF_SetFillTrans   (int OnOff);




 
int  GUI_BMP_Draw        (const void * pFileData,                  int x0, int y0);
int  GUI_BMP_DrawEx      (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0);
int  GUI_BMP_DrawScaled  (const void * pFileData,                  int x0, int y0, int Num, int Denom);
int  GUI_BMP_DrawScaledEx(GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0, int Num, int Denom);
int  GUI_BMP_GetXSize    (const void * pFileData);
int  GUI_BMP_GetXSizeEx  (GUI_GET_DATA_FUNC * pfGetData, void * p);
int  GUI_BMP_GetYSize    (const void * pFileData);
int  GUI_BMP_GetYSizeEx  (GUI_GET_DATA_FUNC * pfGetData, void * p);
void GUI_BMP_EnableAlpha (void);
void GUI_BMP_DisableAlpha(void);




 
int GUI_PNG_Draw      (const void * pFileData, int DataSize, int x0, int y0);
int GUI_PNG_DrawEx    (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0);
int GUI_PNG_GetXSize  (const void * pFileData, int FileSize);
int GUI_PNG_GetXSizeEx(GUI_GET_DATA_FUNC * pfGetData, void * p);
int GUI_PNG_GetYSize  (const void * pFileData, int FileSize);
int GUI_PNG_GetYSizeEx(GUI_GET_DATA_FUNC * pfGetData, void * p);




 
typedef struct {
  int XSize;
  int YSize;
} GUI_JPEG_INFO;

int GUI_JPEG_Draw        (const void * pFileData, int DataSize,    int x0, int y0);
int GUI_JPEG_DrawEx      (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0);
int GUI_JPEG_DrawScaled  (const void * pFileData, int DataSize,    int x0, int y0, int Num, int Denom);
int GUI_JPEG_DrawScaledEx(GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0, int Num, int Denom);
int GUI_JPEG_GetInfo     (const void * pFileData, int DataSize,    GUI_JPEG_INFO * pInfo);
int GUI_JPEG_GetInfoEx   (GUI_GET_DATA_FUNC * pfGetData, void * p, GUI_JPEG_INFO * pInfo);




 






typedef signed long GUI_MOVIE_HANDLE;

typedef void GUI_MOVIE_FUNC(GUI_MOVIE_HANDLE hMovie, int Notification, unsigned long CurrentFrame);

typedef struct {
  int xSize;         
  int ySize;         
  int msPerFrame;    
  unsigned long NumFrames;     
} GUI_MOVIE_INFO;

GUI_MOVIE_HANDLE GUI_MOVIE_Create       (const void * pFileData, unsigned long FileSize, GUI_MOVIE_FUNC * pfNotify);
GUI_MOVIE_HANDLE GUI_MOVIE_CreateEx     (GUI_GET_DATA_FUNC * pfGetData, void * pParam, GUI_MOVIE_FUNC * pfNotify);
int              GUI_MOVIE_Delete       (GUI_MOVIE_HANDLE hMovie);
unsigned long              GUI_MOVIE_GetFrameIndex(GUI_MOVIE_HANDLE hMovie);
int              GUI_MOVIE_GetInfo      (const void * pFileData, unsigned long FileSize, GUI_MOVIE_INFO * pInfo);
int              GUI_MOVIE_GetInfoEx    (GUI_GET_DATA_FUNC * pfGetData, void * pParam, GUI_MOVIE_INFO * pInfo);
int              GUI_MOVIE_GetPos       (GUI_MOVIE_HANDLE hMovie, int * pxPos, int * pyPos, int * pxSize, int * pySize);
int              GUI_MOVIE_GotoFrame    (GUI_MOVIE_HANDLE hMovie, unsigned long Frame);
int              GUI_MOVIE_Pause        (GUI_MOVIE_HANDLE hMovie);
int              GUI_MOVIE_Play         (GUI_MOVIE_HANDLE hMovie);
int              GUI_MOVIE_SetPeriod    (GUI_MOVIE_HANDLE hMovie, unsigned Period);
int              GUI_MOVIE_SetPos       (GUI_MOVIE_HANDLE hMovie, int xPos, int yPos);
int              GUI_MOVIE_ShowScaled   (GUI_MOVIE_HANDLE hMovie, int xPos, int yPos, int num, int denom, int DoLoop);
int              GUI_MOVIE_Show         (GUI_MOVIE_HANDLE hMovie, int xPos, int yPos, int DoLoop);




 



typedef struct {
  const GUI_BITMAP  * pBitmap;
  int                 xHot;
  int                 yHot;
} GUI_CURSOR;

typedef struct {
  const GUI_BITMAP ** ppBm;
  int                 xHot;
  int                 yHot;
  unsigned            Period;
  const unsigned    * pPeriod;
  int                 NumItems;
} GUI_CURSOR_ANIM;


  int                GUI_CURSOR_GetState     (void);
  int                GUI_CURSOR_GetStateEx   (int Layer);
  void               GUI_CURSOR_Hide         (void);
  void               GUI_CURSOR_HideEx       (int Layer);
  const GUI_CURSOR * GUI_CURSOR_Select       (const GUI_CURSOR * pCursor);
  const GUI_CURSOR * GUI_CURSOR_SelectEx     (const GUI_CURSOR * pCursor, int Layer);
  int                GUI_CURSOR_SelectAnim   (const GUI_CURSOR_ANIM * pCursorAnim);
  int                GUI_CURSOR_SelectAnimEx (const GUI_CURSOR_ANIM * pCursorAnim, int LayerIndex);
  int                GUI_CURSOR_SetBitmap    (const GUI_BITMAP * pBM);
  int                GUI_CURSOR_SetBitmapEx  (const GUI_BITMAP * pBM, int Layer);
  void               GUI_CURSOR_SetPosition  (int x, int y);
  void               GUI_CURSOR_SetPositionEx(int xNewPos, int yNewPos, int Layer);
  void               GUI_CURSOR_Show         (void);
  void               GUI_CURSOR_ShowEx       (int Layer);
  GUI_HSPRITE        GUI_CURSOR__GetSpriteEx (int LayerIndex, int * pxPos, int * pyPos);
  void               GUI_CURSOR__SetSpriteEx (GUI_HSPRITE hSprite, const GUI_CURSOR * pCursor, int LayerIndex);








 






GUI_HSPRITE GUI_SPRITE__CreateEx           (const GUI_BITMAP * pBM, int x, int y, int Layer, unsigned short Flags);  
void        GUI_SPRITE__SetCallback        (GUI_HSPRITE hSprite, signed long hContext, void (* pCB)(GUI_HSPRITE, int));
GUI_HSPRITE GUI_SPRITE_Create              (const GUI_BITMAP * pBM, int x, int y);
GUI_HSPRITE GUI_SPRITE_CreateAnim          (const GUI_BITMAP ** ppBm, int x, int y, unsigned Period, const unsigned * pPeriod, int NumItems);
GUI_HSPRITE GUI_SPRITE_CreateEx            (const GUI_BITMAP * pBM, int x, int y, int Layer);
GUI_HSPRITE GUI_SPRITE_CreateExAnim        (const GUI_BITMAP ** ppBm, int x, int y, unsigned Period, const unsigned * pPeriod, int NumItems, int LayerIndex);
GUI_HSPRITE GUI_SPRITE_CreateHidden        (const GUI_BITMAP * pBM, int x, int y);
GUI_HSPRITE GUI_SPRITE_CreateHiddenEx      (const GUI_BITMAP * pBM, int x, int y, int Layer);
void        GUI_SPRITE_Delete              (GUI_HSPRITE hSprite);
int         GUI_SPRITE_GetState            (GUI_HSPRITE hSprite);
void        GUI_SPRITE_Hide                (GUI_HSPRITE hSprite);
int         GUI_SPRITE_SetBitmap           (GUI_HSPRITE hSprite, const GUI_BITMAP * pBM);
int         GUI_SPRITE_SetBitmapAndPosition(GUI_HSPRITE hSprite, const GUI_BITMAP * pBM, int x, int y);
int         GUI_SPRITE_SetLoop             (GUI_HSPRITE hSprite, int OnOff);
void        GUI_SPRITE_SetPosition         (GUI_HSPRITE hSprite, int x, int y);
int         GUI_SPRITE_StartAnim           (GUI_HSPRITE hSprite);
int         GUI_SPRITE_StopAnim            (GUI_HSPRITE hSprite);
void        GUI_SPRITE_Show                (GUI_HSPRITE hSprite);




 
extern const GUI_CURSOR GUI_CursorArrowS,  GUI_CursorArrowSI;
extern const GUI_CURSOR GUI_CursorArrowM,  GUI_CursorArrowMI;
extern const GUI_CURSOR GUI_CursorArrowL,  GUI_CursorArrowLI;
extern const GUI_CURSOR GUI_CursorCrossS,  GUI_CursorCrossSI;
extern const GUI_CURSOR GUI_CursorCrossM,  GUI_CursorCrossMI;
extern const GUI_CURSOR GUI_CursorCrossL,  GUI_CursorCrossLI;
extern const GUI_CURSOR GUI_CursorHeaderM, GUI_CursorHeaderMI;

extern const GUI_BITMAP GUI_BitmapArrowS, GUI_BitmapArrowSI;
extern const GUI_BITMAP GUI_BitmapArrowM, GUI_BitmapArrowMI;
extern const GUI_BITMAP GUI_BitmapArrowL, GUI_BitmapArrowLI;
extern const GUI_BITMAP GUI_BitmapCrossS, GUI_BitmapCrossSI;
extern const GUI_BITMAP GUI_BitmapCrossM, GUI_BitmapCrossMI;
extern const GUI_BITMAP GUI_BitmapCrossL, GUI_BitmapCrossLI;

extern const GUI_CURSOR_ANIM GUI_CursorAnimHourglassM;




 
typedef enum { GUI_WRAPMODE_NONE, GUI_WRAPMODE_WORD, GUI_WRAPMODE_CHAR } GUI_WRAPMODE;




 
void  GUI_DispCEOL             (void);
void  GUI_DispChar             (unsigned short c);
void  GUI_DispCharAt           (unsigned short c, signed short x, signed short y);
void  GUI_DispChars            (unsigned short c, int Cnt);
void  GUI_DispNextLine         (void);
void  GUI_DispString           (const char * s);
void  GUI_DispStringAt         (const char * s, int x, int y);
void  GUI_DispStringAtCEOL     (const char * s, int x, int y);
void  GUI_DispStringHCenterAt  (const char * s, int x, int y);
void  GUI__DispStringInRect    (const char * s, GUI_RECT * pRect, int TextAlign, int MaxNumChars);
void  GUI_DispStringInRect     (const char * s, GUI_RECT * pRect, int TextAlign);

  void  GUI_DispStringInRectEx (const char * s, GUI_RECT * pRect, int TextAlign, int MaxLen, const tLCD_APIList * pLCD_Api);

void  GUI_DispStringInRectMax  (const char * s, GUI_RECT * pRect, int TextAlign, int MaxLen);  
void  GUI_DispStringInRectWrap (const char * s, GUI_RECT * pRect, int TextAlign, GUI_WRAPMODE WrapMode);  
void  GUI_DispStringLen        (const char * s, int Len);
void  GUI_GetTextExtend        (GUI_RECT* pRect, const char * s, int Len);
int   GUI_GetYAdjust           (void);
int   GUI_GetDispPosX          (void);
int   GUI_GetDispPosY          (void);
const GUI_FONT * GUI_GetFont(void);
int   GUI_GetCharDistX         (unsigned short c);
int   GUI_GetCharDistXEx       (unsigned short c, int * pSizeX);
int   GUI_GetStringDistX       (const char * s);
GUI_DRAWMODE GUI_GetDrawMode   (void);
int   GUI_GetFontDistY         (void);
int   GUI_GetFontSizeY         (void);
void  GUI_GetFontInfo          (const GUI_FONT * pFont, GUI_FONTINFO * pfi);
void  GUI_GetOrg               (int * px, int * py);
int   GUI_GetYSizeOfFont       (const GUI_FONT * pFont);
int   GUI_GetYDistOfFont       (const GUI_FONT * pFont);
int   GUI_GetTextAlign         (void);
int   GUI_GetTextMode          (void);
char  GUI_IsInFont             (const GUI_FONT * pFont, unsigned short c);
int   GUI_SetTextAlign         (int Align);
int   GUI_SetTextMode          (int Mode);
char  GUI_SetTextStyle         (char Style);
int   GUI_SetLBorder           (int x);
const GUI_FONT * GUI_SetFont(const GUI_FONT * pNewFont);
char  GUI_GotoXY               (int x, int y);
char  GUI_GotoX                (int x);
char  GUI_GotoY                (int y);
int   GUI_WrapGetNumLines      (const char * pText, int xSize, GUI_WRAPMODE WrapMode);

int   GUI_GetLeadingBlankCols (unsigned short c);
int   GUI_GetTrailingBlankCols(unsigned short c);





 
void GUI_SIF_CreateFont(const void * pFontData, GUI_FONT * pFont, const tGUI_SIF_APIList * pFontType);
void GUI_SIF_DeleteFont(GUI_FONT * pFont);




 
int  GUI_XBF_CreateFont(GUI_FONT * pFont, GUI_XBF_DATA * pXBF, const tGUI_XBF_APIList * pFontType, GUI_XBF_GET_DATA_FUNC * pfGetData, void * pVoid);
void GUI_XBF_DeleteFont(GUI_FONT * pFont);




 
int  GUI_TTF_CreateFont   (GUI_FONT * pFont, GUI_TTF_CS * pCS);
int  GUI_TTF_CreateFontAA (GUI_FONT * pFont, GUI_TTF_CS * pCS);
void GUI_TTF_DestroyCache (void);
void GUI_TTF_Done         (void);
int  GUI_TTF_GetFamilyName(GUI_FONT * pFont, char * pBuffer, int NumBytes);
int  GUI_TTF_GetStyleName (GUI_FONT * pFont, char * pBuffer, int NumBytes);
void GUI_TTF_SetCacheSize (unsigned MaxFaces, unsigned MaxSizes, unsigned long MaxBytes);




 
int          GUI_LANG_GetLang          (void);
int          GUI_LANG_GetNumItems      (int IndexLang);
const char * GUI_LANG_GetText          (int IndexText);
int          GUI_LANG_GetTextBuffered  (int IndexText, char * pBuffer, int SizeOfBuffer);
int          GUI_LANG_GetTextBufferedEx(int IndexText, int IndexLang, char * pBuffer, int SizeOfBuffer);
const char * GUI_LANG_GetTextEx        (int IndexText, int IndexLang);
int          GUI_LANG_LoadCSV          (unsigned char * pFileData, unsigned long FileSize);
int          GUI_LANG_LoadCSVEx        (GUI_GET_DATA_FUNC * pfGetData, void * p);
int          GUI_LANG_LoadText         (unsigned char * pFileData, unsigned long FileSize, int IndexLang);
int          GUI_LANG_LoadTextEx       (GUI_GET_DATA_FUNC * pfGetData, void * p, int IndexLang);
int          GUI_LANG_SetLang          (int IndexLang);
unsigned     GUI_LANG_SetMaxNumLang    (unsigned MaxNumLang);
unsigned short          GUI_LANG_SetSep           (unsigned short Sep);




 
int   GUI_UC_ConvertUC2UTF8   (const unsigned short * s, int Len, char * pBuffer, int BufferSize);
int   GUI_UC_ConvertUTF82UC   (const char * s, int Len, unsigned short * pBuffer, int BufferSize);
int   GUI_UC_Encode           (char * s, unsigned short Char);
int   GUI_UC_GetCharSize      (const char * s);
unsigned short   GUI_UC_GetCharCode      (const char * s);
void  GUI_UC_SetEncodeNone    (void);
void  GUI_UC_SetEncodeUTF8    (void);
int   GUI_UC_EnableBIDI       (int OnOff);

void GUI_UC_DispString(const unsigned short * s);
void GUI_UC2DB (unsigned short Code, unsigned char * pOut);
unsigned short  GUI_DB2UC (unsigned char Byte0, unsigned char Byte1);




 
void GUI_DispBin  (unsigned long  v, unsigned char Len);
void GUI_DispBinAt(unsigned long  v, signed short x, signed short y, unsigned char Len);
void GUI_DispDec  (signed long v, unsigned char Len);
void GUI_DispDecAt (signed long v, signed short x, signed short y, unsigned char Len);
void GUI_DispDecMin(signed long v);
void GUI_DispDecShift(signed long v, unsigned char Len, unsigned char Shift);
void GUI_DispDecSpace(signed long v, unsigned char MaxDigits);
void GUI_DispHex  (unsigned long v, unsigned char Len);
void GUI_DispHexAt(unsigned long v, signed short x, signed short y, unsigned char Len);
void GUI_DispSDec(signed long v, unsigned char Len);
void GUI_DispSDecShift(signed long v, unsigned char Len, unsigned char Shift);




 
void GUI_DispFloat    (float v, char Len);
void GUI_DispFloatFix (float v, char Len, char Fract);
void GUI_DispFloatMin (float v, char Fract);
void GUI_DispSFloatFix(float v, char Len, char Fract);
void GUI_DispSFloatMin(float v, char Fract);




 
typedef struct {
  unsigned long TotalBytes;
  unsigned long FreeBytes;
  unsigned long UsedBytes;
  unsigned long AllocSize;
  unsigned long NumFixedBytes;
  unsigned long MaxUsedBytes;
} GUI_ALLOC_INFO;

signed long GUI_ALLOC_GetNumFreeBlocks(void);
signed long GUI_ALLOC_GetNumFreeBytes (void);
signed long GUI_ALLOC_GetNumUsedBlocks(void);
signed long GUI_ALLOC_GetNumUsedBytes (void);
signed long GUI_ALLOC_GetMaxUsedBytes (void);

void GUI_ALLOC_GetMemInfo  (GUI_ALLOC_INFO * pInfo);
void GUI_ALLOC_SuppressPeak(int OnOff);

signed long           GUI_ALLOC_AllocInit       (const void * pInitData, signed long Size);
signed long           GUI_ALLOC_AllocNoInit     (signed long size);
signed long           GUI_ALLOC_AllocZero       (signed long size);
void               GUI_ALLOC_AssignMemory    (void * p, unsigned long NumBytes);
void               GUI_ALLOC_Free            (signed long  hMem);
void               GUI_ALLOC_FreeFixedBlock  (void * p);
void               GUI_ALLOC_FreePtrArray    (signed long * pArray, int NumElems);
void               GUI_ALLOC_FreePtr         (signed long * phMem);
void *             GUI_ALLOC_GetFixedBlock   (signed long Size);
signed long GUI_ALLOC_GetMaxSize      (void);
signed long GUI_ALLOC_GetSize         (signed long  hMem);
void *             GUI_ALLOC_h2p             (signed long  hMem);
signed long           GUI_ALLOC_p2h             (void * p);
void               GUI_ALLOC_Init            (void);
void               GUI_ALLOC_Lock            (void);
void *             GUI_ALLOC_LockH           (signed long  hMem);
signed long           GUI_ALLOC_Realloc         (signed long hOld, int NewSize);
signed long GUI_ALLOC_RequestSize     (void);
void               GUI_ALLOC_SetAvBlockSize  (unsigned long BlockSize);
void               GUI_ALLOC_Unlock          (void);
void *             GUI_ALLOC_UnlockH         (void ** pp);
int                GUI_ALLOC_SetMaxPercentage(int MaxPercentage);




 



typedef signed long GUI_MEMDEV_Handle;
typedef void     GUI_CALLBACK_VOID_P        (void * p);
typedef int      GUI_ANIMATION_CALLBACK_FUNC(int TimeRem, void * pVoid);
typedef void     GUI_DRAWMEMDEV_16BPP_FUNC  (void * pDst, const void * pSrc, int xSize, int ySize, int BytesPerLineDst, int BytesPerLineSrc);

extern GUI_ANIMATION_CALLBACK_FUNC * GUI_MEMDEV__pCbAnimation;
extern void                        * GUI_MEMDEV__pVoid;

typedef struct {
  GUI_RECT rView, rPrev;
  char FirstCall;
} GUI_AUTODEV;

typedef struct {
  char DrawFixed;
  char IsMeasurement;
} GUI_AUTODEV_INFO;

int  GUI_MEMDEV_CreateAuto(GUI_AUTODEV * pAutoDev);
void GUI_MEMDEV_DeleteAuto(GUI_AUTODEV * pAutoDev);
int  GUI_MEMDEV_DrawAuto  (GUI_AUTODEV * pAutoDev, GUI_AUTODEV_INFO * pAutoDevInfo, GUI_CALLBACK_VOID_P * pfDraw, void * pData);

 
GUI_MEMDEV_Handle GUI_MEMDEV_Create       (int x0, int y0, int xSize, int ySize);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateEx     (int x0, int y0, int xSize, int ySize, int Flags);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateFixed  (int x0, int y0, int xSize, int ySize, int Flags,
                                           const GUI_DEVICE_API     * pDeviceAPI,
                                           const LCD_API_COLOR_CONV * pColorConvAPI);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateFixed32(int x0, int y0, int xSize, int ySize);

void GUI_MEMDEV_Clear                (GUI_MEMDEV_Handle hMem);
int  GUI_MEMDEV_ClearAlpha           (GUI_MEMDEV_Handle hMemData, GUI_MEMDEV_Handle hMemMask);
void GUI_MEMDEV_CopyFromLCD          (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_CopyFromLCDAA        (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_CopyToLCD            (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_CopyToLCDAA          (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_CopyToLCDAt          (GUI_MEMDEV_Handle hMem, int x, int y);
int  GUI_MEMDEV_CompareWithLCD       (GUI_MEMDEV_Handle hMem, int * px, int * py, int * pExp, int * pAct);
void GUI_MEMDEV_Delete               (GUI_MEMDEV_Handle MemDev);
void GUI_MEMDEV_DrawPerspectiveX     (GUI_MEMDEV_Handle hMem, int x, int y, int h0, int h1, int dx, int dy);
int  GUI_MEMDEV_GetXPos              (GUI_MEMDEV_Handle hMem);
int  GUI_MEMDEV_GetXSize             (GUI_MEMDEV_Handle hMem);
int  GUI_MEMDEV_GetYPos              (GUI_MEMDEV_Handle hMem);
int  GUI_MEMDEV_GetYSize             (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_MarkDirty            (GUI_MEMDEV_Handle hMem, int x0, int y0, int x1, int y1);
void GUI_MEMDEV_ReduceYSize          (GUI_MEMDEV_Handle hMem, int YSize);
void GUI_MEMDEV_Rotate               (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag);
void GUI_MEMDEV_RotateAlpha          (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag, unsigned char Alpha);
void GUI_MEMDEV_RotateHR             (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, signed long dx, signed long dy, int a, int Mag);
void GUI_MEMDEV__Rotate              (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag, unsigned long Mask);
void GUI_MEMDEV__RotateHR            (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, signed long dx, signed long dy, int a, int Mag, unsigned long Mask);
void GUI_MEMDEV_RotateHQ             (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag);
void GUI_MEMDEV_RotateHQAlpha        (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag, unsigned char Alpha);
void GUI_MEMDEV_RotateHQHR           (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, signed long dx, signed long dy, int a, int Mag);
void GUI_MEMDEV_RotateHQT            (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag);
void GUI_MEMDEV_RotateHQTI           (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag);
GUI_MEMDEV_Handle GUI_MEMDEV_Select  (GUI_MEMDEV_Handle hMem);   
void  GUI_MEMDEV_SetOrg              (GUI_MEMDEV_Handle hMem, int x0, int y0);
void  GUI_MEMDEV_WriteAt             (GUI_MEMDEV_Handle hMem, int x, int y);
void  GUI_MEMDEV_Write               (GUI_MEMDEV_Handle hMem);
void  GUI_MEMDEV_WriteAlphaAt        (GUI_MEMDEV_Handle hMem, int Alpha, int x, int y);
void  GUI_MEMDEV_WriteAlpha          (GUI_MEMDEV_Handle hMem, int Alpha);
void  GUI_MEMDEV_WriteExAt           (GUI_MEMDEV_Handle hMem, int x, int y, int xMag, int yMag, int Alpha);
void  GUI_MEMDEV_WriteEx             (GUI_MEMDEV_Handle hMem, int xMag, int yMag, int Alpha);
void  GUI_MEMDEV_WriteOpaque         (GUI_MEMDEV_Handle hMem);
void  GUI_MEMDEV_WriteOpaqueAt       (GUI_MEMDEV_Handle hMem, int x, int y);
int   GUI_MEMDEV_Draw                (GUI_RECT * pRect, GUI_CALLBACK_VOID_P * pfDraw, void * pData, int NumLines, int Flags);
void* GUI_MEMDEV_GetDataPtr          (GUI_MEMDEV_Handle hMem);
void  GUI_MEMDEV_SetColorConv        (GUI_MEMDEV_Handle hMem, const LCD_API_COLOR_CONV * pColorConvAPI);
const LCD_API_COLOR_CONV * GUI_MEMDEV_GetColorConv(GUI_MEMDEV_Handle hMemDev);
int   GUI_MEMDEV_GetBitsPerPixel     (GUI_MEMDEV_Handle hMemDev);
int   GUI_MEMDEV_FadeInDevices       (GUI_MEMDEV_Handle hMem0, GUI_MEMDEV_Handle hMem1, int Period);
int   GUI_MEMDEV_FadeOutDevices      (GUI_MEMDEV_Handle hMem0, GUI_MEMDEV_Handle hMem1, int Period);
void  GUI_MEMDEV_SerializeBMP        (GUI_MEMDEV_Handle hDev, GUI_CALLBACK_VOID_U8_P * pfSerialize, void * p);
void  GUI_MEMDEV_SetAnimationCallback(GUI_ANIMATION_CALLBACK_FUNC * pCbAnimation, void * pVoid);
void  GUI_MEMDEV__FadeDevice         (GUI_MEMDEV_Handle hMemWin, GUI_MEMDEV_Handle hMemBk, GUI_MEMDEV_Handle hMemDst, unsigned char Intens);
void  GUI_MEMDEV__FadeDeviceEx       (GUI_MEMDEV_Handle hMemWin, GUI_MEMDEV_Handle hMemBk, GUI_MEMDEV_Handle hMemDst, unsigned char Intens, int xPosWin, int yPosWin);
int   GUI_MEMDEV_PunchOutDevice      (GUI_MEMDEV_Handle hMemData, GUI_MEMDEV_Handle hMemMask);
void  GUI_MEMDEV_SetTimePerFrame     (unsigned TimePerFrame);

void  GUI_SelectLCD(void);

 
GUI_MEMDEV_Handle GUI_MEMDEV_CreateBlurredDevice32  (GUI_MEMDEV_Handle hMem, unsigned char Depth);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateBlurredDevice32HQ(GUI_MEMDEV_Handle hMem, unsigned char Depth);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateBlurredDevice32LQ(GUI_MEMDEV_Handle hMem, unsigned char Depth);
void              GUI_MEMDEV_SetBlurHQ              (void);
void              GUI_MEMDEV_SetBlurLQ              (void);
int               GUI_MEMDEV_BlendColor32           (GUI_MEMDEV_Handle hMem, unsigned long BlendColor, unsigned char BlendIntens);
int               GUI_MEMDEV_Dither32               (GUI_MEMDEV_Handle hMem, const LCD_API_COLOR_CONV * pColorConvAPI);

 
void GUI_MEMDEV_SetDrawMemdev16bppFunc(GUI_DRAWMEMDEV_16BPP_FUNC * pfDrawMemdev16bppFunc);




 
typedef struct {
  unsigned long UserAlpha;
} GUI_ALPHA_STATE;



unsigned GUI_EnableAlpha         (unsigned OnOff);
unsigned long      GUI_RestoreUserAlpha    (GUI_ALPHA_STATE * pAlphaState);
unsigned GUI_SetAlpha            (unsigned char Alpha);
unsigned long      GUI_SetUserAlpha        (GUI_ALPHA_STATE * pAlphaState, unsigned long UserAlpha);
void     GUI_SetFuncAlphaBlending(void (* pfAlphaBlending)(LCD_COLOR *, LCD_COLOR *, LCD_COLOR *, unsigned long));
void     GUI_SetFuncMixColors    (LCD_COLOR (* pFunc)(LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens));
void     GUI_SetFuncMixColorsBulk(void (* pFunc)(unsigned long * pFG, unsigned long * pBG, unsigned long * pDst, unsigned OffFG, unsigned OffBG, unsigned OffDest, unsigned xSize, unsigned ySize, unsigned char Intens));
unsigned GUI_PreserveTrans       (unsigned OnOff);




 
unsigned GUI_SelectLayer(unsigned Index);
unsigned GUI_GetSelLayer(void);

int  GUI_SetLayerPosEx  (unsigned Index, int xPos, int yPos);
int  GUI_SetLayerSizeEx (unsigned Index, int xSize, int ySize);
int  GUI_SetLayerVisEx  (unsigned Index, int OnOff);
int  GUI_SetLayerAlphaEx(unsigned Index, int Alpha);
void GUI_GetLayerPosEx  (unsigned Index, int * pxPos, int * pyPos);

void     GUI_AssignCursorLayer(unsigned Index, unsigned CursorLayer);
unsigned GUI_GetCursorLayer   (unsigned Index);




 
void GUI_SetOrg(int x, int y);

void GUI_MULTIBUF_Begin          (void);
void GUI_MULTIBUF_BeginEx        (int LayerIndex);
void GUI_MULTIBUF_End            (void);
void GUI_MULTIBUF_EndEx          (int LayerIndex);
void GUI_MULTIBUF_Config         (int NumBuffers);
void GUI_MULTIBUF_ConfigEx       (int LayerIndex, int NumBuffers);
void GUI_MULTIBUF_Confirm        (int Index);
void GUI_MULTIBUF_ConfirmEx      (int LayerIndex, int BufferIndex);
int  GUI_MULTIBUF_GetNumBuffers  (void);
int  GUI_MULTIBUF_GetNumBuffersEx(int LayerIndex);
void GUI_MULTIBUF_UseSingleBuffer(void);




 
int  GUI_SPY_Process      (GUI_tSend pfSend, GUI_tRecv pfRecv, void * pConnectInfo);
void GUI_SPY_SetMemHandler(GUI_tMalloc pMalloc, GUI_tFree pFree);
int  GUI_SPY_StartServer  (void);
int  GUI_SPY_X_StartServer(void);




 













typedef signed long GUI_ANIM_HANDLE;

typedef signed long (* GUI_ANIM_GETPOS_FUNC)(int ts, int te, int tNow);

typedef struct {
  int Pos;
  int State;
  GUI_ANIM_HANDLE hAnim;
  int Period;
} GUI_ANIM_INFO;

typedef void GUI_ANIMATION_FUNC(GUI_ANIM_INFO * pInfo, void * pVoid);

signed long GUI_ANIM__Linear    (int ts, int te, int tNow);
signed long GUI_ANIM__Decel     (int ts, int te, int tNow);
signed long GUI_ANIM__Accel     (int ts, int te, int tNow);
signed long GUI_ANIM__AccelDecel(int ts, int te, int tNow);

int             GUI_ANIM_AddItem(GUI_ANIM_HANDLE hAnim, int ts, int te, GUI_ANIM_GETPOS_FUNC pfGetPos, void * pVoid, GUI_ANIMATION_FUNC * pfAnim);
GUI_ANIM_HANDLE GUI_ANIM_Create (int Period, unsigned MinTimePerFrame, void * pVoid, void (* pfSliceInfo)(int State, void * pVoid));
void            GUI_ANIM_Delete (GUI_ANIM_HANDLE hAnim);
int             GUI_ANIM_Exec   (GUI_ANIM_HANDLE hAnim);
void            GUI_ANIM_Start  (GUI_ANIM_HANDLE hAnim);




 



 
typedef struct {
  void     (* pfDrawBitmap   )(GUI_DEVICE * pDevice, int x0, int y0, int xsize, int ysize, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, int Diff, const unsigned long * pTrans);
  void     (* pfDrawHLine    )(GUI_DEVICE * pDevice, int x0, int y0,  int x1);
  void     (* pfDrawVLine    )(GUI_DEVICE * pDevice, int x , int y0,  int y1);
  void     (* pfFillRect     )(GUI_DEVICE * pDevice, int x0, int y0, int x1, int y1);
  unsigned (* pfGetPixelIndex)(GUI_DEVICE * pDevice, int x, int y);
  void     (* pfSetPixelIndex)(GUI_DEVICE * pDevice, int x, int y, int ColorIndex);
  void     (* pfXorPixel     )(GUI_DEVICE * pDevice, int x, int y);
  int      BytesPerPixel;
} GUI_ORIENTATION_API;

extern const GUI_ORIENTATION_API GUI_OrientationAPI_C0;
extern const GUI_ORIENTATION_API GUI_OrientationAPI_C8;
extern const GUI_ORIENTATION_API GUI_OrientationAPI_C16;
extern const GUI_ORIENTATION_API GUI_OrientationAPI_C32;






int GUI_SetOrientation        (int Orientation);
int GUI_SetOrientationEx      (int Orientation, int LayerIndex);
int GUI_SetOrientationExCached(int Orientation, int LayerIndex, const GUI_ORIENTATION_API * pAPI);




 
typedef signed long GUI_MEASDEV_Handle;

GUI_MEASDEV_Handle GUI_MEASDEV_Create (void);
void               GUI_MEASDEV_Delete (GUI_MEASDEV_Handle hMemDev);
void               GUI_MEASDEV_Select (GUI_MEASDEV_Handle hMem);
void               GUI_MEASDEV_GetRect(GUI_MEASDEV_Handle hMem, GUI_RECT * pRect);
void               GUI_MEASDEV_ClearRect(GUI_MEASDEV_Handle hMem);




 
void GUI_RotatePolygon (GUI_POINT * pDest, const GUI_POINT * pSrc, int NumPoints, float Angle);
void GUI_MagnifyPolygon(GUI_POINT * pDest, const GUI_POINT * pSrc, int NumPoints, int Mag);
void GUI_EnlargePolygon(GUI_POINT * pDest, const GUI_POINT * pSrc, int NumPoints, int Len);




 






int GUI_CreateBitmapFromStreamIDX(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLE4(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLE8(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream565(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM565(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream555(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM555(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamA565(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamAM565(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamA555(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamAM555(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLE16(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLEM16(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream24(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamAlpha(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM8888I(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLEAlpha(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLE32(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream444_12(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM444_12(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream444_12_1(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM444_12_1(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream444_16(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM444_16(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);

int  GUI_CreateBitmapFromStream   (GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
void GUI_DrawStreamedBitmap       (const void * p, int x, int y);
void GUI_DrawStreamedBitmapAuto   (const void * p, int x, int y);
int  GUI_DrawStreamedBitmapEx     (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapExAuto (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmap555Ex  (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapM555Ex (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmap565Ex  (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapM565Ex (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapA555Ex (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapAM555Ex(GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapA565Ex (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapAM565Ex(GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmap24Ex   (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
void GUI_GetStreamedBitmapInfo    (const void * p, GUI_BITMAPSTREAM_INFO * pInfo);
int  GUI_GetStreamedBitmapInfoEx  (GUI_GET_DATA_FUNC * pfGetData, const void * p, GUI_BITMAPSTREAM_INFO * pInfo);
void GUI_SetStreamedBitmapHook    (GUI_BITMAPSTREAM_CALLBACK pfStreamedBitmapHook);

void LCD__RLE4_SetFunc (GUI_GET_DATA_FUNC * pfGetData, void * pVoid, unsigned long Off, const LCD_LOGPALETTE * pLogPal);
void LCD__RLE8_SetFunc (GUI_GET_DATA_FUNC * pfGetData, void * pVoid, unsigned long Off, const LCD_LOGPALETTE * pLogPal);
void LCD__RLE16_SetFunc(GUI_GET_DATA_FUNC * pfGetData, void * pVoid, unsigned long Off);
void LCD__RLE32_SetFunc(GUI_GET_DATA_FUNC * pfGetData, void * pVoid, unsigned long Off);




 
void GUI_BMP_Serialize     (GUI_CALLBACK_VOID_U8_P * pfSerialize, void * p);
void GUI_BMP_SerializeEx   (GUI_CALLBACK_VOID_U8_P * pfSerialize, int x0, int y0, int xSize, int ySize, void * p);
void GUI_BMP_SerializeExBpp(GUI_CALLBACK_VOID_U8_P * pfSerialize, int x0, int y0, int xSize, int ySize, void * p, int BitsPerPixel);




 
void           GUI_Delay  (int Period);
int GUI_GetTime(void);
int            GUI_Exec(void);          
int            GUI_Exec1(void);         




 
int     GUI_MessageBox   (const char * sMessage, const char * sCaption, int Flags);









 



typedef signed long GUI_TIMER_HANDLE;

typedef struct {
  int   Time;
  unsigned long              Context;
  GUI_TIMER_HANDLE hTimer;
} GUI_TIMER_MESSAGE;

typedef void GUI_TIMER_CALLBACK(  GUI_TIMER_MESSAGE* pTM);

GUI_TIMER_HANDLE GUI_TIMER_Create   (GUI_TIMER_CALLBACK * cb, int Time, unsigned long Context, unsigned short Flags);
void             GUI_TIMER_Delete   (GUI_TIMER_HANDLE hObj);

 
int GUI_TIMER_GetPeriod(GUI_TIMER_HANDLE hObj);
void           GUI_TIMER_SetPeriod(GUI_TIMER_HANDLE hObj, int Period);
void           GUI_TIMER_SetTime  (GUI_TIMER_HANDLE hObj, int Period);
void           GUI_TIMER_SetDelay (GUI_TIMER_HANDLE hObj, int Delay);
void           GUI_TIMER_Restart  (GUI_TIMER_HANDLE hObj);
int            GUI_TIMER_GetFlag  (GUI_TIMER_HANDLE hObj, int Flag);  
int            GUI_TIMER_Exec     (void);




 



void GUI_AA_DisableHiRes     (void);
void GUI_AA_EnableHiRes      (void);
int  GUI_AA_GetFactor        (void);
void GUI_AA_SetFactor        (int Factor);
void GUI_AA_DrawArc          (int x0, int y0, int rx, int ry, int a0, int a1);
void GUI_AA_DrawLine         (int x0, int y0, int x1, int y1);
void GUI_AA_DrawPolyOutline  (const GUI_POINT * pSrc, int NumPoints, int Thickness, int x, int y);
void GUI_AA_DrawPolyOutlineEx(const GUI_POINT * pSrc, int NumPoints, int Thickness, int x, int y, GUI_POINT * pBuffer);
void GUI_AA_DrawRoundedRect  (int x0, int y0, int x1, int y1, int r);
void GUI_AA_DrawRoundedRectEx(GUI_RECT * pRect, int r);
void GUI_AA_FillCircle       (int x0, int y0, int r);
void GUI_AA_FillEllipse      (int x0, int y0, int rx, int ry);
void GUI_AA_FillPolygon      (GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GUI_AA_FillRoundedRect  (int x0, int y0, int x1, int y1, int r);
void GUI_AA_FillRoundedRectEx(GUI_RECT * pRect, int r);

int  GUI_AA_SetDrawMode      (int Mode);
void GUI_AA_SetpfDrawCharAA4 (int (* pfDrawChar)(int LayerIndex, int x, int y, unsigned char const * p, int xSize, int ySize, int BytesPerLine));






 
 
void GUI_StoreKeyMsg(int Key, int Pressed);
void GUI_SendKeyMsg (int Key, int Pressed);
int  GUI_PollKeyMsg (void);
void GUI_GetKeyState(GUI_KEY_STATE * pState);

void GUI_KEY__SetHook(void (* pfHook)(const GUI_KEY_STATE *));

 
int  GUI_GetKey(void);
int  GUI_WaitKey(void);
void GUI_StoreKey(int c);
void GUI_ClearKeyBuffer(void);




 
void GUI_WaitEvent            (void);
void GUI_SignalEvent          (void);
void GUI_SetSignalEventFunc   (GUI_SIGNAL_EVENT_FUNC     pfSignalEvent);
void GUI_SetWaitEventFunc     (GUI_WAIT_EVENT_FUNC       pfWaitEvent);
void GUI_SetWaitEventTimedFunc(GUI_WAIT_EVENT_TIMED_FUNC pfWaitEventTimed);




 
void GUI_JOYSTICK_StoreState(const GUI_PID_STATE * pState);




 
void GUI_PID_StoreState     (const GUI_PID_STATE * pState);
int  GUI_PID_GetState       (      GUI_PID_STATE * pState);
void GUI_PID_GetCurrentState(      GUI_PID_STATE * pState);
int  GUI_PID_IsEmpty        (void);
int  GUI_PID_IsPressed      (void);
void GUI_PID__SetHook       (void (* pfHook)(const GUI_PID_STATE *));




 
int  GUI_MOUSE_GetState  (      GUI_PID_STATE * pState);
void GUI_MOUSE_StoreState(const GUI_PID_STATE * pState);




 
int  GUI_TOUCH_GetLayer     (void);
int  GUI_TOUCH_GetState     (GUI_PID_STATE * pState);
void GUI_TOUCH_GetUnstable  (int * px, int * py);   
void GUI_TOUCH_SetLayer     (int Layer);
void GUI_TOUCH_StoreState   (int x, int y);
void GUI_TOUCH_StoreStateEx (const GUI_PID_STATE * pState);
void GUI_TOUCH_StoreUnstable(int x, int y);




 
void GUI_MOUSE_DRIVER_PS2_Init(void);                
void GUI_MOUSE_DRIVER_PS2_OnRx(unsigned char Data);




 
int  GUI_TOUCH_CalcCoefficients (int NumPoints, int * pxRef, int * pyRef, int * pxSample, int * pySample, int xSize, int ySize);
int  GUI_TOUCH_Calibrate        (int Coord, int Log0, int Log1, int Phys0, int Phys1);
int  GUI_TOUCH_CalibratePoint   (int * px, int * py);
void GUI_TOUCH_EnableCalibration(int OnOff);
void GUI_TOUCH_Exec             (void);
int  GUI_TOUCH_GetxPhys         (void);     
int  GUI_TOUCH_GetyPhys         (void);     
void GUI_TOUCH_SetCalibration   (int (* pFunc)(int *, int *));  
void GUI_TOUCH_SetOrientation   (unsigned Orientation);
int  GUI_TOUCH_TransformPoint   (int * px, int * py);           








 
void GUI_TOUCH_X_ActivateX(void);
void GUI_TOUCH_X_ActivateY(void);
void GUI_TOUCH_X_Disable  (void);
int  GUI_TOUCH_X_MeasureX (void);
int  GUI_TOUCH_X_MeasureY (void);














 



void GUI_X_Config(void);
void GUI_X_Init  (void);




int GUI_X_GetTime(void);
void           GUI_X_Delay  (int Period);




void GUI_X_Unlock   (void);
void GUI_X_Lock     (void);
unsigned long  GUI_X_GetTaskId(void);
void GUI_X_InitOS   (void);




void GUI_X_ExecIdle      (void);
void GUI_X_WaitEvent     (void);
void GUI_X_WaitEventTimed(int Period);
void GUI_X_SignalEvent   (void);




void GUI_X_Log     (const char * s);
void GUI_X_Warn    (const char * s);
void GUI_X_ErrorOut(const char * s);




 
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE4;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE4Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE8;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE8Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE16;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE16Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLEM16;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLEM16Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE32;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE32Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLEAlpha;

extern const GUI_BITMAP_METHODS GUI_BitmapMethods444_12;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM444_12;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods444_12_1;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM444_12_1;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods444_16;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM444_16;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods555;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM555;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods565;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM565;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods24;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods888;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM888;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods8888;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM8888I;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsA565;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsAM565;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsA555;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsAM555;




#line 1504 "..\\STemWin\\inc\\GUI.h"

#line 1524 "..\\STemWin\\inc\\GUI.h"

extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_Ext;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_Frm;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_AA2;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_AA4;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_AA2_EXT;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_AA4_EXT;

extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop;
extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop_Ext;
extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop_Frm;
extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop_AA2_Ext;
extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop_AA4_Ext;









 
#line 1566 "..\\STemWin\\inc\\GUI.h"












 
#line 1587 "..\\STemWin\\inc\\GUI.h"




#line 1601 "..\\STemWin\\inc\\GUI.h"

#line 1612 "..\\STemWin\\inc\\GUI.h"

#line 1623 "..\\STemWin\\inc\\GUI.h"

#line 1634 "..\\STemWin\\inc\\GUI.h"






#line 1648 "..\\STemWin\\inc\\GUI.h"

#line 1659 "..\\STemWin\\inc\\GUI.h"

#line 1670 "..\\STemWin\\inc\\GUI.h"














































#line 1726 "..\\STemWin\\inc\\GUI.h"

#line 1737 "..\\STemWin\\inc\\GUI.h"



#line 1750 "..\\STemWin\\inc\\GUI.h"










 








 








 









 







 
#line 1864 "..\\STemWin\\inc\\GUI.h"

#line 1878 "..\\STemWin\\inc\\GUI.h"



#line 1888 "..\\STemWin\\inc\\GUI.h"











 










extern T_GUI_MTOUCH_STOREEVENT GUI_MTOUCH__pStoreEvent;




void GUI_MTOUCH_Enable          (int OnOff);
int  GUI_MTOUCH_GetEvent        (GUI_MTOUCH_EVENT * pEvent);
int  GUI_MTOUCH_GetTouchInput   (GUI_MTOUCH_EVENT * pEvent, GUI_MTOUCH_INPUT * pBuffer, unsigned Index);
int  GUI_MTOUCH_IsEmpty         (void);
void GUI_MTOUCH_SetOrientation  (int Orientation);
void GUI_MTOUCH_SetOrientationEx(int Orientation, int LayerIndex);
void GUI_MTOUCH_StoreEvent      (GUI_MTOUCH_EVENT * pEvent, GUI_MTOUCH_INPUT * pInput);




 






 



extern const GUI_FONT GUI_Font8_ASCII,        GUI_Font8_1;
extern const GUI_FONT GUI_Font10S_ASCII,      GUI_Font10S_1;
extern const GUI_FONT GUI_Font10_ASCII,       GUI_Font10_1;
extern const GUI_FONT GUI_Font13_ASCII,       GUI_Font13_1;
extern const GUI_FONT GUI_Font13B_ASCII,      GUI_Font13B_1;
extern const GUI_FONT GUI_Font13H_ASCII,      GUI_Font13H_1;
extern const GUI_FONT GUI_Font13HB_ASCII,     GUI_Font13HB_1;
extern const GUI_FONT GUI_Font16_ASCII,       GUI_Font16_1,       GUI_Font16_HK,    GUI_Font16_1HK;
extern const GUI_FONT GUI_Font16B_ASCII,      GUI_Font16B_1;
extern const GUI_FONT GUI_Font20_ASCII,       GUI_Font20_1;
extern const GUI_FONT GUI_Font20B_ASCII,      GUI_Font20B_1;
extern const GUI_FONT GUI_Font24_ASCII,       GUI_Font24_1;
extern const GUI_FONT GUI_Font24B_ASCII,      GUI_Font24B_1;
extern const GUI_FONT GUI_Font32_ASCII,       GUI_Font32_1;
extern const GUI_FONT GUI_Font32B_ASCII,      GUI_Font32B_1;




extern const GUI_FONT GUI_Font20F_ASCII;




extern const GUI_FONT GUI_Font4x6;
extern const GUI_FONT GUI_Font6x8,            GUI_Font6x9;
extern const GUI_FONT GUI_Font6x8_ASCII,      GUI_Font6x8_1;
extern const GUI_FONT GUI_Font8x8,            GUI_Font8x9;
extern const GUI_FONT GUI_Font8x8_ASCII,      GUI_Font8x8_1;
extern const GUI_FONT GUI_Font8x10_ASCII;
extern const GUI_FONT GUI_Font8x12_ASCII;
extern const GUI_FONT GUI_Font8x13_ASCII,     GUI_Font8x13_1;
extern const GUI_FONT GUI_Font8x15B_ASCII,    GUI_Font8x15B_1;
extern const GUI_FONT GUI_Font8x16,           GUI_Font8x17,       GUI_Font8x18;
extern const GUI_FONT GUI_Font8x16x1x2,       GUI_Font8x16x2x2,   GUI_Font8x16x3x3;
extern const GUI_FONT GUI_Font8x16_ASCII,     GUI_Font8x16_1;




extern const GUI_FONT GUI_FontD24x32;
extern const GUI_FONT GUI_FontD32;
extern const GUI_FONT GUI_FontD36x48;
extern const GUI_FONT GUI_FontD48;
extern const GUI_FONT GUI_FontD48x64;
extern const GUI_FONT GUI_FontD64;
extern const GUI_FONT GUI_FontD60x80;
extern const GUI_FONT GUI_FontD80;




extern const GUI_FONT GUI_FontComic18B_ASCII, GUI_FontComic18B_1;
extern const GUI_FONT GUI_FontComic24B_ASCII, GUI_FontComic24B_1;




 



#line 2031 "..\\STemWin\\inc\\GUI.h"









#line 2063 "..\\STemWin\\inc\\GUI.h"




#line 2075 "..\\STemWin\\inc\\GUI.h"
















 
#line 2100 "..\\STemWin\\inc\\GUI.h"

#line 2109 "..\\STemWin\\inc\\GUI.h"

 






 






 









 
#line 2146 "..\\STemWin\\inc\\GUI.h"




 
#line 2407 "..\\STemWin\\inc\\GUI.h"




 
#line 2423 "..\\STemWin\\inc\\GUI.h"



 
#line 60 "..\\STemWin\\inc\\LCD_Private.h"




 
extern const struct tLCDDEV_APIList_struct *   LCD_aAPI[2];




 














 
#line 101 "..\\STemWin\\inc\\LCD_Private.h"
    
void LCD_DIST0_SetPixelIndex(int x, int y, int PixelIndex); unsigned LCD_DIST0_GetPixelIndex(int x, int y); void LCD_DIST0_XorPixel (int x, int y); void LCD_DIST0_DrawHLine (int x0, int y, int x1); void LCD_DIST0_DrawVLine (int x, int y0, int y1); void LCD_DIST0_FillRect (int x0, int y0, int x1, int y1); void LCD_DIST0_DrawBitmap (int x0, int y0, int xsize, int ysize, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, int Diff, const unsigned long * pTrans); void LCD_DIST0_SetOrg (int x, int y); void LCD_DIST0_On (void); void LCD_DIST0_Off (void); int LCD_DIST0_Init (void); void LCD_DIST0_SetLUTEntry (unsigned char Pos, LCD_COLOR Color); void * LCD_DIST0_GetDevFunc (int Index); void LCD_DIST0_ReInit (void);
void LCD_DIST1_SetPixelIndex(int x, int y, int PixelIndex); unsigned LCD_DIST1_GetPixelIndex(int x, int y); void LCD_DIST1_XorPixel (int x, int y); void LCD_DIST1_DrawHLine (int x0, int y, int x1); void LCD_DIST1_DrawVLine (int x, int y0, int y1); void LCD_DIST1_FillRect (int x0, int y0, int x1, int y1); void LCD_DIST1_DrawBitmap (int x0, int y0, int xsize, int ysize, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, int Diff, const unsigned long * pTrans); void LCD_DIST1_SetOrg (int x, int y); void LCD_DIST1_On (void); void LCD_DIST1_Off (void); int LCD_DIST1_Init (void); void LCD_DIST1_SetLUTEntry (unsigned char Pos, LCD_COLOR Color); void * LCD_DIST1_GetDevFunc (int Index); void LCD_DIST1_ReInit (void);
void LCD_DIST2_SetPixelIndex(int x, int y, int PixelIndex); unsigned LCD_DIST2_GetPixelIndex(int x, int y); void LCD_DIST2_XorPixel (int x, int y); void LCD_DIST2_DrawHLine (int x0, int y, int x1); void LCD_DIST2_DrawVLine (int x, int y0, int y1); void LCD_DIST2_FillRect (int x0, int y0, int x1, int y1); void LCD_DIST2_DrawBitmap (int x0, int y0, int xsize, int ysize, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, int Diff, const unsigned long * pTrans); void LCD_DIST2_SetOrg (int x, int y); void LCD_DIST2_On (void); void LCD_DIST2_Off (void); int LCD_DIST2_Init (void); void LCD_DIST2_SetLUTEntry (unsigned char Pos, LCD_COLOR Color); void * LCD_DIST2_GetDevFunc (int Index); void LCD_DIST2_ReInit (void);
void LCD_DIST3_SetPixelIndex(int x, int y, int PixelIndex); unsigned LCD_DIST3_GetPixelIndex(int x, int y); void LCD_DIST3_XorPixel (int x, int y); void LCD_DIST3_DrawHLine (int x0, int y, int x1); void LCD_DIST3_DrawVLine (int x, int y0, int y1); void LCD_DIST3_FillRect (int x0, int y0, int x1, int y1); void LCD_DIST3_DrawBitmap (int x0, int y0, int xsize, int ysize, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, int Diff, const unsigned long * pTrans); void LCD_DIST3_SetOrg (int x, int y); void LCD_DIST3_On (void); void LCD_DIST3_Off (void); int LCD_DIST3_Init (void); void LCD_DIST3_SetLUTEntry (unsigned char Pos, LCD_COLOR Color); void * LCD_DIST3_GetDevFunc (int Index); void LCD_DIST3_ReInit (void);



 
#line 38 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
#line 1 "..\\STemWin\\inc\\GUI_Private.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\GUI_Private.h"
#line 59 "..\\STemWin\\inc\\GUI_Private.h"
#line 1 "..\\STemWin\\inc\\GUI_Debug.h"



































 


















 
  



#line 61 "..\\STemWin\\inc\\GUI_Debug.h"

#line 63 "..\\STemWin\\inc\\GUI_Debug.h"

#line 70 "..\\STemWin\\inc\\GUI_Debug.h"

#line 78 "..\\STemWin\\inc\\GUI_Debug.h"









 












 

 

#line 127 "..\\STemWin\\inc\\GUI_Debug.h"






 

 

#line 160 "..\\STemWin\\inc\\GUI_Debug.h"






 
 

#line 192 "..\\STemWin\\inc\\GUI_Debug.h"






 











 
#line 60 "..\\STemWin\\inc\\GUI_Private.h"
#line 1 "..\\STemWin\\inc\\WM_GUI.h"
































 


















 
  







int       WM__InitIVRSearch(const GUI_RECT* pMaxRect);
int       WM__GetNextIVR   (void);
int       WM__GetOrgX_AA(void);
int       WM__GetOrgY_AA(void);










#line 84 "..\\STemWin\\inc\\WM_GUI.h"








 
#line 62 "..\\STemWin\\inc\\GUI_Private.h"














 








 

























 











 
#line 133 "..\\STemWin\\inc\\GUI_Private.h"






 
extern const unsigned char GUI__aConvert_15_255[(1 << 4)];
extern const unsigned char GUI__aConvert_31_255[(1 << 5)];
extern const unsigned char GUI__aConvert_63_255[(1 << 6)];
extern const unsigned char GUI__aConvert_255_15[(1 << 8)];
extern const unsigned char GUI__aConvert_255_31[(1 << 8)];
extern const unsigned char GUI__aConvert_255_63[(1 << 8)];






 
typedef signed long GUI_USAGE_Handle;
typedef struct tsUSAGE_APIList tUSAGE_APIList;
typedef struct GUI_Usage GUI_USAGE;




typedef GUI_USAGE_Handle tUSAGE_CreateCompatible(GUI_USAGE * p);
typedef void        tUSAGE_AddPixel        (GUI_USAGE * p, int x, int y);
typedef void        tUSAGE_AddHLine        (GUI_USAGE * p, int x0, int y0, int len);
typedef void        tUSAGE_Clear           (GUI_USAGE * p);
typedef void        tUSAGE_Delete          (GUI_USAGE_Handle h);
typedef int         tUSAGE_GetNextDirty    (GUI_USAGE * p, int * pxOff, int yOff);



void GUI_USAGE_DecUseCnt(GUI_USAGE_Handle  hUsage);

GUI_USAGE_Handle GUI_USAGE_BM_Create(int x0, int y0, int xsize, int ysize, int Flags);
void    GUI_USAGE_Select(GUI_USAGE_Handle hUsage);
void    GUI_USAGE_AddRect(GUI_USAGE * pUsage, int x0, int y0, int xSize, int ySize);






struct tsUSAGE_APIList {
  tUSAGE_AddPixel         * pfAddPixel;
  tUSAGE_AddHLine         * pfAddHLine;
  tUSAGE_Clear            * pfClear;
  tUSAGE_CreateCompatible * pfCreateCompatible;
  tUSAGE_Delete           * pfDelete;
  tUSAGE_GetNextDirty     * pfGetNextDirty;
} ;

struct GUI_Usage {
  signed short x0, y0, XSize, YSize;
  const tUSAGE_APIList * pAPI;
  signed short UseCnt;
};







 


typedef struct {
  GUI_DEVICE * pDevice;
  signed short                   x0, y0, XSize, YSize;
  unsigned               BytesPerLine;
  unsigned               BitsPerPixel;
  signed long               hUsage;
} GUI_MEMDEV;



void         GUI_MEMDEV__CopyFromLCD (GUI_MEMDEV_Handle hMem);
void         GUI_MEMDEV__GetRect     (GUI_RECT * pRect);
unsigned     GUI_MEMDEV__Color2Index (LCD_COLOR Color);
LCD_COLOR    GUI_MEMDEV__Index2Color (int Index);
unsigned int GUI_MEMDEV__GetIndexMask(void);
void         GUI_MEMDEV__SetAlphaCallback(unsigned(* pcbSetAlpha)(unsigned char));

GUI_MEMDEV_Handle GUI_MEMDEV__CreateFixed(int x0, int y0, int xSize, int ySize, int Flags,
                                          const GUI_DEVICE_API     * pDeviceAPI,
                                          const LCD_API_COLOR_CONV * pColorConvAPI);

void              GUI_MEMDEV__DrawSizedAt        (GUI_MEMDEV_Handle hMem, int xPos, int yPos, int xSize, int ySize);
GUI_MEMDEV_Handle GUI_MEMDEV__GetEmptyCopy32     (GUI_MEMDEV_Handle hMem, int * pxSize, int * pySize, int * pxPos, int * pyPos);
void              GUI_MEMDEV__ReadLine           (int x0, int y, int x1, unsigned long * pBuffer);
void              GUI_MEMDEV__WriteToActiveAlpha (GUI_MEMDEV_Handle hMem,int x, int y);
void              GUI_MEMDEV__WriteToActiveAt    (GUI_MEMDEV_Handle hMem,int x, int y);
void              GUI_MEMDEV__WriteToActiveOpaque(GUI_MEMDEV_Handle hMem,int x, int y);
void            * GUI_MEMDEV__XY2PTR             (int x,int y);
void            * GUI_MEMDEV__XY2PTREx           (GUI_MEMDEV * pDev, int x,int y);
void              GUI_MEMDEV__BlendColor32       (GUI_MEMDEV_Handle hMem, unsigned long BlendColor, unsigned char BlendIntens);

unsigned GUI__AlphaPreserveTrans(int OnOff);

extern unsigned GUI_MEMDEV__TimePerFrame;













 









 

int  GUI_cos(int angle);
int  GUI_sin(int angle);
extern const unsigned long GUI_Pow10[10];

 
void GUI_MTOUCH__ManagePID(int OnOff);

 
int  GUI_AA_Init       (int x0, int x1);
int  GUI_AA_Init_HiRes (int x0, int x1);
void GUI_AA_Exit       (void);
signed short  GUI_AA_HiRes2Pixel(int HiRes);

void GL_FillCircleAA_HiRes (int x0, int y0, int r);
void GL_FillEllipseAA_HiRes(int x0, int y0, int rx, int ry);

void GUI_AA__DrawCharAA2(int x0, int y0, int XSize, int YSize, int BytesPerLine, const unsigned char * pData);
void GUI_AA__DrawCharAA4(int x0, int y0, int XSize, int YSize, int BytesPerLine, const unsigned char * pData);
void GUI_AA__DrawCharAA8(int x0, int y0, int XSize, int YSize, int BytesPerLine, const unsigned char * pData);

 


int      GUI__GetAlphaBuffer    (unsigned long ** ppCurrent, unsigned long ** ppConvert, unsigned long ** ppData, int * pVXSizeMax);
int      GUI__AllocAlphaBuffer  (int AllocDataBuffer);
unsigned long    * GUI__DoAlphaBlending   (int x, int y, unsigned long * pData, int xSize, tLCDDEV_Index2Color * pfIndex2Color_DEV, int * pDone);
unsigned GUI__SetAlphaBufferSize(int xSize);

 
int        GUI_SIF__GetCharDistX       (unsigned short c, int * pSizeX);
void       GUI_SIF__GetFontInfo        (const GUI_FONT * pFont, GUI_FONTINFO * pfi);
char       GUI_SIF__IsInFont           (const GUI_FONT * pFont, unsigned short c);
const unsigned char * GUI_SIF__GetpCharInfo       (const GUI_FONT * pFont, unsigned short c, unsigned SizeOfCharInfo);
int        GUI_SIF__GetNumCharAreas    (const GUI_FONT * pFont);
int        GUI_SIF__GetCharDistX_ExtFrm(unsigned short c, int * pSizeX);
void       GUI_SIF__GetFontInfo_ExtFrm (const GUI_FONT * pFont, GUI_FONTINFO * pfi);
char       GUI_SIF__IsInFont_ExtFrm    (const GUI_FONT * pFont, unsigned short c);
int        GUI_SIF__GetCharInfo_ExtFrm (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void       GUI_SIF__ClearLine_ExtFrm   (const char * s, int Len);

 
int        GUI_XBF__GetOff       (const GUI_XBF_DATA * pXBF_Data, unsigned c, unsigned long * pOff);
int        GUI_XBF__GetOffAndSize(const GUI_XBF_DATA * pXBF_Data, unsigned c, unsigned long * pOff, unsigned short * pSize);
int        GUI_XBF__GetCharDistX (unsigned short c, int * pSizeX);
void       GUI_XBF__GetFontInfo  (const GUI_FONT * pFont, GUI_FONTINFO * pInfo);
char       GUI_XBF__IsInFont     (const GUI_FONT * pFont, unsigned short c);
int        GUI_XBF__GetCharInfo  (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void       GUI_XBF__ClearLine    (const char * s, int Len);

 
void GUI_AddHex     (unsigned long v, unsigned char Len, char ** ps);
void GUI_AddBin     (unsigned long v, unsigned char Len, char ** ps);
void GUI_AddDecMin  (signed long v, char ** ps);
void GUI_AddDec     (signed long v, unsigned char Len, char ** ps);
void GUI_AddDecShift(signed long v, unsigned char Len, unsigned char Shift, char ** ps);
long GUI_AddSign    (long v, char ** ps);
int  GUI_Long2Len   (signed long v);




int   GUI_UC__CalcSizeOfChar   (unsigned short Char);
unsigned short   GUI_UC__GetCharCodeInc   (const char ** ps);
int   GUI_UC__NumChars2NumBytes(const char * s, int NumChars);
int   GUI_UC__NumBytes2NumChars(const char * s, int NumBytes);

int  GUI__GetLineNumChars  (const char * s, int MaxNumChars);
int  GUI__GetNumChars      (const char * s);
int  GUI__GetOverlap       (unsigned short Char);
int  GUI__GetLineDistX     (const char * s, int Len);
int  GUI__GetFontSizeY     (void);
int  GUI__HandleEOLine     (const char ** ps);
void GUI__DispLine         (const char * s, int Len, const GUI_RECT * pr);
void GUI__AddSpaceHex      (unsigned long v, unsigned char Len, char ** ps);
void GUI__CalcTextRect     (const char * pText, const GUI_RECT * pTextRectIn, GUI_RECT * pTextRectOut, int TextAlign);

void GUI__ClearTextBackground(int xDist, int yDist);

int  GUI__WrapGetNumCharsDisp       (const char * pText, int xSize, GUI_WRAPMODE WrapMode);
int  GUI__WrapGetNumCharsToNextLine (const char * pText, int xSize, GUI_WRAPMODE WrapMode);
int  GUI__WrapGetNumBytesToNextLine (const char * pText, int xSize, GUI_WRAPMODE WrapMode);
void GUI__memset    (unsigned char  * p, unsigned char Fill, int NumBytes);
void GUI__memset16  (unsigned short * p, unsigned short Fill, int NumWords);
int  GUI__strlen    (const char * s);
int  GUI__strcmp    (const char * s0, const char * s1);
int  GUI__strcmp_hp (signed long hs0, const char * s1);

 
int  GUI__GetCursorPosX     (const char * s, int Index, int MaxNumChars);
int  GUI__GetCursorPosChar  (const char * s, int x, int NumCharsToNextLine);
unsigned short  GUI__GetCursorCharacter(const char * s, int Index, int MaxNumChars, int * pIsRTL);

 
unsigned short  GUI__GetPresentationForm     (unsigned short Char, unsigned short Next, unsigned short Prev, int * pIgnoreNext, const char * s);
int  GUI__IsArabicCharacter       (unsigned short c);

 
int  GUI__BIDI_Log2Vis           (const char * s, int NumChars, char * pBuffer, int BufferSize);
int  GUI__BIDI_GetCursorPosX     (const char * s, int NumChars, int Index);
int  GUI__BIDI_GetCursorPosChar  (const char * s, int NumChars, int x);
unsigned short  GUI__BIDI_GetLogChar        (const char * s, int NumChars, int Index);
int  GUI__BIDI_GetCharDir        (const char * s, int NumChars, int Index);
int  GUI__BIDI_IsNSM             (unsigned short Char);
unsigned short  GUI__BIDI_GetCursorCharacter(const char * s, int Index, int MaxNumChars, int * pIsRTL);
int  GUI__BIDI_GetWordWrap       (const char * s, int xSize, int * pxDist);
int  GUI__BIDI_GetCharWrap       (const char * s, int xSize);

const char * GUI__BIDI_Log2VisBuffered(const char * s, int * pMaxNumChars);

extern int GUI__BIDI_Enabled;

extern int (* _pfGUI__BIDI_Log2Vis         )(const char * s, int NumChars, char * pBuffer, int BufferSize);
extern int (* _pfGUI__BIDI_GetCursorPosX   )(const char * s, int NumChars, int Index);
extern int (* _pfGUI__BIDI_GetCursorPosChar)(const char * s, int NumChars, int x);
extern unsigned short (* _pfGUI__BIDI_GetLogChar      )(const char * s, int NumChars, int Index);
extern int (* _pfGUI__BIDI_GetCharDir      )(const char * s, int NumChars, int Index);
extern int (* _pfGUI__BIDI_IsNSM           )(unsigned short Char);

 
extern const char * (* GUI_CharLine_pfLog2Vis)(const char * s, int * pMaxNumChars);

extern int (* GUI__GetCursorPos_pfGetPosX)     (const char * s, int MaxNumChars, int Index);
extern int (* GUI__GetCursorPos_pfGetPosChar)  (const char * s, int MaxNumChars, int x);
extern unsigned short (* GUI__GetCursorPos_pfGetCharacter)(const char * s, int MaxNumChars, int Index, int * pIsRTL);

extern int (* GUI__Wrap_pfGetWordWrap)(const char * s, int xSize, int * pxDist);
extern int (* GUI__Wrap_pfGetCharWrap)(const char * s, int xSize);

 
const GUI_FONT_PROP * GUIPROP__FindChar(const GUI_FONT_PROP * pProp, unsigned short c);

 
const GUI_FONT_PROP_EXT * GUIPROP_EXT__FindChar(const GUI_FONT_PROP_EXT * pPropExt, unsigned short c);
void  GUIPROP_EXT__DispLine      (const char * s, int Len);
void  GUIPROP_EXT__ClearLine     (const char * s, int Len);
void  GUIPROP_EXT__SetfpClearLine(void (* fpClearLine)(const char * s, int Len));

 
unsigned short GUI__Read16(const unsigned char ** ppData);
unsigned long GUI__Read32(const unsigned char ** ppData);

 
void GUI__GetOrg(int * px, int * py);
void GUI__SetOrgHook(void(* pfHook)(int x, int y));

 
int              GUI_TIMER__IsActive       (void);
int   GUI_TIMER__GetPeriod      (void);
GUI_TIMER_HANDLE GUI_TIMER__GetNextTimer   (GUI_TIMER_HANDLE hTimer, unsigned long * pContext);
GUI_TIMER_HANDLE GUI_TIMER__GetFirstTimer  (unsigned long * pContext);
GUI_TIMER_HANDLE GUI_TIMER__GetNextTimerLin(GUI_TIMER_HANDLE hTimer, unsigned long * pContext);

 
tLCDDEV_Index2Color * GUI_GetpfIndex2ColorEx(int LayerIndex);
tLCDDEV_Color2Index * GUI_GetpfColor2IndexEx(int LayerIndex);

int GUI_GetBitsPerPixelEx(int LayerIndex);

unsigned long * LCD_GetpPalConvTable        (const LCD_LOGPALETTE * pLogPal);
unsigned long * LCD_GetpPalConvTableUncached(const LCD_LOGPALETTE * pLogPal);
unsigned long * LCD_GetpPalConvTableBM      (const LCD_LOGPALETTE * pLogPal, const GUI_BITMAP * pBitmap, int LayerIndex);

 
void GUI_SetFuncGetpPalConvTable(unsigned long * (* pFunc)(const LCD_LOGPALETTE * pLogPal, const GUI_BITMAP * pBitmap, int LayerIndex));







 
#line 459 "..\\STemWin\\inc\\GUI_Private.h"


void GUI__ReadHeaderFromStream  (GUI_BITMAP_STREAM * pBitmapHeader, const unsigned char * pData);
void GUI__CreateBitmapFromStream(const GUI_BITMAP_STREAM * pBitmapHeader, const void * pData, GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const GUI_BITMAP_METHODS * pMethods);

 
int GUI__ManageCache  (int Cmd);
int GUI__ManageCacheEx(int LayerIndex, int Cmd);






 
void GL_DispChar         (unsigned short c);
void GL_DrawArc          (int x0, int y0, int rx, int ry, int a0, int a1);
void GL_DrawBitmap       (const GUI_BITMAP * pBM, int x0, int y0);
void GL_DrawCircle       (int x0, int y0, int r);
void GL_DrawEllipse      (int x0, int y0, int rx, int ry, int w);
void GL_DrawHLine        (int y0, int x0, int x1);
void GL_DrawPolygon      (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GL_DrawPoint        (int x,  int y);
void GL_DrawLine1        (int x0, int y0, int x1, int y1);
void GL_DrawLine1Ex      (int x0, int y0, int x1, int y1, unsigned * pPixelCnt);
void GL_DrawLineRel      (int dx, int dy);
void GL_DrawLineTo       (int x,  int y);
void GL_DrawLineToEx     (int x,  int y, unsigned * pPixelCnt);
void GL_DrawLine         (int x0, int y0, int x1, int y1);
void GL_DrawLineEx       (int x0, int y0, int x1, int y1, unsigned * pPixelCnt);
void GL_MoveTo           (int x,  int y);
void GL_FillCircle       (int x0, int y0, int r);
void GL_FillCircleAA     (int x0, int y0, int r);
void GL_FillEllipse      (int x0, int y0, int rx, int ry);
void GL_FillPolygon      (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GL_SetDefault       (void);








 
typedef int  GUI_tfTimer(void);
typedef int  WM_tfHandlePID(void);







 
extern const unsigned char  GUI_Pixels_ArrowS[45];
extern const unsigned char  GUI_Pixels_ArrowM[60];
extern const unsigned char  GUI_Pixels_ArrowL[150];
extern const unsigned char  GUI_Pixels_CrossS[33];
extern const unsigned char  GUI_Pixels_CrossM[126];
extern const unsigned char  GUI_Pixels_CrossL[248];
extern const unsigned char  GUI_PixelsHeaderM[5 * 17];

extern const GUI_LOGPALETTE GUI_CursorPal;
extern const GUI_LOGPALETTE GUI_CursorPalI;







 
extern GUI_RECT  GUI_RectDispString;  






 
extern unsigned char GUI__CharHasTrans;






 
extern int GUITASK__EntranceCnt;






 

int       GUI_GetBitmapPixelIndex(const GUI_BITMAP * pBMP, unsigned x, unsigned y);
GUI_COLOR GUI_GetBitmapPixelColor(const GUI_BITMAP * pBMP, unsigned x, unsigned y);
int       GUI_GetBitmapPixelIndexEx(int BitsPerPixel, int BytesPerLine, const unsigned char * pData, unsigned x, unsigned y);

void      GUI__DrawBitmap16bpp (int x0, int y0, int xsize, int ysize, const unsigned char * pPixel, const LCD_LOGPALETTE * pLogPal, int xMag, int yMag, tLCDDEV_Index2Color * pfIndex2Color, const LCD_API_COLOR_CONV * pColorConvAPI);
void      GUI__DrawBitmapA16bpp(int x0, int y0, int xSize, int ySize, const unsigned char * pPixel, const LCD_LOGPALETTE * pLogPal, int xMag, int yMag, tLCDDEV_Index2Color * pfIndex2Color);
void      GUI__SetPixelAlpha   (int x, int y, unsigned char Alpha, LCD_COLOR Color);
LCD_COLOR GUI__MixColors       (LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens);
void      GUI__MixColorsBulk   (unsigned long * pFG, unsigned long * pBG, unsigned long * pDst, unsigned OffFG, unsigned OffBG, unsigned OffDest, unsigned xSize, unsigned ySize, unsigned char Intens);

extern const GUI_UC_ENC_APILIST GUI_UC_None;






 



#line 589 "..\\STemWin\\inc\\GUI_Private.h"

void LCD_ReadRect  (int x0, int y0, int x1, int y1, unsigned long * pBuffer, GUI_DEVICE * pDevice);
void GUI_ReadRect  (int x0, int y0, int x1, int y1, unsigned long * pBuffer, GUI_DEVICE * pDevice);
void GUI_ReadRectEx(int x0, int y0, int x1, int y1, unsigned long * pBuffer, GUI_DEVICE * pDevice);

void LCD_ReadRectNoClip(int x0, int y0, int x1, int y1, unsigned long * pBuffer, GUI_DEVICE * pDevice);






 
typedef struct {
  void         (* pfSetColor)   (LCD_COLOR Index);
  void         (* pfSetBkColor) (LCD_COLOR Index);
  LCD_DRAWMODE (* pfSetDrawMode)(LCD_DRAWMODE dm);
} LCD_SET_COLOR_API;

extern const LCD_SET_COLOR_API * LCD__pSetColorAPI;






 








 
extern const GUI_FONT * GUI__pFontDefault;

extern  GUI_CONTEXT * GUI_pContext;

extern GUI_DEVICE * GUI__apDevice[2];




extern unsigned long * (* GUI_pfGetpPalConvTable)(const LCD_LOGPALETTE * pLogPal, const GUI_BITMAP * pBitmap, int LayerIndex);




extern LCD_COLOR (* GUI__pfMixColors)(LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens);




extern void (* GUI__pfMixColorsBulk)(unsigned long * pFG, unsigned long * pBG, unsigned long * pDst, unsigned OffFG, unsigned OffBG, unsigned OffDest, unsigned xSize, unsigned ySize, unsigned char Intens);




extern const GUI_MULTIBUF_API    GUI_MULTIBUF_APIList;
extern const GUI_MULTIBUF_API_EX GUI_MULTIBUF_APIListEx;







extern   int  (* GUI_pfUpdateSoftLayer)(void);





extern void (* GUI_pfHookMTOUCH)(const GUI_MTOUCH_STATE * pState);

extern const GUI_UC_ENC_APILIST * GUI_pUC_API;  

extern  char             GUI_DecChar;
extern           GUI_tfTimer    * GUI_pfTimerExec;
extern           WM_tfHandlePID * WM_pfHandlePID;
extern   void (* GUI_pfDispCharStyle)(unsigned short Char);

extern           int GUI__BufferSize; 
extern           int GUI_AA__ClipX0;  

extern           signed char  GUI__aNumBuffers[2]; 
extern           unsigned char  GUI__PreserveTrans;
extern           unsigned char  GUI__IsInitialized;


  extern const tLCD_APIList * GUI_pLCD_APIList;  


extern signed short GUI_OrgX, GUI_OrgY;









 
#line 39 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
#line 1 "..\\STemWin\\inc\\LCD_SIM.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LCD_SIM.h"






 











 
void  LCDSIM_PreInit(void);
char* LCDSIM_Init(void);
void  LCDSIM_Exit(void);
int   LCDSIM_GetMouseState(LCD_tMouseState *pState);
void  LCDSIM_SetMouseState(int x, int y, int KeyStat, int LayerIndex);
void  LCDSIM_CheckMouseState(int LayerIndex);
int   LCDSIM_SaveBMP   (const char * sFileName);
int   LCDSIM_SaveBMPEx (const char * sFileName, int x0, int y0, int xSize, int ySize);
int   LCDSIM_SaveSBMP  (const char * sFileName);
int   LCDSIM_SaveSBMPEx(const char * sFileName, int x0, int y0, int xSize, int ySize);
void  LCDSIM_SetRGBOrder(unsigned RGBOrder);






 
void LCDSIM_FillRect(int x0, int y0, int x1, int y1, int Index, int LayerIndex);
int  LCDSIM_GetModifyCnt(int LayerIndex);
int  LCDSIM_GetModifyCntInfo(int LayerIndex);
int  LCDSIM_GetPixelColor(int x, int y, int LayerIndex);
int  LCDSIM_GetPixelIndex(int x, int y, int LayerIndex);
int  LCDSIM_Index2Color(int Index, int LayerIndex);
int  LCDSIM_RLUT_GetPixelIndex(int x, int y, int LayerIndex);
void LCDSIM_RLUT_SetPixelIndex(int x, int y, int Index, int LayerIndex);
void LCDSIM_SetLUTEntry(unsigned char Pos, LCD_COLOR color, int LayerIndex);
void LCDSIM_SetPixelIndex(int x, int y, int Index, int LayerIndex);
void LCDSIM_SetPixelColor(int x, int y, LCD_COLOR PixelColor, int LayerIndex);
void LCDSIM_SetSubPixel(int x, int y, unsigned char Value, int LayerIndex);
void LCDSIM_SetPixelPhys(int x, int y, int Index, int LayerIndex);
int  LCDSIM_GetPixelPhys(int xPhys, int yPhys, int LayerIndex);
void LCDSIM_FillRectPhys(int x0Phys, int y0Phys, int x1Phys, int y1Phys, int Index, int LayerIndex);
void LCDSIM_SetOrg(int x, int y, int LayerIndex);
void LCDSIM_SetAlpha(int Alpha, int LayerIndex);
int  LCDSIM_GetAlpha(int LayerIndex);
void LCDSIM_SetLayerPos(int xPos, int yPos, int LayerIndex);
void LCDSIM_SetLayerVis(int OnOff, int LayerIndex);
void LCDSIM_SetSize(int LayerIndex, int xSize, int ySize);
void LCDSIM_SetTransMode(int LayerIndex, int TransMode);
void LCDSIM_SetChroma(int LayerIndex, LCD_COLOR ChromaMin, LCD_COLOR ChromaMax);
void LCDSIM_SetCompositeColor(unsigned long Color);
void LCDSIM_SetCompositeSize(int xSize, int ySize);
void LCDSIM_CopyBuffer(int LayerIndex, int IndexSrc, int IndexDst);
void LCDSIM_Invalidate(int LayerIndex);






 
void SIM_GUI_SetCompositeSize(int xSize, int ySize);
void SIM_GUI_SetCompositeColor(unsigned long Color);
unsigned long  SIM_GUI_GetCompositeColor(void);
void SIM_GUI_SetLCDPos(int xPos, int yPos);
int  SIM_GUI_SetTransColor(int Color);
int  SIM_GUI_SetLCDColorBlack (unsigned int Index, int Color);
int  SIM_GUI_SetLCDColorWhite (unsigned int Index, int Color);
void SIM_GUI_SetMag(int MagX, int MagY);
int  SIM_GUI_GetMagX(void);
int  SIM_GUI_GetMagY(void);
int  SIM_GUI_GetForwardRButton(void);
void SIM_GUI_SetForwardRButton(int OnOff);
void SIM_GUI_SetTransMode(int LayerIndex, int TransMode);
void SIM_GUI_SetChroma(int LayerIndex, unsigned long ChromaMin, unsigned long ChromaMax);
void SIM_GUI_UseCustomBitmaps(void);
void SIM_GUI_SetAccellerator(int Accellerator);
void SIM_GUI_SetMainScreenOffset(int x, int y);
void SIM_GUI_SetCompositeTouch(int LayerIndex);
int  SIM_GUI_GetCompositeTouch(void);






 
void SIM_X_Config(void);    






 
void SIM_GUI_Delay (int ms);
void SIM_GUI_ExecIdle(void);
int  SIM_GUI_GetTime(void);
int  SIM_GUI_GetKey(void);
int  SIM_GUI_WaitKey(void);
void SIM_GUI_StoreKey(int);






 
void SIM_GUI_Log(const char *s);
void SIM_GUI_Log1(const char *s, int p0);
void SIM_GUI_Log2(const char *s, int p0, int p1);
void SIM_GUI_Log3(const char *s, int p0, int p1, int p2);
void SIM_GUI_Log4(const char *s, int p0, int p1, int p2,int p3);
void SIM_GUI_Warn(const char *s);
void SIM_GUI_Warn1(const char *s, int p0);
void SIM_GUI_Warn2(const char *s, int p0, int p1);
void SIM_GUI_Warn3(const char *s, int p0, int p1, int p2);
void SIM_GUI_Warn4(const char *s, int p0, int p1, int p2, int p3);
void SIM_GUI_ErrorOut(const char *s);
void SIM_GUI_ErrorOut1(const char *s, int p0);
void SIM_GUI_ErrorOut2(const char *s, int p0, int p1);
void SIM_GUI_ErrorOut3(const char *s, int p0, int p1, int p2);
void SIM_GUI_ErrorOut4(const char *s, int p0, int p1, int p2, int p3);
void SIM_GUI_EnableMessageBoxOnError(int Status);






 
const char *SIM_GUI_GetCmdLine(void);






 
void SIM_GUI_CreateTask(char * pName, void * pFunc);
void SIM_GUI_Start(void);
unsigned long SIM_GUI_GetTaskID(void);
void SIM_GUI_Lock(void);
void SIM_GUI_Unlock(void);
void SIM_GUI_InitOS(void);






#line 40 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
#line 1 "..\\STemWin\\inc\\LCD_ConfDefaults.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LCD_ConfDefaults.h"




 
#line 90 "..\\STemWin\\inc\\LCD_ConfDefaults.h"



 
#line 41 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
#line 1 "..\\User\\bsp\\bsp_tft_lcd.h"











 









 












 
enum
{
	IC_5420		= 0x5420,
	IC_4001		= 0x4001,
	IC_61509 	= 0xB509,
	IC_8875 	= 0x0075,	
	IC_9488 	= 0x9488,
	IC_7789 	= 0x7789,
	IC_UNKN 	= 0xEEEE
};















 




enum
{
	CL_WHITE        = (((255 >> 3) << 11) | ((255 >> 2) << 5) | (255 >> 3)),	 
	CL_BLACK        = (((0 >> 3) << 11) | ((0 >> 2) << 5) | (0 >> 3)),	 
	CL_RED          = (((255 >> 3) << 11) | ((0 >> 2) << 5) | (0 >> 3)),	 
	CL_GREEN        = (((0 >> 3) << 11) | ((255 >> 2) << 5) | (0 >> 3)),	 
	CL_BLUE         = (((0 >> 3) << 11) | ((0 >> 2) << 5) | (255 >> 3)),	 
	CL_YELLOW       = (((255 >> 3) << 11) | ((255 >> 2) << 5) | (0 >> 3)),	 

	CL_GREY			= (((98 >> 3) << 11) | ((98 >> 2) << 5) | (98 >> 3)), 	 
	CL_GREY1		= (((150 >> 3) << 11) | ((150 >> 2) << 5) | (150 >> 3)), 	 
	CL_GREY2		= (((180 >> 3) << 11) | ((180 >> 2) << 5) | (180 >> 3)), 	 
	CL_GREY3		= (((200 >> 3) << 11) | ((200 >> 2) << 5) | (200 >> 3)), 	 
	CL_GREY4		= (((230 >> 3) << 11) | ((230 >> 2) << 5) | (230 >> 3)), 	 

	CL_BUTTON_GREY	= (((220 >> 3) << 11) | ((220 >> 2) << 5) | (220 >> 3)),  

	CL_MAGENTA      = 0xF81F,	 
	CL_CYAN         = 0x7FFF,	 

	CL_BLUE1        = (((0 >> 3) << 11) | ((0 >> 2) << 5) | (240 >> 3)),		 
	CL_BLUE2        = (((0 >> 3) << 11) | ((0 >> 2) << 5) | (128 >> 3)),		 
	CL_BLUE3        = (((68 >> 3) << 11) | ((68 >> 2) << 5) | (255 >> 3)),		 
	CL_BLUE4        = (((0 >> 3) << 11) | ((64 >> 2) << 5) | (128 >> 3)),		 

	 
	CL_BTN_FACE		= (((236 >> 3) << 11) | ((233 >> 2) << 5) | (216 >> 3)),	 
	
	CL_BTN_FONT		= CL_BLACK,				 
	
	CL_BOX_BORDER1	= (((172 >> 3) << 11) | ((168 >> 2) << 5) | (153 >> 3)),	 
	CL_BOX_BORDER2	= (((255 >> 3) << 11) | ((255 >> 2) << 5) | (255 >> 3)),	 


	CL_MASK			= 0x9999	 
};

 
enum
{
	ALIGN_LEFT = 0,
	ALIGN_CENTER = 1,
	ALIGN_RIGHT = 2
};

 
enum
{
	EDIT_BORDER_COLOR		= CL_BLUE2,		 
	EDIT_BACK_COLOR			= CL_WHITE,			 
};

 
enum
{
	BUTTON_BORDER_COLOR		= CL_BLUE2,			 
	BUTTON_BORDER1_COLOR	= CL_WHITE,			 
	BUTTON_BORDER2_COLOR	= CL_GREY1,			 
	BUTTON_BACK_COLOR		= CL_GREY3,			 
	BUTTON_ACTIVE_COLOR		= CL_CYAN,			 
};

 
enum
{
	WIN_BORDER_COLOR	= CL_BLUE4,		 
	WIN_TITLE_COLOR		= CL_BLUE3,		 
	WIN_CAPTION_COLOR	= CL_WHITE,		 
	WIN_BODY_COLOR		= CL_GREY2,		 
};

 
enum
{
	CHECK_BOX_BORDER_COLOR	= CL_BLUE2,		 
	CHECK_BOX_BACK_COLOR	= CL_GREY3,		 
	CHECK_BOX_CHECKED_COLOR	= CL_RED,		 

	CHECK_BOX_H			= 24,				 
	CHECK_BOX_W			= 24,				 
};

 
typedef enum
{
	FC_ST_12 = 0,		 
	FC_ST_16,			 
	FC_ST_24,			 
	FC_ST_32,			 	
	
	FC_RA8875_16,		 
	FC_RA8875_24,		 
	FC_RA8875_32		 	
}FONT_CODE_E;

 
typedef struct
{
	FONT_CODE_E FontCode;	 
	uint16_t FrontColor; 
	uint16_t BackColor;	 
	uint16_t Space;		 
}FONT_T;

 
typedef enum
{
	ID_ICON		= 1,
	ID_WIN		= 2,
	ID_LABEL	= 3,
	ID_BUTTON	= 4,
	ID_CHECK 	= 5,
	ID_EDIT 	= 6,
	ID_GROUP 	= 7,
}CONTROL_ID_T;

 
typedef struct
{
	uint8_t id;
	uint16_t Left;		 
	uint16_t Top;		 
	uint16_t Height;	 
	uint16_t Width;		 
	uint16_t *pBmp;		 
	char  Text[16];	 
}ICON_T;

 
typedef struct
{
	uint8_t id;
	uint16_t Left;
	uint16_t Top;
	uint16_t Height;
	uint16_t Width;
	uint16_t Color;
	FONT_T *Font;
	char *pCaption;
}WIN_T;

 
typedef struct
{
	uint8_t id;
	uint16_t Left;			 
	uint16_t Top;			 
	uint16_t Height;		 
	uint16_t Width;			 
	uint16_t MaxLen;		 
	FONT_T *Font;			 
	char  *pCaption;
}LABEL_T;

 
typedef struct
{
	uint8_t id;
	uint16_t Left;
	uint16_t Top;
	uint16_t Height;
	uint16_t Width;
	 
	FONT_T *Font;			 
	char *pCaption;
	uint8_t Focus;			 
}BUTTON_T;

 
typedef struct
{
	uint8_t id;
	uint16_t Left;
	uint16_t Top;
	uint16_t Height;
	uint16_t Width;
	uint16_t Color;
	FONT_T *Font;			 
	char   *pCaption;
	char Text[32];			 
}EDIT_T;

 
typedef struct
{
	uint8_t id;
	uint16_t Left;			 
	uint16_t Top;			 
	uint16_t Height;		 
	uint16_t Width;			 
	uint16_t Color;			 
	FONT_T *Font;			 
	char  *pCaption;
	uint8_t Checked;		 
}CHECK_T;

 
typedef struct
{
	uint8_t id;
	uint16_t Left;			 
	uint16_t Top;			 
	uint16_t Height;		 
	uint16_t Width;			 
	FONT_T *Font;			 
	char  *pCaption;
}GROUP_T;

 





 
void LCD_InitHard(void);
void LCD_GetChipDescribe(char *_str);
uint16_t LCD_GetHeight(void);
uint16_t LCD_GetWidth(void);
void LCD_DispOn(void);
void LCD_DispOff(void);
void LCD_ClrScr(uint16_t _usColor);
void LCD_DispStr(uint16_t _usX, uint16_t _usY, char *_ptr, FONT_T *_tFont);
void LCD_PutPixel(uint16_t _usX, uint16_t _usY, uint16_t _usColor);
uint16_t LCD_GetPixel(uint16_t _usX, uint16_t _usY);
void LCD_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint16_t _usColor);
void LCD_Draw_VLine(uint16_t _usX1, uint16_t _usY1, uint16_t _usY2, uint16_t _usColor);
void LCD_Draw_HLine(uint16_t _usX1, uint16_t _usY1, uint16_t _usX2, uint16_t _usColor);
void LCD_Draw_HColorLine(uint16_t _usX1, uint16_t _usY1, uint16_t _usWidth, const uint16_t * _pColor);
void LCD_DrawPoints(uint16_t *x, uint16_t *y, uint16_t _usSize, uint16_t _usColor);
void LCD_DrawRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);
void LCD_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint16_t _usColor);
void LCD_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t *_ptr);
void LCD_SetBackLight(uint8_t _bright);
uint8_t LCD_GetBackLight(void);

void LCD_Fill_Rect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);

void LCD_DrawWin(WIN_T *_pWin);
void LCD_DrawIcon(const ICON_T *_tIcon, FONT_T *_tFont, uint8_t _ucFocusMode);
void LCD_DrawEdit(EDIT_T *_pEdit);
void LCD_DrawButton(BUTTON_T *_pBtn);
void LCD_DrawLabel(LABEL_T *_pLabel);
void LCD_DrawCheckBox(CHECK_T *_pCheckBox);
void LCD_DrawGroupBox(GROUP_T *_pBox);

void LCD_DispControl(void *_pControl);

void LCD_DrawIcon32(const ICON_T *_tIcon, FONT_T *_tFont, uint8_t _ucFocusMode);
void LCD_DrawBmp32(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint8_t *_pBmp);

uint16_t LCD_GetFontWidth(FONT_T *_tFont);
uint16_t LCD_GetFontHeight(FONT_T *_tFont);
uint16_t LCD_GetStrWidth(char *_ptr, FONT_T *_tFont);
void LCD_DispStrEx(uint16_t _usX, uint16_t _usY, char *_ptr, FONT_T *_tFont, uint16_t _Width,
	uint8_t _Align);

void LCD_SetDirection(uint8_t _dir);
uint8_t LCD_ButtonTouchDown(BUTTON_T *_btn, uint16_t _usX, uint16_t _usY);
uint8_t LCD_ButtonTouchRelease(BUTTON_T *_btn, uint16_t _usX, uint16_t _usY);
void LCD_InitButton(BUTTON_T *_btn, uint16_t _x, uint16_t _y, uint16_t _h, uint16_t _w, char *_pCaption, FONT_T *_pFont);

 
extern uint16_t g_ChipID;			 
extern uint16_t g_LcdHeight;		 
extern uint16_t g_LcdWidth;			 
extern uint8_t g_LcdDirection;		 




#line 42 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
#line 1 "..\\User\\bsp\\bsp_lcd_st7789v.h"











 







 
void ST7789V_SoftReset(void);
uint32_t ST7789V_ReadID(void);
uint32_t ST7789V_InitHard(void);
void ST7789V_DispOn(void);
void ST7789V_DispOff(void);
void ST7789V_ClrScr(uint16_t _usColor);
void ST7789V_PutPixel(uint16_t _usX, uint16_t _usY, uint16_t _usColor);
uint16_t ST7789V_GetPixel(uint16_t _usX, uint16_t _usY);
void ST7789V_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint16_t _usColor);
void ST7789V_DrawHLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usColor);
void ST7789V_DrawHColorLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor);
void ST7789V_DrawHTransLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor);
void ST7789V_DrawRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);
void ST7789V_FillRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);
void ST7789V_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint16_t _usColor);
void ST7789V_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t *_ptr);

void ST7789V_SetBackLight(uint8_t _bright);
void ST7789V_SetDirection(uint8_t _ucDir);

void ST7789V_DrawVLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usY2 , uint16_t _usColor);



 
#line 43 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"








 



 
#line 94 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"






 
typedef struct {
  unsigned long VRAMAddr;
  int xSize, ySize;
  int vxSize, vySize;
  int vxSizePhys;
  int BitsPerPixel;
} DRIVER_CONTEXT_TEMPLATE;






 








 
static void _SetPixelIndex(GUI_DEVICE * pDevice, int x, int y, int PixelIndex) {



    
    
    
#line 140 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
    (void)pDevice;
    (void)x;
    (void)y;
    (void)PixelIndex;
    {
      
      
      
      
      
        LCD_PutPixel(x, y, PixelIndex);
    }





}









 
static unsigned int _GetPixelIndex(GUI_DEVICE * pDevice, int x, int y) {
  unsigned int PixelIndex;



    
    
    
#line 185 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
    (void)pDevice;
    (void)x;
    (void)y;
    {
      
      
      
      
      
        PixelIndex = LCD_GetPixel(x, y);
    }





  return PixelIndex;
}




 
static void _XorPixel(GUI_DEVICE * pDevice, int x, int y) {
  unsigned long PixelIndex;
  unsigned long IndexMask;

  PixelIndex = _GetPixelIndex(pDevice, x, y);
  IndexMask  = pDevice->pColorConvAPI->pfGetIndexMask();
  _SetPixelIndex(pDevice, x, y, PixelIndex ^ IndexMask);
}




 
static void _DrawHLine  (GUI_DEVICE * pDevice, int x0, int y,  int x1) {
  unsigned long ColorIndex;
	
  if (GUI_pContext->DrawMode & (1<<0)) {
    for (; x0 <= x1; x0++) {
      _XorPixel(pDevice, x0, y);
    }
  } else {

		ColorIndex = (*GUI_pContext->LCD_pColorIndex);
		LCD_Draw_HLine(x0, y, x1, ColorIndex);
#line 239 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
    }
}




 
static void _DrawVLine  (GUI_DEVICE * pDevice, int x, int y0,  int y1) {
  unsigned long ColorIndex;
  if (GUI_pContext->DrawMode & (1<<0)) {
    for (; y0 <= y1; y0++) {
      _XorPixel(pDevice, x, y0);
    }
  } else {

		ColorIndex = (*GUI_pContext->LCD_pColorIndex);
		LCD_Draw_VLine(x, y0, y1, ColorIndex);
#line 262 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
  }
}




 
static void _FillRect(GUI_DEVICE * pDevice, int x0, int y0, int x1, int y1) {
    unsigned long ColorIndex;
	int x;
	
	if (GUI_pContext->DrawMode & (1<<0)) 
	{
		for (; y0 <= y1; y0++) 
		{
			for (x = x0; x <= x1; x++) 
			{
				_XorPixel(pDevice, x, y0);
			}
		}
	} 
	else
	{
	   

		ColorIndex = (*GUI_pContext->LCD_pColorIndex);
		LCD_Fill_Rect(x0, y0, y1-y0+1, x1-x0+1, ColorIndex);
#line 295 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
	}
}





 
static void _DrawBitLine1BPP(GUI_DEVICE * pDevice, int x, int y, unsigned char const * p, int Diff, int xsize, const unsigned long * pTrans) {
  unsigned long IndexMask, Index0, Index1, Pixel;

  Index0 = *(pTrans + 0);
  Index1 = *(pTrans + 1);
  x += Diff;
  switch (GUI_pContext->DrawMode & ((1<<1) | (1<<0))) {
  case 0:
    do {
      _SetPixelIndex(pDevice, x++, y, (*p & (0x80 >> Diff)) ? Index1 : Index0);
      if (++Diff == 8) {
        Diff = 0;
        p++;
      }
    } while (--xsize);
    break;
  case (1<<1):
    do {
      if (*p & (0x80 >> Diff))
        _SetPixelIndex(pDevice, x, y, Index1);
      x++;
      if (++Diff == 8) {
        Diff = 0;
        p++;
      }
    } while (--xsize);
    break;
  case (1<<0) | (1<<1):
  case (1<<0):
    IndexMask = pDevice->pColorConvAPI->pfGetIndexMask();
    do {
      if (*p & (0x80 >> Diff)) {
        Pixel = _GetPixelIndex(pDevice, x, y);
        _SetPixelIndex(pDevice, x, y, Pixel ^ IndexMask);
      }
      x++;
      if (++Diff == 8) {
        Diff = 0;
        p++;
      }
    } while (--xsize);
    break;
  }
}




 
static void  _DrawBitLine2BPP(GUI_DEVICE * pDevice, int x, int y, unsigned char const * p, int Diff, int xsize, const unsigned long * pTrans) {
  unsigned long Pixels, PixelIndex;
  int CurrentPixel, Shift, Index;

  Pixels = *p;
  CurrentPixel = Diff;
  x += Diff;
  switch (GUI_pContext->DrawMode & ((1<<1) | (1<<0))) {
  case 0:
    if (pTrans) {
      do {
        Shift = (3 - CurrentPixel) << 1;
        Index = (Pixels & (0xC0 >> (6 - Shift))) >> Shift;
        PixelIndex = *(pTrans + Index);
        _SetPixelIndex(pDevice, x++, y, PixelIndex);
        if (++CurrentPixel == 4) {
          CurrentPixel = 0;
          Pixels = *(++p);
        }
      } while (--xsize);
    } else {
      do {
        Shift = (3 - CurrentPixel) << 1;
        Index = (Pixels & (0xC0 >> (6 - Shift))) >> Shift;
        _SetPixelIndex(pDevice, x++, y, Index);
        if (++CurrentPixel == 4) {
          CurrentPixel = 0;
          Pixels = *(++p);
        }
      } while (--xsize);
    }
    break;
  case (1<<1):
    if (pTrans) {
      do {
        Shift = (3 - CurrentPixel) << 1;
        Index = (Pixels & (0xC0 >> (6 - Shift))) >> Shift;
        if (Index) {
          PixelIndex = *(pTrans + Index);
          _SetPixelIndex(pDevice, x, y, PixelIndex);
        }
        x++;
        if (++CurrentPixel == 4) {
          CurrentPixel = 0;
          Pixels = *(++p);
        }
      } while (--xsize);
    } else {
      do {
        Shift = (3 - CurrentPixel) << 1;
        Index = (Pixels & (0xC0 >> (6 - Shift))) >> Shift;
        if (Index) {
          _SetPixelIndex(pDevice, x, y, Index);
        }
        x++;
        if (++CurrentPixel == 4) {
          CurrentPixel = 0;
          Pixels = *(++p);
        }
      } while (--xsize);
    }
    break;
  }
}




 
static void  _DrawBitLine4BPP(GUI_DEVICE * pDevice, int x, int y, unsigned char const * p, int Diff, int xsize, const unsigned long * pTrans) {
  unsigned long Pixels, PixelIndex;
  int CurrentPixel, Shift, Index;

  Pixels = *p;
  CurrentPixel = Diff;
  x += Diff;
  switch (GUI_pContext->DrawMode & ((1<<1) | (1<<0))) {
  case 0:
    if (pTrans) {
      do {
        Shift = (1 - CurrentPixel) << 2;
        Index = (Pixels & (0xF0 >> (4 - Shift))) >> Shift;
        PixelIndex = *(pTrans + Index);
        _SetPixelIndex(pDevice, x++, y, PixelIndex);
        if (++CurrentPixel == 2) {
          CurrentPixel = 0;
          Pixels = *(++p);
        }
      } while (--xsize);
    } else {
      do {
        Shift = (1 - CurrentPixel) << 2;
        Index = (Pixels & (0xF0 >> (4 - Shift))) >> Shift;
        _SetPixelIndex(pDevice, x++, y, Index);
        if (++CurrentPixel == 2) {
          CurrentPixel = 0;
          Pixels = *(++p);
        }
      } while (--xsize);
    }
    break;
  case (1<<1):
    if (pTrans) {
      do {
        Shift = (1 - CurrentPixel) << 2;
        Index = (Pixels & (0xF0 >> (4 - Shift))) >> Shift;
        if (Index) {
          PixelIndex = *(pTrans + Index);
          _SetPixelIndex(pDevice, x, y, PixelIndex);
        }
        x++;
        if (++CurrentPixel == 2) {
          CurrentPixel = 0;
          Pixels = *(++p);
        }
      } while (--xsize);
    } else {
      do {
        Shift = (1 - CurrentPixel) << 2;
        Index = (Pixels & (0xF0 >> (4 - Shift))) >> Shift;
        if (Index) {
          _SetPixelIndex(pDevice, x, y, Index);
        }
        x++;
        if (++CurrentPixel == 2) {
          CurrentPixel = 0;
          Pixels = *(++p);
        }
      } while (--xsize);
    }
    break;
  }
}




 
static void  _DrawBitLine8BPP(GUI_DEVICE * pDevice, int x, int y, unsigned char const * p, int xsize, const unsigned long * pTrans) {
  unsigned long Pixel;

  switch (GUI_pContext->DrawMode & ((1<<1) | (1<<0))) {
  case 0:
    if (pTrans) {
      for (; xsize > 0; xsize--, x++, p++) {
        Pixel = *p;
        _SetPixelIndex(pDevice, x, y, *(pTrans + Pixel));
      }
    } else {
      for (; xsize > 0; xsize--, x++, p++) {
        _SetPixelIndex(pDevice, x, y, *p);
      }
    }
    break;
  case (1<<1):
    if (pTrans) {
      for (; xsize > 0; xsize--, x++, p++) {
        Pixel = *p;
        if (Pixel) {
          _SetPixelIndex(pDevice, x, y, *(pTrans + Pixel));
        }
      }
    } else {
      for (; xsize > 0; xsize--, x++, p++) {
        Pixel = *p;
        if (Pixel) {
          _SetPixelIndex(pDevice, x, y, Pixel);
        }
      }
    }
    break;
  }
}








 
static void _DrawBitLine16BPP(GUI_DEVICE * pDevice, int x, int y, unsigned short const * p, int xsize) {

	LCD_Draw_HColorLine(x, y, xsize, (uint16_t *)p);
#line 543 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
}








 
static void _DrawBitLine32BPP(GUI_DEVICE * pDevice, int x, int y, unsigned long const * p, int xsize) {

	LCD_Draw_HColorLine(x, y, xsize, (uint16_t *)p);
#line 562 "..\\STemWin\\DisplayDriver\\GUIDRV_Template.c"
}




 
static void _DrawBitmap(GUI_DEVICE * pDevice, int x0, int y0,
                       int xSize, int ySize,
                       int BitsPerPixel, 
                       int BytesPerLine,
                       const unsigned char * pData, int Diff,
                       const unsigned long * pTrans) {
  int i;

  switch (BitsPerPixel) {
  case 1:
    for (i = 0; i < ySize; i++) {
      _DrawBitLine1BPP(pDevice, x0, i + y0, pData, Diff, xSize, pTrans);
      pData += BytesPerLine;
    }
    break;
  case 2:
    for (i = 0; i < ySize; i++) {
      _DrawBitLine2BPP(pDevice, x0, i + y0, pData, Diff, xSize, pTrans);
      pData += BytesPerLine;
    }
    break;
  case 4:
    for (i = 0; i < ySize; i++) {
      _DrawBitLine4BPP(pDevice, x0, i + y0, pData, Diff, xSize, pTrans);
      pData += BytesPerLine;
    }
    break;
  case 8:
    for (i = 0; i < ySize; i++) {
      _DrawBitLine8BPP(pDevice, x0, i + y0, pData, xSize, pTrans);
      pData += BytesPerLine;
    }
    break;
  
  
  
  case 16:
    for (i = 0; i < ySize; i++) {
      _DrawBitLine16BPP(pDevice, x0, i + y0, (const unsigned short *)pData, xSize);
      pData += BytesPerLine;
    }
    break;
  
  
  
  case 32:
    for (i = 0; i < ySize; i++) {
      _DrawBitLine32BPP(pDevice, x0, i + y0, (const unsigned long *)pData, xSize);
      pData += BytesPerLine;
    }
    break;
  }
}










 
static int _InitOnce(GUI_DEVICE * pDevice) {
  DRIVER_CONTEXT_TEMPLATE * pContext;

  if (pDevice->u.pContext == 0) {
    pDevice->u.pContext = GUI_ALLOC_GetFixedBlock(sizeof(DRIVER_CONTEXT_TEMPLATE));
    pContext = (DRIVER_CONTEXT_TEMPLATE *)pDevice->u.pContext;
    pContext->BitsPerPixel = LCD__GetBPP(pDevice->pColorConvAPI->pfGetIndexMask());
  }
  return pDevice->u.pContext ? 0 : 1;
}




 
static signed long _GetDevProp(GUI_DEVICE * pDevice, int Index) {
  DRIVER_CONTEXT_TEMPLATE * pContext;

  pContext = (DRIVER_CONTEXT_TEMPLATE *)pDevice->u.pContext;
  switch (Index) {
  case 0x01:
    return pContext->xSize;
  case 0x02:
    return pContext->ySize;
  case 0x03:
    return pContext->vxSize;
  case 0x04:
    return pContext->vySize;
  case 0x08:
    return pContext->BitsPerPixel;
  case 0x09:
    return 0;
  case 0x0A:
    return 1;
  case 0x0B:
    return 1;
  case 0x0C:
    return 0;
  case 0x0D:
    return 0;
  case 0x0E:
    return 0;
  }
  return -1;
}




 
static void * _GetDevData(GUI_DEVICE * pDevice, int Index) {
  (void)pDevice;

    switch (Index) {
    case 0x01:
      return (void *)&GUI_MEMDEV_DEVICE_16; 
    }



  return 0;
}




 
static void _GetRect(GUI_DEVICE * pDevice, LCD_RECT * pRect) {
  DRIVER_CONTEXT_TEMPLATE * pContext;

  pContext = (DRIVER_CONTEXT_TEMPLATE *)pDevice->u.pContext;
  pRect->x0 = 0;
  pRect->y0 = 0;
  pRect->x1 = pContext->vxSize - 1;
  pRect->y1 = pContext->vySize - 1;
}




 
static void _SetOrg(GUI_DEVICE * pDevice, int x, int y) {
  LCD_X_SETORG_INFO Data = {0};

  Data.xPos = x;
  Data.yPos = y;
  LCD_X_DisplayDriver(pDevice->LayerIndex, 0x03, (void *)&Data);
}






 



 
static void _SetVRAMAddr(GUI_DEVICE * pDevice, void * pVRAM) {
  DRIVER_CONTEXT_TEMPLATE * pContext;
  LCD_X_SETVRAMADDR_INFO Data = {0};

  _InitOnce(pDevice);
  if (pDevice->u.pContext) {
    pContext = (DRIVER_CONTEXT_TEMPLATE *)pDevice->u.pContext;
    pContext->VRAMAddr = (unsigned long)pVRAM;
    Data.pVRAM = pVRAM;
    LCD_X_DisplayDriver(pDevice->LayerIndex, 0x02, (void *)&Data);
  }
}




 
static void _SetVSize(GUI_DEVICE * pDevice, int xSize, int ySize) {
  DRIVER_CONTEXT_TEMPLATE * pContext;

  _InitOnce(pDevice);
  if (pDevice->u.pContext) {
    pContext = (DRIVER_CONTEXT_TEMPLATE *)pDevice->u.pContext;
    pContext->vxSize = xSize;
    pContext->vySize = ySize;
    pContext->vxSizePhys = xSize;
  }
}




 
static void _SetSize(GUI_DEVICE * pDevice, int xSize, int ySize) {
  DRIVER_CONTEXT_TEMPLATE * pContext;
  LCD_X_SETSIZE_INFO Data = {0};

  _InitOnce(pDevice);
  if (pDevice->u.pContext) {
    pContext = (DRIVER_CONTEXT_TEMPLATE *)pDevice->u.pContext;
    pContext->vxSizePhys = (pContext->vxSizePhys == 0) ? xSize : pContext->vxSizePhys;
    pContext->xSize = xSize;
    pContext->ySize = ySize;
    Data.xSize = xSize;
    Data.ySize = ySize;
    LCD_X_DisplayDriver(pDevice->LayerIndex, 0x07, (void *)&Data);
  }
}



 
static int  _Init(GUI_DEVICE * pDevice) {
  int r;

  r = _InitOnce(pDevice);
  r |= LCD_X_DisplayDriver(pDevice->LayerIndex, 0x01, 0);
  return r;
}




 
static void _On (GUI_DEVICE * pDevice) {
  LCD_X_DisplayDriver(pDevice->LayerIndex, 0x05, 0);
}




 
static void _Off (GUI_DEVICE * pDevice) {
  LCD_X_DisplayDriver(pDevice->LayerIndex, 0x06, 0);
}




 
static void _SetLUTEntry(GUI_DEVICE * pDevice, unsigned char Pos, LCD_COLOR Color) {
  LCD_X_SETLUTENTRY_INFO Data = {0};

  Data.Pos   = Pos;
  Data.Color = Color;
  LCD_X_DisplayDriver(pDevice->LayerIndex, 0x04, (void *)&Data);
}




 
static void (* _GetDevFunc(GUI_DEVICE ** ppDevice, int Index))(void) {
  (void)ppDevice;
  switch (Index) {
  case 0x09:
    return (void (*)(void))_SetVRAMAddr;
  case 0x0A:
    return (void (*)(void))_SetVSize;
  case 0x0B:
    return (void (*)(void))_SetSize;
  case 0x0C:
    return (void (*)(void))_Init;
  case 0x0E:
    return (void (*)(void))_On;
  case 0x0F:
    return (void (*)(void))_Off;
  case 0x10:
    return (void (*)(void))_SetLUTEntry;
  }
  return 0;
}






 



 
const GUI_DEVICE_API GUIDRV_Template_API = {
  
  
  
  DEVICE_CLASS_DRIVER,
  
  
  
  _DrawBitmap,
  _DrawHLine,
  _DrawVLine,
  _FillRect,
  _GetPixelIndex,
  _SetPixelIndex,
  _XorPixel,
  
  
  
  _SetOrg,
  
  
  
  _GetDevFunc,
  _GetDevProp,
  _GetDevData,
  _GetRect,
};

 
