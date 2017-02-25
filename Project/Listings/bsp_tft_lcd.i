#line 1 "..\\User\\bsp\\bsp_tft_lcd.c"














































 

#line 1 "..\\User\\bsp\\bsp.h"











 







 




 


 




#line 1 "..\\FreeRTOS\\include\\FreeRTOS.h"



































































 






 
#line 1 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 77 "..\\FreeRTOS\\include\\FreeRTOS.h"













 
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






 
#line 92 "..\\FreeRTOS\\include\\FreeRTOS.h"





 
#line 1 "..\\User\\FreeRTOSConfig.h"



































































 














 
 
  
#line 88 "..\\User\\FreeRTOSConfig.h"
 extern volatile uint32_t ulHighFrequencyTimerTicks;


#line 104 "..\\User\\FreeRTOSConfig.h"


 






 




 

#line 127 "..\\User\\FreeRTOSConfig.h"

 
#line 135 "..\\User\\FreeRTOSConfig.h"


 





 



 



 



 




 








#line 99 "..\\FreeRTOS\\include\\FreeRTOS.h"

 
#line 1 "..\\FreeRTOS\\include\\projdefs.h"



































































 







 
typedef void (*TaskFunction_t)( void * );



 












 




 











 
#line 152 "..\\FreeRTOS\\include\\projdefs.h"


 







#line 102 "..\\FreeRTOS\\include\\FreeRTOS.h"

 
#line 1 "..\\FreeRTOS\\include\\portable.h"



































































 



 













 
#line 1 "..\\FreeRTOS\\include\\deprecated_definitions.h"



































































 












 











































































































































































#line 260 "..\\FreeRTOS\\include\\deprecated_definitions.h"

#line 268 "..\\FreeRTOS\\include\\deprecated_definitions.h"







#line 282 "..\\FreeRTOS\\include\\deprecated_definitions.h"








































#line 88 "..\\FreeRTOS\\include\\portable.h"




 
#line 1 "..\\FreeRTOS\\portable\\RVDS\\ARM_CM3\\portmacro.h"



































































 

















 

 
#line 96 "..\\FreeRTOS\\portable\\RVDS\\ARM_CM3\\portmacro.h"

typedef uint32_t StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;





	typedef uint32_t TickType_t;


	
 


 

 




 


 

 
#line 135 "..\\FreeRTOS\\portable\\RVDS\\ARM_CM3\\portmacro.h"
 





 

 
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );

#line 153 "..\\FreeRTOS\\portable\\RVDS\\ARM_CM3\\portmacro.h"

 

 

	extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );


 

 






	 




	 



	 




 



 


 


	void vPortValidateInterruptPriority( void );



 








 

static __forceinline void vPortSetBASEPRI( uint32_t ulBASEPRI )
{
	__asm
	{
		
 
		msr basepri, ulBASEPRI
	}
}
 

static __forceinline void vPortRaiseBASEPRI( void )
{
uint32_t ulNewBASEPRI = ( 0x01 << (8 - 4) );

	__asm
	{
		
 
		msr basepri, ulNewBASEPRI
		dsb
		isb
	}
}
 

static __forceinline void vPortClearBASEPRIFromISR( void )
{
	__asm
	{
		

 
		msr basepri, #0
	}
}
 

static __forceinline uint32_t ulPortRaiseBASEPRI( void )
{
uint32_t ulReturn, ulNewBASEPRI = ( 0x01 << (8 - 4) );

	__asm
	{
		
 
		mrs ulReturn, basepri
		msr basepri, ulNewBASEPRI
		dsb
		isb
	}

	return ulReturn;
}
 

static __forceinline BaseType_t xPortIsInsideInterrupt( void )
{
uint32_t ulCurrentInterrupt;
BaseType_t xReturn;

	 
	__asm
	{
		mrs ulCurrentInterrupt, ipsr
	}

	if( ulCurrentInterrupt == 0 )
	{
		xReturn = ( ( BaseType_t ) 0 );
	}
	else
	{
		xReturn = ( ( BaseType_t ) 1 );
	}

	return xReturn;
}








#line 95 "..\\FreeRTOS\\include\\portable.h"






































#line 1 "..\\FreeRTOS\\include\\mpu_wrappers.h"



































































 





 
#line 192 "..\\FreeRTOS\\include\\mpu_wrappers.h"










#line 134 "..\\FreeRTOS\\include\\portable.h"






 



	StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters ) ;


 
typedef struct HeapRegion
{
	uint8_t *pucStartAddress;
	size_t xSizeInBytes;
} HeapRegion_t;











 
void vPortDefineHeapRegions( const HeapRegion_t * const pxHeapRegions ) ;




 
void *pvPortMalloc( size_t xSize ) ;
void vPortFree( void *pv ) ;
void vPortInitialiseBlocks( void ) ;
size_t xPortGetFreeHeapSize( void ) ;
size_t xPortGetMinimumEverFreeHeapSize( void ) ;




 
BaseType_t xPortStartScheduler( void ) ;





 
void vPortEndScheduler( void ) ;







 











#line 105 "..\\FreeRTOS\\include\\FreeRTOS.h"

 




 







 



























































































































































#line 281 "..\\FreeRTOS\\include\\FreeRTOS.h"

 
#line 298 "..\\FreeRTOS\\include\\FreeRTOS.h"



































 

	
 




	
 




	
 




	
 




	 




	 




	
 




	



 




	


 




	


 




	


 







 












































































































































































































































































































































































	 




	 



 














#line 809 "..\\FreeRTOS\\include\\FreeRTOS.h"
	
 







 




#line 843 "..\\FreeRTOS\\include\\FreeRTOS.h"

	
 













 













 
struct xSTATIC_LIST_ITEM
{
	TickType_t xDummy1;
	void *pvDummy2[ 4 ];
};
typedef struct xSTATIC_LIST_ITEM StaticListItem_t;

 
struct xSTATIC_MINI_LIST_ITEM
{
	TickType_t xDummy1;
	void *pvDummy2[ 2 ];
};
typedef struct xSTATIC_MINI_LIST_ITEM StaticMiniListItem_t;

 
typedef struct xSTATIC_LIST
{
	UBaseType_t uxDummy1;
	void *pvDummy2;
	StaticMiniListItem_t xDummy3;
} StaticList_t;













 
typedef struct xSTATIC_TCB
{
	void				*pxDummy1;



	StaticListItem_t	xDummy3[ 2 ];
	UBaseType_t			uxDummy5;
	void				*pxDummy6;
	uint8_t				ucDummy7[ ( 16 ) ];
#line 927 "..\\FreeRTOS\\include\\FreeRTOS.h"
		UBaseType_t		uxDummy10[ 2 ];


		UBaseType_t		uxDummy12[ 2 ];
#line 939 "..\\FreeRTOS\\include\\FreeRTOS.h"
		uint32_t		ulDummy16;





		uint32_t 		ulDummy18;
		uint8_t 		ucDummy19;





} StaticTask_t;














 
typedef struct xSTATIC_QUEUE
{
	void *pvDummy1[ 3 ];

	union
	{
		void *pvDummy2;
		UBaseType_t uxDummy2;
	} u;

	StaticList_t xDummy3[ 2 ];
	UBaseType_t uxDummy4[ 3 ];
	uint8_t ucDummy5[ 2 ];










		UBaseType_t uxDummy8;
		uint8_t ucDummy9;


} StaticQueue_t;
typedef StaticQueue_t StaticSemaphore_t;














 
typedef struct xSTATIC_EVENT_GROUP
{
	TickType_t xDummy1;
	StaticList_t xDummy2;


		UBaseType_t uxDummy3;






} StaticEventGroup_t;














 
typedef struct xSTATIC_TIMER
{
	void				*pvDummy1;
	StaticListItem_t	xDummy2;
	TickType_t			xDummy3;
	UBaseType_t			uxDummy4;
	void 				*pvDummy5[ 2 ];

		UBaseType_t		uxDummy6;






} StaticTimer_t;







#line 35 "..\\User\\bsp\\bsp.h"
#line 1 "..\\FreeRTOS\\include\\task.h"



































































 









#line 1 "..\\FreeRTOS\\include\\list.h"



































































 



























 



































 












 

	 
#line 176 "..\\FreeRTOS\\include\\list.h"




 
struct xLIST_ITEM
{
				 
	 TickType_t xItemValue;			 
	struct xLIST_ITEM *  pxNext;		 
	struct xLIST_ITEM *  pxPrevious;	 
	void * pvOwner;										 
	void *  pvContainer;				 
				 
};
typedef struct xLIST_ITEM ListItem_t;					 

struct xMINI_LIST_ITEM
{
				 
	 TickType_t xItemValue;
	struct xLIST_ITEM *  pxNext;
	struct xLIST_ITEM *  pxPrevious;
};
typedef struct xMINI_LIST_ITEM MiniListItem_t;



 
typedef struct xLIST
{
					 
	 UBaseType_t uxNumberOfItems;
	ListItem_t *  pxIndex;			 
	MiniListItem_t xListEnd;							 
					 
} List_t;







 








 








 









 








 







 







 







 








 




 





















 
#line 330 "..\\FreeRTOS\\include\\list.h"

















 










 







 






 











 
void vListInitialise( List_t * const pxList ) ;









 
void vListInitialiseItem( ListItem_t * const pxItem ) ;











 
void vListInsert( List_t * const pxList, ListItem_t * const pxNewListItem ) ;



















 
void vListInsertEnd( List_t * const pxList, ListItem_t * const pxNewListItem ) ;













 
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove ) ;







#line 79 "..\\FreeRTOS\\include\\task.h"







 















 
typedef void * TaskHandle_t;




 
typedef BaseType_t (*TaskHookFunction_t)( void * );

 
typedef enum
{
	eRunning = 0,	 
	eReady,			 
	eBlocked,		 
	eSuspended,		 
	eDeleted,		 
	eInvalid			 
} eTaskState;

 
typedef enum
{
	eNoAction = 0,				 
	eSetBits,					 
	eIncrement,					 
	eSetValueWithOverwrite,		 
	eSetValueWithoutOverwrite	 
} eNotifyAction;



 
typedef struct xTIME_OUT
{
	BaseType_t xOverflowCount;
	TickType_t xTimeOnEntering;
} TimeOut_t;



 
typedef struct xMEMORY_REGION
{
	void *pvBaseAddress;
	uint32_t ulLengthInBytes;
	uint32_t ulParameters;
} MemoryRegion_t;



 
typedef struct xTASK_PARAMETERS
{
	TaskFunction_t pvTaskCode;
	const char * const pcName;	 
	uint16_t usStackDepth;
	void *pvParameters;
	UBaseType_t uxPriority;
	StackType_t *puxStackBuffer;
	MemoryRegion_t xRegions[ 1 ];
} TaskParameters_t;


 
typedef struct xTASK_STATUS
{
	TaskHandle_t xHandle;			 
	const char *pcTaskName;			   
	UBaseType_t xTaskNumber;		 
	eTaskState eCurrentState;		 
	UBaseType_t uxCurrentPriority;	 
	UBaseType_t uxBasePriority;		 
	uint32_t ulRunTimeCounter;		 
	StackType_t *pxStackBase;		 
	uint16_t usStackHighWaterMark;	 
} TaskStatus_t;

 
typedef enum
{
	eAbortSleep = 0,		 
	eStandardSleep,			 
	eNoTasksWaitingTimeout	 
} eSleepModeStatus;





 









 













 














 









 









 




 







 





























































































 

	BaseType_t xTaskCreate(	TaskFunction_t pxTaskCode,
							const char * const pcName,
							const uint16_t usStackDepth,
							void * const pvParameters,
							UBaseType_t uxPriority,
							TaskHandle_t * const pxCreatedTask ) ;  












































































































 
#line 484 "..\\FreeRTOS\\include\\task.h"



































































 

















































 
void vTaskAllocateMPURegions( TaskHandle_t xTask, const MemoryRegion_t * const pxRegions ) ;







































 
void vTaskDelete( TaskHandle_t xTaskToDelete ) ;



 














































 
void vTaskDelay( const TickType_t xTicksToDelay ) ;

























































 
void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement ) ;























 
BaseType_t xTaskAbortDelay( TaskHandle_t xTask ) ;













































 
UBaseType_t uxTaskPriorityGet( TaskHandle_t xTask ) ;






 
UBaseType_t uxTaskPriorityGetFromISR( TaskHandle_t xTask ) ;
















 
eTaskState eTaskGetState( TaskHandle_t xTask ) ;






















































 
void vTaskGetInfo( TaskHandle_t xTask, TaskStatus_t *pxTaskStatus, BaseType_t xGetFreeStackSpace, eTaskState eState ) ;








































 
void vTaskPrioritySet( TaskHandle_t xTask, UBaseType_t uxNewPriority ) ;

















































 
void vTaskSuspend( TaskHandle_t xTaskToSuspend ) ;















































 
void vTaskResume( TaskHandle_t xTaskToResume ) ;



























 
BaseType_t xTaskResumeFromISR( TaskHandle_t xTaskToResume ) ;



 



























 
void vTaskStartScheduler( void ) ;






















































 
void vTaskEndScheduler( void ) ;

















































 
void vTaskSuspendAll( void ) ;




















































 
BaseType_t xTaskResumeAll( void ) ;



 









 
TickType_t xTaskGetTickCount( void ) ;














 
TickType_t xTaskGetTickCountFromISR( void ) ;












 
UBaseType_t uxTaskGetNumberOfTasks( void ) ;











 
char *pcTaskGetName( TaskHandle_t xTaskToQuery ) ;  














 
TaskHandle_t xTaskGetHandle( const char *pcNameToQuery ) ;  



















 
UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask ) ;






 
#line 1397 "..\\FreeRTOS\\include\\task.h"

#line 1409 "..\\FreeRTOS\\include\\task.h"











 
BaseType_t xTaskCallApplicationTaskHook( TaskHandle_t xTask, void *pvParameter ) ;







 
TaskHandle_t xTaskGetIdleTaskHandle( void ) ;

































































































 
UBaseType_t uxTaskGetSystemState( TaskStatus_t * const pxTaskStatusArray, const UBaseType_t uxArraySize, uint32_t * const pulTotalRunTime ) ;













































 
void vTaskList( char * pcWriteBuffer ) ;  




















































 
void vTaskGetRunTimeStats( char *pcWriteBuffer ) ;  















































































 
BaseType_t xTaskGenericNotify( TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t *pulPreviousNotificationValue ) ;

























































































 
BaseType_t xTaskGenericNotifyFromISR( TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t *pulPreviousNotificationValue, BaseType_t *pxHigherPriorityTaskWoken ) ;











































































 
BaseType_t xTaskNotifyWait( uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait ) ;












































 






















































 
void vTaskNotifyGiveFromISR( TaskHandle_t xTaskToNotify, BaseType_t *pxHigherPriorityTaskWoken ) ;



































































 
uint32_t ulTaskNotifyTake( BaseType_t xClearCountOnExit, TickType_t xTicksToWait ) ;














 
BaseType_t xTaskNotifyStateClear( TaskHandle_t xTask );



 















 
BaseType_t xTaskIncrementTick( void ) ;































 
void vTaskPlaceOnEventList( List_t * const pxEventList, const TickType_t xTicksToWait ) ;
void vTaskPlaceOnUnorderedEventList( List_t * pxEventList, const TickType_t xItemValue, const TickType_t xTicksToWait ) ;











 
void vTaskPlaceOnEventListRestricted( List_t * const pxEventList, TickType_t xTicksToWait, const BaseType_t xWaitIndefinitely ) ;
























 
BaseType_t xTaskRemoveFromEventList( const List_t * const pxEventList ) ;
BaseType_t xTaskRemoveFromUnorderedEventList( ListItem_t * pxEventListItem, const TickType_t xItemValue ) ;








 
void vTaskSwitchContext( void ) ;




 
TickType_t uxTaskResetEventItemValue( void ) ;



 
TaskHandle_t xTaskGetCurrentTaskHandle( void ) ;



 
void vTaskSetTimeOutState( TimeOut_t * const pxTimeOut ) ;




 
BaseType_t xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut, TickType_t * const pxTicksToWait ) ;




 
void vTaskMissedYield( void ) ;




 
BaseType_t xTaskGetSchedulerState( void ) ;




 
void vTaskPriorityInherit( TaskHandle_t const pxMutexHolder ) ;




 
BaseType_t xTaskPriorityDisinherit( TaskHandle_t const pxMutexHolder ) ;



 
UBaseType_t uxTaskGetTaskNumber( TaskHandle_t xTask ) ;




 
void vTaskSetTaskNumber( TaskHandle_t xTask, const UBaseType_t uxHandle ) ;








 
void vTaskStepTick( const TickType_t xTicksToJump ) ;














 
eSleepModeStatus eTaskConfirmSleepModeStatus( void ) ;




 
void *pvTaskIncrementMutexHeldCount( void ) ;








#line 36 "..\\User\\bsp\\bsp.h"
#line 43 "..\\User\\bsp\\bsp.h"

 



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




 

















 









 

  

 

 
#line 49 "..\\User\\bsp\\bsp.h"
#line 50 "..\\User\\bsp\\bsp.h"
#line 1 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 51 "..\\User\\bsp\\bsp.h"
#line 1 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 52 "..\\User\\bsp\\bsp.h"
#line 1 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"





 
#line 53 "..\\User\\bsp\\bsp.h"
#line 1 "..\\User\\emWinTask\\MainTask.h"













 




#line 1 "..\\STemWin\\inc\\GUI.h"
































 


















 
  



#line 1 "..\\STemWin\\inc\\GUI_ConfDefaults.h"




































 


















 
  



#line 1 "..\\STemWin\\Config\\GUIConf.h"
































 


















 
 






 





 









 







 





 





#line 62 "..\\STemWin\\inc\\GUI_ConfDefaults.h"

#line 70 "..\\STemWin\\inc\\GUI_ConfDefaults.h"



#line 79 "..\\STemWin\\inc\\GUI_ConfDefaults.h"






 







 







 







 
















































 
#line 166 "..\\STemWin\\inc\\GUI_ConfDefaults.h"

 





 











 




 






 
#line 58 "..\\STemWin\\inc\\GUI.h"
#line 1 "..\\STemWin\\inc\\GUI_Type.h"




































 


















 
  



#line 1 "..\\STemWin\\inc\\LCD.h"
































 


















 
  



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



 
#line 20 "..\\User\\emWinTask\\MainTask.h"
#line 1 "..\\STemWin\\inc\\DIALOG.h"
































 


















 
  



#line 1 "..\\STemWin\\inc\\WM.h"
































 


















 
  




#line 59 "..\\STemWin\\inc\\WM.h"
#line 60 "..\\STemWin\\inc\\WM.h"
#line 1 "..\\STemWin\\inc\\WM_GUI.h"
































 


















 
  







int       WM__InitIVRSearch(const GUI_RECT* pMaxRect);
int       WM__GetNextIVR   (void);
int       WM__GetOrgX_AA(void);
int       WM__GetOrgY_AA(void);










#line 84 "..\\STemWin\\inc\\WM_GUI.h"








 
#line 61 "..\\STemWin\\inc\\WM.h"
#line 62 "..\\STemWin\\inc\\WM.h"





 




 





 
#line 86 "..\\STemWin\\inc\\WM.h"




 






 








 




 




#line 123 "..\\STemWin\\inc\\WM.h"




 








 
typedef struct WM_WINDOW_INFO WM_WINDOW_INFO;

struct WM_WINDOW_INFO {
  signed long hWin;
  signed long hParent;
  signed long hFirstChild;
  signed long hNext;
  GUI_RECT Rect;
  unsigned long      Status;
  unsigned long      DebugId;
  WM_WINDOW_INFO * pNext;
};

typedef struct {
  int Key, PressedCnt;
} WM_KEY_INFO;

typedef struct {
  int NumItems, v, PageSize;
} WM_SCROLL_STATE;

typedef struct {
  int Done;
  int ReturnValue;
} WM_DIALOG_STATUS;

typedef struct {
  int x,y;
  unsigned char  State;
  unsigned char  StatePrev;
} WM_PID_STATE_CHANGED_INFO;

typedef struct {
  int Cmd;
  int dx, dy, da;
  int xPos, yPos;
  int Period;
  int SnapX;
  int SnapY;
  int FinalMove;
  unsigned long Flags;
  GUI_PID_STATE * pState;
  signed long hContext;
} WM_MOTION_INFO;

typedef struct {
  signed long       FactorMin;   
  signed long       FactorMax;   
  unsigned long       xSize;       
  unsigned long       ySize;       
  unsigned long       xSizeParent; 
  unsigned long       ySizeParent; 
  signed long       Factor0;     
  int       xPos0;       
  int       yPos0;       
  GUI_POINT Center0;     
} WM_ZOOM_INFO;

typedef struct {
  int            Flags;     
  GUI_POINT      Point;     
  GUI_POINT      Center;    
  signed long            Angle;     
  signed long            Factor;    
  WM_ZOOM_INFO * pZoomInfo; 
} WM_GESTURE_INFO;

typedef struct {
  int dx, dy;
} WM_MOVE_INFO;




 











 







































#line 271 "..\\STemWin\\inc\\WM.h"

#line 279 "..\\STemWin\\inc\\WM.h"

















 








 








 
#line 325 "..\\STemWin\\inc\\WM.h"







 









 










 
#line 361 "..\\STemWin\\inc\\WM.h"

 




 




























 
typedef struct WM_Obj     WM_Obj;
typedef struct WM_MESSAGE WM_MESSAGE;

typedef void WM_CALLBACK( WM_MESSAGE * pMsg);

struct WM_MESSAGE {
  int MsgId;             
  GUI_HWIN hWin;          
  GUI_HWIN hWinSrc;       
  union {
    const void * p;             
    int v;
    GUI_COLOR Color;
  } Data;
};

struct WM_Obj {
  GUI_RECT Rect;         
  GUI_RECT InvalidRect;  
  WM_CALLBACK* cb;       
  GUI_HWIN hNextLin;      
  GUI_HWIN hParent;
  GUI_HWIN hFirstChild;
  GUI_HWIN hNext;

    GUI_MEMDEV_Handle hMem;  

  unsigned long Status;            



};

typedef void WM_tfPollPID(void);
typedef void WM_tfForEach(GUI_HWIN hWin, void * pData);

typedef void (* WM_tfInvalidateParent)  (const GUI_RECT * pInvalidRect, GUI_HWIN hParent, GUI_HWIN hStop);
typedef void (* WM_tfInvalidateDrawFunc)(GUI_HWIN hWin);
typedef void (* WM_tfPaint1Func)        (GUI_HWIN hWin);

typedef struct {
  signed long  hTimer;
  GUI_HWIN  hWin;
  int      UserId;
} WM_TIMER_OBJ;




 
void WM_Activate  (void);
void WM_Deactivate(void);
void WM_Init      (void);
int  WM_Exec      (void);     
unsigned long  WM_SetCreateFlags(unsigned long Flags);
WM_tfPollPID * WM_SetpfPollPID(WM_tfPollPID * pf);




 
void    WM_AttachWindow              (GUI_HWIN hWin, GUI_HWIN hParent);
void    WM_AttachWindowAt            (GUI_HWIN hWin, GUI_HWIN hParent, int x, int y);
int     WM_CheckScrollPos            (WM_SCROLL_STATE * pScrollState, int Pos, int LowerDist, int UpperDist);  
void    WM_ClrHasTrans               (GUI_HWIN hWin);
GUI_HWIN WM_CreateWindow              (int x0, int y0, int xSize, int ySize, unsigned long Style, WM_CALLBACK * cb, int NumExtraBytes);
GUI_HWIN WM_CreateWindowAsChild       (int x0, int y0, int xSize, int ySize, GUI_HWIN hWinParent, unsigned long Style, WM_CALLBACK* cb, int NumExtraBytes);
void    WM_DeleteWindow              (GUI_HWIN hWin);
void    WM_DetachWindow              (GUI_HWIN hWin);
void    WM_EnableGestures            (GUI_HWIN hWin, int OnOff);
int     WM_GetHasTrans               (GUI_HWIN hWin);
GUI_HWIN WM_GetFocussedWindow         (void);
int     WM_GetInvalidRect            (GUI_HWIN hWin, GUI_RECT * pRect);
int     WM_GetStayOnTop              (GUI_HWIN hWin);
void    WM_HideWindow                (GUI_HWIN hWin);
void    WM_InvalidateArea            (const GUI_RECT * pRect);
void    WM_InvalidateRect            (GUI_HWIN hWin, const GUI_RECT * pRect);
void    WM_InvalidateWindow          (GUI_HWIN hWin);
void    WM_InvalidateWindowAndDescsEx(GUI_HWIN hWin, const GUI_RECT * pInvalidRect, unsigned short Flags);
void    WM_InvalidateWindowAndDescs  (GUI_HWIN hWin);     
int     WM_IsEnabled                 (GUI_HWIN hObj);
char    WM_IsCompletelyCovered       (GUI_HWIN hWin);     
char    WM_IsCompletelyVisible       (GUI_HWIN hWin);     
int     WM_IsFocussable              (GUI_HWIN hWin);
int     WM_IsVisible                 (GUI_HWIN hWin);
int     WM_IsWindow                  (GUI_HWIN hWin);     
void    WM_SetAnchor                 (GUI_HWIN hWin, unsigned short AnchorFlags);
void    WM_SetHasTrans               (GUI_HWIN hWin);
void    WM_SetId                     (GUI_HWIN hObj, int Id);
void    WM_SetStayOnTop              (GUI_HWIN hWin, int OnOff);
void    WM_SetTransState             (GUI_HWIN hWin, unsigned State);
void    WM_ShowWindow                (GUI_HWIN hWin);
void    WM_ValidateRect              (GUI_HWIN hWin, const GUI_RECT * pRect);
void    WM_ValidateWindow            (GUI_HWIN hWin);

 
void WM_GESTURE_Enable  (int OnOff);
int  WM_GESTURE_EnableEx(int OnOff, int MaxFactor);
void WM_GESTURE_Exec    (void);
signed long  WM_GESTURE_SetThresholdAngle(signed long ThresholdAngle);
signed long  WM_GESTURE_SetThresholdDist (signed long ThresholdDist);

 
void     WM_MOTION_Enable          (int OnOff);
void     WM_MOTION_SetMovement     (GUI_HWIN hWin, int Axis, signed long Velocity, signed long Dist);
void     WM_MOTION_SetMotion       (GUI_HWIN hWin, int Axis, signed long Velocity, signed long Deceleration);
void     WM_MOTION_SetMoveable     (GUI_HWIN hWin, unsigned long Flags, int OnOff);
void     WM_MOTION_SetDeceleration (GUI_HWIN hWin, int Axis, signed long Deceleration);
unsigned WM_MOTION_SetDefaultPeriod(unsigned Period);
void     WM_MOTION_SetSpeed        (GUI_HWIN hWin, int Axis, signed long Velocity);

 
signed long WM_MOTION__CreateContext(void);
void    WM_MOTION__DeleteContext(signed long hContext);

 
void     WM__SetMotionCallback (void(* cbMotion) (GUI_PID_STATE * pState, void * p));

 






  int               GUI_MEMDEV_BlendWinBk       (GUI_HWIN hWin, int Period, unsigned long BlendColor, unsigned char BlendIntens);
  int               GUI_MEMDEV_BlurAndBlendWinBk(GUI_HWIN hWin, int Period, unsigned char BlurDepth, unsigned long BlendColor, unsigned char BlendIntens);
  int               GUI_MEMDEV_BlurWinBk        (GUI_HWIN hWin, int Period, unsigned char BlurDepth);
  void              GUI_MEMDEV_CreateStatic     (GUI_HWIN hWin);
  GUI_MEMDEV_Handle GUI_MEMDEV_CreateWindowDevice(GUI_HWIN hWin);
  int               GUI_MEMDEV_FadeInWindow     (GUI_HWIN hWin, int Period);
  int               GUI_MEMDEV_FadeOutWindow    (GUI_HWIN hWin, int Period);
  GUI_MEMDEV_Handle GUI_MEMDEV_GetStaticDevice  (GUI_HWIN hWin);
  GUI_MEMDEV_Handle GUI_MEMDEV_GetWindowDevice  (GUI_HWIN hWin);
  int               GUI_MEMDEV_MoveInWindow     (GUI_HWIN hWin, int x, int y, int a180, int Period);
  int               GUI_MEMDEV_MoveOutWindow    (GUI_HWIN hWin, int x, int y, int a180, int Period);
  void              GUI_MEMDEV_Paint1Static     (GUI_HWIN hWin);                                      
  int               GUI_MEMDEV_ShiftInWindow    (GUI_HWIN hWin, int Period, int Direction);
  int               GUI_MEMDEV_ShiftOutWindow   (GUI_HWIN hWin, int Period, int Direction);
  int               GUI_MEMDEV_SwapWindow       (GUI_HWIN hWin, int Period, int Edge);

  void              GUI_MEMDEV__CreateStatic    (GUI_HWIN hWin);


 
void WM_MoveWindow                (GUI_HWIN hWin, int dx, int dy);
void WM_ResizeWindow              (GUI_HWIN hWin, int dx, int dy);
void WM_MoveTo                    (GUI_HWIN hWin, int x, int y);
void WM_MoveChildTo               (GUI_HWIN hWin, int x, int y);
void WM_SetSize                   (GUI_HWIN hWin, int XSize, int YSize);
void WM_SetWindowPos              (GUI_HWIN hWin, int xPos, int yPos, int xSize, int ySize);
int  WM_SetXSize                  (GUI_HWIN hWin, int xSize);
int  WM_SetYSize                  (GUI_HWIN hWin, int ySize);
int  WM_SetScrollbarH             (GUI_HWIN hWin, int OnOff);  
int  WM_SetScrollbarV             (GUI_HWIN hWin, int OnOff);  

 








typedef signed long WM_TOOLTIP_HANDLE;

typedef struct {
  int          Id;
  const char * pText;
} TOOLTIP_INFO;

int               WM_TOOLTIP_AddTool         (WM_TOOLTIP_HANDLE hToolTip, GUI_HWIN hTool, const char * pText);
WM_TOOLTIP_HANDLE WM_TOOLTIP_Create          (GUI_HWIN hDlg, const TOOLTIP_INFO * pInfo, unsigned NumItems);
void              WM_TOOLTIP_Delete          (WM_TOOLTIP_HANDLE hToolTip);
GUI_COLOR         WM_TOOLTIP_SetDefaultColor (unsigned Index, GUI_COLOR Color);
const GUI_FONT *  WM_TOOLTIP_SetDefaultFont  (const GUI_FONT * pFont);
unsigned          WM_TOOLTIP_SetDefaultPeriod(unsigned Index, unsigned Period);

 
void WM__SetToolTipCallback(void(* cbToolTip)(GUI_PID_STATE * pState, GUI_HWIN));

 




  signed long WM_CreateTimer (GUI_HWIN hWin, int UserID, int Period, int Mode);  
  void    WM_DeleteTimer (signed long hTimer);  
  void    WM_RestartTimer(signed long hTimer, int Period);

int WM_GetTimerId(signed long hTimer);

 
int WM_GetNumWindows(void);
int WM_GetNumInvalidWindows(void);

 
void WM_CheckScrollBounds(WM_SCROLL_STATE * pScrollState);  
int  WM_GetScrollPosH    (GUI_HWIN hWin);
int  WM_GetScrollPosV    (GUI_HWIN hWin);
void WM_SetScrollPosH    (GUI_HWIN hWin, unsigned ScrollPos);
void WM_SetScrollPosV    (GUI_HWIN hWin, unsigned ScrollPos);
int  WM_SetScrollValue   (WM_SCROLL_STATE * pScrollState, int v);  

 
WM_CALLBACK * WM_SetCallback(GUI_HWIN hWin, WM_CALLBACK * cb);
WM_CALLBACK * WM_GetCallback(GUI_HWIN hWin);

 
void      WM_GetClientRect           (GUI_RECT * pRect);
void      WM_GetClientRectEx         (GUI_HWIN hWin, GUI_RECT * pRect);
void      WM_GetInsideRect           (GUI_RECT * pRect);
void      WM_GetInsideRectEx         (GUI_HWIN hWin, GUI_RECT * pRect);
void      WM_GetInsideRectExScrollbar(GUI_HWIN hWin, GUI_RECT * pRect);  
void      WM_GetWindowRect           (GUI_RECT * pRect);
void      WM_GetWindowRectEx         (GUI_HWIN hWin, GUI_RECT * pRect);
int       WM_GetOrgX                 (void);
int       WM_GetOrgY                 (void);
int       WM_GetWindowOrgX           (GUI_HWIN hWin);
int       WM_GetWindowOrgY           (GUI_HWIN hWin);
int       WM_GetWindowSizeX          (GUI_HWIN hWin);
int       WM_GetWindowSizeY          (GUI_HWIN hWin);
GUI_HWIN   WM_GetFirstChild           (GUI_HWIN hWin);
GUI_HWIN   WM_GetNextSibling          (GUI_HWIN hWin);
GUI_HWIN   WM_GetParent               (GUI_HWIN hWin);
GUI_HWIN   WM_GetPrevSibling          (GUI_HWIN hWin);
int       WM_GetId                   (GUI_HWIN hWin);
GUI_HWIN   WM_GetScrollbarV           (GUI_HWIN hWin);
GUI_HWIN   WM_GetScrollbarH           (GUI_HWIN hWin);
GUI_HWIN   WM_GetScrollPartner        (GUI_HWIN hWin);
GUI_HWIN   WM_GetClientWindow         (GUI_HWIN hObj);
GUI_COLOR WM_GetBkColor              (GUI_HWIN hObj);

 
void WM_BringToBottom(GUI_HWIN hWin);
void WM_BringToTop(GUI_HWIN hWin);

GUI_COLOR WM_SetDesktopColor  (GUI_COLOR Color);
GUI_COLOR WM_SetDesktopColorEx(GUI_COLOR Color, unsigned int LayerIndex);
void      WM_SetDesktopColors (GUI_COLOR Color);

 
GUI_HWIN WM_SelectWindow           (GUI_HWIN  hWin);
GUI_HWIN WM_GetActiveWindow        (void);
void    WM_Paint                  (GUI_HWIN hObj);
void    WM_Update                 (GUI_HWIN hWin);
void    WM_PaintWindowAndDescs    (GUI_HWIN hWin);
void    WM_UpdateWindowAndDescs   (GUI_HWIN hWin);

 
GUI_HWIN WM_GetDesktopWindow  (void);
GUI_HWIN WM_GetDesktopWindowEx(unsigned int LayerIndex);

 
const GUI_RECT * WM_SetUserClipRect(const GUI_RECT * pRect);
void             WM_SetDefault     (void);

 
void WM_EnableMemdev              (GUI_HWIN hWin);
void WM_DisableMemdev             (GUI_HWIN hWin);

 
int WM_MULTIBUF_Enable(int OnOff);

extern const GUI_MULTIBUF_API * WM_MULTIBUF__pAPI;

typedef void (* T_WM_EXEC_GESTURE)(void);

extern T_WM_EXEC_GESTURE WM__pExecGestures;

 
int WM_OnKey(int Key, int Pressed);
void WM_MakeModal(GUI_HWIN hWin);
int WM_SetModalLayer(int LayerIndex);
int WM_GetModalLayer(void);







 
void      WM_NotifyParent         (GUI_HWIN hWin, int Notification);
void      WM_SendMessage          (GUI_HWIN hWin, WM_MESSAGE * p);
void      WM_SendMessageNoPara    (GUI_HWIN hWin, int MsgId);              
void      WM_DefaultProc          (WM_MESSAGE * pMsg);
int       WM_BroadcastMessage     (WM_MESSAGE * pMsg);
void      WM_SetScrollState       (GUI_HWIN hWin, const WM_SCROLL_STATE * pState);
void      WM_SetEnableState       (GUI_HWIN hItem, int State);
void      WM_SendToParent         (GUI_HWIN hWin, WM_MESSAGE * pMsg);
int       WM_HasFocus             (GUI_HWIN hWin);
int       WM_SetFocus             (GUI_HWIN hWin);
GUI_HWIN   WM_SetFocusOnNextChild  (GUI_HWIN hParent);      
GUI_HWIN   WM_SetFocusOnPrevChild  (GUI_HWIN hParent);      
GUI_HWIN   WM_GetDialogItem        (GUI_HWIN hWin, int Id);
void      WM_EnableWindow         (GUI_HWIN hWin);
void      WM_DisableWindow        (GUI_HWIN hWin);
void      WM_GetScrollState       (GUI_HWIN hObj, WM_SCROLL_STATE * pScrollState);




 
int       WM_GetUserData   (GUI_HWIN hWin, void * pDest, int SizeOfBuffer);
int       WM_SetUserData   (GUI_HWIN hWin, const void * pSrc, int SizeOfBuffer);
int       WM__GetUserDataEx(GUI_HWIN hWin, void * pDest, int NumBytes, int SizeOfObject);
int       WM__SetUserDataEx(GUI_HWIN hWin, const void * pSrc, int NumBytes, int SizeOfObject);




 
int  WM_HasCaptured   (GUI_HWIN hWin);
void WM_SetCapture    (GUI_HWIN hObj, int AutoRelease);
void WM_SetCaptureMove(GUI_HWIN hWin, const GUI_PID_STATE * pState, int MinVisibility, int LimitTop);  
void WM_ReleaseCapture(void);




 
int       WM_HandlePID      (void);
GUI_HWIN   WM_Screen2hWin    (int x, int y);
GUI_HWIN   WM_Screen2hWinEx  (GUI_HWIN hStop, int x, int y);
void      WM_ForEachDesc    (GUI_HWIN hWin, WM_tfForEach * pcb, void * pData);
void      WM_SetScreenSize  (int xSize, int ySize);
int       WM_PollSimMsg     (void);
int       WM_GetWindowInfo  (WM_WINDOW_INFO * pInfo, int FirstWindow);




 







 












#line 765 "..\\STemWin\\inc\\WM.h"

















 
#line 58 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\BUTTON.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\BUTTON.h"
#line 1 "..\\STemWin\\inc\\DIALOG_Intern.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\DIALOG_Intern.h"












 
typedef struct  GUI_WIDGET_CREATE_INFO_struct GUI_WIDGET_CREATE_INFO;
typedef GUI_HWIN GUI_WIDGET_CREATE_FUNC        (const GUI_WIDGET_CREATE_INFO * pCreate, GUI_HWIN hWin, int x0, int y0, WM_CALLBACK * cb);






 
struct GUI_WIDGET_CREATE_INFO_struct {
  GUI_WIDGET_CREATE_FUNC * pfCreateIndirect;
  const char             * pName;            
  signed short                      Id;               
  signed short                      x0;               
  signed short                      y0;               
  signed short                      xSize;            
  signed short                      ySize;            
  unsigned short                      Flags;            
  signed long                      Para;             
  unsigned long                      NumExtraBytes;    
};






 
GUI_HWIN            GUI_CreateDialogBox   (const GUI_WIDGET_CREATE_INFO * paWidget, int NumWidgets, WM_CALLBACK * cb, GUI_HWIN hParent, int x0, int y0);
void               GUI_EndDialog         (GUI_HWIN hWin, int r);
int                GUI_ExecDialogBox     (const GUI_WIDGET_CREATE_INFO * paWidget, int NumWidgets, WM_CALLBACK * cb, GUI_HWIN hParent, int x0, int y0);
int                GUI_ExecCreatedDialog (GUI_HWIN hDialog);
WM_DIALOG_STATUS * GUI_GetDialogStatusPtr(GUI_HWIN hDialog);                                    
void               GUI_SetDialogStatusPtr(GUI_HWIN hDialog, WM_DIALOG_STATUS * pDialogStatus);  




 
LCD_COLOR          DIALOG_GetBkColor(void);
LCD_COLOR          DIALOG_SetBkColor(LCD_COLOR BkColor);








 
#line 59 "..\\STemWin\\inc\\BUTTON.h"
#line 1 "..\\STemWin\\inc\\WIDGET.h"
































 


















 
  







#line 1 "..\\STemWin\\inc\\WM_Intern.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\WM_Intern.h"
#line 1 "..\\STemWin\\inc\\GUI_Private.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\GUI_Private.h"
#line 1 "..\\STemWin\\inc\\LCD_Protected.h"
































 


















 
  



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









 
#line 59 "..\\STemWin\\inc\\WM_Intern.h"

















 


 






#line 92 "..\\STemWin\\inc\\WM_Intern.h"








#line 107 "..\\STemWin\\inc\\WM_Intern.h"






 
typedef struct {
  GUI_HWIN hOld;
  GUI_HWIN hNew;
} WM_NOTIFY_CHILD_HAS_FOCUS_INFO;

typedef struct WM_CRITICAL_HANDLE {
  struct  WM_CRITICAL_HANDLE * pNext;
  volatile GUI_HWIN hWin;
} WM_CRITICAL_HANDLE;






 
extern unsigned long            WM__CreateFlags;
extern GUI_HWIN        WM__ahCapture[2];
extern GUI_HWIN        WM__ahWinFocus[2];
extern char           WM__CaptureReleaseAuto;
extern WM_tfPollPID * WM_pfPollPID;
extern unsigned char             WM__PaintCallbackCnt;       
extern GUI_HWIN        WM__hCreateStatic;


  extern int     WM__TransWindowCnt;
  extern GUI_HWIN WM__hATransWindow;






extern WM_CRITICAL_HANDLE     WM__aCHWinModal[2];
extern WM_CRITICAL_HANDLE     WM__aCHWinLast[2];
extern int                    WM__ModalLayer;


  extern WM_CRITICAL_HANDLE   WM__aCHWinMouseOver[2];









  extern unsigned                  WM__TouchedLayer;





extern unsigned short     WM__NumWindows;
extern unsigned short     WM__NumInvalidWindows;
extern GUI_HWIN WM__FirstWin;
extern WM_CRITICAL_HANDLE * WM__pFirstCriticalHandle;

extern GUI_HWIN   WM__ahDesktopWin[2];
extern GUI_COLOR WM__aBkColor[2];








 
void    WM__ActivateClipRect        (void);
int     WM__ClipAtParentBorders     (GUI_RECT * pRect, GUI_HWIN hWin);
void    WM__Client2Screen           (const WM_Obj * pWin, GUI_RECT * pRect);
void    WM__DeleteAssocTimer        (GUI_HWIN hWin);
void    WM__DeleteSecure            (GUI_HWIN hWin);
void    WM__DetachWindow            (GUI_HWIN hChild);
void    WM__ForEachDesc             (GUI_HWIN hWin, WM_tfForEach * pcb, void * pData);
void    WM__GetClientRectWin        (const WM_Obj * pWin, GUI_RECT * pRect);
void    WM__GetClientRectEx         (GUI_HWIN hWin, GUI_RECT * pRect);
GUI_HWIN WM__GetFirstSibling         (GUI_HWIN hWin);
GUI_HWIN WM__GetFocussedChild        (GUI_HWIN hWin);
int     WM__GetHasFocus             (GUI_HWIN hWin);
GUI_HWIN WM__GetLastSibling          (GUI_HWIN hWin);
GUI_HWIN WM__GetPrevSibling          (GUI_HWIN hWin);
int     WM__GetTopLevelLayer        (GUI_HWIN hWin);
int     WM__GetWindowSizeX          (const WM_Obj * pWin);
int     WM__GetWindowSizeY          (const WM_Obj * pWin);
void    WM__InsertWindowIntoList    (GUI_HWIN hWin, GUI_HWIN hParent);
void    WM__Invalidate1Abs          (GUI_HWIN hWin, const GUI_RECT * pRect);
void    WM__InvalidateAreaBelow     (const GUI_RECT * pRect, GUI_HWIN StopWin);
void    WM__InvalidateRectEx        (const GUI_RECT * pInvalidRect, GUI_HWIN hParent, GUI_HWIN hStop);
void    WM__InvalidateTransAreaAbove(const GUI_RECT * pRect, GUI_HWIN StopWin);
int     WM__IntersectRect           (GUI_RECT * pDest, const GUI_RECT * pr0, const GUI_RECT * pr1);
int     WM__IsAncestor              (GUI_HWIN hChild, GUI_HWIN hParent);
int     WM__IsAncestorOrSelf        (GUI_HWIN hChild, GUI_HWIN hParent);
int     WM__IsChild                 (GUI_HWIN hWin, GUI_HWIN hParent);
int     WM__IsEnabled               (GUI_HWIN hWin);
int     WM__IsInModalArea           (GUI_HWIN hWin);
int     WM__IsInWindow              (WM_Obj * pWin, int x, int y);
int     WM__IsWindow                (GUI_HWIN hWin);
void    WM__LeaveIVRSearch          (void);
void    WM__MoveTo                  (GUI_HWIN hWin, int x, int y);
void    WM__MoveWindow              (GUI_HWIN hWin, int dx, int dy);
void    WM__NotifyVisChanged        (GUI_HWIN hWin, GUI_RECT * pRect);
int     WM__RectIsNZ                (const GUI_RECT * pr);
void    WM__RemoveWindowFromList    (GUI_HWIN hWin);
void    WM__Screen2Client           (const WM_Obj * pWin, GUI_RECT * pRect);
void    WM__SelectTopLevelLayer     (GUI_HWIN  hWin);
void    WM__SendMsgNoData           (GUI_HWIN hWin, unsigned char MsgId);
void    WM__SendMessage             (GUI_HWIN hWin, WM_MESSAGE * pm);
void    WM__SendMessageIfEnabled    (GUI_HWIN hWin, WM_MESSAGE * pm);
void    WM__SendMessageNoPara       (GUI_HWIN hWin, int MsgId);
void    WM__SendPIDMessage          (GUI_HWIN hWin, WM_MESSAGE * pMsg);
int     WM__SetScrollbarH           (GUI_HWIN hWin, int OnOff);
int     WM__SetScrollbarV           (GUI_HWIN hWin, int OnOff);
void    WM__UpdateChildPositions    (WM_Obj * pObj, int dx0, int dy0, int dx1, int dy1);
void    WM_PID__GetPrevState        (GUI_PID_STATE * pPrevState, int Layer);
void    WM_PID__SetPrevState        (GUI_PID_STATE * pPrevState, int Layer);
void    WM__SendTouchMessage        (GUI_HWIN hWin, WM_MESSAGE * pMsg);

unsigned short     WM_GetFlags                 (GUI_HWIN hWin);
int     WM__Paint                   (GUI_HWIN hWin);
void    WM__Paint1                  (GUI_HWIN hWin);
void    WM__AddCriticalHandle       (WM_CRITICAL_HANDLE * pCH);
void    WM__RemoveCriticalHandle    (WM_CRITICAL_HANDLE * pCH);
void    WM__SetLastTouched          (GUI_HWIN hWin);


  void    WM__InvalidateDrawAndDescs(GUI_HWIN hWin);




 

  typedef struct {
    int xSize, ySize; 
  } EFFECT_CONTEXT;

  int  GUI_MEMDEV__CalcParaFadeIn    (int Period, int TimeUsed);
  void GUI_MEMDEV__ClipBK            (EFFECT_CONTEXT * pContext);
  void GUI_MEMDEV__RemoveStaticDevice(GUI_HWIN hWin);
  void GUI_MEMDEV__UndoClipBK        (EFFECT_CONTEXT * pContext);


void WM__InvalidateParent(const GUI_RECT * pInvalidRect, GUI_HWIN hParent, GUI_HWIN hStop);
void WM__InvalidateRect  (const GUI_RECT * pInvalidRect, GUI_HWIN hParent, GUI_HWIN hStop, unsigned short Flags);

WM_tfInvalidateParent   WM__SetInvalidateParentFunc(WM_tfInvalidateParent pfInvalidateParentFunc);
WM_tfInvalidateDrawFunc WM__SetInvalidateDrawFunc  (WM_tfInvalidateDrawFunc pfInvalidateDrawFunc);
WM_tfPaint1Func         WM__SetPaint1Func          (WM_tfPaint1Func pfPaint1Func);









 
#line 62 "..\\STemWin\\inc\\WIDGET.h"








 
typedef struct {
  GUI_HWIN    hWin;
  int        Cmd;          
  int        ItemIndex;
  int        Col;
  int        x0, y0, x1, y1;
  void     * p;
} WIDGET_ITEM_DRAW_INFO;

typedef int  WIDGET_DRAW_ITEM_FUNC(const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
typedef void WIDGET_PAINT         (GUI_HWIN hObj);
typedef void WIDGET_CREATE        (GUI_HWIN hObj);

typedef struct {
  WIDGET_PAINT  * pfPaint;
  WIDGET_CREATE * pfCreate;
  void          * pSkinPrivate;
} WIDGET_SKIN;







 
#line 1 "..\\STemWin\\inc\\SCROLLBAR.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\SCROLLBAR.h"
#line 59 "..\\STemWin\\inc\\SCROLLBAR.h"
#line 1 "..\\STemWin\\inc\\WIDGET.h"
































 


















 
  
#line 396 "..\\STemWin\\inc\\WIDGET.h"




#line 60 "..\\STemWin\\inc\\SCROLLBAR.h"












 







 





 






 








 
typedef signed long SCROLLBAR_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorUpper[2];
  GUI_COLOR aColorLower[2];
  GUI_COLOR aColorShaft[2];
  GUI_COLOR ColorArrow;
  GUI_COLOR ColorGrasp;
} SCROLLBAR_SKINFLEX_PROPS;

typedef struct {
  int IsVertical;
  int State;
} SCROLLBAR_SKINFLEX_INFO;






 
SCROLLBAR_Handle SCROLLBAR_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int WinFlags, int SpecialFlags);
SCROLLBAR_Handle SCROLLBAR_CreateAttached(GUI_HWIN hParent, int SpecialFlags);
SCROLLBAR_Handle SCROLLBAR_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
SCROLLBAR_Handle SCROLLBAR_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
SCROLLBAR_Handle SCROLLBAR_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void SCROLLBAR_Callback(WM_MESSAGE * pMsg);






 

 

void      SCROLLBAR_AddValue   (SCROLLBAR_Handle hObj, int Add);
void      SCROLLBAR_Dec        (SCROLLBAR_Handle hObj);
void      SCROLLBAR_Inc        (SCROLLBAR_Handle hObj);
int       SCROLLBAR_GetUserData(SCROLLBAR_Handle hObj, void * pDest, int NumBytes);
GUI_COLOR SCROLLBAR_SetColor   (SCROLLBAR_Handle hObj, int Index, GUI_COLOR Color);
void      SCROLLBAR_SetNumItems(SCROLLBAR_Handle hObj, int NumItems);
void      SCROLLBAR_SetPageSize(SCROLLBAR_Handle hObj, int PageSize);
void      SCROLLBAR_SetValue   (SCROLLBAR_Handle hObj, int v);
int       SCROLLBAR_SetWidth   (SCROLLBAR_Handle hObj, int Width);
void      SCROLLBAR_SetState   (SCROLLBAR_Handle hObj, const WM_SCROLL_STATE* pState);
int       SCROLLBAR_SetUserData(SCROLLBAR_Handle hObj, const void * pSrc, int NumBytes);






 
void SCROLLBAR_GetSkinFlexProps     (SCROLLBAR_SKINFLEX_PROPS * pProps, int Index);
void SCROLLBAR_SetSkinClassic       (SCROLLBAR_Handle hObj);
void SCROLLBAR_SetSkin              (SCROLLBAR_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  SCROLLBAR_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void SCROLLBAR_SetSkinFlexProps     (const SCROLLBAR_SKINFLEX_PROPS * pProps, int Index);
void SCROLLBAR_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * SCROLLBAR_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
int       SCROLLBAR_GetDefaultWidth(void);
GUI_COLOR SCROLLBAR_SetDefaultColor(GUI_COLOR Color, unsigned int Index);  
int       SCROLLBAR_SetDefaultWidth(int DefaultWidth);






 
int       SCROLLBAR_GetThumbSizeMin(void);
int       SCROLLBAR_SetThumbSizeMin(int ThumbSizeMin);






 
int SCROLLBAR_GetNumItems(SCROLLBAR_Handle hObj);
int SCROLLBAR_GetPageSize(SCROLLBAR_Handle hObj);
int SCROLLBAR_GetValue   (SCROLLBAR_Handle hObj);






 











 
#line 98 "..\\STemWin\\inc\\WIDGET.h"






 



 
#line 136 "..\\STemWin\\inc\\WIDGET.h"






 

#line 167 "..\\STemWin\\inc\\WIDGET.h"




 












 
#line 215 "..\\STemWin\\inc\\WIDGET.h"
                                           







 






 








 
typedef struct {
  int EffectSize;
  void (* pfDrawUp)      (void);
  void (* pfDrawUpRect)  (const GUI_RECT * pRect);
  void (* pfDrawDown)    (void);
  void (* pfDrawDownRect)(const GUI_RECT * pRect);
  void (* pfDrawFlat)    (void);
  void (* pfDrawFlatRect)(const GUI_RECT * pRect);
} WIDGET_EFFECT;

typedef struct {
  WM_Obj      Win;
  const WIDGET_EFFECT* pEffect;
  signed short Id;
  unsigned short State;



} WIDGET;









 
 
typedef struct GUI_DRAW GUI_DRAW;
typedef void   GUI_DRAW_SELF_CB (GUI_HWIN hWin);
typedef signed long GUI_DRAW_HANDLE;

 
typedef struct {
  void (* pfDraw)    (GUI_DRAW_HANDLE hDrawObj, GUI_HWIN hObj, int x, int y);
  int  (* pfGetXSize)(GUI_DRAW_HANDLE hDrawObj);
  int  (* pfGetYSize)(GUI_DRAW_HANDLE hDrawObj);
} GUI_DRAW_CONSTS;

 
struct GUI_DRAW {
  const GUI_DRAW_CONSTS* pConsts;
  union {
    const void * pData;
    GUI_DRAW_SELF_CB* pfDraw;
  } Data;
  signed short xOff, yOff;
};

 
void GUI_DRAW__Draw    (GUI_DRAW_HANDLE hDrawObj, GUI_HWIN hObj, int x, int y);
int  GUI_DRAW__GetXSize(GUI_DRAW_HANDLE hDrawObj);
int  GUI_DRAW__GetYSize(GUI_DRAW_HANDLE hDrawObj);

 
signed long GUI_DRAW_BITMAP_Create  (const GUI_BITMAP* pBitmap, int x, int y);
signed long GUI_DRAW_BMP_Create     (const void* pBMP, int x, int y);
signed long GUI_DRAW_STREAMED_Create(const GUI_BITMAP_STREAM * pBitmap, int x, int y);
signed long GUI_DRAW_SELF_Create(GUI_DRAW_SELF_CB* pfDraw, int x, int y);






 

extern const WIDGET_EFFECT WIDGET_Effect_3D;
extern const WIDGET_EFFECT WIDGET_Effect_3D1L;
extern const WIDGET_EFFECT WIDGET_Effect_3D2L;
extern const WIDGET_EFFECT WIDGET_Effect_None;
extern const WIDGET_EFFECT WIDGET_Effect_Simple;






 

void      WIDGET__DrawFocusRect      (WIDGET * pWidget, const GUI_RECT * pRect, int Dist);
void      WIDGET__DrawHLine          (WIDGET * pWidget, int y, int x0, int x1);
void      WIDGET__DrawTriangle       (WIDGET * pWidget, int x, int y, int Size, int Inc);
void      WIDGET__DrawVLine          (WIDGET * pWidget, int x, int y0, int y1);
void      WIDGET__EFFECT_DrawDownRect(WIDGET * pWidget, GUI_RECT * pRect);
void      WIDGET__EFFECT_DrawDown    (WIDGET * pWidget);
void      WIDGET__EFFECT_DrawUpRect  (WIDGET * pWidget, GUI_RECT * pRect);
void      WIDGET__FillRectEx         (WIDGET * pWidget, const GUI_RECT * pRect);
int       WIDGET__GetWindowSizeX     (GUI_HWIN hWin);
GUI_COLOR WIDGET__GetBkColor         (GUI_HWIN hObj);
int       WIDGET__GetXSize           (const WIDGET * pWidget);
int       WIDGET__GetYSize           (const WIDGET * pWidget);
void      WIDGET__GetClientRect      (WIDGET * pWidget, GUI_RECT * pRect);
void      WIDGET__GetInsideRect      (WIDGET * pWidget, GUI_RECT * pRect);
void      WIDGET__Init               (WIDGET * pWidget, int Id, unsigned short State);
void      WIDGET__RotateRect90       (WIDGET * pWidget, GUI_RECT * pDest, const GUI_RECT * pRect);
void      WIDGET__SetScrollState     (GUI_HWIN hWin, const WM_SCROLL_STATE * pVState, const WM_SCROLL_STATE * pState);
void      WIDGET__FillStringInRect   (const char * pText, const GUI_RECT * pFillRect, const GUI_RECT * pTextRectMax, const GUI_RECT * pTextRectAct);






 
void  WIDGET_SetState     (GUI_HWIN hObj, int State);
void  WIDGET_AndState     (GUI_HWIN hObj, int State);
void  WIDGET_OrState      (GUI_HWIN hObj, int State);
int   WIDGET_HandleActive (GUI_HWIN hObj, WM_MESSAGE* pMsg);
int   WIDGET_GetState     (GUI_HWIN hObj);
int   WIDGET_SetWidth     (GUI_HWIN hObj, int Width);

void  WIDGET_EFFECT_3D_DrawUp(void);

const WIDGET_EFFECT* WIDGET_SetDefaultEffect(const WIDGET_EFFECT* pEffect);

void  WIDGET_SetEffect              (GUI_HWIN hObj, const WIDGET_EFFECT* pEffect);

const WIDGET_EFFECT* WIDGET_GetDefaultEffect(void);

void WIDGET_EFFECT_3D_SetColor    (unsigned Index, GUI_COLOR Color);
void WIDGET_EFFECT_3D1L_SetColor  (unsigned Index, GUI_COLOR Color);
void WIDGET_EFFECT_3D2L_SetColor  (unsigned Index, GUI_COLOR Color);
void WIDGET_EFFECT_Simple_SetColor(unsigned Index, GUI_COLOR Color);

GUI_COLOR WIDGET_EFFECT_3D_GetColor    (unsigned Index);
GUI_COLOR WIDGET_EFFECT_3D1L_GetColor  (unsigned Index);
GUI_COLOR WIDGET_EFFECT_3D2L_GetColor  (unsigned Index);
GUI_COLOR WIDGET_EFFECT_Simple_GetColor(unsigned Index);

int WIDGET_EFFECT_3D_GetNumColors(void);
int WIDGET_EFFECT_3D1L_GetNumColors(void);
int WIDGET_EFFECT_3D2L_GetNumColors(void);
int WIDGET_EFFECT_Simple_GetNumColors(void);






 

















#line 60 "..\\STemWin\\inc\\BUTTON.h"










 
 







 







 







 






 










 
typedef signed long BUTTON_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorUpper[2];
  GUI_COLOR aColorLower[2];
  int Radius;
} BUTTON_SKINFLEX_PROPS;







 

BUTTON_Handle BUTTON_Create        (int x0, int y0, int xSize, int ySize, int ID, int Flags);
BUTTON_Handle BUTTON_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags);
BUTTON_Handle BUTTON_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
BUTTON_Handle BUTTON_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
BUTTON_Handle BUTTON_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);






 
GUI_COLOR        BUTTON_GetDefaultBkColor   (unsigned Index);
const GUI_FONT * BUTTON_GetDefaultFont      (void);
int              BUTTON_GetDefaultTextAlign (void);
GUI_COLOR        BUTTON_GetDefaultTextColor (unsigned Index);
void             BUTTON_SetDefaultBkColor   (GUI_COLOR Color, unsigned Index);
GUI_COLOR        BUTTON_SetDefaultFocusColor(GUI_COLOR Color);
void             BUTTON_SetDefaultFont      (const GUI_FONT * pFont);
void             BUTTON_SetDefaultTextAlign (int Align);
void             BUTTON_SetDefaultTextColor (GUI_COLOR Color, unsigned Index);







 
void BUTTON_Callback(WM_MESSAGE *pMsg);






 
GUI_COLOR          BUTTON_GetBkColor         (BUTTON_Handle hObj, unsigned int Index);
const GUI_BITMAP * BUTTON_GetBitmap(BUTTON_Handle hObj,unsigned int Index);
const GUI_FONT   * BUTTON_GetFont  (BUTTON_Handle hObj);
GUI_COLOR          BUTTON_GetFrameColor      (BUTTON_Handle hObj);
WIDGET           * BUTTON_GetpWidget         (BUTTON_Handle hObj);
void               BUTTON_GetText            (BUTTON_Handle hObj, char * pBuffer, int MaxLen);
GUI_COLOR          BUTTON_GetTextColor       (BUTTON_Handle hObj, unsigned int Index);
int                BUTTON_GetTextAlign       (BUTTON_Handle hObj);
int                BUTTON_GetUserData        (BUTTON_Handle hObj, void * pDest, int NumBytes);
unsigned           BUTTON_IsPressed          (BUTTON_Handle hObj);
void               BUTTON_SetBitmap          (BUTTON_Handle hObj, unsigned int Index, const GUI_BITMAP * pBitmap);
void               BUTTON_SetBitmapEx        (BUTTON_Handle hObj, unsigned int Index, const GUI_BITMAP * pBitmap, int x, int y);
void               BUTTON_SetBkColor         (BUTTON_Handle hObj, unsigned int Index, GUI_COLOR Color);
void               BUTTON_SetBMP             (BUTTON_Handle hObj, unsigned int Index, const void * pBitmap);
void               BUTTON_SetBMPEx           (BUTTON_Handle hObj, unsigned int Index, const void * pBitmap, int x, int y);
void               BUTTON_SetFont            (BUTTON_Handle hObj, const GUI_FONT * pfont);
void               BUTTON_SetFrameColor      (BUTTON_Handle hObj, GUI_COLOR Color);
void               BUTTON_SetState           (BUTTON_Handle hObj, int State);                                     
void               BUTTON_SetPressed         (BUTTON_Handle hObj, int State);
GUI_COLOR          BUTTON_SetFocusColor      (BUTTON_Handle hObj, GUI_COLOR Color);
void               BUTTON_SetFocussable      (BUTTON_Handle hObj, int State);
void               BUTTON_SetStreamedBitmap  (BUTTON_Handle hObj, unsigned int Index, const GUI_BITMAP_STREAM * pBitmap);
void               BUTTON_SetStreamedBitmapEx(BUTTON_Handle hObj, unsigned int Index, const GUI_BITMAP_STREAM * pBitmap, int x, int y);
int                BUTTON_SetText            (BUTTON_Handle hObj, const char* s);
void               BUTTON_SetTextAlign       (BUTTON_Handle hObj, int Align);
void               BUTTON_SetTextColor       (BUTTON_Handle hObj, unsigned int Index, GUI_COLOR Color);
void               BUTTON_SetTextOffset      (BUTTON_Handle hObj, int xPos, int yPos);
void               BUTTON_SetSelfDrawEx      (BUTTON_Handle hObj, unsigned int Index, GUI_DRAW_SELF_CB * pDraw, int x, int y);  
void               BUTTON_SetSelfDraw        (BUTTON_Handle hObj, unsigned int Index, GUI_DRAW_SELF_CB * pDraw);                
void               BUTTON_SetReactOnLevel    (void);
void               BUTTON_SetReactOnTouch    (void);
int                BUTTON_SetUserData        (BUTTON_Handle hObj, const void * pSrc, int NumBytes);






 
void BUTTON_GetSkinFlexProps     (BUTTON_SKINFLEX_PROPS * pProps, int Index);
void BUTTON_SetSkinClassic       (BUTTON_Handle hObj);
void BUTTON_SetSkin              (BUTTON_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  BUTTON_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void BUTTON_SetSkinFlexProps     (const BUTTON_SKINFLEX_PROPS * pProps, int Index);
void BUTTON_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * BUTTON_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);










 
#line 59 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\CALENDAR.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\CALENDAR.h"












 






















 








 



 
typedef struct {
  int Year;
  int Month;
  int Day;
} CALENDAR_DATE;




 
typedef struct {
  GUI_COLOR aColorFrame[3]; 
  GUI_COLOR aColorUpper[2]; 
  GUI_COLOR aColorLower[2]; 
  GUI_COLOR ColorArrow;     
} CALENDAR_SKINFLEX_PROPS;






 
GUI_HWIN CALENDAR_Create           (GUI_HWIN hParent, int xPos, int yPos, unsigned Year, unsigned Month, unsigned Day, unsigned FirstDayOfWeek, int Id, int Flags);
void    CALENDAR_GetDate          (GUI_HWIN hWin, CALENDAR_DATE * pDate);
void    CALENDAR_GetSel           (GUI_HWIN hWin, CALENDAR_DATE * pDate);
void    CALENDAR_SetDate          (GUI_HWIN hWin, CALENDAR_DATE * pDate);
void    CALENDAR_SetSel           (GUI_HWIN hWin, CALENDAR_DATE * pDate);
void    CALENDAR_ShowDate         (GUI_HWIN hWin, CALENDAR_DATE * pDate);




 
void    CALENDAR_SetDefaultBkColor(unsigned Index, GUI_COLOR Color);
void    CALENDAR_SetDefaultColor  (unsigned Index, GUI_COLOR Color);
void    CALENDAR_SetDefaultDays   (const char ** apDays);
void    CALENDAR_SetDefaultFont   (unsigned Index, const GUI_FONT * pFont);
void    CALENDAR_SetDefaultMonths (const char ** apMonths);
void    CALENDAR_SetDefaultSize   (unsigned Index, unsigned Size);




 
void    CALENDAR_GetSkinFlexProps (CALENDAR_SKINFLEX_PROPS * pProps, int Index);
void    CALENDAR_SetSkinFlexProps (const CALENDAR_SKINFLEX_PROPS * pProps, int Index);







 
void CALENDAR_Callback(WM_MESSAGE * pMsg);








 
#line 60 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\CHECKBOX.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\CHECKBOX.h"
#line 59 "..\\STemWin\\inc\\CHECKBOX.h"
#line 60 "..\\STemWin\\inc\\CHECKBOX.h"












 




 






 
#line 91 "..\\STemWin\\inc\\CHECKBOX.h"




 








 
typedef signed long CHECKBOX_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorInner[2];
  GUI_COLOR ColorCheck;
  int       ButtonSize;
} CHECKBOX_SKINFLEX_PROPS;






 
CHECKBOX_Handle CHECKBOX_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags);
CHECKBOX_Handle CHECKBOX_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
CHECKBOX_Handle CHECKBOX_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
CHECKBOX_Handle CHECKBOX_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void CHECKBOX_Callback(WM_MESSAGE * pMsg);






 

int              CHECKBOX_GetDefaultAlign     (void);
GUI_COLOR        CHECKBOX_GetDefaultBkColor   (void);
const GUI_FONT * CHECKBOX_GetDefaultFont      (void);
int              CHECKBOX_GetDefaultSpacing   (void);
int              CHECKBOX_GetDefaultTextAlign (void);
GUI_COLOR        CHECKBOX_GetDefaultTextColor (void);
int              CHECKBOX_GetUserData         (CHECKBOX_Handle hObj, void * pDest, int NumBytes);
void             CHECKBOX_SetDefaultAlign     (int Align);
void             CHECKBOX_SetDefaultBkColor   (GUI_COLOR Color);
GUI_COLOR        CHECKBOX_SetDefaultFocusColor(GUI_COLOR Color);
void             CHECKBOX_SetDefaultFont      (const GUI_FONT * pFont);
void             CHECKBOX_SetDefaultImage     (const GUI_BITMAP * pBitmap, unsigned int Index);
void             CHECKBOX_SetDefaultSpacing   (int Spacing);
void             CHECKBOX_SetDefaultTextAlign (int Align);
void             CHECKBOX_SetDefaultTextColor (GUI_COLOR Color);






 

int       CHECKBOX_GetState     (CHECKBOX_Handle hObj);
int       CHECKBOX_GetText      (CHECKBOX_Handle hObj, char * pBuffer, int MaxLen);
int       CHECKBOX_IsChecked    (CHECKBOX_Handle hObj);
void      CHECKBOX_SetBkColor   (CHECKBOX_Handle hObj, GUI_COLOR Color);
GUI_COLOR CHECKBOX_SetBoxBkColor(CHECKBOX_Handle hObj, GUI_COLOR Color, int Index);
GUI_COLOR CHECKBOX_SetFocusColor(CHECKBOX_Handle hObj, GUI_COLOR Color);
void      CHECKBOX_SetFont      (CHECKBOX_Handle hObj, const GUI_FONT * pFont);
void      CHECKBOX_SetImage     (CHECKBOX_Handle hObj, const GUI_BITMAP * pBitmap, unsigned int Index);
void      CHECKBOX_SetNumStates (CHECKBOX_Handle hObj, unsigned NumStates);
void      CHECKBOX_SetSpacing   (CHECKBOX_Handle hObj, unsigned Spacing);
void      CHECKBOX_SetState     (CHECKBOX_Handle hObj, unsigned State);
void      CHECKBOX_SetText      (CHECKBOX_Handle hObj, const char * pText);
void      CHECKBOX_SetTextAlign (CHECKBOX_Handle hObj, int Align);
void      CHECKBOX_SetTextColor (CHECKBOX_Handle hObj, GUI_COLOR Color);
int       CHECKBOX_SetUserData  (CHECKBOX_Handle hObj, const void * pSrc, int NumBytes);






 
void CHECKBOX_GetSkinFlexProps      (CHECKBOX_SKINFLEX_PROPS * pProps, int Index);
void CHECKBOX_SetSkinClassic        (CHECKBOX_Handle hObj);
void CHECKBOX_SetSkin               (CHECKBOX_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  CHECKBOX_DrawSkinFlex          (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void CHECKBOX_SetSkinFlexProps      (const CHECKBOX_SKINFLEX_PROPS * pProps, int Index);
void CHECKBOX_SetDefaultSkinClassic (void);
int  CHECKBOX_GetSkinFlexButtonSize (CHECKBOX_Handle hObj);
void CHECKBOX_SetSkinFlexButtonSize (CHECKBOX_Handle hObj, int ButtonSize);
WIDGET_DRAW_ITEM_FUNC * CHECKBOX_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 














 
#line 61 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\CHOOSECOLOR.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\CHOOSECOLOR.h"












 










 



 
typedef struct {
  unsigned  aBorder[2];
  unsigned  aSpace[2];
  unsigned  aButtonSize[2];
  GUI_COLOR aColor[2];
} CHOOSECOLOR_PROPS;




 
typedef struct {
  unsigned long               LastColor;
  const GUI_COLOR * pColor;
  unsigned          NumColors;
  unsigned          NumColorsPerLine;
  int               SelOld;
  int               Sel;
  GUI_HWIN           hParent;
  CHOOSECOLOR_PROPS Props;
} CHOOSECOLOR_CONTEXT;






 
GUI_HWIN CHOOSECOLOR_Create(GUI_HWIN           hParent,
                           int               xPos,
                           int               yPos,
                           int               xSize,
                           int               ySize,
                           const GUI_COLOR * pColor,
                           unsigned          NumColors,
                           unsigned          NumColorsPerLine,
                           int               Sel,
                           const char      * sCaption,
                           int               Flags);

int  CHOOSECOLOR_GetSel(GUI_HWIN hObj);
void CHOOSECOLOR_SetSel(GUI_HWIN hObj, int Sel);

void CHOOSECOLOR_SetDefaultColor     (unsigned Index, GUI_COLOR Color);
void CHOOSECOLOR_SetDefaultSpace     (unsigned Index, unsigned Space);
void CHOOSECOLOR_SetDefaultBorder    (unsigned Index, unsigned Border);
void CHOOSECOLOR_SetDefaultButtonSize(unsigned Index, unsigned ButtonSize);







 
void CHOOSECOLOR_Callback(WM_MESSAGE * pMsg);







#line 62 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\CHOOSEFILE.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\CHOOSEFILE.h"












 


















 



 
typedef struct CHOOSEFILE_INFO CHOOSEFILE_INFO;

struct CHOOSEFILE_INFO {
  int               Cmd;                                 
  int               Id;                                  
  const char      * pMask;                               
  char            * pName;                               
  char            * pExt;                                
  char            * pAttrib;                             
  WM_TOOLTIP_HANDLE hToolTip;                            
  unsigned long               SizeL;                               
  unsigned long               SizeH;                               
  unsigned long               Flags;                               
  char              pRoot[256];            
  int            (* pfGetData)(CHOOSEFILE_INFO * pInfo); 
};






 
GUI_HWIN CHOOSEFILE_Create(GUI_HWIN           hParent,  
                          int               xPos,     
                          int               yPos,     
                          int               xSize,    
                          int               ySize,    
                          const char      * apRoot[], 
                          int               NumRoot,  
                          int               SelRoot,  
                          const char      * sCaption, 
                          int               Flags,    
                          CHOOSEFILE_INFO * pInfo     
                          );

void    CHOOSEFILE_Callback            (WM_MESSAGE * pMsg);
void    CHOOSEFILE_EnableToolTips      (void);
void    CHOOSEFILE_SetButtonText       (GUI_HWIN hWin, unsigned ButtonIndex, const char * pText);
void    CHOOSEFILE_SetDefaultButtonText(unsigned ButtonIndex, const char * pText);
void    CHOOSEFILE_SetDelim            (char Delim);
void    CHOOSEFILE_SetToolTips         (const TOOLTIP_INFO * pInfo, int NumItems);
void    CHOOSEFILE_SetTopMode          (unsigned OnOff);







#line 63 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\DROPDOWN.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\DROPDOWN.h"
#line 59 "..\\STemWin\\inc\\DROPDOWN.h"
#line 1 "..\\STemWin\\inc\\LISTBOX.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LISTBOX.h"
#line 59 "..\\STemWin\\inc\\LISTBOX.h"
#line 60 "..\\STemWin\\inc\\LISTBOX.h"












 






 








 
typedef signed long LISTBOX_Handle;







 





 
#line 112 "..\\STemWin\\inc\\LISTBOX.h"






 

LISTBOX_Handle LISTBOX_Create        (const GUI_ConstString * ppText, int x0, int y0, int xSize, int ySize, int Flags);
LISTBOX_Handle LISTBOX_CreateAsChild (const GUI_ConstString * ppText, GUI_HWIN hWinParent, int x0, int y0, int xSize, int ySize, int Flags);
LISTBOX_Handle LISTBOX_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const GUI_ConstString * ppText);
LISTBOX_Handle LISTBOX_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const GUI_ConstString * ppText, int NumExtraBytes);
LISTBOX_Handle LISTBOX_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void LISTBOX_Callback(WM_MESSAGE * pMsg);






 

int          LISTBOX_AddKey          (LISTBOX_Handle hObj, int Key);
void         LISTBOX_AddString       (LISTBOX_Handle hObj, const char * s);
void         LISTBOX_AddStringH      (LISTBOX_Handle hObj, signed long hString);  
void         LISTBOX_DecSel          (LISTBOX_Handle hObj);
void         LISTBOX_DeleteItem      (LISTBOX_Handle hObj, unsigned int Index);
void         LISTBOX_EnableWrapMode  (LISTBOX_Handle hObj, int OnOff);
unsigned     LISTBOX_GetItemSpacing  (LISTBOX_Handle hObj);
unsigned     LISTBOX_GetNumItems     (LISTBOX_Handle hObj);
int          LISTBOX_GetSel          (LISTBOX_Handle hObj);
const GUI_FONT * LISTBOX_GetFont     (LISTBOX_Handle hObj);
int          LISTBOX_GetItemDisabled (LISTBOX_Handle hObj, unsigned Index);
int          LISTBOX_GetItemSel      (LISTBOX_Handle hObj, unsigned Index);
void         LISTBOX_GetItemText     (LISTBOX_Handle hObj, unsigned Index, char * pBuffer, int MaxSize);
int          LISTBOX_GetMulti        (LISTBOX_Handle hObj);
int          LISTBOX_GetScrollStepH  (LISTBOX_Handle hObj);
int          LISTBOX_GetTextAlign    (LISTBOX_Handle hObj);
int          LISTBOX_GetUserData     (LISTBOX_Handle hObj, void * pDest, int NumBytes);
void         LISTBOX_IncSel          (LISTBOX_Handle hObj);
void         LISTBOX_InsertString    (LISTBOX_Handle hObj, const char * s, unsigned int Index);
void         LISTBOX_InvalidateItem  (LISTBOX_Handle hObj, int Index);
int          LISTBOX_OwnerDraw       (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void         LISTBOX_SetAutoScrollH  (LISTBOX_Handle hObj, int OnOff);
void         LISTBOX_SetAutoScrollV  (LISTBOX_Handle hObj, int OnOff);
void         LISTBOX_SetBkColor      (LISTBOX_Handle hObj, unsigned int Index, GUI_COLOR color);
void         LISTBOX_SetFont         (LISTBOX_Handle hObj, const GUI_FONT * pFont);
void         LISTBOX_SetItemDisabled (LISTBOX_Handle hObj, unsigned Index, int OnOff);
void         LISTBOX_SetItemSel      (LISTBOX_Handle hObj, unsigned Index, int OnOff);
void         LISTBOX_SetItemSpacing  (LISTBOX_Handle hObj, unsigned Value);
void         LISTBOX_SetMulti        (LISTBOX_Handle hObj, int Mode);
void         LISTBOX_SetOwner        (LISTBOX_Handle hObj, GUI_HWIN hOwner);
void         LISTBOX_SetOwnerDraw    (LISTBOX_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawItem);
void         LISTBOX_SetScrollStepH  (LISTBOX_Handle hObj, int Value);
void         LISTBOX_SetSel          (LISTBOX_Handle hObj, int Sel);
void         LISTBOX_SetScrollbarColor(LISTBOX_Handle hObj, unsigned Index, GUI_COLOR Color);
void         LISTBOX_SetScrollbarWidth(LISTBOX_Handle hObj, unsigned Width);
void         LISTBOX_SetString       (LISTBOX_Handle hObj, const char * s, unsigned int Index);
void         LISTBOX_SetText         (LISTBOX_Handle hObj, const GUI_ConstString * ppText);
void         LISTBOX_SetTextAlign    (LISTBOX_Handle hObj, int Align);
GUI_COLOR    LISTBOX_SetTextColor    (LISTBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
int          LISTBOX_SetUserData     (LISTBOX_Handle hObj, const void * pSrc, int NumBytes);
int          LISTBOX_UpdateScrollers (LISTBOX_Handle hObj);






 

const GUI_FONT * LISTBOX_GetDefaultFont(void);
int              LISTBOX_GetDefaultScrollStepH (void);
GUI_COLOR        LISTBOX_GetDefaultBkColor     (unsigned Index);
int              LISTBOX_GetDefaultTextAlign   (void);
GUI_COLOR        LISTBOX_GetDefaultTextColor   (unsigned Index);
void             LISTBOX_SetDefaultFont        (const GUI_FONT * pFont);
void             LISTBOX_SetDefaultScrollStepH (int Value);
void             LISTBOX_SetDefaultBkColor     (unsigned Index, GUI_COLOR Color);
void             LISTBOX_SetDefaultTextAlign   (int Align);
void             LISTBOX_SetDefaultTextColor   (unsigned Index, GUI_COLOR Color);






 











 
#line 60 "..\\STemWin\\inc\\DROPDOWN.h"










 






 










 










 
typedef signed long DROPDOWN_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorUpper[2];
  GUI_COLOR aColorLower[2];
  GUI_COLOR ColorArrow;
  GUI_COLOR ColorText;
  GUI_COLOR ColorSep;
  int Radius;
} DROPDOWN_SKINFLEX_PROPS;






 
DROPDOWN_Handle DROPDOWN_Create        (GUI_HWIN hWinParent, int x0, int y0, int xSize, int ySize, int Flags);
DROPDOWN_Handle DROPDOWN_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
DROPDOWN_Handle DROPDOWN_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
DROPDOWN_Handle DROPDOWN_CreateIndirect(const GUI_WIDGET_CREATE_INFO* pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK* cb);







 
void DROPDOWN_Callback(WM_MESSAGE * pMsg);






 
void     DROPDOWN_AddKey           (DROPDOWN_Handle hObj, int Key);
void     DROPDOWN_AddString        (DROPDOWN_Handle hObj, const char* s);
void     DROPDOWN_Collapse         (DROPDOWN_Handle hObj);
void     DROPDOWN_DecSel           (DROPDOWN_Handle hObj);
void     DROPDOWN_DecSelExp        (DROPDOWN_Handle hObj);
void     DROPDOWN_DeleteItem       (DROPDOWN_Handle hObj, unsigned int Index);
void     DROPDOWN_Expand           (DROPDOWN_Handle hObj);
unsigned DROPDOWN_GetItemDisabled  (DROPDOWN_Handle hObj, unsigned Index);
unsigned DROPDOWN_GetItemSpacing   (DROPDOWN_Handle hObj);
int      DROPDOWN_GetItemText      (DROPDOWN_Handle hObj, unsigned Index, char * pBuffer, int MaxSize);
LISTBOX_Handle DROPDOWN_GetListbox (DROPDOWN_Handle hObj);
int      DROPDOWN_GetNumItems      (DROPDOWN_Handle hObj);
int      DROPDOWN_GetSel           (DROPDOWN_Handle hObj);
int      DROPDOWN_GetSelExp        (DROPDOWN_Handle hObj);
int      DROPDOWN_GetUserData      (DROPDOWN_Handle hObj, void * pDest, int NumBytes);
void     DROPDOWN_IncSel           (DROPDOWN_Handle hObj);
void     DROPDOWN_IncSelExp        (DROPDOWN_Handle hObj);
void     DROPDOWN_InsertString     (DROPDOWN_Handle hObj, const char* s, unsigned int Index);
void     DROPDOWN_SetAutoScroll    (DROPDOWN_Handle hObj, int OnOff);
void     DROPDOWN_SetBkColor       (DROPDOWN_Handle hObj, unsigned int Index, GUI_COLOR color);
void     DROPDOWN_SetColor         (DROPDOWN_Handle hObj, unsigned int Index, GUI_COLOR Color);
void     DROPDOWN_SetFont          (DROPDOWN_Handle hObj, const GUI_FONT * pfont);
void     DROPDOWN_SetItemDisabled  (DROPDOWN_Handle hObj, unsigned Index, int OnOff);
void     DROPDOWN_SetItemSpacing   (DROPDOWN_Handle hObj, unsigned Value);
int      DROPDOWN_SetListHeight    (DROPDOWN_Handle hObj, unsigned Height);
void     DROPDOWN_SetScrollbarColor(DROPDOWN_Handle hObj, unsigned Index, GUI_COLOR Color);
void     DROPDOWN_SetScrollbarWidth(DROPDOWN_Handle hObj, unsigned Width);
void     DROPDOWN_SetSel           (DROPDOWN_Handle hObj, int Sel);
void     DROPDOWN_SetSelExp        (DROPDOWN_Handle hObj, int Sel);
void     DROPDOWN_SetTextAlign     (DROPDOWN_Handle hObj, int Align);
void     DROPDOWN_SetTextColor     (DROPDOWN_Handle hObj, unsigned int index, GUI_COLOR color);
void     DROPDOWN_SetTextHeight    (DROPDOWN_Handle hObj, unsigned TextHeight);
int      DROPDOWN_SetUpMode        (DROPDOWN_Handle hObj, int OnOff);
int      DROPDOWN_SetUserData      (DROPDOWN_Handle hObj, const void * pSrc, int NumBytes);






 
void DROPDOWN_GetSkinFlexProps     (DROPDOWN_SKINFLEX_PROPS * pProps, int Index);
void DROPDOWN_SetSkinClassic       (DROPDOWN_Handle hObj);
void DROPDOWN_SetSkin              (DROPDOWN_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  DROPDOWN_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void DROPDOWN_SetSkinFlexProps     (const DROPDOWN_SKINFLEX_PROPS * pProps, int Index);
void DROPDOWN_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * DROPDOWN_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
GUI_COLOR        DROPDOWN_GetDefaultBkColor       (int Index);
GUI_COLOR        DROPDOWN_GetDefaultColor         (int Index);
const GUI_FONT * DROPDOWN_GetDefaultFont          (void);
GUI_COLOR        DROPDOWN_GetDefaultScrollbarColor(int Index);
void             DROPDOWN_SetDefaultFont          (const GUI_FONT * pFont);
GUI_COLOR        DROPDOWN_SetDefaultBkColor       (int Index, GUI_COLOR Color);
GUI_COLOR        DROPDOWN_SetDefaultColor         (int Index, GUI_COLOR Color);
GUI_COLOR        DROPDOWN_SetDefaultScrollbarColor(int Index, GUI_COLOR Color);








 
#line 64 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\EDIT.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\EDIT.h"
#line 59 "..\\STemWin\\inc\\EDIT.h"















 





 



#line 90 "..\\STemWin\\inc\\EDIT.h"






































 
typedef signed long EDIT_Handle;
typedef void tEDIT_AddKeyEx    (EDIT_Handle hObj, int Key);
typedef void tEDIT_UpdateBuffer(EDIT_Handle hObj);




 
EDIT_Handle EDIT_Create        (int x0, int y0, int xSize, int ySize, int Id, int MaxLen, int Flags);
EDIT_Handle EDIT_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int MaxLen);
EDIT_Handle EDIT_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int MaxLen);
EDIT_Handle EDIT_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int MaxLen, int NumExtraBytes);
EDIT_Handle EDIT_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void EDIT_Callback(WM_MESSAGE * pMsg);






 
void EDIT_SetDefaultBkColor  (unsigned int Index, GUI_COLOR Color);
void EDIT_SetDefaultFont     (const GUI_FONT * pFont);
void EDIT_SetDefaultTextAlign(int Align);
void EDIT_SetDefaultTextColor(unsigned int Index, GUI_COLOR Color);




 



GUI_COLOR        EDIT_GetDefaultBkColor(unsigned int Index);
const GUI_FONT * EDIT_GetDefaultFont(void);
int              EDIT_GetDefaultTextAlign(void);
GUI_COLOR        EDIT_GetDefaultTextColor(unsigned int Index);



void EDIT_AddKey           (EDIT_Handle hObj, int Key);
void EDIT_EnableBlink      (EDIT_Handle hObj, int Period, int OnOff);
GUI_COLOR EDIT_GetBkColor  (EDIT_Handle hObj, unsigned int Index);
void EDIT_SetBkColor       (EDIT_Handle hObj, unsigned int Index, GUI_COLOR color);
void EDIT_SetCursorAtChar  (EDIT_Handle hObj, int Pos);
void EDIT_SetCursorAtPixel (EDIT_Handle hObj, int xPos);
void EDIT_SetFocussable    (EDIT_Handle hObj, int State);
void EDIT_SetFont          (EDIT_Handle hObj, const GUI_FONT * pFont);
int  EDIT_SetInsertMode    (EDIT_Handle hObj, int OnOff);
void EDIT_SetMaxLen        (EDIT_Handle hObj, int MaxLen);
void EDIT_SetpfAddKeyEx    (EDIT_Handle hObj, tEDIT_AddKeyEx * pfAddKeyEx);
void EDIT_SetpfUpdateBuffer(EDIT_Handle hObj, tEDIT_UpdateBuffer * pfUpdateBuffer);
void EDIT_SetText          (EDIT_Handle hObj, const char * s);
void EDIT_SetTextAlign     (EDIT_Handle hObj, int Align);
GUI_COLOR EDIT_GetTextColor(EDIT_Handle hObj, unsigned int Index);
void EDIT_SetTextColor     (EDIT_Handle hObj, unsigned int Index, GUI_COLOR Color);
void EDIT_SetSel           (EDIT_Handle hObj, int FirstChar, int LastChar);
int  EDIT_SetUserData      (EDIT_Handle hObj, const void * pSrc, int NumBytes);
int  EDIT_EnableInversion  (EDIT_Handle hObj, int OnOff);



int   EDIT_GetCursorCharPos  (EDIT_Handle hObj);
void  EDIT_GetCursorPixelPos (EDIT_Handle hObj, int * pxPos, int * pyPos);
float EDIT_GetFloatValue     (EDIT_Handle hObj);
const GUI_FONT * EDIT_GetFont(EDIT_Handle hObj);
int   EDIT_GetNumChars       (EDIT_Handle hObj);
void  EDIT_GetText           (EDIT_Handle hObj, char * sDest, int MaxLen);
signed long   EDIT_GetValue          (EDIT_Handle hObj);
void  EDIT_SetFloatValue     (EDIT_Handle hObj, float Value);
int   EDIT_GetUserData       (EDIT_Handle hObj, void * pDest, int NumBytes);
void  EDIT_SetValue          (EDIT_Handle hObj, signed long Value);






 
void  EDIT_SetHexMode  (EDIT_Handle hEdit, unsigned long Value, unsigned long Min, unsigned long Max);
void  EDIT_SetBinMode  (EDIT_Handle hEdit, unsigned long Value, unsigned long Min, unsigned long Max);
void  EDIT_SetDecMode  (EDIT_Handle hEdit, signed long Value, signed long Min, signed long Max, int Shift, unsigned char Flags);
void  EDIT_SetFloatMode(EDIT_Handle hEdit, float Value, float Min, float Max, int Shift, unsigned char Flags);
void  EDIT_SetTextMode (EDIT_Handle hEdit);
void  EDIT_SetUlongMode(EDIT_Handle hEdit, unsigned long Value, unsigned long Min, unsigned long Max);

unsigned long   GUI_EditHex      (unsigned long Value, unsigned long Min, unsigned long Max, int Len, int xSize);
unsigned long   GUI_EditBin      (unsigned long Value, unsigned long Min, unsigned long Max, int Len, int xSize);
signed long   GUI_EditDec      (signed long Value, signed long Min, signed long Max, int Len, int xSize, int Shift, unsigned char Flags);
float GUI_EditFloat    (float Value, float Min, float Max, int Len, int xSize, int Shift, unsigned char Flags);
void  GUI_EditString   (char * pString, int Len, int xSize);








 
#line 65 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\FRAMEWIN.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\FRAMEWIN.h"
#line 59 "..\\STemWin\\inc\\FRAMEWIN.h"
#line 61 "..\\STemWin\\inc\\FRAMEWIN.h"










 



 







 






 
#line 97 "..\\STemWin\\inc\\FRAMEWIN.h"

#line 104 "..\\STemWin\\inc\\FRAMEWIN.h"




 






 






 










 
typedef signed long FRAMEWIN_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorTitle[2];
  int Radius;
  int SpaceX;
  int BorderSizeL;
  int BorderSizeR;
  int BorderSizeT;
  int BorderSizeB;
} FRAMEWIN_SKINFLEX_PROPS;






 
FRAMEWIN_Handle FRAMEWIN_Create        (const char * pTitle, WM_CALLBACK * cb, int Flags, int x0, int y0, int xSize, int ySize);
FRAMEWIN_Handle FRAMEWIN_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, const char * pText, WM_CALLBACK * cb, int Flags);
FRAMEWIN_Handle FRAMEWIN_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const char * pTitle, WM_CALLBACK * cb);
FRAMEWIN_Handle FRAMEWIN_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const char * pTitle, WM_CALLBACK * cb, int NumExtraBytes);
FRAMEWIN_Handle FRAMEWIN_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void FRAMEWIN_Callback(WM_MESSAGE * pMsg);






 
GUI_HWIN FRAMEWIN_AddButton     (FRAMEWIN_Handle hObj, int Flags, int Off, int Id);
GUI_HWIN FRAMEWIN_AddCloseButton(FRAMEWIN_Handle hObj, int Flags, int Off);
GUI_HWIN FRAMEWIN_AddMaxButton  (FRAMEWIN_Handle hObj, int Flags, int Off);
void    FRAMEWIN_AddMenu       (FRAMEWIN_Handle hObj, GUI_HWIN hMenu);
GUI_HWIN FRAMEWIN_AddMinButton  (FRAMEWIN_Handle hObj, int Flags, int Off);
void    FRAMEWIN_Minimize      (FRAMEWIN_Handle hObj);
void    FRAMEWIN_Maximize      (FRAMEWIN_Handle hObj);
void    FRAMEWIN_Restore       (FRAMEWIN_Handle hObj);
void    FRAMEWIN_SetActive     (FRAMEWIN_Handle hObj, int State);
void    FRAMEWIN_SetBarColor   (FRAMEWIN_Handle hObj, unsigned Index, GUI_COLOR Color);
void    FRAMEWIN_SetBorderSize (FRAMEWIN_Handle hObj, unsigned Size);
void    FRAMEWIN_SetClientColor(FRAMEWIN_Handle hObj, GUI_COLOR Color);
void    FRAMEWIN_SetFont       (FRAMEWIN_Handle hObj, const GUI_FONT * pFont);
void    FRAMEWIN_SetMoveable   (FRAMEWIN_Handle hObj, int State);
void    FRAMEWIN_SetOwnerDraw  (FRAMEWIN_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawItem);
void    FRAMEWIN_SetResizeable (FRAMEWIN_Handle hObj, int State);
void    FRAMEWIN_SetText       (FRAMEWIN_Handle hObj, const char* s);
void    FRAMEWIN_SetTextAlign  (FRAMEWIN_Handle hObj, int Align);
void    FRAMEWIN_SetTextColor  (FRAMEWIN_Handle hObj, GUI_COLOR Color);
void    FRAMEWIN_SetTextColorEx(FRAMEWIN_Handle hObj, unsigned Index, GUI_COLOR Color);
void    FRAMEWIN_SetTitleVis   (FRAMEWIN_Handle hObj, int Show);
int     FRAMEWIN_SetTitleHeight(FRAMEWIN_Handle hObj, int Height);
int     FRAMEWIN_SetUserData   (FRAMEWIN_Handle hObj, const void * pSrc, int NumBytes);






 
void FRAMEWIN_GetSkinFlexProps     (FRAMEWIN_SKINFLEX_PROPS * pProps, int Index);
void FRAMEWIN_SetSkinClassic       (FRAMEWIN_Handle hObj);
void FRAMEWIN_SetSkin              (FRAMEWIN_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  FRAMEWIN_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void FRAMEWIN_SetSkinFlexProps     (const FRAMEWIN_SKINFLEX_PROPS * pProps, int Index);
void FRAMEWIN_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * FRAMEWIN_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
const GUI_FONT * FRAMEWIN_GetFont(FRAMEWIN_Handle hObj);

int       FRAMEWIN_GetActive      (FRAMEWIN_Handle hObj);
int       FRAMEWIN_GetTitleHeight (FRAMEWIN_Handle hObj);
GUI_COLOR FRAMEWIN_GetBarColor    (FRAMEWIN_Handle hObj, unsigned Index);
int       FRAMEWIN_GetBorderSize  (FRAMEWIN_Handle hObj);
int       FRAMEWIN_GetBorderSizeEx(FRAMEWIN_Handle hObj, unsigned Edge);
void      FRAMEWIN_GetText        (FRAMEWIN_Handle hObj, char * pBuffer, int MaxLen);
int       FRAMEWIN_GetTextAlign   (FRAMEWIN_Handle hObj);
int       FRAMEWIN_GetUserData    (FRAMEWIN_Handle hObj, void * pDest, int NumBytes);
int       FRAMEWIN_IsMinimized    (FRAMEWIN_Handle hObj);
int       FRAMEWIN_IsMaximized    (FRAMEWIN_Handle hObj);






 
GUI_COLOR        FRAMEWIN_GetDefaultBarColor   (unsigned Index);
int              FRAMEWIN_GetDefaultBorderSize (void);
int              FRAMEWIN_GetDefaultTitleHeight(void);
GUI_COLOR        FRAMEWIN_GetDefaultClientColor(void);
const GUI_FONT * FRAMEWIN_GetDefaultFont       (void);
GUI_COLOR        FRAMEWIN_GetDefaultTextColor  (unsigned Index);
int              FRAMEWIN_OwnerDraw            (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void             FRAMEWIN_SetDefaultBarColor   (unsigned Index, GUI_COLOR Color);
void             FRAMEWIN_SetDefaultBorderSize (int DefaultBorderSize);
void             FRAMEWIN_SetDefaultTitleHeight(int DefaultTitleHeight);
void             FRAMEWIN_SetDefaultClientColor(GUI_COLOR Color);
void             FRAMEWIN_SetDefaultFont       (const GUI_FONT * pFont);
int              FRAMEWIN_SetDefaultTextAlign  (int TextAlign);
void             FRAMEWIN_SetDefaultTextColor  (unsigned Index, GUI_COLOR Color);






 
#line 266 "..\\STemWin\\inc\\FRAMEWIN.h"








 
#line 66 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\GRAPH.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\GRAPH.h"
#line 59 "..\\STemWin\\inc\\GRAPH.h"
#line 60 "..\\STemWin\\inc\\GRAPH.h"












 




































 
typedef signed long GRAPH_Handle;
typedef signed long GRAPH_DATA_Handle;
typedef signed long GRAPH_SCALE_Handle;






 

GRAPH_Handle GRAPH_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
GRAPH_Handle GRAPH_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
GRAPH_Handle GRAPH_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);

GRAPH_DATA_Handle  GRAPH_DATA_XY_Create(GUI_COLOR Color, unsigned MaxNumItems, GUI_POINT * pData, unsigned NumItems);
GRAPH_DATA_Handle  GRAPH_DATA_YT_Create(GUI_COLOR Color, unsigned MaxNumItems, signed short * pData, unsigned NumItems);
GRAPH_SCALE_Handle GRAPH_SCALE_Create  (int Pos, int TextAlign, unsigned Flags, unsigned TickDist);







 
void GRAPH_Callback(WM_MESSAGE * pMsg);






 
void      GRAPH_AttachData             (GRAPH_Handle hObj, GRAPH_DATA_Handle hData);
void      GRAPH_AttachScale            (GRAPH_Handle hObj, GRAPH_SCALE_Handle hScale);
void      GRAPH_DetachData             (GRAPH_Handle hObj, GRAPH_DATA_Handle hData);
void      GRAPH_DetachScale            (GRAPH_Handle hObj, GRAPH_SCALE_Handle hScale);
signed long       GRAPH_GetScrollValue         (GRAPH_Handle hObj, unsigned char Coord);
int       GRAPH_GetUserData            (GRAPH_Handle hObj, void * pDest, int NumBytes);
void      GRAPH_SetAutoScrollbar       (GRAPH_Handle hObj, unsigned char Coord, unsigned char OnOff);
void      GRAPH_SetBorder              (GRAPH_Handle hObj, unsigned BorderL, unsigned BorderT, unsigned BorderR, unsigned BorderB);
GUI_COLOR GRAPH_SetColor               (GRAPH_Handle hObj, GUI_COLOR Color, unsigned Index);
unsigned  GRAPH_SetGridFixedX          (GRAPH_Handle hObj, unsigned OnOff);
unsigned  GRAPH_SetGridOffY            (GRAPH_Handle hObj, unsigned Value);
unsigned  GRAPH_SetGridVis             (GRAPH_Handle hObj, unsigned OnOff);
unsigned  GRAPH_SetGridDistX           (GRAPH_Handle hObj, unsigned Value);
unsigned  GRAPH_SetGridDistY           (GRAPH_Handle hObj, unsigned Value);
unsigned char        GRAPH_SetLineStyleH          (GRAPH_Handle hObj, unsigned char Value);
unsigned char        GRAPH_SetLineStyleV          (GRAPH_Handle hObj, unsigned char Value);
void      GRAPH_SetLineStyle           (GRAPH_Handle hObj, unsigned char Value);
void      GRAPH_SetScrollValue         (GRAPH_Handle hObj, unsigned char Coord, unsigned long Value);
unsigned  GRAPH_SetVSizeX              (GRAPH_Handle hObj, unsigned Value);
unsigned  GRAPH_SetVSizeY              (GRAPH_Handle hObj, unsigned Value);
int       GRAPH_SetUserData            (GRAPH_Handle hObj, const void * pSrc, int NumBytes);
void      GRAPH_SetUserDraw            (GRAPH_Handle hObj, void (* pOwnerDraw)(GUI_HWIN, int));

void      GRAPH_DATA_YT_AddValue       (GRAPH_DATA_Handle hDataObj, signed short Value);
void      GRAPH_DATA_YT_Clear          (GRAPH_DATA_Handle hDataObj);
void      GRAPH_DATA_YT_Delete         (GRAPH_DATA_Handle hDataObj);
void      GRAPH_DATA_YT_SetAlign       (GRAPH_DATA_Handle hDataObj, int Align);
void      GRAPH_DATA_YT_SetOffY        (GRAPH_DATA_Handle hDataObj, int Off);
void      GRAPH_DATA_YT_MirrorX        (GRAPH_DATA_Handle hDataObj, int OnOff);

void      GRAPH_DATA_XY_AddPoint       (GRAPH_DATA_Handle hDataObj, GUI_POINT * pPoint);
void      GRAPH_DATA_XY_Clear          (GRAPH_DATA_Handle hDataObj);
void      GRAPH_DATA_XY_Delete         (GRAPH_DATA_Handle hDataObj);
unsigned  GRAPH_DATA_XY_GetLineVis     (GRAPH_DATA_Handle hDataObj);
unsigned  GRAPH_DATA_XY_GetPointVis    (GRAPH_DATA_Handle hDataObj);
void      GRAPH_DATA_XY_SetLineStyle   (GRAPH_DATA_Handle hDataObj, unsigned char LineStyle);
unsigned  GRAPH_DATA_XY_SetLineVis     (GRAPH_DATA_Handle hDataObj, unsigned OnOff);
void      GRAPH_DATA_XY_SetOffX        (GRAPH_DATA_Handle hDataObj, int Off);
void      GRAPH_DATA_XY_SetOffY        (GRAPH_DATA_Handle hDataObj, int Off);
void      GRAPH_DATA_XY_SetPenSize     (GRAPH_DATA_Handle hDataObj, unsigned char PenSize);
void      GRAPH_DATA_XY_SetPointSize   (GRAPH_DATA_Handle hDataObj, unsigned PointSize);
unsigned  GRAPH_DATA_XY_SetPointVis    (GRAPH_DATA_Handle hDataObj, unsigned OnOff);
void      GRAPH_DATA_XY_SetOwnerDraw   (GRAPH_DATA_Handle hDataObj, WIDGET_DRAW_ITEM_FUNC * pOwnerDraw);

void             GRAPH_SCALE_Delete      (GRAPH_SCALE_Handle hScaleObj);
float            GRAPH_SCALE_SetFactor   (GRAPH_SCALE_Handle hScaleObj, float Factor);
const GUI_FONT * GRAPH_SCALE_SetFont     (GRAPH_SCALE_Handle hScaleObj, const GUI_FONT * pFont);
int              GRAPH_SCALE_SetNumDecs  (GRAPH_SCALE_Handle hScaleObj, int NumDecs);
int              GRAPH_SCALE_SetOff      (GRAPH_SCALE_Handle hScaleObj, int Off);
int              GRAPH_SCALE_SetPos      (GRAPH_SCALE_Handle hScaleObj, int Pos);
GUI_COLOR        GRAPH_SCALE_SetTextColor(GRAPH_SCALE_Handle hScaleObj, GUI_COLOR Color);
unsigned         GRAPH_SCALE_SetTickDist (GRAPH_SCALE_Handle hScaleObj, unsigned Value);








 
#line 67 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\HEADER.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\HEADER.h"
#line 59 "..\\STemWin\\inc\\HEADER.h"
#line 60 "..\\STemWin\\inc\\HEADER.h"












 

typedef signed long HEADER_Handle;

typedef struct {
  GUI_COLOR aColorFrame[2];
  GUI_COLOR aColorUpper[2];
  GUI_COLOR aColorLower[2];
  GUI_COLOR ColorArrow;
} HEADER_SKINFLEX_PROPS;






 

HEADER_Handle HEADER_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int SpecialFlags);
HEADER_Handle HEADER_CreateAttached(GUI_HWIN hParent, int Id, int SpecialFlags);
HEADER_Handle HEADER_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
HEADER_Handle HEADER_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
HEADER_Handle HEADER_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void HEADER_Callback(WM_MESSAGE * pMsg);






 
 
GUI_COLOR          HEADER_SetDefaultArrowColor(GUI_COLOR Color);
GUI_COLOR          HEADER_SetDefaultBkColor   (GUI_COLOR Color);
const GUI_CURSOR * HEADER_SetDefaultCursor    (const GUI_CURSOR * pCursor);
const GUI_FONT *   HEADER_SetDefaultFont      (const GUI_FONT * pFont);
int                HEADER_SetDefaultBorderH   (int Spacing);
int                HEADER_SetDefaultBorderV   (int Spacing);
GUI_COLOR          HEADER_SetDefaultTextColor (GUI_COLOR Color);

 
GUI_COLOR          HEADER_GetDefaultArrowColor(void);
GUI_COLOR          HEADER_GetDefaultBkColor   (void);
const GUI_CURSOR * HEADER_GetDefaultCursor    (void);
const GUI_FONT *   HEADER_GetDefaultFont      (void);
int                HEADER_GetDefaultBorderH   (void);
int                HEADER_GetDefaultBorderV   (void);
GUI_COLOR          HEADER_GetDefaultTextColor (void);






 
void      HEADER_AddItem            (HEADER_Handle hObj, int Width, const char * s, int Align);
void      HEADER_DeleteItem         (HEADER_Handle hObj, unsigned Index);
GUI_COLOR HEADER_GetArrowColor      (HEADER_Handle hObj);
GUI_COLOR HEADER_GetBkColor         (HEADER_Handle hObj);
int       HEADER_GetHeight          (HEADER_Handle hObj);
int       HEADER_GetItemWidth       (HEADER_Handle hObj, unsigned int Index);
int       HEADER_GetNumItems        (HEADER_Handle hObj);
int       HEADER_GetSel             (HEADER_Handle hObj);
GUI_COLOR HEADER_GetTextColor       (HEADER_Handle hObj);
int       HEADER_GetUserData        (HEADER_Handle hObj, void * pDest, int NumBytes);
void      HEADER_SetArrowColor      (HEADER_Handle hObj, GUI_COLOR Color);
void      HEADER_SetBitmap          (HEADER_Handle hObj, unsigned int Index, const GUI_BITMAP * pBitmap);
void      HEADER_SetBitmapEx        (HEADER_Handle hObj, unsigned int Index, const GUI_BITMAP * pBitmap, int x, int y);
void      HEADER_SetBkColor         (HEADER_Handle hObj, GUI_COLOR Color);
void      HEADER_SetBMP             (HEADER_Handle hObj, unsigned int Index, const void * pBitmap);
void      HEADER_SetBMPEx           (HEADER_Handle hObj, unsigned int Index, const void * pBitmap, int x, int y);
void      HEADER_SetDirIndicator    (HEADER_Handle hObj, int Column, int Reverse);  
void      HEADER_SetDragLimit       (HEADER_Handle hObj, unsigned DragLimit);
unsigned  HEADER_SetFixed           (HEADER_Handle hObj, unsigned Fixed);
void      HEADER_SetFont            (HEADER_Handle hObj, const GUI_FONT * pFont);
void      HEADER_SetHeight          (HEADER_Handle hObj, int Height);
void      HEADER_SetTextAlign       (HEADER_Handle hObj, unsigned int Index, int Align);
void      HEADER_SetItemText        (HEADER_Handle hObj, unsigned int Index, const char * s);
void      HEADER_SetItemWidth       (HEADER_Handle hObj, unsigned int Index, int Width);
void      HEADER_SetScrollPos       (HEADER_Handle hObj, int ScrollPos);
void      HEADER_SetStreamedBitmap  (HEADER_Handle hObj, unsigned int Index, const GUI_BITMAP_STREAM * pBitmap);
void      HEADER_SetStreamedBitmapEx(HEADER_Handle hObj, unsigned int Index, const GUI_BITMAP_STREAM * pBitmap, int x, int y);
void      HEADER_SetTextColor       (HEADER_Handle hObj, GUI_COLOR Color);
int       HEADER_SetUserData        (HEADER_Handle hObj, const void * pSrc, int NumBytes);






 
void HEADER_GetSkinFlexProps     (HEADER_SKINFLEX_PROPS * pProps, int Index);
void HEADER_SetSkinClassic       (HEADER_Handle hObj);
void HEADER_SetSkin              (HEADER_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  HEADER_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void HEADER_SetSkinFlexProps     (const HEADER_SKINFLEX_PROPS * pProps, int Index);
void HEADER_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * HEADER_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 

#line 198 "..\\STemWin\\inc\\HEADER.h"








 
#line 68 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\ICONVIEW.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\ICONVIEW.h"
#line 59 "..\\STemWin\\inc\\ICONVIEW.h"
#line 60 "..\\STemWin\\inc\\ICONVIEW.h"












 

































 
typedef signed long ICONVIEW_Handle;






 
ICONVIEW_Handle ICONVIEW_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int xSizeItems, int ySizeItems);
ICONVIEW_Handle ICONVIEW_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int xSizeItems, int ySizeItems, int NumExtraBytes);
ICONVIEW_Handle ICONVIEW_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);

int  ICONVIEW_AddBitmapItem           (ICONVIEW_Handle hObj, const GUI_BITMAP * pBitmap, const char * pText);
int  ICONVIEW_AddBMPItem              (ICONVIEW_Handle hObj, const unsigned char * pBMP, const char * pText);
int  ICONVIEW_AddBMPItemEx            (ICONVIEW_Handle hObj, const void * pBMP, GUI_GET_DATA_FUNC * pfGetData, const char * pText);
int  ICONVIEW_AddStreamedBitmapItem   (ICONVIEW_Handle hObj, const void * pStreamedBitmap, const char * pText);
void ICONVIEW_DeleteItem              (ICONVIEW_Handle hObj, unsigned Index);
void ICONVIEW_EnableStreamAuto        (void);
unsigned long  ICONVIEW_GetItemUserData         (ICONVIEW_Handle hObj, int Index);
int  ICONVIEW_GetNumItems             (ICONVIEW_Handle hObj);
int  ICONVIEW_GetItemText             (ICONVIEW_Handle hObj, int Index, char * pBuffer, int MaxSize);
int  ICONVIEW_GetSel                  (ICONVIEW_Handle hObj);
int  ICONVIEW_GetUserData             (ICONVIEW_Handle hObj, void * pDest, int NumBytes);
int  ICONVIEW_InsertBitmapItem        (ICONVIEW_Handle hObj, const GUI_BITMAP * pBitmap, const char * pText, int Index);
int  ICONVIEW_InsertBMPItem           (ICONVIEW_Handle hObj, const unsigned char * pBMP, const char * pText, int Index);
int  ICONVIEW_InsertBMPItemEx         (ICONVIEW_Handle hObj, const void * pBMP, GUI_GET_DATA_FUNC * pfGetData, const char * pText, int Index);
int  ICONVIEW_InsertStreamedBitmapItem(ICONVIEW_Handle hObj, const void * pStreamedBitmap, const char * pText, int Index);
int  ICONVIEW_SetBitmapItem           (ICONVIEW_Handle hObj, int Index, const GUI_BITMAP * pBitmap);
void ICONVIEW_SetBkColor              (ICONVIEW_Handle hObj, int Index, GUI_COLOR Color);
int  ICONVIEW_SetBMPItem              (ICONVIEW_Handle hObj, const unsigned char * pBMP, int Index);
int  ICONVIEW_SetBMPItemEx            (ICONVIEW_Handle hObj, const void * pBMP, GUI_GET_DATA_FUNC * pfGetData, int Index);
void ICONVIEW_SetFont                 (ICONVIEW_Handle hObj, const GUI_FONT * pFont);
void ICONVIEW_SetFrame                (ICONVIEW_Handle hObj, int Coord, int Value);
void ICONVIEW_SetItemText             (ICONVIEW_Handle hObj, int Index, const char * pText);
void ICONVIEW_SetItemUserData         (ICONVIEW_Handle hObj, int Index, unsigned long UserData);
void ICONVIEW_SetSel                  (ICONVIEW_Handle hObj, int Sel);
void ICONVIEW_SetSpace                (ICONVIEW_Handle hObj, int Coord, int Value);
int  ICONVIEW_SetStreamedBitmapItem   (ICONVIEW_Handle hObj, int Index, const void * pStreamedBitmap);
void ICONVIEW_SetIconAlign            (ICONVIEW_Handle hObj, int IconAlign);
void ICONVIEW_SetTextAlign            (ICONVIEW_Handle hObj, int TextAlign);
void ICONVIEW_SetTextColor            (ICONVIEW_Handle hObj, int Index, GUI_COLOR Color);
int  ICONVIEW_SetUserData             (ICONVIEW_Handle hObj, const void * pSrc, int NumBytes);
void ICONVIEW_SetWrapMode             (ICONVIEW_Handle hObj, GUI_WRAPMODE WrapMode);

void ICONVIEW_Callback(WM_MESSAGE * pMsg);








 
#line 69 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\IMAGE.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\IMAGE.h"
#line 59 "..\\STemWin\\inc\\IMAGE.h"
#line 60 "..\\STemWin\\inc\\IMAGE.h"












 











 
typedef signed long IMAGE_Handle;






 
IMAGE_Handle IMAGE_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
IMAGE_Handle IMAGE_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
IMAGE_Handle IMAGE_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);

void IMAGE_Callback(WM_MESSAGE * pMsg);






 
int  IMAGE_GetUserData(IMAGE_Handle hObj, void * pDest, int NumBytes);
void IMAGE_SetBitmap  (IMAGE_Handle hWin, const GUI_BITMAP * pBitmap);
void IMAGE_SetBMP     (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetBMPEx   (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
void IMAGE_SetDTA     (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetDTAEx   (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
void IMAGE_SetGIF     (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetGIFEx   (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
void IMAGE_SetJPEG    (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetJPEGEx  (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
void IMAGE_SetPNG     (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetPNGEx   (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
int  IMAGE_SetUserData(IMAGE_Handle hObj, const void * pSrc, int NumBytes);









 
#line 70 "..\\STemWin\\inc\\DIALOG.h"
#line 71 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\LISTVIEW.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LISTVIEW.h"
#line 59 "..\\STemWin\\inc\\LISTVIEW.h"
#line 60 "..\\STemWin\\inc\\LISTVIEW.h"
#line 61 "..\\STemWin\\inc\\LISTVIEW.h"










 





 








 











 
typedef signed long LISTVIEW_Handle;






 
LISTVIEW_Handle LISTVIEW_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int SpecialFlags);
LISTVIEW_Handle LISTVIEW_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
LISTVIEW_Handle LISTVIEW_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
LISTVIEW_Handle LISTVIEW_CreateAttached(GUI_HWIN hParent, int Id, int SpecialFlags);
LISTVIEW_Handle LISTVIEW_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void LISTVIEW_Callback(WM_MESSAGE * pMsg);






 
int              LISTVIEW_AddColumn           (LISTVIEW_Handle hObj, int Width, const char * s, int Align);
int              LISTVIEW_AddRow              (LISTVIEW_Handle hObj, const GUI_ConstString * ppText);
int              LISTVIEW_CompareText         (const void * p0, const void * p1);
int              LISTVIEW_CompareDec          (const void * p0, const void * p1);
void             LISTVIEW_DecSel              (LISTVIEW_Handle hObj);
void             LISTVIEW_DeleteAllRows       (LISTVIEW_Handle hObj);
void             LISTVIEW_DeleteColumn        (LISTVIEW_Handle hObj, unsigned Index);
void             LISTVIEW_DeleteRow           (LISTVIEW_Handle hObj, unsigned Index);
void             LISTVIEW_DeleteRowSorted     (LISTVIEW_Handle hObj, int Row);
void             LISTVIEW_DisableRow          (LISTVIEW_Handle hObj, unsigned Row);
void             LISTVIEW_DisableSort         (LISTVIEW_Handle hObj);
void             LISTVIEW_EnableCellSelect    (LISTVIEW_Handle hObj, unsigned OnOff);  
void             LISTVIEW_EnableRow           (LISTVIEW_Handle hObj, unsigned Row);
void             LISTVIEW_EnableSort          (LISTVIEW_Handle hObj);
GUI_COLOR        LISTVIEW_GetBkColor          (LISTVIEW_Handle hObj, unsigned Index);
const GUI_FONT * LISTVIEW_GetFont             (LISTVIEW_Handle hObj);
HEADER_Handle    LISTVIEW_GetHeader           (LISTVIEW_Handle hObj);
void             LISTVIEW_GetItemRect         (LISTVIEW_Handle hObj, unsigned long Col, unsigned long Row, GUI_RECT * pRect);
void             LISTVIEW_GetItemText         (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, char * pBuffer, unsigned MaxSize);
unsigned         LISTVIEW_GetItemTextLen      (LISTVIEW_Handle hObj, unsigned Column, unsigned Row);
void             LISTVIEW_GetItemTextSorted   (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, char * pBuffer, unsigned MaxSize);
unsigned         LISTVIEW_GetLBorder          (LISTVIEW_Handle hObj);
unsigned         LISTVIEW_GetNumColumns       (LISTVIEW_Handle hObj);
unsigned         LISTVIEW_GetNumRows          (LISTVIEW_Handle hObj);
unsigned         LISTVIEW_GetRBorder          (LISTVIEW_Handle hObj);
int              LISTVIEW_GetSel              (LISTVIEW_Handle hObj);
int              LISTVIEW_GetSelCol           (LISTVIEW_Handle hObj);
int              LISTVIEW_GetSelUnsorted      (LISTVIEW_Handle hObj);
int              LISTVIEW_GetTextAlign        (LISTVIEW_Handle hObj, unsigned ColIndex);
GUI_COLOR        LISTVIEW_GetTextColor        (LISTVIEW_Handle hObj, unsigned Index);
int              LISTVIEW_GetUserData         (LISTVIEW_Handle hObj, void * pDest, int NumBytes);
unsigned long              LISTVIEW_GetUserDataRow      (LISTVIEW_Handle hObj, unsigned Row);
GUI_WRAPMODE     LISTVIEW_GetWrapMode         (LISTVIEW_Handle hObj);
void             LISTVIEW_IncSel              (LISTVIEW_Handle hObj);
int              LISTVIEW_InsertRow           (LISTVIEW_Handle hObj, unsigned Index, const GUI_ConstString * ppText);
int              LISTVIEW_OwnerDraw           (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
unsigned         LISTVIEW_RowIsDisabled       (LISTVIEW_Handle hObj, unsigned Row);
void             LISTVIEW_SetAutoScrollH      (LISTVIEW_Handle hObj, int OnOff);
void             LISTVIEW_SetAutoScrollV      (LISTVIEW_Handle hObj, int OnOff);
void             LISTVIEW_SetItemBitmap       (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, int xOff, int yOff, const GUI_BITMAP * pBitmap);
void             LISTVIEW_SetBkColor          (LISTVIEW_Handle hObj, unsigned int Index, GUI_COLOR Color);
void             LISTVIEW_SetColumnWidth      (LISTVIEW_Handle hObj, unsigned int Index, int Width);
void             LISTVIEW_SetCompareFunc      (LISTVIEW_Handle hObj, unsigned Column, int (* fpCompare)(const void * p0, const void * p1));
unsigned         LISTVIEW_SetFixed            (LISTVIEW_Handle hObj, unsigned Fixed);
void             LISTVIEW_SetFont             (LISTVIEW_Handle hObj, const GUI_FONT * pFont);
int              LISTVIEW_SetGridVis          (LISTVIEW_Handle hObj, int Show);
void             LISTVIEW_SetHeaderHeight     (LISTVIEW_Handle hObj, unsigned HeaderHeight);
void             LISTVIEW_SetItemBkColor      (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, unsigned int Index, GUI_COLOR Color);
void             LISTVIEW_SetItemText         (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, const char * s);
void             LISTVIEW_SetItemTextColor    (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, unsigned int Index, GUI_COLOR Color);
void             LISTVIEW_SetItemTextSorted   (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, const char * pText);
void             LISTVIEW_SetLBorder          (LISTVIEW_Handle hObj, unsigned BorderSize);
void             LISTVIEW_SetOwnerDraw        (LISTVIEW_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawItem);
void             LISTVIEW_SetRBorder          (LISTVIEW_Handle hObj, unsigned BorderSize);
unsigned         LISTVIEW_SetRowHeight        (LISTVIEW_Handle hObj, unsigned RowHeight);
void             LISTVIEW_SetSel              (LISTVIEW_Handle hObj, int Sel);
void             LISTVIEW_SetSelCol           (LISTVIEW_Handle hObj, int NewCol);
void             LISTVIEW_SetSelUnsorted      (LISTVIEW_Handle hObj, int Sel);
unsigned         LISTVIEW_SetSort             (LISTVIEW_Handle hObj, unsigned Column, unsigned Reverse);
void             LISTVIEW_SetTextAlign        (LISTVIEW_Handle hObj, unsigned int Index, int Align);
void             LISTVIEW_SetTextColor        (LISTVIEW_Handle hObj, unsigned int Index, GUI_COLOR Color);
int              LISTVIEW_SetUserData         (LISTVIEW_Handle hObj, const void * pSrc, int NumBytes);
void             LISTVIEW_SetUserDataRow      (LISTVIEW_Handle hObj, unsigned Row, unsigned long UserData);
void             LISTVIEW_SetWrapMode         (LISTVIEW_Handle hObj, GUI_WRAPMODE WrapMode);






 

GUI_COLOR        LISTVIEW_SetDefaultBkColor  (unsigned  Index, GUI_COLOR Color);
const GUI_FONT * LISTVIEW_SetDefaultFont     (const GUI_FONT * pFont);
GUI_COLOR        LISTVIEW_SetDefaultGridColor(GUI_COLOR Color);
GUI_COLOR        LISTVIEW_SetDefaultTextColor(unsigned  Index, GUI_COLOR Color);








 
#line 72 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\LISTWHEEL.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LISTWHEEL.h"
#line 59 "..\\STemWin\\inc\\LISTWHEEL.h"
#line 60 "..\\STemWin\\inc\\LISTWHEEL.h"












 








 
typedef signed long LISTWHEEL_Handle;






 





 
LISTWHEEL_Handle LISTWHEEL_Create        (const GUI_ConstString * ppText, int x0, int y0, int xSize, int ySize, int Flags);
LISTWHEEL_Handle LISTWHEEL_CreateAsChild (const GUI_ConstString * ppText, GUI_HWIN hWinParent, int x0, int y0, int xSize, int ySize, int Flags);
LISTWHEEL_Handle LISTWHEEL_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);
LISTWHEEL_Handle LISTWHEEL_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent,
                                          int WinFlags, int ExFlags, int Id, const GUI_ConstString * ppText);
LISTWHEEL_Handle LISTWHEEL_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent,
                                          int WinFlags, int ExFlags, int Id, const GUI_ConstString * ppText, int NumExtraBytes);







 
void LISTWHEEL_Callback(WM_MESSAGE * pMsg);






 
void      LISTWHEEL_AddString      (LISTWHEEL_Handle hObj, const char * s);
void *    LISTWHEEL_GetItemData    (LISTWHEEL_Handle hObj, unsigned Index);  
void      LISTWHEEL_GetItemText    (LISTWHEEL_Handle hObj, unsigned Index, char * pBuffer, int MaxSize);
int       LISTWHEEL_GetItemFromPos (LISTWHEEL_Handle hObj, int yPos);
int       LISTWHEEL_GetLBorder     (LISTWHEEL_Handle hObj);
unsigned  LISTWHEEL_GetLineHeight  (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetNumItems    (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetPos         (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetRBorder     (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetSel         (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetSnapPosition(LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetTextAlign   (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetUserData    (LISTWHEEL_Handle hObj, void * pDest, int NumBytes);
int       LISTWHEEL_IsMoving       (LISTWHEEL_Handle hObj);
void      LISTWHEEL_MoveToPos      (LISTWHEEL_Handle hObj, unsigned int Index);
int       LISTWHEEL_OwnerDraw      (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void      LISTWHEEL_SetBkColor     (LISTWHEEL_Handle hObj, unsigned int Index, GUI_COLOR Color);
void      LISTWHEEL_SetDeceleration(LISTWHEEL_Handle hObj, unsigned Deceleration);
void      LISTWHEEL_SetFont        (LISTWHEEL_Handle hObj, const GUI_FONT * pFont);
void      LISTWHEEL_SetItemData    (LISTWHEEL_Handle hObj, unsigned Index, void * pData);  
void      LISTWHEEL_SetLBorder     (LISTWHEEL_Handle hObj, unsigned BorderSize);
void      LISTWHEEL_SetLineHeight  (LISTWHEEL_Handle hObj, unsigned LineHeight);
void      LISTWHEEL_SetOwnerDraw   (LISTWHEEL_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfOwnerDraw);
void      LISTWHEEL_SetPos         (LISTWHEEL_Handle hObj, unsigned int Index);
void      LISTWHEEL_SetRBorder     (LISTWHEEL_Handle hObj, unsigned BorderSize);
void      LISTWHEEL_SetSel         (LISTWHEEL_Handle hObj, int Sel);
void      LISTWHEEL_SetSnapPosition(LISTWHEEL_Handle hObj, int SnapPosition);
void      LISTWHEEL_SetText        (LISTWHEEL_Handle hObj, const GUI_ConstString * ppText);
void      LISTWHEEL_SetTextAlign   (LISTWHEEL_Handle hObj, int Align);
void      LISTWHEEL_SetTextColor   (LISTWHEEL_Handle hObj, unsigned int Index, GUI_COLOR Color);
void      LISTWHEEL_SetTimerPeriod (LISTWHEEL_Handle hObj, int TimerPeriod);
int       LISTWHEEL_SetUserData    (LISTWHEEL_Handle hObj, const void * pSrc, int NumBytes);
void      LISTWHEEL_SetVelocity    (LISTWHEEL_Handle hObj, int Velocity);

const GUI_FONT * LISTWHEEL_GetFont(LISTWHEEL_Handle hObj);








 
#line 73 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\MENU.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\MENU.h"
#line 59 "..\\STemWin\\inc\\MENU.h"
#line 60 "..\\STemWin\\inc\\MENU.h"












 





 









 






 









 








 
#line 121 "..\\STemWin\\inc\\MENU.h"
                                            






 











 

typedef signed long MENU_Handle;

typedef struct {
  
  
  
  GUI_COLOR aBkColorH[2];
  GUI_COLOR BkColorV;
  GUI_COLOR FrameColorH;
  GUI_COLOR FrameColorV;
  
  
  
  GUI_COLOR aSelColorH[2];
  GUI_COLOR aSelColorV[2];
  GUI_COLOR FrameColorSelH;
  GUI_COLOR FrameColorSelV;
  
  
  
  GUI_COLOR aSepColorH[2];
  GUI_COLOR aSepColorV[2];
  
  
  
  GUI_COLOR ArrowColor;
  
  
  
  GUI_COLOR TextColor;
} MENU_SKINFLEX_PROPS;




 
typedef struct {
  unsigned short MsgType;
  unsigned short ItemId;
} MENU_MSG_DATA;




 
typedef struct {
  const char  * pText;
  unsigned short           Id;
  unsigned short           Flags;
  MENU_Handle   hSubmenu;
} MENU_ITEM_DATA;






 
MENU_Handle MENU_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);
MENU_Handle MENU_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
MENU_Handle MENU_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);







 
void MENU_Callback(WM_MESSAGE * pMsg);






 
void      MENU_AddItem      (MENU_Handle hObj, const MENU_ITEM_DATA * pItemData);
void      MENU_Attach       (MENU_Handle hObj, GUI_HWIN hDestWin, int x, int y, int xSize, int ySize, int Flags);
void      MENU_DeleteItem   (MENU_Handle hObj, unsigned short ItemId);
void      MENU_DisableItem  (MENU_Handle hObj, unsigned short ItemId);
void      MENU_EnableItem   (MENU_Handle hObj, unsigned short ItemId);
void      MENU_GetItem      (MENU_Handle hObj, unsigned short ItemId, MENU_ITEM_DATA * pItemData);
void      MENU_GetItemText  (MENU_Handle hObj, unsigned short ItemId, char * pBuffer, unsigned BufferSize);
unsigned  MENU_GetNumItems  (MENU_Handle hObj);
GUI_HWIN   MENU_GetOwner     (MENU_Handle hObj);
int       MENU_GetUserData  (MENU_Handle hObj, void * pDest, int NumBytes);
void      MENU_InsertItem   (MENU_Handle hObj, unsigned short ItemId, const MENU_ITEM_DATA * pItemData);
void      MENU_Popup        (MENU_Handle hObj, GUI_HWIN hDestWin, int x, int y, int xSize, int ySize, int Flags);
void      MENU_SetBkColor   (MENU_Handle hObj, unsigned ColorIndex, GUI_COLOR Color);
void      MENU_SetBorderSize(MENU_Handle hObj, unsigned BorderIndex, unsigned char BorderSize);
void      MENU_SetFont      (MENU_Handle hObj, const GUI_FONT * pFont);
void      MENU_SetItem      (MENU_Handle hObj, unsigned short ItemId, const MENU_ITEM_DATA * pItemData);
void      MENU_SetOwner     (MENU_Handle hObj, GUI_HWIN hOwner);
int       MENU_SetSel       (MENU_Handle hObj, int Sel);
void      MENU_SetTextColor (MENU_Handle hObj, unsigned ColorIndex, GUI_COLOR Color);
int       MENU_SetUserData  (MENU_Handle hObj, const void * pSrc, int NumBytes);






 
GUI_COLOR             MENU_GetDefaultTextColor  (unsigned ColorIndex);
GUI_COLOR             MENU_GetDefaultBkColor    (unsigned ColorIndex);
unsigned char                    MENU_GetDefaultBorderSize (unsigned BorderIndex);
const WIDGET_EFFECT * MENU_GetDefaultEffect     (void);
const GUI_FONT      * MENU_GetDefaultFont       (void);
void                  MENU_SetDefaultTextColor  (unsigned ColorIndex, GUI_COLOR Color);
void                  MENU_SetDefaultBkColor    (unsigned ColorIndex, GUI_COLOR Color);
void                  MENU_SetDefaultBorderSize (unsigned BorderIndex, unsigned char BorderSize);
void                  MENU_SetDefaultEffect     (const WIDGET_EFFECT * pEffect);
void                  MENU_SetDefaultFont       (const GUI_FONT * pFont);






 
int                     MENU_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void                    MENU_GetSkinFlexProps     (MENU_SKINFLEX_PROPS * pProps, int Index);
WIDGET_DRAW_ITEM_FUNC * MENU_SetDefaultSkin       (WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
void                    MENU_SetDefaultSkinClassic(void);
void                    MENU_SetSkinClassic       (MENU_Handle hObj);
void                    MENU_SetSkin              (MENU_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
void                    MENU_SetSkinFlexProps     (const MENU_SKINFLEX_PROPS * pProps, int Index);
void                    MENU_SkinEnableArrow      (MENU_Handle hObj, int OnOff);








 
#line 74 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\MULTIEDIT.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\MULTIEDIT.h"
#line 59 "..\\STemWin\\inc\\MULTIEDIT.h"






















 








 

typedef signed long MULTIEDIT_HANDLE;






 
MULTIEDIT_HANDLE MULTIEDIT_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int ExFlags, const char * pText, int MaxLen);
MULTIEDIT_HANDLE MULTIEDIT_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int BufferSize, const char * pText);
MULTIEDIT_HANDLE MULTIEDIT_CreateIndirect(const GUI_WIDGET_CREATE_INFO* pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);
MULTIEDIT_HANDLE MULTIEDIT_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int BufferSize, const char * pText, int NumExtraBytes);







 
void MULTIEDIT_Callback(WM_MESSAGE * pMsg);






 

int  MULTIEDIT_AddKey           (MULTIEDIT_HANDLE hObj, unsigned short Key);
int  MULTIEDIT_AddText          (MULTIEDIT_HANDLE hObj, const char * s);
void MULTIEDIT_EnableBlink      (MULTIEDIT_HANDLE hObj, int Period, int OnOff);
int  MULTIEDIT_GetCursorCharPos (MULTIEDIT_HANDLE hObj);
void MULTIEDIT_GetCursorPixelPos(MULTIEDIT_HANDLE hObj, int * pxPos, int * pyPos);
void MULTIEDIT_GetPrompt        (MULTIEDIT_HANDLE hObj, char* sDest, int MaxNumChars);
int  MULTIEDIT_GetTextSize      (MULTIEDIT_HANDLE hObj);
void MULTIEDIT_GetText          (MULTIEDIT_HANDLE hObj, char* sDest, int MaxNumChars);
int  MULTIEDIT_GetUserData      (MULTIEDIT_HANDLE hObj, void * pDest, int NumBytes);
void MULTIEDIT_SetTextAlign     (MULTIEDIT_HANDLE hObj, int Align);
void MULTIEDIT_SetAutoScrollH   (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetAutoScrollV   (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetBkColor       (MULTIEDIT_HANDLE hObj, unsigned Index, GUI_COLOR color);
void MULTIEDIT_SetCursorCharPos (MULTIEDIT_HANDLE hObj, int x, int y);        
void MULTIEDIT_SetCursorPixelPos(MULTIEDIT_HANDLE hObj, int x, int y);        
void MULTIEDIT_SetCursorOffset  (MULTIEDIT_HANDLE hObj, int Offset);
void MULTIEDIT_SetHBorder       (MULTIEDIT_HANDLE hObj, unsigned HBorder);
void MULTIEDIT_SetFocussable    (MULTIEDIT_HANDLE hObj, int State);
void MULTIEDIT_SetFont          (MULTIEDIT_HANDLE hObj, const GUI_FONT * pFont);
void MULTIEDIT_SetInsertMode    (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetBufferSize    (MULTIEDIT_HANDLE hObj, int BufferSize);
void MULTIEDIT_SetMaxNumChars   (MULTIEDIT_HANDLE hObj, unsigned MaxNumChars);
void MULTIEDIT_SetPrompt        (MULTIEDIT_HANDLE hObj, const char* sPrompt);
void MULTIEDIT_SetReadOnly      (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetPasswordMode  (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetText          (MULTIEDIT_HANDLE hObj, const char* s);
void MULTIEDIT_SetTextColor     (MULTIEDIT_HANDLE hObj, unsigned Index, GUI_COLOR color);
int  MULTIEDIT_SetUserData      (MULTIEDIT_HANDLE hObj, const void * pSrc, int NumBytes);
void MULTIEDIT_SetWrapNone      (MULTIEDIT_HANDLE hObj);
void MULTIEDIT_SetWrapChar      (MULTIEDIT_HANDLE hObj);
void MULTIEDIT_SetWrapWord      (MULTIEDIT_HANDLE hObj);






 











 
#line 75 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\MULTIPAGE.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\MULTIPAGE.h"
#line 1 "..\\STemWin\\inc\\DIALOG.h"
































 


















 
  
#line 112 "..\\STemWin\\inc\\DIALOG.h"

 
#line 59 "..\\STemWin\\inc\\MULTIPAGE.h"












 



 
































 
typedef signed long MULTIPAGE_Handle;

typedef struct {
  GUI_COLOR BkColor;
  GUI_COLOR aBkUpper[2];
  GUI_COLOR aBkLower[2];
  GUI_COLOR FrameColor;
  GUI_COLOR TextColor;
} MULTIPAGE_SKINFLEX_PROPS;

typedef struct {
  unsigned char  SelSideBorderInc;         
  unsigned char  SelTopBorderInc;          
} MULTIPAGE_SKIN_PROPS;

typedef struct {

    tLCD_APIList  * pRotation;

  unsigned          Align;
  int               Sel;
  unsigned short               State;
  unsigned char                FrameFlags;    
  unsigned char                PageStatus;
  GUI_DRAW_HANDLE * pDrawObj;
} MULTIPAGE_SKIN_INFO;






 
MULTIPAGE_Handle MULTIPAGE_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int SpecialFlags);
MULTIPAGE_Handle MULTIPAGE_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
MULTIPAGE_Handle MULTIPAGE_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
MULTIPAGE_Handle MULTIPAGE_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void MULTIPAGE_Callback(WM_MESSAGE * pMsg);






 
void             MULTIPAGE_AddEmptyPage   (MULTIPAGE_Handle hObj, GUI_HWIN hWin ,const char * pText);
void             MULTIPAGE_AddPage        (MULTIPAGE_Handle hObj, GUI_HWIN hWin ,const char * pText);
GUI_HWIN          MULTIPAGE_AttachWindow   (MULTIPAGE_Handle hObj, unsigned Index, GUI_HWIN hWin);
void             MULTIPAGE_DeletePage     (MULTIPAGE_Handle hObj, unsigned Index, int Delete);
void             MULTIPAGE_DisablePage    (MULTIPAGE_Handle hObj, unsigned Index);
void             MULTIPAGE_EnablePage     (MULTIPAGE_Handle hObj, unsigned Index);
void             MULTIPAGE_EnableScrollbar(MULTIPAGE_Handle hObj, unsigned OnOff);
const GUI_FONT * MULTIPAGE_GetFont        (MULTIPAGE_Handle hObj);
int              MULTIPAGE_GetSelection   (MULTIPAGE_Handle hObj);
int              MULTIPAGE_GetPageText    (MULTIPAGE_Handle hObj, unsigned Index, char * pBuffer, int MaxLen);
int              MULTIPAGE_GetUserData    (MULTIPAGE_Handle hObj, void * pDest, int NumBytes);
GUI_HWIN          MULTIPAGE_GetWindow      (MULTIPAGE_Handle hObj, unsigned Index);
int              MULTIPAGE_IsPageEnabled  (MULTIPAGE_Handle hObj, unsigned Index);
void             MULTIPAGE_SelectPage     (MULTIPAGE_Handle hObj, unsigned Index);
void             MULTIPAGE_SetAlign       (MULTIPAGE_Handle hObj, unsigned Align);
int              MULTIPAGE_SetBitmapEx    (MULTIPAGE_Handle hObj, const GUI_BITMAP * pBitmap, int x, int y, int Index, int State);
int              MULTIPAGE_SetBitmap      (MULTIPAGE_Handle hObj, const GUI_BITMAP * pBitmap, int Index, int State);
void             MULTIPAGE_SetBkColor     (MULTIPAGE_Handle hObj, GUI_COLOR Color, unsigned Index);
void             MULTIPAGE_SetFont        (MULTIPAGE_Handle hObj, const GUI_FONT * pFont);
void             MULTIPAGE_SetRotation    (MULTIPAGE_Handle hObj, unsigned Rotation);
void             MULTIPAGE_SetTabWidth    (MULTIPAGE_Handle hObj, int Width, int Index);
void             MULTIPAGE_SetTabHeight   (MULTIPAGE_Handle hObj, int Height);
void             MULTIPAGE_SetTextAlign   (MULTIPAGE_Handle hObj, unsigned Align);
void             MULTIPAGE_SetText        (MULTIPAGE_Handle hObj, const char * pText, unsigned Index);
void             MULTIPAGE_SetTextColor   (MULTIPAGE_Handle hObj, GUI_COLOR Color, unsigned Index);
int              MULTIPAGE_SetUserData    (MULTIPAGE_Handle hObj, const void * pSrc, int NumBytes);






 
unsigned         MULTIPAGE_GetDefaultAlign      (void);
GUI_COLOR        MULTIPAGE_GetDefaultBkColor    (unsigned Index);
const GUI_FONT * MULTIPAGE_GetDefaultFont       (void);
GUI_COLOR        MULTIPAGE_GetDefaultTextColor  (unsigned Index);

void             MULTIPAGE_SetDefaultAlign      (unsigned Align);
void             MULTIPAGE_SetDefaultBkColor    (GUI_COLOR Color, unsigned Index);
void             MULTIPAGE_SetDefaultBorderSizeX(unsigned Size);
void             MULTIPAGE_SetDefaultBorderSizeY(unsigned Size);
void             MULTIPAGE_SetDefaultFont       (const GUI_FONT * pFont);
void             MULTIPAGE_SetDefaultTextColor  (GUI_COLOR Color, unsigned Index);

void             MULTIPAGE_SetEffectColor       (unsigned Index, GUI_COLOR Color);
GUI_COLOR        MULTIPAGE_GetEffectColor       (unsigned Index);
int              MULTIPAGE_GetNumEffectColors   (void);






 
int                     MULTIPAGE_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void                    MULTIPAGE_GetSkinFlexProps     (MULTIPAGE_SKINFLEX_PROPS * pProps, int Index);
WIDGET_DRAW_ITEM_FUNC * MULTIPAGE_SetDefaultSkin       (WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
void                    MULTIPAGE_SetDefaultSkinClassic(void);
void                    MULTIPAGE_SetSkinClassic       (MULTIPAGE_Handle hObj);
void                    MULTIPAGE_SetSkin              (MULTIPAGE_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
void                    MULTIPAGE_SetSkinFlexProps     (const MULTIPAGE_SKINFLEX_PROPS * pProps, int Index);










 
#line 76 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\PROGBAR.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\PROGBAR.h"
#line 59 "..\\STemWin\\inc\\PROGBAR.h"
#line 60 "..\\STemWin\\inc\\PROGBAR.h"












 



 







 








 
typedef signed long PROGBAR_Handle;

typedef struct {
  GUI_COLOR aColorUpperL[2];
  GUI_COLOR aColorLowerL[2];
  GUI_COLOR aColorUpperR[2];
  GUI_COLOR aColorLowerR[2];
  GUI_COLOR ColorFrame;
  GUI_COLOR ColorText;
} PROGBAR_SKINFLEX_PROPS;

typedef struct {
  int IsVertical;
  int Index;
  const char * pText;
} PROGBAR_SKINFLEX_INFO;






 

PROGBAR_Handle PROGBAR_Create        (int x0, int y0, int xSize, int ySize, int Flags);
PROGBAR_Handle PROGBAR_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags);
PROGBAR_Handle PROGBAR_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
PROGBAR_Handle PROGBAR_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
PROGBAR_Handle PROGBAR_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void PROGBAR_Callback(WM_MESSAGE * pMsg);






 

void PROGBAR_GetMinMax   (PROGBAR_Handle hObj, int * pMin, int * pMax);
int  PROGBAR_GetUserData (PROGBAR_Handle hObj, void * pDest, int NumBytes);
int  PROGBAR_GetValue    (PROGBAR_Handle hObj);
void PROGBAR_SetBarColor (PROGBAR_Handle hObj, unsigned int index, GUI_COLOR color);
void PROGBAR_SetFont     (PROGBAR_Handle hObj, const GUI_FONT * pfont);
void PROGBAR_SetMinMax   (PROGBAR_Handle hObj, int Min, int Max);
void PROGBAR_SetText     (PROGBAR_Handle hObj, const char* s);
void PROGBAR_SetTextAlign(PROGBAR_Handle hObj, int Align);
void PROGBAR_SetTextColor(PROGBAR_Handle hObj, unsigned int index, GUI_COLOR color);
void PROGBAR_SetTextPos  (PROGBAR_Handle hObj, int XOff, int YOff);
void PROGBAR_SetValue    (PROGBAR_Handle hObj, int v);
int  PROGBAR_SetUserData (PROGBAR_Handle hObj, const void * pSrc, int NumBytes);






 
void PROGBAR_GetSkinFlexProps     (PROGBAR_SKINFLEX_PROPS * pProps, int Index);
void PROGBAR_SetSkinClassic       (PROGBAR_Handle hObj);
void PROGBAR_SetSkin              (PROGBAR_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  PROGBAR_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void PROGBAR_SetSkinFlexProps     (const PROGBAR_SKINFLEX_PROPS * pProps, int Index);
void PROGBAR_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * PROGBAR_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);










 
#line 77 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\RADIO.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\RADIO.h"
#line 59 "..\\STemWin\\inc\\RADIO.h"
#line 60 "..\\STemWin\\inc\\RADIO.h"










 







 











 






 








 
typedef signed long RADIO_Handle;

typedef struct {
  GUI_COLOR aColorButton[4];
  int       ButtonSize;
} RADIO_SKINFLEX_PROPS;






 

RADIO_Handle RADIO_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, unsigned Para);
RADIO_Handle RADIO_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumItems, int Spacing);
RADIO_Handle RADIO_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumItems, int Spacing, int NumExtraBytes);
RADIO_Handle RADIO_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void RADIO_Callback(WM_MESSAGE * pMsg);






 

void             RADIO_SetDefaultFont      (const GUI_FONT * pFont);
GUI_COLOR        RADIO_SetDefaultFocusColor(GUI_COLOR Color);
void             RADIO_SetDefaultImage     (const GUI_BITMAP * pBitmap, unsigned int Index);
void             RADIO_SetDefaultTextColor (GUI_COLOR TextColor);
const GUI_FONT * RADIO_GetDefaultFont      (void);
GUI_COLOR        RADIO_GetDefaultTextColor (void);






 

void      RADIO_AddValue     (RADIO_Handle hObj, int Add);
void      RADIO_Dec          (RADIO_Handle hObj);
int       RADIO_GetText      (RADIO_Handle hObj, unsigned Index, char * pBuffer, int MaxLen);
int       RADIO_GetUserData  (RADIO_Handle hObj, void * pDest, int NumBytes);
void      RADIO_Inc          (RADIO_Handle hObj);
void      RADIO_SetBkColor   (RADIO_Handle hObj, GUI_COLOR Color);
GUI_COLOR RADIO_SetFocusColor(RADIO_Handle hObj, GUI_COLOR Color);
void      RADIO_SetFont      (RADIO_Handle hObj, const GUI_FONT * pFont);
void      RADIO_SetGroupId   (RADIO_Handle hObj, unsigned char GroupId);
void      RADIO_SetImage     (RADIO_Handle hObj, const GUI_BITMAP * pBitmap, unsigned int Index);
void      RADIO_SetText      (RADIO_Handle hObj, const char* pText, unsigned Index);
void      RADIO_SetTextColor (RADIO_Handle hObj, GUI_COLOR Color);
void      RADIO_SetValue     (RADIO_Handle hObj, int v);
int       RADIO_SetUserData  (RADIO_Handle hObj, const void * pSrc, int NumBytes);

const GUI_BITMAP * RADIO_GetImage(RADIO_Handle hObj, unsigned int Index);






 
void RADIO_GetSkinFlexProps     (RADIO_SKINFLEX_PROPS * pProps, int Index);
void RADIO_SetSkinClassic       (RADIO_Handle hObj);
void RADIO_SetSkin              (RADIO_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  RADIO_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void RADIO_SetSkinFlexProps     (const RADIO_SKINFLEX_PROPS * pProps, int Index);
void RADIO_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * RADIO_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
int RADIO_GetValue(RADIO_Handle hObj);








 
#line 78 "..\\STemWin\\inc\\DIALOG.h"
#line 79 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\SLIDER.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\SLIDER.h"
#line 59 "..\\STemWin\\inc\\SLIDER.h"
#line 60 "..\\STemWin\\inc\\SLIDER.h"












 



 





 






 








 
typedef signed long SLIDER_Handle;

typedef struct {
  GUI_COLOR aColorFrame[2];
  GUI_COLOR aColorInner[2];
  GUI_COLOR aColorShaft[3];
  GUI_COLOR ColorTick;
  GUI_COLOR ColorFocus;
  int TickSize;
  int ShaftSize;
} SLIDER_SKINFLEX_PROPS;

typedef struct {
  int Width;
  int NumTicks;
  int Size;
  int IsPressed;
  int IsVertical;
} SLIDER_SKINFLEX_INFO;






 
SLIDER_Handle SLIDER_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int WinFlags, int SpecialFlags);
SLIDER_Handle SLIDER_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
SLIDER_Handle SLIDER_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
SLIDER_Handle SLIDER_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void SLIDER_Callback(WM_MESSAGE * pMsg);






 
void      SLIDER_Dec            (SLIDER_Handle hObj);
void      SLIDER_EnableFocusRect(SLIDER_Handle hObj, int OnOff);
GUI_COLOR SLIDER_GetBarColor    (SLIDER_Handle hObj);
GUI_COLOR SLIDER_GetBkColor     (SLIDER_Handle hObj);
unsigned char        SLIDER_GetFlag        (SLIDER_Handle hObj, unsigned char Flag);
GUI_COLOR SLIDER_GetFocusColor  (SLIDER_Handle hObj);
void      SLIDER_GetRange       (SLIDER_Handle hObj, int * pMin, int * pMax);
GUI_COLOR SLIDER_GetTickColor   (SLIDER_Handle hObj);
int       SLIDER_GetUserData    (SLIDER_Handle hObj, void * pDest, int NumBytes);
int       SLIDER_GetValue       (SLIDER_Handle hObj);
void      SLIDER_Inc            (SLIDER_Handle hObj);
void      SLIDER_SetBarColor    (SLIDER_Handle hObj, GUI_COLOR Color);
void      SLIDER_SetBkColor     (SLIDER_Handle hObj, GUI_COLOR Color);
GUI_COLOR SLIDER_SetFocusColor  (SLIDER_Handle hObj, GUI_COLOR Color);
void      SLIDER_SetNumTicks    (SLIDER_Handle hObj, int NumTicks);
void      SLIDER_SetRange       (SLIDER_Handle hObj, int Min, int Max);
void      SLIDER_SetTickColor   (SLIDER_Handle hObj, GUI_COLOR Color);
int       SLIDER_SetUserData    (SLIDER_Handle hObj, const void * pSrc, int NumBytes);
void      SLIDER_SetValue       (SLIDER_Handle hObj, int v);
void      SLIDER_SetWidth       (SLIDER_Handle hObj, int Width);






 
void SLIDER_GetSkinFlexProps     (SLIDER_SKINFLEX_PROPS * pProps, int Index);
void SLIDER_SetSkinClassic       (SLIDER_Handle hObj);
void SLIDER_SetSkin              (SLIDER_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  SLIDER_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void SLIDER_SetSkinFlexProps     (const SLIDER_SKINFLEX_PROPS * pProps, int Index);
void SLIDER_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * SLIDER_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
GUI_COLOR SLIDER_GetDefaultBkColor   (void);
GUI_COLOR SLIDER_GetDefaultBarColor  (void);
GUI_COLOR SLIDER_GetDefaultFocusColor(void);
GUI_COLOR SLIDER_GetDefaultTickColor (void);
void      SLIDER_SetDefaultBkColor   (GUI_COLOR Color);
void      SLIDER_SetDefaultBarColor  (GUI_COLOR Color);
GUI_COLOR SLIDER_SetDefaultFocusColor(GUI_COLOR Color);
void      SLIDER_SetDefaultTickColor (GUI_COLOR Color);








 
#line 80 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\SPINBOX.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\SPINBOX.h"
#line 59 "..\\STemWin\\inc\\SPINBOX.h"
#line 60 "..\\STemWin\\inc\\SPINBOX.h"
#line 61 "..\\STemWin\\inc\\SPINBOX.h"
#line 62 "..\\STemWin\\inc\\SPINBOX.h"












 



 

















 







 













 
typedef signed long SPINBOX_Handle;

typedef struct {
  GUI_COLOR aColorFrame[2];   
  GUI_COLOR aColorUpper[2];   
  GUI_COLOR aColorLower[2];   
  GUI_COLOR ColorArrow;       
  GUI_COLOR ColorBk;          
  GUI_COLOR ColorText;        
  GUI_COLOR ColorButtonFrame; 
} SPINBOX_SKINFLEX_PROPS;






 



 
SPINBOX_Handle SPINBOX_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int Id, int Min, int Max);
SPINBOX_Handle SPINBOX_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int Id, int Min, int Max, int NumExtraBytes);
SPINBOX_Handle SPINBOX_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);




 
void           SPINBOX_Callback(WM_MESSAGE * pMsg);




 
void        SPINBOX_EnableBlink     (SPINBOX_Handle hObj, int Period, int OnOff);
GUI_COLOR   SPINBOX_GetBkColor      (SPINBOX_Handle hObj, unsigned int Index);
GUI_COLOR   SPINBOX_GetButtonBkColor(SPINBOX_Handle hObj, unsigned int Index);
EDIT_Handle SPINBOX_GetEditHandle   (SPINBOX_Handle hObj);
int         SPINBOX_GetUserData     (SPINBOX_Handle hObj, void * pDest, int NumBytes);
signed long         SPINBOX_GetValue        (SPINBOX_Handle hObj);
void        SPINBOX_SetBkColor      (SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
void        SPINBOX_SetButtonBkColor(SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
void        SPINBOX_SetButtonSize   (SPINBOX_Handle hObj, unsigned ButtonSize);
void        SPINBOX_SetEdge         (SPINBOX_Handle hObj, unsigned char Edge);
void        SPINBOX_SetEditMode     (SPINBOX_Handle hObj, unsigned char EditMode);
void        SPINBOX_SetFont         (SPINBOX_Handle hObj, const GUI_FONT * pFont);
void        SPINBOX_SetRange        (SPINBOX_Handle hObj, signed long Min, signed long Max);
unsigned short         SPINBOX_SetStep         (SPINBOX_Handle hObj, unsigned short Step);
void        SPINBOX_SetTextColor    (SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
int         SPINBOX_SetUserData     (SPINBOX_Handle hObj, const void * pSrc, int NumBytes);
void        SPINBOX_SetValue        (SPINBOX_Handle hObj, signed long Value);




 
unsigned short  SPINBOX_GetDefaultButtonSize(void);
void SPINBOX_SetDefaultButtonSize(unsigned short ButtonSize);




 
void                    SPINBOX_GetSkinFlexProps     (SPINBOX_SKINFLEX_PROPS * pProps, int Index);
void                    SPINBOX_SetSkinClassic       (SPINBOX_Handle hObj);
void                    SPINBOX_SetSkin              (SPINBOX_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int                     SPINBOX_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void                    SPINBOX_SetSkinFlexProps     (const SPINBOX_SKINFLEX_PROPS * pProps, int Index);
void                    SPINBOX_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * SPINBOX_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
#line 81 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\TEXT.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\TEXT.h"
#line 59 "..\\STemWin\\inc\\TEXT.h"
#line 60 "..\\STemWin\\inc\\TEXT.h"
#line 61 "..\\STemWin\\inc\\TEXT.h"












 




 















 
typedef signed long TEXT_Handle;






 
TEXT_Handle TEXT_Create        (int x0, int y0, int xSize, int ySize, int Id, int Flags, const char * s, int Align);
TEXT_Handle TEXT_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, const char * s, int Align);
TEXT_Handle TEXT_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const char * pText);
TEXT_Handle TEXT_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const char * pText, int NumExtraBytes);
TEXT_Handle TEXT_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void TEXT_Callback(WM_MESSAGE * pMsg);






 

 

GUI_COLOR        TEXT_GetBkColor  (TEXT_Handle hObj); 
const GUI_FONT * TEXT_GetFont     (TEXT_Handle hObj);
int              TEXT_GetNumLines (TEXT_Handle hObj);
int              TEXT_GetText     (TEXT_Handle hObj, char * pDest, unsigned long BufferSize);
int              TEXT_GetTextAlign(TEXT_Handle hObj);
GUI_COLOR        TEXT_GetTextColor(TEXT_Handle hObj);
int              TEXT_GetUserData (TEXT_Handle hObj, void * pDest, int NumBytes);
GUI_WRAPMODE     TEXT_GetWrapMode (TEXT_Handle hObj);
void             TEXT_SetBkColor  (TEXT_Handle hObj, GUI_COLOR Color);
void             TEXT_SetFont     (TEXT_Handle hObj, const GUI_FONT * pFont);
int              TEXT_SetText     (TEXT_Handle hObj, const char * s);
void             TEXT_SetTextAlign(TEXT_Handle hObj, int Align);
void             TEXT_SetTextColor(TEXT_Handle hObj, GUI_COLOR Color);
int              TEXT_SetUserData (TEXT_Handle hObj, const void * pSrc, int NumBytes);
void             TEXT_SetWrapMode (TEXT_Handle hObj, GUI_WRAPMODE WrapMode);






 

const GUI_FONT * TEXT_GetDefaultFont     (void);
GUI_COLOR        TEXT_GetDefaultTextColor(void);
GUI_WRAPMODE     TEXT_GetDefaultWrapMode (void);
void             TEXT_SetDefaultFont     (const GUI_FONT * pFont);
void             TEXT_SetDefaultTextColor(GUI_COLOR Color);
GUI_WRAPMODE     TEXT_SetDefaultWrapMode (GUI_WRAPMODE WrapMode);








 
#line 82 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\TREEVIEW.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\TREEVIEW.h"
#line 59 "..\\STemWin\\inc\\TREEVIEW.h"
#line 60 "..\\STemWin\\inc\\TREEVIEW.h"












 
 
#line 82 "..\\STemWin\\inc\\TREEVIEW.h"

 
#line 90 "..\\STemWin\\inc\\TREEVIEW.h"

 




 




 
#line 108 "..\\STemWin\\inc\\TREEVIEW.h"

 



 








 
typedef signed long TREEVIEW_Handle;
typedef signed long TREEVIEW_ITEM_Handle;

typedef struct {
  int IsNode;
  int IsExpanded;
  int HasLines;
  int HasRowSelect;
  int Level;
} TREEVIEW_ITEM_INFO;

typedef struct {
  GUI_COLOR ColorBk;
  GUI_COLOR ColorText;
  GUI_COLOR ColorTextBk;
  GUI_COLOR ColorLines;
  GUI_RECT rText;
  TREEVIEW_ITEM_Handle hItem;
  const GUI_FONT * pFont;
  char * pText;
  unsigned char NumLines;
  signed short ax0[3];
  signed short ay0[3];
  signed short ax1[3];
  signed short ay1[3];
  unsigned char NumConnectors;
  signed short axc[16];
  const GUI_BITMAP * pBmPM;
  const GUI_BITMAP * pBmOCL;
  signed short xPosPM, xPosOCL;
  unsigned char IndexPM;
  unsigned char IndexOCL;
} TREEVIEW_ITEM_DRAW_INFO;






 
TREEVIEW_Handle      TREEVIEW_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
TREEVIEW_Handle      TREEVIEW_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
TREEVIEW_Handle      TREEVIEW_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void TREEVIEW_Callback(WM_MESSAGE * pMsg);






 
int                  TREEVIEW_AttachItem     (TREEVIEW_Handle hObj, TREEVIEW_ITEM_Handle hItem, TREEVIEW_ITEM_Handle hItemAt, int Position);
void                 TREEVIEW_DecSel         (TREEVIEW_Handle hObj);
TREEVIEW_ITEM_Handle TREEVIEW_GetItem        (TREEVIEW_Handle hObj, TREEVIEW_ITEM_Handle hItem, int Flags);
TREEVIEW_ITEM_Handle TREEVIEW_GetSel         (TREEVIEW_Handle hObj);
int                  TREEVIEW_GetUserData    (TREEVIEW_Handle hObj, void * pDest, int NumBytes);
void                 TREEVIEW_IncSel         (TREEVIEW_Handle hObj);
TREEVIEW_ITEM_Handle TREEVIEW_InsertItem     (TREEVIEW_Handle hObj, int IsNode, TREEVIEW_ITEM_Handle hItemPrev, int Position, const char * s);
int                  TREEVIEW_OwnerDraw      (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void                 TREEVIEW_ScrollToSel    (TREEVIEW_Handle hObj);
void                 TREEVIEW_SetAutoScrollH (TREEVIEW_Handle hObj, int State);
void                 TREEVIEW_SetAutoScrollV (TREEVIEW_Handle hObj, int State);
void                 TREEVIEW_SetBitmapOffset(TREEVIEW_Handle hObj, int Index, int xOff, int yOff);
void                 TREEVIEW_SetBkColor     (TREEVIEW_Handle hObj, int Index, GUI_COLOR Color);
void                 TREEVIEW_SetFont        (TREEVIEW_Handle hObj, const GUI_FONT * pFont);
void                 TREEVIEW_SetHasLines    (TREEVIEW_Handle hObj, int State);
void                 TREEVIEW_SetImage       (TREEVIEW_Handle hObj, int Index, const GUI_BITMAP * pBitmap);
int                  TREEVIEW_SetIndent      (TREEVIEW_Handle hObj, int Indent);
void                 TREEVIEW_SetLineColor   (TREEVIEW_Handle hObj, int Index, GUI_COLOR Color);
void                 TREEVIEW_SetOwnerDraw   (TREEVIEW_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawItem);
void                 TREEVIEW_SetSel         (TREEVIEW_Handle hObj, TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_SetSelMode     (TREEVIEW_Handle hObj, int Mode);
void                 TREEVIEW_SetTextColor   (TREEVIEW_Handle hObj, int Index, GUI_COLOR Color);
int                  TREEVIEW_SetTextIndent  (TREEVIEW_Handle hObj, int TextIndent);
int                  TREEVIEW_SetUserData    (TREEVIEW_Handle hObj, const void * pSrc, int NumBytes);







 
void                 TREEVIEW_ITEM_Collapse   (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_CollapseAll(TREEVIEW_ITEM_Handle hItem);
TREEVIEW_ITEM_Handle TREEVIEW_ITEM_Create     (int IsNode, const char * s, unsigned long UserData);
void                 TREEVIEW_ITEM_Delete     (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_Detach     (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_Expand     (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_ExpandAll  (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_GetInfo    (TREEVIEW_ITEM_Handle hItem, TREEVIEW_ITEM_INFO * pInfo);
void                 TREEVIEW_ITEM_GetText    (TREEVIEW_ITEM_Handle hItem, unsigned char * pBuffer, int MaxNumBytes);
unsigned long                  TREEVIEW_ITEM_GetUserData(TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_SetImage   (TREEVIEW_ITEM_Handle hItem, int Index, const GUI_BITMAP * pBitmap);
TREEVIEW_ITEM_Handle TREEVIEW_ITEM_SetText    (TREEVIEW_ITEM_Handle hItem, const char * s);
void                 TREEVIEW_ITEM_SetUserData(TREEVIEW_ITEM_Handle hItem, unsigned long UserData);






 
GUI_COLOR        TREEVIEW_GetDefaultBkColor  (int Index);
const GUI_FONT * TREEVIEW_GetDefaultFont     (void);
GUI_COLOR        TREEVIEW_GetDefaultLineColor(int Index);
GUI_COLOR        TREEVIEW_GetDefaultTextColor(int Index);
void             TREEVIEW_SetDefaultBkColor  (int Index, GUI_COLOR Color);
void             TREEVIEW_SetDefaultFont     (const GUI_FONT * pFont);
void             TREEVIEW_SetDefaultLineColor(int Index, GUI_COLOR Color);
void             TREEVIEW_SetDefaultTextColor(int Index, GUI_COLOR Color);








 
#line 83 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\KNOB.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\KNOB.h"
#line 59 "..\\STemWin\\inc\\KNOB.h"
#line 60 "..\\STemWin\\inc\\KNOB.h"
#line 61 "..\\STemWin\\inc\\KNOB.h"












 
typedef signed long KNOB_Handle;






 
KNOB_Handle KNOB_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int Id);
KNOB_Handle KNOB_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int Id, int NumExtraBytes);
KNOB_Handle KNOB_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void KNOB_Callback(WM_MESSAGE * pMsg);






 
void KNOB_AddValue   (KNOB_Handle hObj, signed long Value);
int  KNOB_GetUserData(KNOB_Handle hObj, void * pDest, int NumBytes);               
signed long  KNOB_GetValue   (KNOB_Handle hObj);                                           
void KNOB_SetBkColor (KNOB_Handle hObj, GUI_COLOR Color);                          
void KNOB_SetBkDevice(KNOB_Handle hObj, GUI_MEMDEV_Handle hMemBk);                 
void KNOB_SetDevice  (KNOB_Handle hObj, GUI_MEMDEV_Handle hMemSrc);                
void KNOB_SetKeyValue(KNOB_Handle hObj, signed long KeyValue);                             
void KNOB_SetOffset  (KNOB_Handle hObj, signed long Offset);                               
void KNOB_SetPeriod  (KNOB_Handle hObj, signed long Period);                               
void KNOB_SetPos     (KNOB_Handle hObj, signed long Pos);                                  
void KNOB_SetRange   (KNOB_Handle hObj, signed long MinRange, signed long MaxRange);               
void KNOB_SetSnap    (KNOB_Handle hObj, signed long Snap);                                 
void KNOB_SetTickSize(KNOB_Handle hObj, signed long TickSize);                             
int  KNOB_SetUserData(KNOB_Handle hObj, const void * pSrc, int NumBytes);          






 








 
#line 84 "..\\STemWin\\inc\\DIALOG.h"










 
GUI_HWIN   WINDOW_CreateEx         (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, WM_CALLBACK * cb);
GUI_HWIN   WINDOW_CreateUser       (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, WM_CALLBACK * cb, int NumExtraBytes);
GUI_HWIN   WINDOW_CreateIndirect   (const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);
GUI_COLOR WINDOW_GetDefaultBkColor(void);
int       WINDOW_GetUserData      (GUI_HWIN hObj, void * pDest, int NumBytes);
void      WINDOW_SetBkColor       (GUI_HWIN hObj, GUI_COLOR Color);
void      WINDOW_SetDefaultBkColor(GUI_COLOR Color);
int       WINDOW_SetUserData      (GUI_HWIN hObj, const void * pSrc, int NumBytes);

void WINDOW_Callback(WM_MESSAGE * pMsg);








 
#line 21 "..\\User\\emWinTask\\MainTask.h"
#line 22 "..\\User\\emWinTask\\MainTask.h"
#line 23 "..\\User\\emWinTask\\MainTask.h"
#line 24 "..\\User\\emWinTask\\MainTask.h"
#line 25 "..\\User\\emWinTask\\MainTask.h"
#line 26 "..\\User\\emWinTask\\MainTask.h"
#line 27 "..\\User\\emWinTask\\MainTask.h"
#line 28 "..\\User\\emWinTask\\MainTask.h"
#line 29 "..\\User\\emWinTask\\MainTask.h"
#line 30 "..\\User\\emWinTask\\MainTask.h"
#line 31 "..\\User\\emWinTask\\MainTask.h"
#line 32 "..\\User\\emWinTask\\MainTask.h"
#line 33 "..\\User\\emWinTask\\MainTask.h"
#line 34 "..\\User\\emWinTask\\MainTask.h"
#line 35 "..\\User\\emWinTask\\MainTask.h"
#line 36 "..\\User\\emWinTask\\MainTask.h"
#line 37 "..\\User\\emWinTask\\MainTask.h"
#line 38 "..\\User\\emWinTask\\MainTask.h"
#line 39 "..\\User\\emWinTask\\MainTask.h"
#line 40 "..\\User\\emWinTask\\MainTask.h"

#line 1 "..\\FatFS\\src\\ff.h"

















 









#line 1 "..\\FatFS\\src\\integer.h"
 
 
 




#line 16 "..\\FatFS\\src\\integer.h"

 
typedef int				INT;
typedef unsigned int	UINT;

 
typedef unsigned char	BYTE;

 
typedef short			SHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;

 
typedef long			LONG;
typedef unsigned long	DWORD;

 
typedef unsigned long long QWORD;



#line 29 "..\\FatFS\\src\\ff.h"
#line 1 "..\\FatFS\\src\\ffconf.h"


 





 





 









 








 




 



 



 



 




 




 



 




 



























 

















 





 











 








 




 


 








 








 









 





 











 





 





 





 













 











 




















 

 


 
#line 30 "..\\FatFS\\src\\ff.h"







 

#line 51 "..\\FatFS\\src\\ff.h"





 

#line 69 "..\\FatFS\\src\\ff.h"
typedef char TCHAR;







 

#line 85 "..\\FatFS\\src\\ff.h"
typedef DWORD FSIZE_t;




 

typedef struct {
	BYTE	fs_type;		 
	BYTE	drv;			 
	BYTE	n_fats;			 
	BYTE	wflag;			 
	BYTE	fsi_flag;		 
	WORD	id;				 
	WORD	n_rootdir;		 
	WORD	csize;			 




	WCHAR*	lfnbuf;			 
#line 114 "..\\FatFS\\src\\ff.h"
	DWORD	last_clst;		 
	DWORD	free_clst;		 
#line 125 "..\\FatFS\\src\\ff.h"
	DWORD	n_fatent;		 
	DWORD	fsize;			 
	DWORD	volbase;		 
	DWORD	fatbase;		 
	DWORD	dirbase;		 
	DWORD	database;		 
	DWORD	winsect;		 
	BYTE	win[512];	 
} FATFS;



 

typedef struct {
	FATFS*	fs;			 
	WORD	id;			 
	BYTE	attr;		 
	BYTE	stat;		 
	DWORD	sclust;		 
	FSIZE_t	objsize;	 
#line 155 "..\\FatFS\\src\\ff.h"
} _FDID;



 

typedef struct {
	_FDID	obj;			 
	BYTE	flag;			 
	BYTE	err;			 
	FSIZE_t	fptr;			 
	DWORD	clust;			 
	DWORD	sect;			 

	DWORD	dir_sect;		 
	BYTE*	dir_ptr;		 





	BYTE	buf[512];	 

} FIL;



 

typedef struct {
	_FDID	obj;			 
	DWORD	dptr;			 
	DWORD	clust;			 
	DWORD	sect;			 
	BYTE*	dir;			 
	BYTE	fn[12];			 

	DWORD	blk_ofs;		 




} DIR;



 

typedef struct {
	FSIZE_t	fsize;			 
	WORD	fdate;			 
	WORD	ftime;			 
	BYTE	fattrib;		 

	TCHAR	altname[13];			 
	TCHAR	fname[255 + 1];	 



} FILINFO;



 

typedef enum {
	FR_OK = 0,				 
	FR_DISK_ERR,			 
	FR_INT_ERR,				 
	FR_NOT_READY,			 
	FR_NO_FILE,				 
	FR_NO_PATH,				 
	FR_INVALID_NAME,		 
	FR_DENIED,				 
	FR_EXIST,				 
	FR_INVALID_OBJECT,		 
	FR_WRITE_PROTECTED,		 
	FR_INVALID_DRIVE,		 
	FR_NOT_ENABLED,			 
	FR_NO_FILESYSTEM,		 
	FR_MKFS_ABORTED,		 
	FR_TIMEOUT,				 
	FR_LOCKED,				 
	FR_NOT_ENOUGH_CORE,		 
	FR_TOO_MANY_OPEN_FILES,	 
	FR_INVALID_PARAMETER	 
} FRESULT;



 
 

FRESULT f_open (FIL* fp, const TCHAR* path, BYTE mode);				 
FRESULT f_close (FIL* fp);											 
FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br);			 
FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw);	 
FRESULT f_lseek (FIL* fp, FSIZE_t ofs);								 
FRESULT f_truncate (FIL* fp);										 
FRESULT f_sync (FIL* fp);											 
FRESULT f_opendir (DIR* dp, const TCHAR* path);						 
FRESULT f_closedir (DIR* dp);										 
FRESULT f_readdir (DIR* dp, FILINFO* fno);							 
FRESULT f_findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern);	 
FRESULT f_findnext (DIR* dp, FILINFO* fno);							 
FRESULT f_mkdir (const TCHAR* path);								 
FRESULT f_unlink (const TCHAR* path);								 
FRESULT f_rename (const TCHAR* path_old, const TCHAR* path_new);	 
FRESULT f_stat (const TCHAR* path, FILINFO* fno);					 
FRESULT f_chmod (const TCHAR* path, BYTE attr, BYTE mask);			 
FRESULT f_utime (const TCHAR* path, const FILINFO* fno);			 
FRESULT f_chdir (const TCHAR* path);								 
FRESULT f_chdrive (const TCHAR* path);								 
FRESULT f_getcwd (TCHAR* buff, UINT len);							 
FRESULT f_getfree (const TCHAR* path, DWORD* nclst, FATFS** fatfs);	 
FRESULT f_getlabel (const TCHAR* path, TCHAR* label, DWORD* vsn);	 
FRESULT f_setlabel (const TCHAR* label);							 
FRESULT f_forward (FIL* fp, UINT(*func)(const BYTE*,UINT), UINT btf, UINT* bf);	 
FRESULT f_expand (FIL* fp, FSIZE_t szf, BYTE opt);					 
FRESULT f_mount (FATFS* fs, const TCHAR* path, BYTE opt);			 
FRESULT f_mkfs (const TCHAR* path, BYTE opt, DWORD au, void* work, UINT len);	 
FRESULT f_fdisk (BYTE pdrv, const DWORD* szt, void* work);			 
int f_putc (TCHAR c, FIL* fp);										 
int f_puts (const TCHAR* str, FIL* cp);								 
int f_printf (FIL* fp, const TCHAR* str, ...);						 
TCHAR* f_gets (TCHAR* buff, int len, FIL* fp);						 

#line 288 "..\\FatFS\\src\\ff.h"








 
 

 

DWORD get_fattime (void);


 

WCHAR ff_convert (WCHAR chr, UINT dir);	 
WCHAR ff_wtoupper (WCHAR chr);			 






 
#line 321 "..\\FatFS\\src\\ff.h"




 
 


 
#line 337 "..\\FatFS\\src\\ff.h"

 


 






 





 











#line 42 "..\\User\\emWinTask\\MainTask.h"
#line 1 "..\\FatFS\\src\\diskio.h"


 








#line 13 "..\\FatFS\\src\\diskio.h"


 
typedef BYTE	DSTATUS;

 
typedef enum {
	RES_OK = 0,		 
	RES_ERROR,		 
	RES_WRPRT,		 
	RES_NOTRDY,		 
	RES_PARERR		 
} DRESULT;


 
 


DSTATUS disk_initialize (BYTE pdrv);
DSTATUS disk_status (BYTE pdrv);
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);


 






 

 






 





 
#line 70 "..\\FatFS\\src\\diskio.h"

 








#line 43 "..\\User\\emWinTask\\MainTask.h"





 
extern FRESULT result;
extern FIL file;
extern FILINFO finfo;
extern DIR DirInf;
extern UINT bw;
extern FATFS fs;

extern void _WriteByte2File(unsigned char Data, void * p); 




 
extern const  GUI_FONT GUI_FontHZ_Song_12;
extern const  GUI_FONT GUI_FontHZ_FangSong_16;
extern const  GUI_FONT GUI_FontHZ_Song_16;
extern const  GUI_FONT GUI_FontHZ_Hei_24;
extern const  GUI_FONT GUI_FontHZ_Kai_24;
extern const  GUI_FONT GUI_FontHZ_Song_24;
extern const  GUI_FONT GUI_FontHZ_SimSun_1616;
extern const  GUI_FONT GUI_FontHZ_SimSun_2424;



 
#line 54 "..\\User\\bsp\\bsp.h"














 




 
#line 1 "..\\User\\bsp\\bsp_dwt.h"














 




void bsp_InitDWT(void);
void bsp_DelayUS(uint32_t _ulDelayTime);
void bsp_DelayMS(uint32_t _ulDelayTime);



 
#line 75 "..\\User\\bsp\\bsp.h"
#line 1 "..\\User\\bsp\\bsp_uart_fifo.h"











 






 

 

















 






 








 
typedef enum
{
	COM1 = 0,	 
	COM2 = 1,	 
	COM3 = 2,	 
	COM4 = 3,	 
	COM5 = 4,	 
}COM_PORT_E;

 






























 
typedef struct
{
	USART_TypeDef *uart;		 
	uint8_t *pTxBuf;			 
	uint8_t *pRxBuf;			 
	uint16_t usTxBufSize;		 
	uint16_t usRxBufSize;		 
	volatile uint16_t usTxWrite;			 
	volatile uint16_t usTxRead;			 
	volatile uint16_t usTxCount;			 

	volatile uint16_t usRxWrite;			 
	volatile uint16_t usRxRead;			 
	volatile uint16_t usRxCount;			 

	void (*SendBefor)(void); 	 
	void (*SendOver)(void); 	 
	void (*ReciveNew)(uint8_t _byte);	 
}UART_T;

void bsp_InitUart(void);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte);
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte);

void comClearTxFifo(COM_PORT_E _ucPort);
void comClearRxFifo(COM_PORT_E _ucPort);

void RS485_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen);
void RS485_SendStr(char *_pBuf);

void bsp_Set485Baud(uint32_t _baud);

void bsp_SetUart1Baud(uint32_t _baud);
void bsp_SetUart2Baud(uint32_t _baud);



 
#line 76 "..\\User\\bsp\\bsp.h"
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




#line 77 "..\\User\\bsp\\bsp.h"
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



 
#line 78 "..\\User\\bsp\\bsp.h"
#line 1 "..\\User\\bsp\\bsp_tim_pwm.h"











 




void bsp_SetTIMOutPWM(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TIM_TypeDef* TIMx, uint8_t _ucChannel,
	 uint32_t _ulFreq, uint32_t _ulDutyCycle);

void bsp_SetTIMOutPWM_N(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TIM_TypeDef* TIMx, uint8_t _ucChannel,
	 uint32_t _ulFreq, uint32_t _ulDutyCycle);
	 
void bsp_SetTIMforInt(TIM_TypeDef* TIMx, uint32_t _ulFreq, uint8_t _PreemptionPriority, uint8_t _SubPriority);



 
#line 79 "..\\User\\bsp\\bsp.h"
#line 1 "..\\User\\bsp\\bsp_cpu_rtc.h"















 

#line 21 "..\\User\\bsp\\bsp_cpu_rtc.h"

typedef struct
{
	uint16_t Year;
	uint8_t Mon;
	uint8_t Day;	
	uint8_t Hour;		
	uint8_t Min;				
	uint8_t Sec;					
	uint8_t Week;	
}RTC_T;

extern RTC_T g_tRTC;


void bsp_InitRTC(void);
uint8_t IS_RTC_LeapYear(uint16_t _year);
void bsp_RTC_SetTime(uint8_t _hour, uint8_t _min, uint8_t _sec);
void bsp_RTC_SetDate(uint16_t _year, uint8_t _mon, uint8_t _day);
uint32_t bsp_RTC_GetSecond(uint16_t _year, uint8_t _mon, uint8_t _day, uint8_t _hour, uint8_t _min, uint8_t _sec);
void bsp_RTC_ReadClock(RTC_DateTypeDef *pDate, RTC_TimeTypeDef   *pTime);
uint8_t bsp_RTC_CalcWeek(uint16_t _year, uint8_t _mon, uint8_t _day);
void bsp_RTC_GetClock(void);



 

#line 80 "..\\User\\bsp\\bsp.h"
#line 1 "..\\User\\bsp\\bsp_user_lib.h"









 




int str_len(char *_str);
void str_cpy(char *_tar, char *_src);
int str_cmp(char * s1, char * s2);
void mem_set(char *_tar, char _data, int _len);
uint8_t dec_to_bcd(uint8_t dat);
void int_to_str(int _iNumber, char *_pBuf, unsigned char _len);
int str_to_int(char *_pStr);

uint16_t BEBufToUint16(uint8_t *_pBuf);
uint16_t LEBufToUint16(uint8_t *_pBuf);

uint32_t BEBufToUint32(uint8_t *_pBuf);
uint32_t LEBufToUint32(uint8_t *_pBuf);

uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen) ;
int32_t  CaculTwoPoint(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x);



 
#line 81 "..\\User\\bsp\\bsp.h"

#line 1 "..\\User\\bsp\\bsp_spi_flash.h"











 






 
enum
{
	SST25VF016B_ID = 0xBF2541,
	MX25L1606E_ID  = 0xC22015,
	W25Q64BV_ID    = 0xEF4017
};

typedef struct
{
	uint32_t ChipID;		 
	char ChipName[16];		 
	uint32_t TotalSize;		 
	uint16_t PageSize;		 
}SFLASH_T;

void bsp_InitSFlash(void);
uint32_t sf_ReadID(void);
void sf_EraseChip(void);
void sf_EraseSector(uint32_t _uiSectorAddr);
void sf_PageWrite(uint8_t * _pBuf, uint32_t _uiWriteAddr, uint16_t _usSize);
uint8_t sf_WriteBuffer(uint8_t* _pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize);
void sf_ReadBuffer(uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize);

extern SFLASH_T g_tSF;



DSTATUS SPI_disk_status(void);
DSTATUS SPI_disk_initialize(void);

DRESULT SPI_disk_read(
       	uint8_t *buff,		 
	uint32_t sector,	 
	uint32_t count	);	 

DRESULT SPI_disk_write(
       	const uint8_t *buff,		 
	uint32_t sector,	 
	uint32_t count	);	 

DRESULT SPI_disk_ioctl(
	uint8_t cmd,		 
	void *buff );	 




 
#line 83 "..\\User\\bsp\\bsp.h"
#line 1 "..\\User\\bsp\\bsp_adc.h"






void bsp_InitADC(void);

int8_t getCurent_IntTempValue(void);
int8_t getCurent_ExtTempValue(void);
uint8_t getLightVLuxValue(void);
uint16_t getMicAmp_dBValue(void);




#line 84 "..\\User\\bsp\\bsp.h"
#line 1 "..\\User\\bsp\\bsp_timer.h"











 







 


 
typedef enum
{
	TMR_ONCE_MODE = 0,		 
	TMR_AUTO_MODE = 1		 
}TMR_MODE_E;

 
typedef struct
{
	volatile uint8_t Mode;		 
	volatile uint8_t Flag;		 
	volatile uint32_t Count;	 
	volatile uint32_t PreLoad;	 
}SOFT_TMR;

 
void bsp_InitTimer(void);


void bsp_StartTimer(uint8_t _id, uint32_t _period);
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period);
void bsp_StopTimer(uint8_t _id);
uint8_t bsp_CheckTimer(uint8_t _id);
int32_t bsp_GetRunTime(void);



 
#line 85 "..\\User\\bsp\\bsp.h"
#line 1 "..\\User\\bsp\\bsp_key.h"











 






 








































 
typedef enum
{
	KID_K1 = 0,
	KID_K2,
	KID_K3,
	KID_JOY_U,
	KID_JOY_D,
	KID_JOY_L,
	KID_JOY_R,
	KID_JOY_OK
}KEY_ID_E;





 





 
typedef struct
{
	 
	uint8_t (*IsKeyDownFunc)(void);  

	uint8_t  Count;			 
	uint16_t LongCount;		 
	uint16_t LongTime;		 
	uint8_t  State;			 
	uint8_t  RepeatSpeed;	 
	uint8_t  RepeatCount;	 
}KEY_T;







 
typedef enum
{
	KEY_NONE = 0,			 

	KEY_1_DOWN,				 
	KEY_1_UP,				 
	KEY_1_LONG,				 

	KEY_2_DOWN,				 
	KEY_2_UP,				 
	KEY_2_LONG,				 

	KEY_3_DOWN,				 
	KEY_3_UP,				 
	KEY_3_LONG,				 

	KEY_4_DOWN,				 
	KEY_4_UP,				 
	KEY_4_LONG,				 

	KEY_5_DOWN,				 
	KEY_5_UP,				 
	KEY_5_LONG,				 

	KEY_6_DOWN,				 
	KEY_6_UP,				 
	KEY_6_LONG,				 

	KEY_7_DOWN,				 
	KEY_7_UP,				 
	KEY_7_LONG,				 

	KEY_8_DOWN,				 
	KEY_8_UP,				 
	KEY_8_LONG,				 

	 
	KEY_9_DOWN,				 
	KEY_9_UP,				 
	KEY_9_LONG,				 

	KEY_10_DOWN,			 
	KEY_10_UP,				 
	KEY_10_LONG,			 
}KEY_ENUM;

 

typedef struct
{
	uint8_t Buf[10];		 
	uint8_t Read;					 
	uint8_t Write;					 
	uint8_t Read2;					 
}KEY_FIFO_T;

 
void bsp_InitKey(void);
void bsp_KeyScan(void);
void bsp_PutKey(uint8_t _KeyCode);
uint8_t bsp_GetKey(void);
uint8_t bsp_GetKey2(void);
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID);
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed);
void bsp_ClearKey(void);



 
#line 86 "..\\User\\bsp\\bsp.h"































































 
void bsp_Init(void);
void bsp_Idle(void);
void BSP_Tick_Init (void);
void bsp_LedToggle(uint8_t led_no);





 
#line 50 "..\\User\\bsp\\bsp_tft_lcd.c"


 
uint16_t g_ChipID = IC_4001;		 
uint16_t g_LcdHeight = 240;			 
uint16_t g_LcdWidth = 320;			 
uint8_t s_ucBright;					 
uint8_t g_LcdDirection;				 

static void LCD_CtrlLinesConfig(void);

static void LCD_HardReset(void);

void LCD_SetPwmBackLight(uint8_t _bright);









 
void LCD_InitHard(void)
{
	 
	LCD_CtrlLinesConfig();

	LCD_HardReset();	 
	
	 
	bsp_DelayMS(20);

	g_ChipID = ST7789V_InitHard();
    if(g_ChipID == IC_7789)
    {
        LCD_SetDirection(2);

        LCD_ClrScr(CL_BLACK);	 

        LCD_SetBackLight(100);	  

#line 99 "..\\User\\bsp\\bsp_tft_lcd.c"
    }
}








 
void LCD_HardReset(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	 
	RCC_AHB1PeriphClockCmd(((uint32_t)0x00000001), ENABLE);
	
	 
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0100);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0000)), &GPIO_InitStructure);

	GPIO_SetBits(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0000)), ((uint16_t)0x0100));
	bsp_DelayMS(1);
	GPIO_ResetBits(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0000)), ((uint16_t)0x0100));
	bsp_DelayMS(10);
	GPIO_SetBits(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0000)), ((uint16_t)0x0100));
 	bsp_DelayMS(120);
}








 
void LCD_GetChipDescribe(char *_str)
{
	switch (g_ChipID)
	{
		case IC_5420:
			strcpy(_str, "SPFD5420A");
			break;

		case IC_4001:
			strcpy(_str, "OTM4001A");
			break;

		case IC_61509:
			strcpy(_str, "R61509V");
			break;

		case IC_8875:
			strcpy(_str, "RA8875");
			break;

		case IC_9488:
			strcpy(_str, "ILI9488");
			break;

		default:
			strcpy(_str, "Unknow");
			break;
	}
}








 
uint16_t LCD_GetHeight(void)
{
	return g_LcdHeight;
}








 
uint16_t LCD_GetWidth(void)
{
	return g_LcdWidth;
}








 
void LCD_DispOn(void)
{
	ST7789V_DispOn();
}








 
void LCD_DispOff(void)
{
	ST7789V_DispOff();
}








 
void LCD_ClrScr(uint16_t _usColor)
{
    ST7789V_ClrScr(_usColor);
}












 
void LCD_DispStr(uint16_t _usX, uint16_t _usY, char *_ptr, FONT_T *_tFont)
{
	LCD_DispStrEx(_usX, _usY, _ptr, _tFont, 0, 0);
}









 
uint16_t LCD_GetFontWidth(FONT_T *_tFont)
{
	uint16_t font_width = 16;

	switch (_tFont->FontCode)
	{
		case FC_ST_12:
			font_width = 12;
			break;

		case FC_ST_16:
		case FC_RA8875_16:			
			font_width = 16;
			break;
			
		case FC_RA8875_24:			
		case FC_ST_24:
			font_width = 24;
			break;
			
		case FC_ST_32:
		case FC_RA8875_32:	
			font_width = 32;
			break;			
	}
	return font_width;
}









 
uint16_t LCD_GetFontHeight(FONT_T *_tFont)
{
	uint16_t height = 16;

	switch (_tFont->FontCode)
	{
		case FC_ST_12:
			height = 12;
			break;

		case FC_ST_16:
		case FC_RA8875_16:			
			height = 16;
			break;
			
		case FC_RA8875_24:			
		case FC_ST_24:
			height = 24;
			break;
			
		case FC_ST_32:
		case FC_RA8875_32:	
			height = 32;
			break;			
	}
	return height;
}










 
uint16_t LCD_GetStrWidth(char *_ptr, FONT_T *_tFont)
{
	char *p = _ptr;
	uint16_t width = 0;
	uint8_t code1, code2;
	uint16_t font_width;

	font_width = LCD_GetFontWidth(_tFont);

	while (*p != 0)
	{
		code1 = *p;	 
		if (code1 < 0x80)	 
		{
			switch(_tFont->FontCode)
			{
				case FC_RA8875_16:
					
					break;
				
				case FC_RA8875_24:
					
					break;
				
				case FC_RA8875_32:
					
					break;
				
				case FC_ST_12:
					font_width = 6;
					break;

				case FC_ST_16:		
					font_width = 8;
					break;
					
				case FC_ST_24:			
					font_width = 12;
					break;
					
				case FC_ST_32:
					font_width = 16;
					break;
				
				default:
					font_width = 8;
					break;					
			}
			
		}
		else	 
		{
			code2 = *++p;
			if (code2 == 0)
			{
				break;
			}
			font_width = LCD_GetFontWidth(_tFont);
			
		}
		width += font_width;
		p++;
	}

	return width;
}











 
static void _LCD_ReadAsciiDot(uint8_t _code, uint8_t _fontcode, uint8_t *_pBuf)
{
	const uint8_t *pAscDot;
	uint8_t font_bytes = 0;

	pAscDot = 0;
	switch (_fontcode)
	{
		case FC_ST_12:		 
			font_bytes = 24;
			
			break;
		
		case FC_ST_24:
		case FC_ST_32:
		case FC_ST_16:
			 
			font_bytes = 32;
			
			break;
		
		case FC_RA8875_16:
		case FC_RA8875_24:
		case FC_RA8875_32:
			return;
	}	

	 
	memcpy(_pBuf, &pAscDot[_code * (font_bytes / 2)], (font_bytes / 2));	
}











 
static void _LCD_ReadHZDot(uint8_t _code1, uint8_t _code2,  uint8_t _fontcode, uint8_t *_pBuf)
{
#line 513 "..\\User\\bsp\\bsp_tft_lcd.c"
		uint8_t *pDot = 0;
		uint8_t font_bytes = 0;
			
		switch (_fontcode)
		{
			case FC_ST_12:		 
				font_bytes = 24;
				
				break;
			
			case FC_ST_16:
				font_bytes = 32;
				
				break;
	
			case FC_ST_24:
				font_bytes = 72;
				
				break;			
				
			case FC_ST_32:	
				font_bytes = 128;
				
				break;						
			
			case FC_RA8875_16:
			case FC_RA8875_24:
			case FC_RA8875_32:
				return;
		}			
	
		 
		if (_code1 >=0xA1 && _code1 <= 0xA9 && _code2 >=0xA1)
		{
			pDot += ((_code1 - 0xA1) * 94 + (_code2 - 0xA1)) * font_bytes;
		}
		else if (_code1 >=0xB0 && _code1 <= 0xF7 && _code2 >=0xA1)
		{
			pDot += ((_code1 - 0xB0) * 94 + (_code2 - 0xA1) + 846) * font_bytes;
		}
		memcpy(_pBuf, pDot, font_bytes);

}
			
















 
void LCD_DispStrEx(uint16_t _usX, uint16_t _usY, char *_ptr, FONT_T *_tFont, uint16_t _Width,
	uint8_t _Align)
{
	uint32_t i;
	uint8_t code1;
	uint8_t code2;
	uint8_t buf[32 * 32 / 8];	 
	uint8_t width;
	uint16_t m;
	uint8_t font_width = 0;
	uint8_t font_height = 0;
	uint16_t x, y;
	uint16_t offset;
	uint16_t str_width;	 
	uint8_t ra8875_use = 0;
	

	switch (_tFont->FontCode)
	{
		case FC_ST_12:		 
			font_height = 12;
			font_width = 12;
			break;
		
		case FC_ST_16:
			font_height = 16;
			font_width = 16;
			break;

		case FC_ST_24:
			font_height = 24;
			font_width = 24;
			break;
						
		case FC_ST_32:	
			font_height = 32;
			font_width = 32;
			break;					
		
		case FC_RA8875_16:
			
			ra8875_use = 1;	 
			break;
			
		case FC_RA8875_24:
			
			ra8875_use = 1;	 
			break;
						
		case FC_RA8875_32:
			
			ra8875_use = 1;	 
			break;
		
		default:
			return;
	}
	
	str_width = LCD_GetStrWidth(_ptr, _tFont);	 
	offset = 0;
	if (_Width > str_width)
	{
		if (_Align == ALIGN_RIGHT)	 
		{
			offset = _Width - str_width;
		}
		else if (_Align == ALIGN_CENTER)	 
		{
			offset = (_Width - str_width) / 2;
		}
		else	 
		{
			;
		}
	}

	 
	if (offset > 0)
	{
		LCD_Fill_Rect(_usX, _usY, LCD_GetFontHeight(_tFont), offset,  _tFont->BackColor);
		_usX += offset;
	}
	
	 
	if (_Width > str_width)
	{
		LCD_Fill_Rect(_usX + str_width, _usY, LCD_GetFontHeight(_tFont), _Width - str_width - offset,  _tFont->BackColor);
	}
	
	if (ra8875_use == 1)	 
	{
		
		
		
		
	}
	else	 
	{
		 
		while (*_ptr != 0)
		{
			code1 = *_ptr;	 
			if (code1 < 0x80)
			{
				 
				
				_LCD_ReadAsciiDot(code1, _tFont->FontCode, buf);	 
				width = font_width / 2;
			}
			else
			{
				code2 = *++_ptr;
				if (code2 == 0)
				{
					break;
				}
				 
				_LCD_ReadHZDot(code1, code2, _tFont->FontCode, buf);
				width = font_width;
			}
	
			y = _usY;
			 
			for (m = 0; m < font_height; m++)	 
			{
				x = _usX;
				for (i = 0; i < width; i++)	 
				{
					if ((buf[m * ((2 * width) / font_width) + i / 8] & (0x80 >> (i % 8 ))) != 0x00)
					{
						LCD_PutPixel(x, y, _tFont->FrontColor);	 
					}
					else
					{
						if (_tFont->BackColor != CL_MASK)	 
						{
							LCD_PutPixel(x, y, _tFont->BackColor);	 
						}
					}
	
					x++;
				}
				y++;
			}
	
			if (_tFont->Space > 0)
			{
				 
			}
			_usX += width + _tFont->Space;	 
			_ptr++;			 
		}
	}
}










 
void LCD_PutPixel(uint16_t _usX, uint16_t _usY, uint16_t _usColor)
{
	ST7789V_PutPixel(_usX, _usY, _usColor);
}










 
uint16_t LCD_GetPixel(uint16_t _usX, uint16_t _usY)
{
	uint16_t usRGB;

	usRGB = ST7789V_GetPixel(_usX, _usY);

	return usRGB;
}











 
void LCD_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint16_t _usColor)
{
	ST7789V_DrawLine(_usX1 , _usY1 , _usX2, _usY2 , _usColor);
}

void LCD_Draw_VLine(uint16_t _usX1, uint16_t _usY1, uint16_t _usY2, uint16_t _usColor)
{
        ST7789V_DrawVLine(_usX1, _usY1, _usY2, _usColor);
}
void LCD_Draw_HLine(uint16_t _usX1, uint16_t _usY1, uint16_t _usX2, uint16_t _usColor)
{
        ST7789V_DrawHLine(_usX1, _usY1, _usX2, _usColor);
}

void LCD_Draw_HColorLine(uint16_t _usX1, uint16_t _usY1, uint16_t _usWidth, const uint16_t * _pColor)
{
        ST7789V_DrawHColorLine(_usX1, _usY1, _usWidth, _pColor);
}










 
void LCD_DrawPoints(uint16_t *x, uint16_t *y, uint16_t _usSize, uint16_t _usColor)
{
	uint16_t i;

	for (i = 0 ; i < _usSize - 1; i++)
	{
		LCD_DrawLine(x[i], y[i], x[i + 1], y[i + 1], _usColor);
	}
}











 
void LCD_DrawRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor)
{
	ST7789V_DrawRect(_usX, _usY, _usHeight, _usWidth, _usColor);
}











 
void LCD_Fill_Rect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor)
{
	ST7789V_FillRect(_usX, _usY, _usHeight, _usWidth, _usColor);
}










 
void LCD_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint16_t _usColor)
{
	ST7789V_DrawCircle(_usX, _usY, _usRadius, _usColor);
}












 
void LCD_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t *_ptr)
{
    ST7789V_DrawBMP(_usX, _usY, _usHeight, _usWidth, _ptr);
}








 
void LCD_DrawWin(WIN_T *_pWin)
{

	uint16_t TitleHegiht;

	TitleHegiht = 20;

	 
	LCD_DrawRect(_pWin->Left, _pWin->Top, _pWin->Height, _pWin->Width, WIN_BORDER_COLOR);
	LCD_DrawRect(_pWin->Left + 1, _pWin->Top + 1, _pWin->Height - 2, _pWin->Width - 2, WIN_BORDER_COLOR);

	 
	LCD_Fill_Rect(_pWin->Left + 2, _pWin->Top + 2, TitleHegiht, _pWin->Width - 4, WIN_TITLE_COLOR);

	 
	LCD_Fill_Rect(_pWin->Left + 2, _pWin->Top + TitleHegiht + 2, _pWin->Height - 4 - TitleHegiht,
		_pWin->Width - 4, WIN_BODY_COLOR);

	LCD_DispStr(_pWin->Left + 3, _pWin->Top + 2, _pWin->pCaption, _pWin->Font);
#line 933 "..\\User\\bsp\\bsp_tft_lcd.c"
}











 
void LCD_DrawIcon(const ICON_T *_tIcon, FONT_T *_tFont, uint8_t _ucFocusMode)
{
	const uint16_t *p;
	uint16_t usNewRGB;
	uint16_t x, y;		 

	p = _tIcon->pBmp;
	for (y = 0; y < _tIcon->Height; y++)
	{
		for (x = 0; x < _tIcon->Width; x++)
		{
			usNewRGB = *p++;	 
			 
			if ((y == 0 && (x < 6 || x > _tIcon->Width - 7)) ||
				(y == 1 && (x < 4 || x > _tIcon->Width - 5)) ||
				(y == 2 && (x < 3 || x > _tIcon->Width - 4)) ||
				(y == 3 && (x < 2 || x > _tIcon->Width - 3)) ||
				(y == 4 && (x < 1 || x > _tIcon->Width - 2)) ||
				(y == 5 && (x < 1 || x > _tIcon->Width - 2))	||

				(y == _tIcon->Height - 1 && (x < 6 || x > _tIcon->Width - 7)) ||
				(y == _tIcon->Height - 2 && (x < 4 || x > _tIcon->Width - 5)) ||
				(y == _tIcon->Height - 3 && (x < 3 || x > _tIcon->Width - 4)) ||
				(y == _tIcon->Height - 4 && (x < 2 || x > _tIcon->Width - 3)) ||
				(y == _tIcon->Height - 5 && (x < 1 || x > _tIcon->Width - 2)) ||
				(y == _tIcon->Height - 6 && (x < 1 || x > _tIcon->Width - 2))
				)
			{
				;
			}
			else
			{
				if (_ucFocusMode != 0)	 
				{
					 
					uint16_t R,G,B;
					uint16_t bright = 15;

					 
					R = (usNewRGB & 0xF800) >> 11;
					G = (usNewRGB & 0x07E0) >> 5;
					B =  usNewRGB & 0x001F;
					if (R > bright)
					{
						R -= bright;
					}
					else
					{
						R = 0;
					}
					if (G > 2 * bright)
					{
						G -= 2 * bright;
					}
					else
					{
						G = 0;
					}
					if (B > bright)
					{
						B -= bright;
					}
					else
					{
						B = 0;
					}
					usNewRGB = (R << 11) + (G << 5) + B;
				}

				LCD_PutPixel(x + _tIcon->Left, y + _tIcon->Top, usNewRGB);
			}
		}
	}

	 
	{
		uint16_t len;
		uint16_t width;

		len = strlen(_tIcon->Text);

		if  (len == 0)
		{
			return;	 
		}

		 
		if (_tFont->FontCode == FC_ST_12)		 
		{
			width = 6 * (len + _tFont->Space);
		}
		else	 
		{
			width = 8 * (len + _tFont->Space);
		}


		 
		x = (_tIcon->Left + _tIcon->Width / 2) - width / 2;
		y = _tIcon->Top + _tIcon->Height + 2;
		LCD_DispStr(x, y, (char *)_tIcon->Text, _tFont);
	}
}










 
uint16_t LCD_Blend565(uint16_t src, uint16_t dst, uint8_t alpha)
{
	uint32_t src2;
	uint32_t dst2;

	src2 = ((src << 16) |src) & 0x07E0F81F;
	dst2 = ((dst << 16) | dst) & 0x07E0F81F;
	dst2 = ((((dst2 - src2) * alpha) >> 5) + src2) & 0x07E0F81F;
	return (dst2 >> 16) | dst2;
}










 
void LCD_DrawIcon32(const ICON_T *_tIcon, FONT_T *_tFont, uint8_t _ucFocusMode)
{
	const uint8_t *p;
	uint16_t usOldRGB, usNewRGB;
	int16_t x, y;		 
	uint8_t R1,G1,B1,A;	 
	uint8_t R0,G0,B0;	 

	p = (const uint8_t *)_tIcon->pBmp;
	p += 54;		 

	 
	for (y = _tIcon->Height - 1; y >= 0; y--)
	{
		for (x = 0; x < _tIcon->Width; x++)
		{
			B1 = *p++;
			G1 = *p++;
			R1 = *p++;
			A = *p++;	 

			if (A == 0x00)	 
			{
				;	 
			}
			else if (A == 0xFF)	 
			{
				usNewRGB = (((R1 >> 3) << 11) | ((G1 >> 2) << 5) | (B1 >> 3));
				if (_ucFocusMode == 1)
				{
					usNewRGB = LCD_Blend565(usNewRGB, CL_YELLOW, 10);
				}
				LCD_PutPixel(x + _tIcon->Left, y + _tIcon->Top, usNewRGB);
			}
			else 	 
			{
				 
				usOldRGB = LCD_GetPixel(x + _tIcon->Left, y + _tIcon->Top);
				
				R0 = ((usOldRGB >> 8) & 0xF8);
				G0 = ((usOldRGB >> 3) & 0xFC);
				B0 = ((usOldRGB << 3) & 0xF8);

				R1 = (R1 * A) / 255 + R0 * (255 - A) / 255;
				G1 = (G1 * A) / 255 + G0 * (255 - A) / 255;
				B1 = (B1 * A) / 255 + B0 * (255 - A) / 255;
				usNewRGB = (((R1 >> 3) << 11) | ((G1 >> 2) << 5) | (B1 >> 3));
				if (_ucFocusMode == 1)
				{
					usNewRGB = LCD_Blend565(usNewRGB, CL_YELLOW, 10);
				}
				LCD_PutPixel(x + _tIcon->Left, y + _tIcon->Top, usNewRGB);
			}
		}
	}

	 
	{
		uint16_t len;
		uint16_t width;

		len = strlen(_tIcon->Text);

		if  (len == 0)
		{
			return;	 
		}

		 
		if (_tFont->FontCode == FC_ST_12)		 
		{
			width = 6 * (len + _tFont->Space);
		}
		else	 
		{
			width = 8 * (len + _tFont->Space);
		}


		 
		x = (_tIcon->Left + _tIcon->Width / 2) - width / 2;
		y = _tIcon->Top + _tIcon->Height + 2;
		LCD_DispStr(x, y, (char *)_tIcon->Text, _tFont);
	}
}










 
void LCD_DrawBmp32(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint8_t *_pBmp)
{
	const uint8_t *p;
	uint16_t usOldRGB, usNewRGB;
	int16_t x, y;		 
	uint8_t R1,G1,B1,A;	 
	uint8_t R0,G0,B0;	 

	p = (const uint8_t *)_pBmp;
	p += 54;		 

	 
	for (y = _usHeight - 1; y >= 0; y--)
	{
		for (x = 0; x < _usWidth; x++)
		{
			B1 = *p++;
			G1 = *p++;
			R1 = *p++;
			A = *p++;	 

			if (A == 0x00)	 
			{
				;	 
			}
			else if (A == 0xFF)	 
			{
				usNewRGB = (((R1 >> 3) << 11) | ((G1 >> 2) << 5) | (B1 >> 3));
				
				
				
				
				LCD_PutPixel(x + _usX, y + _usY, usNewRGB);
			}
			else 	 
			{
				 
				usOldRGB = LCD_GetPixel(x + _usX, y + _usY);
				R0 = ((usOldRGB >> 8) & 0xF8);
				G0 = ((usOldRGB >> 3) & 0xFC);
				B0 = ((usOldRGB << 3) & 0xF8);

				R1 = (R1 * A) / 255 + R0 * (255 - A) / 255;
				G1 = (G1 * A) / 255 + G0 * (255 - A) / 255;
				B1 = (B1 * A) / 255 + B0 * (255 - A) / 255;
				usNewRGB = (((R1 >> 3) << 11) | ((G1 >> 2) << 5) | (B1 >> 3));
				
				
				
				
				LCD_PutPixel(x + _usX, y + _usY, usNewRGB);
			}
		}
	}
}








 
void LCD_DrawLabel(LABEL_T *_pLabel)
{

	char dispbuf[256];
	uint16_t i;
	uint16_t NewLen;

	NewLen = strlen(_pLabel->pCaption);

	if (NewLen > _pLabel->MaxLen)
	{
		LCD_DispStr(_pLabel->Left, _pLabel->Top, _pLabel->pCaption, _pLabel->Font);
		_pLabel->MaxLen = NewLen;
	}
	else
	{
		for (i = 0; i < NewLen; i++)
		{
			dispbuf[i] = _pLabel->pCaption[i];
		}
		for (; i < _pLabel->MaxLen; i++)
		{
			dispbuf[i] = ' ';		 
		}
		dispbuf[i] = 0;
		LCD_DispStr(_pLabel->Left, _pLabel->Top, dispbuf, _pLabel->Font);
	}
#line 1283 "..\\User\\bsp\\bsp_tft_lcd.c"
}








 
void LCD_DrawCheckBox(CHECK_T *_pCheckBox)
{

	uint16_t x, y;

	 

	 
	x = _pCheckBox->Left;
	LCD_DrawRect(x, _pCheckBox->Top, CHECK_BOX_H, CHECK_BOX_W, CHECK_BOX_BORDER_COLOR);
	LCD_DrawRect(x + 1, _pCheckBox->Top + 1, CHECK_BOX_H - 2, CHECK_BOX_W - 2, CHECK_BOX_BORDER_COLOR);
	LCD_Fill_Rect(x + 2, _pCheckBox->Top + 2, CHECK_BOX_H - 4, CHECK_BOX_W - 4, CHECK_BOX_BACK_COLOR);

	 
	x = _pCheckBox->Left + CHECK_BOX_W + 2;
	y = _pCheckBox->Top + CHECK_BOX_H / 2 - 8;
	LCD_DispStr(x, y, _pCheckBox->pCaption, _pCheckBox->Font);

	if (_pCheckBox->Checked)
	{
		FONT_T font;

	    font.FontCode = FC_ST_16;
		font.BackColor = CL_MASK;
		font.FrontColor = CHECK_BOX_CHECKED_COLOR;	 
		font.Space = 0;
		x = _pCheckBox->Left;
		LCD_DispStr(x + 3, _pCheckBox->Top + 3, "¡Ì", &font);
	}
#line 1353 "..\\User\\bsp\\bsp_tft_lcd.c"

}








 
void LCD_DrawEdit(EDIT_T *_pEdit)
{

	uint16_t len, x, y;
	uint8_t width;

	 
	LCD_DrawRect(_pEdit->Left, _pEdit->Top, _pEdit->Height, _pEdit->Width, EDIT_BORDER_COLOR);
	LCD_Fill_Rect(_pEdit->Left + 1, _pEdit->Top + 1, _pEdit->Height - 2, _pEdit->Width - 2, EDIT_BACK_COLOR);

	 
	if (_pEdit->Font->FontCode == FC_ST_12)
	{
		width = 6;
	}
	else
	{
		width = 8;
	}
	len = strlen(_pEdit->pCaption);
	x = _pEdit->Left +  (_pEdit->Width - len * width) / 2;
	y = _pEdit->Top + _pEdit->Height / 2 - width;

	LCD_DispStr(x, y, _pEdit->pCaption, _pEdit->Font);
#line 1412 "..\\User\\bsp\\bsp_tft_lcd.c"
}












 
void LCD_DrawButton(BUTTON_T *_pBtn)
{

	uint16_t x, y, h;
	FONT_T font;	 

	font.FontCode = _pBtn->Font->FontCode;
	font.FrontColor = _pBtn->Font->FrontColor;
	font.BackColor = _pBtn->Font->BackColor;
	font.Space = _pBtn->Font->Space;	
			
	if (_pBtn->Focus == 1)
	{
		font.BackColor = BUTTON_ACTIVE_COLOR;
	}
	else
	{
		 
		font.BackColor = BUTTON_BACK_COLOR;
	}
	
	 
	LCD_DrawRect(_pBtn->Left, _pBtn->Top, _pBtn->Height, _pBtn->Width, BUTTON_BORDER_COLOR);
	LCD_DrawRect(_pBtn->Left + 1, _pBtn->Top + 1, _pBtn->Height - 2, _pBtn->Width - 2, BUTTON_BORDER1_COLOR);
	LCD_DrawRect(_pBtn->Left + 2, _pBtn->Top + 2, _pBtn->Height - 4, _pBtn->Width - 4, BUTTON_BORDER2_COLOR);

	h =  LCD_GetFontHeight(&font);
	x = _pBtn->Left + 3;
	y = _pBtn->Top + _pBtn->Height / 2 - h / 2;		
	
	LCD_Fill_Rect(_pBtn->Left + 3, _pBtn->Top + 3, _pBtn->Height - 6, _pBtn->Width - 6, font.BackColor);	 
	LCD_DispStrEx(x, y, _pBtn->pCaption, &font, _pBtn->Width - 6, ALIGN_CENTER);	 		

#line 1535 "..\\User\\bsp\\bsp_tft_lcd.c"
}








 
void LCD_DrawGroupBox(GROUP_T *_pBox)
{
	uint16_t x, y;

	 
	LCD_DrawRect(_pBox->Left + 1, _pBox->Top + 5, _pBox->Height, _pBox->Width - 1, CL_BOX_BORDER2);

	 
	LCD_DrawRect(_pBox->Left, _pBox->Top + 4, _pBox->Height, _pBox->Width - 1, CL_BOX_BORDER1);

	 
	x = _pBox->Left + 9;
	y = _pBox->Top;
	LCD_DispStr(x, y, _pBox->pCaption, _pBox->Font);
}








 
void LCD_DispControl(void *_pControl)
{
	uint8_t id;

	id = *(uint8_t *)_pControl;	 

	switch (id)
	{
		case ID_ICON:
			
			break;

		case ID_WIN:
			LCD_DrawWin((WIN_T *)_pControl);
			break;

		case ID_LABEL:
			LCD_DrawLabel((LABEL_T *)_pControl);
			break;

		case ID_BUTTON:
			LCD_DrawButton((BUTTON_T *)_pControl);
			break;

		case ID_CHECK:
			LCD_DrawCheckBox((CHECK_T *)_pControl);
			break;

		case ID_EDIT:
			LCD_DrawEdit((EDIT_T *)_pControl);
			break;

		case ID_GROUP:
			LCD_DrawGroupBox((GROUP_T *)_pControl);
			break;
	}
}








 
static void LCD_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	 
	RCC_AHB1PeriphClockCmd(((uint32_t)0x00000002) | ((uint32_t)0x00000004), ENABLE);

	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	 
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0xFFFF);
	GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0400)), &GPIO_InitStructure);
        GPIO_Write(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0400)), 0XFF); 

         
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0040)           
		                        | ((uint16_t)0x0080)            
		                        | ((uint16_t)0x0100)           
		                        | ((uint16_t)0x0200);          
	GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0800)), &GPIO_InitStructure);
        GPIO_SetBits(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0800)), ((uint16_t)0x0040));
        GPIO_SetBits(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0800)), ((uint16_t)0x0080));
        GPIO_SetBits(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0800)), ((uint16_t)0x0100));
        GPIO_SetBits(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0800)), ((uint16_t)0x0200));
}

#line 1702 "..\\User\\bsp\\bsp_tft_lcd.c"









 
void LCD_SetPwmBackLight(uint8_t _bright)
{
	 
	
	bsp_SetTIMOutPWM_N(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0000)), ((uint16_t)0x0004), ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000)), 3, 2000, _bright);	
}









 
void LCD_SetBackLight(uint8_t _bright)
{
	s_ucBright =  _bright;	 

	LCD_SetPwmBackLight(_bright);
}








 
uint8_t LCD_GetBackLight(void)
{
	return s_ucBright;
}







 
void LCD_SetDirection(uint8_t _dir)
{
	g_LcdDirection =  _dir;		 

	ST7789V_SetDirection(_dir);
}










 
uint8_t LCD_ButtonTouchDown(BUTTON_T *_btn, uint16_t _usX, uint16_t _usY)
{
	if ((_usX > _btn->Left) && (_usX < _btn->Left + _btn->Width)
		&& (_usY > _btn->Top) && (_usY < _btn->Top + _btn->Height))
	{
		;	 
		_btn->Focus = 1;
		LCD_DrawButton(_btn);
		return 1;
	}
	else
	{
		return 0;
	}
}









 
uint8_t LCD_ButtonTouchRelease(BUTTON_T *_btn, uint16_t _usX, uint16_t _usY)
{
	_btn->Focus = 0;
	LCD_DrawButton(_btn);

	if ((_usX > _btn->Left) && (_usX < _btn->Left + _btn->Width)
		&& (_usY > _btn->Top) && (_usY < _btn->Top + _btn->Height))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}











 
void LCD_InitButton(BUTTON_T *_btn, uint16_t _x, uint16_t _y, uint16_t _h, uint16_t _w, char *_pCaption, FONT_T *_pFont)
{
	_btn->Left = _x;
	_btn->Top = _y;
	_btn->Height = _h;
	_btn->Width = _w;
	_btn->pCaption = _pCaption;	
	_btn->Font = _pFont;
	_btn->Focus = 0;
}

 
