#line 1 "..\\FreeRTOS\\portable\\MemMang\\heap_4.c"



































































 








 
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





 
#line 79 "..\\FreeRTOS\\portable\\MemMang\\heap_4.c"



 


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







#line 86 "..\\FreeRTOS\\portable\\MemMang\\heap_4.c"
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








#line 87 "..\\FreeRTOS\\portable\\MemMang\\heap_4.c"







 


 


 





	static uint8_t ucHeap[ ( ( size_t ) ( 17 * 1024 ) ) ];



 
typedef struct A_BLOCK_LINK
{
	struct A_BLOCK_LINK *pxNextFreeBlock;	 
	size_t xBlockSize;						 
} BlockLink_t;

 






 
static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert );




 
static void prvHeapInit( void );

 


 
static const size_t xHeapStructSize	= ( sizeof( BlockLink_t ) + ( ( size_t ) ( 8 - 1 ) ) ) & ~( ( size_t ) ( 0x0007 ) );

 
static BlockLink_t xStart, *pxEnd = 0;


 
static size_t xFreeBytesRemaining = 0U;
static size_t xMinimumEverFreeBytesRemaining = 0U;




 
static size_t xBlockAllocatedBit = 0;

 

void *pvPortMalloc( size_t xWantedSize )
{
BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
void *pvReturn = 0;

	vTaskSuspendAll();
	{
		
 
		if( pxEnd == 0 )
		{
			prvHeapInit();
		}
		else
		{
			;
		}

		


 
		if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
		{
			
 
			if( xWantedSize > 0 )
			{
				xWantedSize += xHeapStructSize;

				
 
				if( ( xWantedSize & ( 0x0007 ) ) != 0x00 )
				{
					 
					xWantedSize += ( 8 - ( xWantedSize & ( 0x0007 ) ) );
					if( ( ( xWantedSize & ( 0x0007 ) ) == 0 ) == 0 ) { vPortRaiseBASEPRI(); for( ;; ); };
				}
				else
				{
					;
				}
			}
			else
			{
				;
			}

			if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )
			{
				
 
				pxPreviousBlock = &xStart;
				pxBlock = xStart.pxNextFreeBlock;
				while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != 0 ) )
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}

				
 
				if( pxBlock != pxEnd )
				{
					
 
					pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );

					
 
					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

					
 
					if( ( pxBlock->xBlockSize - xWantedSize ) > ( ( size_t ) ( xHeapStructSize << 1 ) ) )
					{
						


 
						pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
						if( ( ( ( ( size_t ) pxNewBlockLink ) & ( 0x0007 ) ) == 0 ) == 0 ) { vPortRaiseBASEPRI(); for( ;; ); };

						
 
						pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
						pxBlock->xBlockSize = xWantedSize;

						 
						prvInsertBlockIntoFreeList( pxNewBlockLink );
					}
					else
					{
						;
					}

					xFreeBytesRemaining -= pxBlock->xBlockSize;

					if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )
					{
						xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
					}
					else
					{
						;
					}

					
 
					pxBlock->xBlockSize |= xBlockAllocatedBit;
					pxBlock->pxNextFreeBlock = 0;
				}
				else
				{
					;
				}
			}
			else
			{
				;
			}
		}
		else
		{
			;
		}

		;
	}
	( void ) xTaskResumeAll();

#line 299 "..\\FreeRTOS\\portable\\MemMang\\heap_4.c"

	if( ( ( ( ( size_t ) pvReturn ) & ( size_t ) ( 0x0007 ) ) == 0 ) == 0 ) { vPortRaiseBASEPRI(); for( ;; ); };
	return pvReturn;
}
 

void vPortFree( void *pv )
{
uint8_t *puc = ( uint8_t * ) pv;
BlockLink_t *pxLink;

	if( pv != 0 )
	{
		
 
		puc -= xHeapStructSize;

		 
		pxLink = ( void * ) puc;

		 
		if( ( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 ) == 0 ) { vPortRaiseBASEPRI(); for( ;; ); };
		if( ( pxLink->pxNextFreeBlock == 0 ) == 0 ) { vPortRaiseBASEPRI(); for( ;; ); };

		if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
		{
			if( pxLink->pxNextFreeBlock == 0 )
			{
				
 
				pxLink->xBlockSize &= ~xBlockAllocatedBit;

				vTaskSuspendAll();
				{
					 
					xFreeBytesRemaining += pxLink->xBlockSize;
					;
					prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
				}
				( void ) xTaskResumeAll();
			}
			else
			{
				;
			}
		}
		else
		{
			;
		}
	}
}
 

size_t xPortGetFreeHeapSize( void )
{
	return xFreeBytesRemaining;
}
 

size_t xPortGetMinimumEverFreeHeapSize( void )
{
	return xMinimumEverFreeBytesRemaining;
}
 

void vPortInitialiseBlocks( void )
{
	 
}
 

static void prvHeapInit( void )
{
BlockLink_t *pxFirstFreeBlock;
uint8_t *pucAlignedHeap;
size_t uxAddress;
size_t xTotalHeapSize = ( ( size_t ) ( 17 * 1024 ) );

	 
	uxAddress = ( size_t ) ucHeap;

	if( ( uxAddress & ( 0x0007 ) ) != 0 )
	{
		uxAddress += ( 8 - 1 );
		uxAddress &= ~( ( size_t ) ( 0x0007 ) );
		xTotalHeapSize -= uxAddress - ( size_t ) ucHeap;
	}

	pucAlignedHeap = ( uint8_t * ) uxAddress;

	
 
	xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
	xStart.xBlockSize = ( size_t ) 0;

	
 
	uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
	uxAddress -= xHeapStructSize;
	uxAddress &= ~( ( size_t ) ( 0x0007 ) );
	pxEnd = ( void * ) uxAddress;
	pxEnd->xBlockSize = 0;
	pxEnd->pxNextFreeBlock = 0;

	
 
	pxFirstFreeBlock = ( void * ) pucAlignedHeap;
	pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
	pxFirstFreeBlock->pxNextFreeBlock = pxEnd;

	 
	xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
	xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

	 
	xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * ( ( size_t ) 8 ) ) - 1 );
}
 

static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert )
{
BlockLink_t *pxIterator;
uint8_t *puc;

	
 
	for( pxIterator = &xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
	{
		 
	}

	
 
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	}
	else
	{
		;
	}

	
 
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	{
		if( pxIterator->pxNextFreeBlock != pxEnd )
		{
			 
			pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		}
		else
		{
			pxBlockToInsert->pxNextFreeBlock = pxEnd;
		}
	}
	else
	{
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}

	


 
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	}
	else
	{
		;
	}
}
