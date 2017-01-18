#line 1 "..\\FatFS\\src\\diskio.c"
 
 
 
 
 
 
 
 

#line 1 "..\\FatFS\\src\\diskio.h"


 








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

 








#line 11 "..\\FatFS\\src\\diskio.c"

 





 
 
 

DSTATUS disk_status (
	BYTE pdrv		 
)
{
	DSTATUS stat;
	

	switch (pdrv) {
	case 0 :
		

		

		return stat;

	case 1 :
		

		

		return stat;

	case 2 :
		

		

		return stat;
	}
	return 0x01;
}



 
 
 

DSTATUS disk_initialize (
	BYTE pdrv				 
)
{
	DSTATUS stat;
	

	switch (pdrv) {
	case 0 :
		

		

		return stat;

	case 1 :
		

		

		return stat;

	case 2 :
		

		

		return stat;
	}
	return 0x01;
}



 
 
 

DRESULT disk_read (
	BYTE pdrv,		 
	BYTE *buff,		 
	DWORD sector,	 
	UINT count		 
)
{
	DRESULT res;
	

	switch (pdrv) {
	case 0 :
		

		

		

		return res;

	case 1 :
		

		

		

		return res;

	case 2 :
		

		

		

		return res;
	}

	return RES_PARERR;
}



 
 
 

DRESULT disk_write (
	BYTE pdrv,			 
	const BYTE *buff,	 
	DWORD sector,		 
	UINT count			 
)
{
	DRESULT res;
	

	switch (pdrv) {
	case 0 :
		

		

		

		return res;

	case 1 :
		

		

		

		return res;

	case 2 :
		

		

		

		return res;
	}

	return RES_PARERR;
}



 
 
 

DRESULT disk_ioctl (
	BYTE pdrv,		 
	BYTE cmd,		 
	void *buff		 
)
{
	DRESULT res;
	

	switch (pdrv) {
	case 0 :

		

		return res;

	case 1 :

		

		return res;

	case 2 :

		

		return res;
	}

	return RES_PARERR;
}









 
DWORD get_fattime (void)
{
	 
#line 248 "..\\FatFS\\src\\diskio.c"
	return	  ((DWORD)(2014 - 1980) << 25)	 
			| ((DWORD)7 << 21)				 
			| ((DWORD)2 << 16)				 
			| ((DWORD)0 << 11)				 
			| ((DWORD)0 << 5)				 
			| ((DWORD)0 >> 1);				 

}

