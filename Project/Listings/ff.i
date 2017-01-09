#line 1 "..\\FatFS\\src\\ff.c"

















 


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
#line 197 "..\\FatFS\\src\\ff.h"
} DIR;



 

typedef struct {
	FSIZE_t	fsize;			 
	WORD	fdate;			 
	WORD	ftime;			 
	BYTE	fattrib;		 




	TCHAR	fname[13];		 

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


 
#line 313 "..\\FatFS\\src\\ff.h"

 
#line 321 "..\\FatFS\\src\\ff.h"




 
 


 
#line 337 "..\\FatFS\\src\\ff.h"

 


 






 





 











#line 22 "..\\FatFS\\src\\ff.c"
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

 








#line 23 "..\\FatFS\\src\\ff.c"






 









 
#line 50 "..\\FatFS\\src\\ff.c"



 
#line 62 "..\\FatFS\\src\\ff.c"


 
#line 73 "..\\FatFS\\src\\ff.c"


 
#line 87 "..\\FatFS\\src\\ff.c"



 

#line 101 "..\\FatFS\\src\\ff.c"

#line 325 "..\\FatFS\\src\\ff.c"


 


















#line 352 "..\\FatFS\\src\\ff.c"


 





 





 
#line 376 "..\\FatFS\\src\\ff.c"


 
#line 385 "..\\FatFS\\src\\ff.c"




 

#line 413 "..\\FatFS\\src\\ff.c"

#line 427 "..\\FatFS\\src\\ff.c"

#line 447 "..\\FatFS\\src\\ff.c"






#line 465 "..\\FatFS\\src\\ff.c"

#line 502 "..\\FatFS\\src\\ff.c"














 



 




static FATFS *FatFs[1];	 
static WORD Fsid;				 









#line 580 "..\\FatFS\\src\\ff.c"














 


 
 
 

static
WORD ld_word (const BYTE* ptr)	 
{
	WORD rv;

	rv = ptr[1];
	rv = rv << 8 | ptr[0];
	return rv;
}

static
DWORD ld_dword (const BYTE* ptr)	 
{
	DWORD rv;

	rv = ptr[3];
	rv = rv << 8 | ptr[2];
	rv = rv << 8 | ptr[1];
	rv = rv << 8 | ptr[0];
	return rv;
}

#line 640 "..\\FatFS\\src\\ff.c"


static
void st_word (BYTE* ptr, WORD val)	 
{
	*ptr++ = (BYTE)val; val >>= 8;
	*ptr++ = (BYTE)val;
}

static
void st_dword (BYTE* ptr, DWORD val)	 
{
	*ptr++ = (BYTE)val; val >>= 8;
	*ptr++ = (BYTE)val; val >>= 8;
	*ptr++ = (BYTE)val; val >>= 8;
	*ptr++ = (BYTE)val;
}

#line 673 "..\\FatFS\\src\\ff.c"



 
 
 

 
static
void mem_cpy (void* dst, const void* src, UINT cnt) {
	BYTE *d = (BYTE*)dst;
	const BYTE *s = (const BYTE*)src;

	if (cnt) {
		do *d++ = *s++; while (--cnt);
	}
}

 
static
void mem_set (void* dst, int val, UINT cnt) {
	BYTE *d = (BYTE*)dst;

	do *d++ = (BYTE)val; while (--cnt);
}

 
static
int mem_cmp (const void* dst, const void* src, UINT cnt) {	 
	const BYTE *d = (const BYTE *)dst, *s = (const BYTE *)src;
	int r = 0;

	do {
		r = *d++ - *s++;
	} while (--cnt && r == 0);

	return r;
}

 
static
int chk_chr (const char* str, int chr) {	 
	while (*str && *str != chr) str++;
	return *str;
}




#line 747 "..\\FatFS\\src\\ff.c"



#line 860 "..\\FatFS\\src\\ff.c"



 
 
 

static
FRESULT sync_window (	 
	FATFS* fs			 
)
{
	DWORD wsect;
	UINT nf;
	FRESULT res = FR_OK;


	if (fs->wflag) {	 
		wsect = fs->winsect;	 
		if (disk_write(fs->drv, fs->win, wsect, 1) != RES_OK) {
			res = FR_DISK_ERR;
		} else {
			fs->wflag = 0;
			if (wsect - fs->fatbase < fs->fsize) {		 
				for (nf = fs->n_fats; nf >= 2; nf--) {	 
					wsect += fs->fsize;
					disk_write(fs->drv, fs->win, wsect, 1);
				}
			}
		}
	}
	return res;
}



static
FRESULT move_window (	 
	FATFS* fs,			 
	DWORD sector		 
)
{
	FRESULT res = FR_OK;


	if (sector != fs->winsect) {	 

		res = sync_window(fs);		 

		if (res == FR_OK) {			 
			if (disk_read(fs->drv, fs->win, sector, 1) != RES_OK) {
				sector = 0xFFFFFFFF;	 
				res = FR_DISK_ERR;
			}
			fs->winsect = sector;
		}
	}
	return res;
}





 
 
 

static
FRESULT sync_fs (	 
	FATFS* fs		 
)
{
	FRESULT res;


	res = sync_window(fs);
	if (res == FR_OK) {
		 
		if (fs->fs_type == 3 && fs->fsi_flag == 1) {
			 
			mem_set(fs->win, 0, ((UINT)512));
			st_word(fs->win + 510, 0xAA55);
			st_dword(fs->win + 0, 0x41615252);
			st_dword(fs->win + 484, 0x61417272);
			st_dword(fs->win + 488, fs->free_clst);
			st_dword(fs->win + 492, fs->last_clst);
			 
			fs->winsect = fs->volbase + 1;
			disk_write(fs->drv, fs->win, fs->winsect, 1);
			fs->fsi_flag = 0;
		}
		 
		if (disk_ioctl(fs->drv, 0, 0) != RES_OK) res = FR_DISK_ERR;
	}

	return res;
}





 
 
 

static
DWORD clust2sect (	 
	FATFS* fs,		 
	DWORD clst		 
)
{
	clst -= 2;
	if (clst >= fs->n_fatent - 2) return 0;		 
	return clst * fs->csize + fs->database;
}




 
 
 

static
DWORD get_fat (	 
	_FDID* obj,	 
	DWORD clst	 
)
{
	UINT wc, bc;
	DWORD val;
	FATFS *fs = obj->fs;


	if (clst < 2 || clst >= fs->n_fatent) {	 
		val = 1;	 

	} else {
		val = 0xFFFFFFFF;	 

		switch (fs->fs_type) {
		case 1 :
			bc = (UINT)clst; bc += bc / 2;
			if (move_window(fs, fs->fatbase + (bc / ((UINT)512))) != FR_OK) break;
			wc = fs->win[bc++ % ((UINT)512)];
			if (move_window(fs, fs->fatbase + (bc / ((UINT)512))) != FR_OK) break;
			wc |= fs->win[bc % ((UINT)512)] << 8;
			val = (clst & 1) ? (wc >> 4) : (wc & 0xFFF);
			break;

		case 2 :
			if (move_window(fs, fs->fatbase + (clst / (((UINT)512) / 2))) != FR_OK) break;
			val = ld_word(fs->win + clst * 2 % ((UINT)512));
			break;

		case 3 :
			if (move_window(fs, fs->fatbase + (clst / (((UINT)512) / 4))) != FR_OK) break;
			val = ld_dword(fs->win + clst * 4 % ((UINT)512)) & 0x0FFFFFFF;
			break;
#line 1045 "..\\FatFS\\src\\ff.c"
		default:
			val = 1;	 
		}
	}

	return val;
}





 
 
 

static
FRESULT put_fat (	 
	FATFS* fs,		 
	DWORD clst,		 
	DWORD val		 
)
{
	UINT bc;
	BYTE *p;
	FRESULT res = FR_INT_ERR;


	if (clst >= 2 && clst < fs->n_fatent) {	 
		switch (fs->fs_type) {
		case 1 :	 
			bc = (UINT)clst; bc += bc / 2;
			res = move_window(fs, fs->fatbase + (bc / ((UINT)512)));
			if (res != FR_OK) break;
			p = fs->win + bc++ % ((UINT)512);
			*p = (clst & 1) ? ((*p & 0x0F) | ((BYTE)val << 4)) : (BYTE)val;
			fs->wflag = 1;
			res = move_window(fs, fs->fatbase + (bc / ((UINT)512)));
			if (res != FR_OK) break;
			p = fs->win + bc % ((UINT)512);
			*p = (clst & 1) ? (BYTE)(val >> 4) : ((*p & 0xF0) | ((BYTE)(val >> 8) & 0x0F));
			fs->wflag = 1;
			break;

		case 2 :	 
			res = move_window(fs, fs->fatbase + (clst / (((UINT)512) / 2)));
			if (res != FR_OK) break;
			st_word(fs->win + clst * 2 % ((UINT)512), (WORD)val);
			fs->wflag = 1;
			break;

		case 3 :	 



			res = move_window(fs, fs->fatbase + (clst / (((UINT)512) / 4)));
			if (res != FR_OK) break;
			if (!0 || fs->fs_type != 4) {
				val = (val & 0x0FFFFFFF) | (ld_dword(fs->win + clst * 4 % ((UINT)512)) & 0xF0000000);
			}
			st_dword(fs->win + clst * 4 % ((UINT)512), val);
			fs->wflag = 1;
			break;
		}
	}
	return res;
}






#line 1224 "..\\FatFS\\src\\ff.c"




 
 
 
static
FRESULT remove_chain (	 
	_FDID* obj,			 
	DWORD clst,			 
	DWORD pclst			 
)
{
	FRESULT res = FR_OK;
	DWORD nxt;
	FATFS *fs = obj->fs;
#line 1247 "..\\FatFS\\src\\ff.c"

	if (clst < 2 || clst >= fs->n_fatent) return FR_INT_ERR;	 

	 
	if (pclst && (!0 || fs->fs_type != 4 || obj->stat != 2)) {
		res = put_fat(fs, pclst, 0xFFFFFFFF);
		if (res != FR_OK) return res;
	}

	 
	do {
		nxt = get_fat(obj, clst);			 
		if (nxt == 0) break;				 
		if (nxt == 1) return FR_INT_ERR;	 
		if (nxt == 0xFFFFFFFF) return FR_DISK_ERR;	 
		if (!0 || fs->fs_type != 4) {
			res = put_fat(fs, clst, 0);		 
			if (res != FR_OK) return res;
		}
		if (fs->free_clst < fs->n_fatent - 2) {	 
			fs->free_clst++;
			fs->fsi_flag |= 1;
		}
#line 1288 "..\\FatFS\\src\\ff.c"
		clst = nxt;					 
	} while (clst < fs->n_fatent);	 

#line 1302 "..\\FatFS\\src\\ff.c"
	return FR_OK;
}




 
 
 
static
DWORD create_chain (	 
	_FDID* obj,			 
	DWORD clst			 
)
{
	DWORD cs, ncl, scl;
	FRESULT res;
	FATFS *fs = obj->fs;


	if (clst == 0) {	 
		scl = fs->last_clst;				 
		if (scl == 0 || scl >= fs->n_fatent) scl = 1;
	}
	else {				 
		cs = get_fat(obj, clst);			 
		if (cs < 2) return 1;				 
		if (cs == 0xFFFFFFFF) return cs;	 
		if (cs < fs->n_fatent) return cs;	 
		scl = clst;
	}

#line 1351 "..\\FatFS\\src\\ff.c"
	{	 
		ncl = scl;	 
		for (;;) {
			ncl++;							 
			if (ncl >= fs->n_fatent) {		 
				ncl = 2;
				if (ncl > scl) return 0;	 
			}
			cs = get_fat(obj, ncl);			 
			if (cs == 0) break;				 
			if (cs == 1 || cs == 0xFFFFFFFF) return cs;	 
			if (ncl == scl) return 0;		 
		}
	}

	if (0 && fs->fs_type == 4 && obj->stat == 2) {	 
		res = FR_OK;						 
	} else {
		res = put_fat(fs, ncl, 0xFFFFFFFF);	 
		if (res == FR_OK && clst) {
			res = put_fat(fs, clst, ncl);	 
		}
	}

	if (res == FR_OK) {			 
		fs->last_clst = ncl;
		if (fs->free_clst < fs->n_fatent - 2) fs->free_clst--;
		fs->fsi_flag |= 1;
	} else {
		ncl = (res == FR_DISK_ERR) ? 0xFFFFFFFF : 1;	 
	}

	return ncl;		 
}






#line 1418 "..\\FatFS\\src\\ff.c"




 
 
 

static
FRESULT dir_sdi (	 
	DIR* dp,		 
	DWORD ofs		 
)
{
	DWORD csz, clst;
	FATFS *fs = dp->obj.fs;


	if (ofs >= (DWORD)((0 && fs->fs_type == 4) ? 0x10000000 : 0x200000) || ofs % 32) {	 
		return FR_INT_ERR;
	}
	dp->dptr = ofs;				 
	clst = dp->obj.sclust;		 
	if (clst == 0 && fs->fs_type >= 3) {	 
		clst = fs->dirbase;
		if (0) dp->obj.stat = 0;	 
	}

	if (clst == 0) {	 
		if (ofs / 32 >= fs->n_rootdir)	return FR_INT_ERR;	 
		dp->sect = fs->dirbase;

	} else {			 
		csz = (DWORD)fs->csize * ((UINT)512);	 
		while (ofs >= csz) {				 
			clst = get_fat(&dp->obj, clst);				 
			if (clst == 0xFFFFFFFF) return FR_DISK_ERR;	 
			if (clst < 2 || clst >= fs->n_fatent) return FR_INT_ERR;	 
			ofs -= csz;
		}
		dp->sect = clust2sect(fs, clst);
	}
	dp->clust = clst;					 
	if (!dp->sect) return FR_INT_ERR;
	dp->sect += ofs / ((UINT)512);			 
	dp->dir = fs->win + (ofs % ((UINT)512));	 

	return FR_OK;
}




 
 
 

static
FRESULT dir_next (	 
	DIR* dp,		 
	int stretch		 
)
{
	DWORD ofs, clst;
	FATFS *fs = dp->obj.fs;

	UINT n;


	ofs = dp->dptr + 32;	 
	if (!dp->sect || ofs >= (DWORD)((0 && fs->fs_type == 4) ? 0x10000000 : 0x200000)) return FR_NO_FILE;	 

	if (ofs % ((UINT)512) == 0) {	 
		dp->sect++;				 

		if (!dp->clust) {		 
			if (ofs / 32 >= fs->n_rootdir) {	 
				dp->sect = 0; return FR_NO_FILE;
			}
		}
		else {					 
			if ((ofs / ((UINT)512) & (fs->csize - 1)) == 0) {		 
				clst = get_fat(&dp->obj, dp->clust);			 
				if (clst <= 1) return FR_INT_ERR;				 
				if (clst == 0xFFFFFFFF) return FR_DISK_ERR;		 
				if (clst >= fs->n_fatent) {						 

					if (!stretch) {								 
						dp->sect = 0; return FR_NO_FILE;
					}
					clst = create_chain(&dp->obj, dp->clust);	 
					if (clst == 0) return FR_DENIED;			 
					if (clst == 1) return FR_INT_ERR;			 
					if (clst == 0xFFFFFFFF) return FR_DISK_ERR;	 
					 
					if (0) dp->obj.stat |= 4;			 
					if (sync_window(fs) != FR_OK) return FR_DISK_ERR;	 
					mem_set(fs->win, 0, ((UINT)512));				 
					for (n = 0, fs->winsect = clust2sect(fs, clst); n < fs->csize; n++, fs->winsect++) {	 
						fs->wflag = 1;
						if (sync_window(fs) != FR_OK) return FR_DISK_ERR;
					}
					fs->winsect -= n;							 




				}
				dp->clust = clst;		 
				dp->sect = clust2sect(fs, clst);
			}
		}
	}
	dp->dptr = ofs;						 
	dp->dir = fs->win + ofs % ((UINT)512);	 

	return FR_OK;
}





 
 
 

static
FRESULT dir_alloc (	 
	DIR* dp,		 
	UINT nent		 
)
{
	FRESULT res;
	UINT n;
	FATFS *fs = dp->obj.fs;


	res = dir_sdi(dp, 0);
	if (res == FR_OK) {
		n = 0;
		do {
			res = move_window(fs, dp->sect);
			if (res != FR_OK) break;



			if (dp->dir[0] == 0xE5 || dp->dir[0] == 0) {

				if (++n == nent) break;	 
			} else {
				n = 0;					 
			}
			res = dir_next(dp, 1);
		} while (res == FR_OK);	 
	}

	if (res == FR_NO_FILE) res = FR_DENIED;	 
	return res;
}






 
 
 

static
DWORD ld_clust (	 
	FATFS* fs,		 
	const BYTE* dir	 
)
{
	DWORD cl;

	cl = ld_word(dir + 26);
	if (fs->fs_type == 3) {
		cl |= (DWORD)ld_word(dir + 20) << 16;
	}

	return cl;
}



static
void st_clust (
	FATFS* fs,	 
	BYTE* dir,	 
	DWORD cl	 
)
{
	st_word(dir + 26, (WORD)cl);
	if (fs->fs_type == 3) {
		st_word(dir + 20, (WORD)(cl >> 16));
	}
}




#line 1737 "..\\FatFS\\src\\ff.c"



#line 1796 "..\\FatFS\\src\\ff.c"



#line 1817 "..\\FatFS\\src\\ff.c"



#line 2080 "..\\FatFS\\src\\ff.c"




 
 
 

static
FRESULT dir_read (
	DIR* dp,		 
	int vol			 
)
{
	FRESULT res = FR_NO_FILE;
	FATFS *fs = dp->obj.fs;
	BYTE a, c;




	while (dp->sect) {
		res = move_window(fs, dp->sect);
		if (res != FR_OK) break;
		c = dp->dir[0];	 
		if (c == 0) { res = FR_NO_FILE; break; }	 
#line 2122 "..\\FatFS\\src\\ff.c"
		{	 
			dp->obj.attr = a = dp->dir[11] & 0x3F;	 
#line 2144 "..\\FatFS\\src\\ff.c"
			if (c != 0xE5 && c != '.' && a != 0x0F && (int)((a & ~0x20) == 0x08) == vol) {	 
				break;
			}

		}
		res = dir_next(dp, 0);		 
		if (res != FR_OK) break;
	}

	if (res != FR_OK) dp->sect = 0;		 
	return res;
}





 
 
 

static
FRESULT dir_find (	 
	DIR* dp			 
)
{
	FRESULT res;
	FATFS *fs = dp->obj.fs;
	BYTE c;




	res = dir_sdi(dp, 0);			 
	if (res != FR_OK) return res;
#line 2196 "..\\FatFS\\src\\ff.c"
	 



	do {
		res = move_window(fs, dp->sect);
		if (res != FR_OK) break;
		c = dp->dir[0];
		if (c == 0) { res = FR_NO_FILE; break; }	 
#line 2227 "..\\FatFS\\src\\ff.c"
		dp->obj.attr = dp->dir[11] & 0x3F;
		if (!(dp->dir[11] & 0x08) && !mem_cmp(dp->dir, dp->fn, 11)) break;	 

		res = dir_next(dp, 0);	 
	} while (res == FR_OK);

	return res;
}





 
 
 

static
FRESULT dir_register (	 
	DIR* dp				 
)
{
	FRESULT res;
	FATFS *fs = dp->obj.fs;
#line 2318 "..\\FatFS\\src\\ff.c"
	res = dir_alloc(dp, 1);		 



	 
	if (res == FR_OK) {
		res = move_window(fs, dp->sect);
		if (res == FR_OK) {
			mem_set(dp->dir, 0, 32);	 
			mem_cpy(dp->dir + 0, dp->fn, 11);	 



			fs->wflag = 1;
		}
	}

	return res;
}






 
 
 

static
FRESULT dir_remove (	 
	DIR* dp				 
)
{
	FRESULT res;
	FATFS *fs = dp->obj.fs;
#line 2375 "..\\FatFS\\src\\ff.c"

	res = move_window(fs, dp->sect);
	if (res == FR_OK) {
		dp->dir[0] = 0xE5;
		fs->wflag = 1;
	}


	return res;
}






 
 
 

static
void get_fileinfo (		 
	DIR* dp,			 
	FILINFO* fno	 	 
)
{
	UINT i, j;
	TCHAR c;
	DWORD tm;






	fno->fname[0] = 0;		 
	if (!dp->sect) return;	 

#line 2471 "..\\FatFS\\src\\ff.c"
	i = j = 0;
	while (i < 11) {		 
		c = (TCHAR)dp->dir[i++];
		if (c == ' ') continue;				 
		if (c == 0x05) c = (TCHAR)0xE5;	 
		if (i == 9) fno->fname[j++] = '.';	 
		fno->fname[j++] = c;
	}
	fno->fname[j] = 0;


	fno->fattrib = dp->dir[11];				 
	fno->fsize = ld_dword(dp->dir + 28);	 
	tm = ld_dword(dp->dir + 22);			 
	fno->ftime = (WORD)tm; fno->fdate = (WORD)(tm >> 16);
}





#line 2562 "..\\FatFS\\src\\ff.c"



 
 
 

static
FRESULT create_name (	 
	DIR* dp,			 
	const TCHAR** path	 
)
{
#line 2694 "..\\FatFS\\src\\ff.c"
	BYTE c, d, *sfn;
	UINT ni, si, i;
	const char *p;

	 
	p = *path; sfn = dp->fn;
	mem_set(sfn, ' ', 11);
	si = i = 0; ni = 8;
#line 2715 "..\\FatFS\\src\\ff.c"
	for (;;) {
		c = (BYTE)p[si++];
		if (c <= ' ') break; 			 
		if (c == '/' || c == '\\') {	 
			while (p[si] == '/' || p[si] == '\\') si++;	 
			break;
		}
		if (c == '.' || i >= ni) {		 
			if (ni == 11 || c != '.') return FR_INVALID_NAME;	 
			i = 8; ni = 11;				 
			continue;
		}
		if (c >= 0x80) {				 
#line 2735 "..\\FatFS\\src\\ff.c"
		}
		if ((((BYTE)(c) >= 0x81 && (BYTE)(c) <= 0x9F) || ((BYTE)(c) >= 0xE0 && (BYTE)(c) <= 0xFC))) {				 
			d = (BYTE)p[si++];			 
			if (!(((BYTE)(d) >= 0x40 && (BYTE)(d) <= 0x7E) || ((BYTE)(d) >= 0x80 && (BYTE)(d) <= 0xFC)) || i >= ni - 1) return FR_INVALID_NAME;	 
			sfn[i++] = c;
			sfn[i++] = d;
		} else {						 
			if (chk_chr("\"*+,:;<=>\?[]|\x7F", c)) return FR_INVALID_NAME;	 
			if ((((c)>= 'a')&&((c)<= 'z'))) c -= 0x20;	 
			sfn[i++] = c;
		}
	}
	*path = p + si;						 
	if (i == 0) return FR_INVALID_NAME;	 

	if (sfn[0] == 0xE5) sfn[0] = 0x05;	 
	sfn[11] = (c <= ' ') ? 0x04 : 0;		 

	return FR_OK;

}




 
 
 

static
FRESULT follow_path (	 
	DIR* dp,			 
	const TCHAR* path	 
)
{
	FRESULT res;
	BYTE ns;
	_FDID *obj = &dp->obj;
	FATFS *fs = obj->fs;







	{										 
		while (*path == '/' || *path == '\\') path++;	 
		obj->sclust = 0;					 
	}
#line 2798 "..\\FatFS\\src\\ff.c"

	if ((UINT)*path < ' ') {				 
		dp->fn[11] = 0x80;
		res = dir_sdi(dp, 0);

	} else {								 
		for (;;) {
			res = create_name(dp, &path);	 
			if (res != FR_OK) break;
			res = dir_find(dp);				 
			ns = dp->fn[11];
			if (res != FR_OK) {				 
				if (res == FR_NO_FILE) {	 
					if (0 && (ns & 0x20)) {	 
						if (!(ns & 0x04)) continue;	 
						dp->fn[11] = 0x80;
						res = FR_OK;
					} else {							 
						if (!(ns & 0x04)) res = FR_NO_PATH;	 
					}
				}
				break;
			}
			if (ns & 0x04) break;			 
			 
			if (!(obj->attr & 0x10)) {		 
				res = FR_NO_PATH; break;
			}
#line 2836 "..\\FatFS\\src\\ff.c"
			{
				obj->sclust = ld_clust(fs, fs->win + dp->dptr % ((UINT)512));	 
			}
		}
	}

	return res;
}




 
 
 

static
int get_ldnumber (		 
	const TCHAR** path	 
)
{
	const TCHAR *tp, *tt;
	UINT i;
	int vol = -1;
#line 2866 "..\\FatFS\\src\\ff.c"


	if (*path) {	 
		for (tt = *path; (UINT)*tt >= (0 ? ' ' : '!') && *tt != ':'; tt++) ;	 
		if (*tt == ':') {	 
			tp = *path;
			i = *tp++ - '0'; 
			if (i < 10 && tp == tt) {	 
				if (i < 1) {	 
					vol = (int)i;
					*path = ++tt;
				}
			}
#line 2895 "..\\FatFS\\src\\ff.c"
			return vol;
		}



		vol = 0;		 

	}
	return vol;
}




 
 
 

static
BYTE check_fs (	 
	FATFS* fs,	 
	DWORD sect	 
)
{
	fs->wflag = 0; fs->winsect = 0xFFFFFFFF;		 
	if (move_window(fs, sect) != FR_OK) return 4;	 

	if (ld_word(fs->win + 510) != 0xAA55) return 3;	 

	if (fs->win[0] == 0xE9 || (fs->win[0] == 0xEB && fs->win[0 + 2] == 0x90)) {
		if ((ld_dword(fs->win + 54) & 0xFFFFFF) == 0x544146) return 0;	 
		if (ld_dword(fs->win + 82) == 0x33544146) return 0;			 
	}



	return 2;
}




 
 
 

static
FRESULT find_volume (	 
	const TCHAR** path,	 
	FATFS** rfs,		 
	BYTE mode			 
)
{
	BYTE fmt, *pt;
	int vol;
	DSTATUS stat;
	DWORD bsect, fasize, tsect, sysect, nclst, szbfat, br[4];
	WORD nrsv;
	FATFS *fs;
	UINT i;


	 
	*rfs = 0;
	vol = get_ldnumber(path);
	if (vol < 0) return FR_INVALID_DRIVE;

	 
	fs = FatFs[vol];					 
	if (!fs) return FR_NOT_ENABLED;		 

	;						 
	*rfs = fs;							 

	mode &= (BYTE)~0x01;				 
	if (fs->fs_type) {					 
		stat = disk_status(fs->drv);
		if (!(stat & 0x01)) {		 
			if (!0 && mode && (stat & 0x04)) {	 
				return FR_WRITE_PROTECTED;
			}
			return FR_OK;				 
		}
	}

	 
	 

	fs->fs_type = 0;					 
	fs->drv = (BYTE)(vol);				 
	stat = disk_initialize(fs->drv);	 
	if (stat & 0x01) { 			 
		return FR_NOT_READY;			 
	}
	if (!0 && mode && (stat & 0x04)) {  
		return FR_WRITE_PROTECTED;
	}




	 
	bsect = 0;
	fmt = check_fs(fs, bsect);			 
	if (fmt == 2 || (fmt < 2 && 0 != 0)) {	 
		for (i = 0; i < 4; i++) {			 
			pt = fs->win + (446 + i * 16);
			br[i] = pt[4] ? ld_dword(pt + 8) : 0;
		}
		i = 0;						 
		if (i) i--;
		do {								 
			bsect = br[i];
			fmt = bsect ? check_fs(fs, bsect) : 3;	 
		} while (!0 && fmt >= 2 && ++i < 4);
	}
	if (fmt == 4) return FR_DISK_ERR;		 
	if (fmt >= 2) return FR_NO_FILESYSTEM;	 

	 

#line 3062 "..\\FatFS\\src\\ff.c"
	{
		if (ld_word(fs->win + 11) != ((UINT)512)) return FR_NO_FILESYSTEM;	 

		fasize = ld_word(fs->win + 22);			 
		if (fasize == 0) fasize = ld_dword(fs->win + 36);
		fs->fsize = fasize;

		fs->n_fats = fs->win[16];					 
		if (fs->n_fats != 1 && fs->n_fats != 2) return FR_NO_FILESYSTEM;	 
		fasize *= fs->n_fats;								 

		fs->csize = fs->win[13];				 
		if (fs->csize == 0 || (fs->csize & (fs->csize - 1))) return FR_NO_FILESYSTEM;	 

		fs->n_rootdir = ld_word(fs->win + 17);	 
		if (fs->n_rootdir % (((UINT)512) / 32)) return FR_NO_FILESYSTEM;	 

		tsect = ld_word(fs->win + 19);			 
		if (tsect == 0) tsect = ld_dword(fs->win + 32);

		nrsv = ld_word(fs->win + 14);			 
		if (nrsv == 0) return FR_NO_FILESYSTEM;				 

		 
		sysect = nrsv + fasize + fs->n_rootdir / (((UINT)512) / 32);	 
		if (tsect < sysect) return FR_NO_FILESYSTEM;		 
		nclst = (tsect - sysect) / fs->csize;				 
		if (nclst == 0) return FR_NO_FILESYSTEM;			 
		fmt = 3;
		if (nclst <= 0xFFF5) fmt = 2;
		if (nclst <= 0xFF5) fmt = 1;

		 
		fs->n_fatent = nclst + 2;							 
		fs->volbase = bsect;								 
		fs->fatbase = bsect + nrsv; 						 
		fs->database = bsect + sysect;						 
		if (fmt == 3) {
			if (ld_word(fs->win + 42) != 0) return FR_NO_FILESYSTEM;	 
			if (fs->n_rootdir) return FR_NO_FILESYSTEM;		 
			fs->dirbase = ld_dword(fs->win + 44);	 
			szbfat = fs->n_fatent * 4;						 
		} else {
			if (fs->n_rootdir == 0)	return FR_NO_FILESYSTEM; 
			fs->dirbase = fs->fatbase + fasize;				 
			szbfat = (fmt == 2) ?					 
				fs->n_fatent * 2 : fs->n_fatent * 3 / 2 + (fs->n_fatent & 1);
		}
		if (fs->fsize < (szbfat + (((UINT)512) - 1)) / ((UINT)512)) return FR_NO_FILESYSTEM;	 


		 
		fs->last_clst = fs->free_clst = 0xFFFFFFFF;		 
		fs->fsi_flag = 0x80;

		if (fmt == 3				 
			&& ld_word(fs->win + 48) == 1
			&& move_window(fs, bsect + 1) == FR_OK)
		{
			fs->fsi_flag = 0;
			if (ld_word(fs->win + 510) == 0xAA55	 
				&& ld_dword(fs->win + 0) == 0x41615252
				&& ld_dword(fs->win + 484) == 0x61417272)
			{

				fs->free_clst = ld_dword(fs->win + 488);


				fs->last_clst = ld_dword(fs->win + 492);

			}
		}


	}

	fs->fs_type = fmt;	 
	fs->id = ++Fsid;	 
#line 3152 "..\\FatFS\\src\\ff.c"
	return FR_OK;
}




 
 
 

static
FRESULT validate (	 
	_FDID* obj,		 
	FATFS** fs		 
)
{
	FRESULT res;


	if (!obj || !obj->fs || !obj->fs->fs_type || obj->fs->id != obj->id || (disk_status(obj->fs->drv) & 0x01)) {
		*fs = 0;				 
		res = FR_INVALID_OBJECT;
	} else {
		*fs = obj->fs;			 
		;		 
		res = FR_OK;
	}
	return res;
}








 



 
 
 

FRESULT f_mount (
	FATFS* fs,			 
	const TCHAR* path,	 
	BYTE opt			 
)
{
	FATFS *cfs;
	int vol;
	FRESULT res;
	const TCHAR *rp = path;


	 
	vol = get_ldnumber(&rp);
	if (vol < 0) return FR_INVALID_DRIVE;
	cfs = FatFs[vol];					 

	if (cfs) {
#line 3221 "..\\FatFS\\src\\ff.c"
		cfs->fs_type = 0;				 
	}

	if (fs) {
		fs->fs_type = 0;				 



	}
	FatFs[vol] = fs;					 

	if (!fs || opt != 1) return FR_OK;	 

	res = find_volume(&path, &fs, 0);	 
	return res;
}




 
 
 

FRESULT f_open (
	FIL* fp,			 
	const TCHAR* path,	 
	BYTE mode			 
)
{
	FRESULT res;
	DIR dj;
	FATFS *fs;

	DWORD dw, cl, bcs, clst, sc;
	FSIZE_t ofs;

	


	if (!fp) return FR_INVALID_OBJECT;

	 
	mode &= 0 ? 0x01 : 0x01 | 0x02 | 0x08 | 0x04 | 0x10 | 0x30 | 0x20;
	res = find_volume(&path, &fs, mode);
	if (res == FR_OK) {
		dj.obj.fs = fs;
		;
		res = follow_path(&dj, path);	 

		if (res == FR_OK) {
			if (dj.fn[11] & 0x80) {	 
				res = FR_INVALID_NAME;
			}





		}
		 
		if (mode & (0x08 | 0x10 | 0x04)) {
			if (res != FR_OK) {					 
				if (res == FR_NO_FILE)			 



					res = dir_register(&dj);

				mode |= 0x08;		 
			}
			else {								 
				if (dj.obj.attr & (0x01 | 0x10)) {	 
					res = FR_DENIED;
				} else {
					if (mode & 0x04) res = FR_EXIST;	 
				}
			}
			if (res == FR_OK && (mode & 0x08)) {	 
				dw = get_fattime();
#line 3325 "..\\FatFS\\src\\ff.c"
				{
					 
					st_dword(dj.dir + 14, dw);	 
					st_dword(dj.dir + 22, dw);	 
					dj.dir[11] = 0x20;			 
					cl = ld_clust(fs, dj.dir);			 
					st_clust(fs, dj.dir, 0);			 
					st_dword(dj.dir + 28, 0);
					fs->wflag = 1;

					if (cl) {							 
						dw = fs->winsect;
						res = remove_chain(&dj.obj, cl, 0);
						if (res == FR_OK) {
							res = move_window(fs, dw);
							fs->last_clst = cl - 1;		 
						}
					}
				}
			}
		}
		else {	 
			if (res == FR_OK) {					 
				if (dj.obj.attr & 0x10) {		 
					res = FR_NO_FILE;
				} else {
					if ((mode & 0x02) && (dj.obj.attr & 0x01)) {  
						res = FR_DENIED;
					}
				}
			}
		}
		if (res == FR_OK) {
			if (mode & 0x08)		 
				mode |= 0x40;
			fp->dir_sect = fs->winsect;			 
			fp->dir_ptr = dj.dir;




		}
#line 3378 "..\\FatFS\\src\\ff.c"

		if (res == FR_OK) {
#line 3390 "..\\FatFS\\src\\ff.c"
			{
				fp->obj.sclust = ld_clust(fs, dj.dir);				 
				fp->obj.objsize = ld_dword(dj.dir + 28);
			}



			fp->obj.fs = fs;	 	 
			fp->obj.id = fs->id;
			fp->flag = mode;		 
			fp->err = 0;			 
			fp->sect = 0;			 
			fp->fptr = 0;			 


			mem_set(fp->buf, 0, 512);	 

			if ((mode & 0x20) && fp->obj.objsize > 0) {	 
				fp->fptr = fp->obj.objsize;			 
				bcs = (DWORD)fs->csize * ((UINT)512);	 
				clst = fp->obj.sclust;				 
				for (ofs = fp->obj.objsize; res == FR_OK && ofs > bcs; ofs -= bcs) {
					clst = get_fat(&fp->obj, clst);
					if (clst <= 1) res = FR_INT_ERR;
					if (clst == 0xFFFFFFFF) res = FR_DISK_ERR;
				}
				fp->clust = clst;
				if (res == FR_OK && ofs % ((UINT)512)) {	 
					if ((sc = clust2sect(fs, clst)) == 0) {
						res = FR_INT_ERR;
					} else {
						fp->sect = sc + (DWORD)(ofs / ((UINT)512));

						if (disk_read(fs->drv, fp->buf, fp->sect, 1) != RES_OK) res = FR_DISK_ERR;

					}
				}
			}

		}

		;
	}

	if (res != FR_OK) fp->obj.fs = 0;	 

	return res;
}




 
 
 

FRESULT f_read (
	FIL* fp, 	 
	void* buff,	 
	UINT btr,	 
	UINT* br	 
)
{
	FRESULT res;
	FATFS *fs;
	DWORD clst, sect;
	FSIZE_t remain;
	UINT rcnt, cc, csect;
	BYTE *rbuff = (BYTE*)buff;


	*br = 0;	 
	res = validate(&fp->obj, &fs);				 
	if (res != FR_OK || (res = (FRESULT)fp->err) != FR_OK) return res;	 
	if (!(fp->flag & 0x01)) return FR_DENIED;  
	remain = fp->obj.objsize - fp->fptr;
	if (btr > remain) btr = (UINT)remain;		 

	for ( ;  btr;								 
		rbuff += rcnt, fp->fptr += rcnt, *br += rcnt, btr -= rcnt) {
		if (fp->fptr % ((UINT)512) == 0) {			 
			csect = (UINT)(fp->fptr / ((UINT)512) & (fs->csize - 1));	 
			if (csect == 0) {					 
				if (fp->fptr == 0) {			 
					clst = fp->obj.sclust;		 
				} else {						 





					{
						clst = get_fat(&fp->obj, fp->clust);	 
					}
				}
				if (clst < 2) { fp->err = (BYTE)(FR_INT_ERR); return FR_INT_ERR; };
				if (clst == 0xFFFFFFFF) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
				fp->clust = clst;				 
			}
			sect = clust2sect(fs, fp->clust);	 
			if (!sect) { fp->err = (BYTE)(FR_INT_ERR); return FR_INT_ERR; };
			sect += csect;
			cc = btr / ((UINT)512);					 
			if (cc) {							 
				if (csect + cc > fs->csize) {	 
					cc = fs->csize - csect;
				}
				if (disk_read(fs->drv, rbuff, sect, cc) != RES_OK) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
#line 3504 "..\\FatFS\\src\\ff.c"
				if ((fp->flag & 0x80) && fp->sect - sect < cc) {
					mem_cpy(rbuff + ((fp->sect - sect) * ((UINT)512)), fp->buf, ((UINT)512));
				}


				rcnt = ((UINT)512) * cc;				 
				continue;
			}

			if (fp->sect != sect) {			 

				if (fp->flag & 0x80) {		 
					if (disk_write(fs->drv, fp->buf, fp->sect, 1) != RES_OK) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
					fp->flag &= (BYTE)~0x80;
				}

				if (disk_read(fs->drv, fp->buf, sect, 1) != RES_OK)	{ fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };	 
			}

			fp->sect = sect;
		}
		rcnt = ((UINT)512) - (UINT)fp->fptr % ((UINT)512);	 
		if (rcnt > btr) rcnt = btr;					 




		mem_cpy(rbuff, fp->buf + fp->fptr % ((UINT)512), rcnt);	 

	}

	return FR_OK;
}





 
 
 

FRESULT f_write (
	FIL* fp,			 
	const void* buff,	 
	UINT btw,			 
	UINT* bw			 
)
{
	FRESULT res;
	FATFS *fs;
	DWORD clst, sect;
	UINT wcnt, cc, csect;
	const BYTE *wbuff = (const BYTE*)buff;


	*bw = 0;	 
	res = validate(&fp->obj, &fs);			 
	if (res != FR_OK || (res = (FRESULT)fp->err) != FR_OK) return res;	 
	if (!(fp->flag & 0x02)) return FR_DENIED;	 

	 
	if ((!0 || fs->fs_type != 4) && (DWORD)(fp->fptr + btw) < (DWORD)fp->fptr) {
		btw = (UINT)(0xFFFFFFFF - (DWORD)fp->fptr);
	}

	for ( ;  btw;							 
		wbuff += wcnt, fp->fptr += wcnt, fp->obj.objsize = (fp->fptr > fp->obj.objsize) ? fp->fptr : fp->obj.objsize, *bw += wcnt, btw -= wcnt) {
		if (fp->fptr % ((UINT)512) == 0) {		 
			csect = (UINT)(fp->fptr / ((UINT)512)) & (fs->csize - 1);	 
			if (csect == 0) {				 
				if (fp->fptr == 0) {		 
					clst = fp->obj.sclust;	 
					if (clst == 0) {		 
						clst = create_chain(&fp->obj, 0);	 
					}
				} else {					 





					{
						clst = create_chain(&fp->obj, fp->clust);	 
					}
				}
				if (clst == 0) break;		 
				if (clst == 1) { fp->err = (BYTE)(FR_INT_ERR); return FR_INT_ERR; };
				if (clst == 0xFFFFFFFF) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
				fp->clust = clst;			 
				if (fp->obj.sclust == 0) fp->obj.sclust = clst;	 
			}



			if (fp->flag & 0x80) {		 
				if (disk_write(fs->drv, fp->buf, fp->sect, 1) != RES_OK) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
				fp->flag &= (BYTE)~0x80;
			}

			sect = clust2sect(fs, fp->clust);	 
			if (!sect) { fp->err = (BYTE)(FR_INT_ERR); return FR_INT_ERR; };
			sect += csect;
			cc = btw / ((UINT)512);				 
			if (cc) {						 
				if (csect + cc > fs->csize) {	 
					cc = fs->csize - csect;
				}
				if (disk_write(fs->drv, wbuff, sect, cc) != RES_OK) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
#line 3620 "..\\FatFS\\src\\ff.c"
				if (fp->sect - sect < cc) {  
					mem_cpy(fp->buf, wbuff + ((fp->sect - sect) * ((UINT)512)), ((UINT)512));
					fp->flag &= (BYTE)~0x80;
				}


				wcnt = ((UINT)512) * cc;		 
				continue;
			}
#line 3635 "..\\FatFS\\src\\ff.c"
			if (fp->sect != sect && 		 
				fp->fptr < fp->obj.objsize &&
				disk_read(fs->drv, fp->buf, sect, 1) != RES_OK) {
					{ fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
			}

			fp->sect = sect;
		}
		wcnt = ((UINT)512) - (UINT)fp->fptr % ((UINT)512);	 
		if (wcnt > btw) wcnt = btw;					 





		mem_cpy(fp->buf + fp->fptr % ((UINT)512), wbuff, wcnt);	 
		fp->flag |= 0x80;

	}

	fp->flag |= 0x40;				 

	return FR_OK;
}




 
 
 

FRESULT f_sync (
	FIL* fp		 
)
{
	FRESULT res;
	FATFS *fs;
	DWORD tm;
	BYTE *dir;
	


	res = validate(&fp->obj, &fs);	 
	if (res == FR_OK) {
		if (fp->flag & 0x40) {	 

			if (fp->flag & 0x80) {	 
				if (disk_write(fs->drv, fp->buf, fp->sect, 1) != RES_OK) return FR_DISK_ERR;
				fp->flag &= (BYTE)~0x80;
			}

			 
			tm = get_fattime();				 
#line 3716 "..\\FatFS\\src\\ff.c"
			{
				res = move_window(fs, fp->dir_sect);
				if (res == FR_OK) {
					dir = fp->dir_ptr;
					dir[11] |= 0x20;						 
					st_clust(fp->obj.fs, dir, fp->obj.sclust);		 
					st_dword(dir + 28, (DWORD)fp->obj.objsize);	 
					st_dword(dir + 22, tm);				 
					st_word(dir + 18, 0);
					fs->wflag = 1;
					res = sync_fs(fs);					 
					fp->flag &= (BYTE)~0x40;
				}
			}
		}
	}

	return res;
}






 
 
 

FRESULT f_close (
	FIL* fp		 
)
{
	FRESULT res;
	FATFS *fs;


	res = f_sync(fp);					 
	if (res == FR_OK)

	{
		res = validate(&fp->obj, &fs);	 
		if (res == FR_OK) {




			{
				fp->obj.fs = 0;			 
			}



		}
	}
	return res;
}




#line 3925 "..\\FatFS\\src\\ff.c"




 
 
 

FRESULT f_lseek (
	FIL* fp,		 
	FSIZE_t ofs		 
)
{
	FRESULT res;
	FATFS *fs;
	DWORD clst, bcs, nsect;
	FSIZE_t ifptr;




	res = validate(&fp->obj, &fs);		 
	if (res != FR_OK || (res = (FRESULT)fp->err) != FR_OK) return res;	 
#line 3999 "..\\FatFS\\src\\ff.c"

	 
	{



		if (ofs > fp->obj.objsize && (0 || !(fp->flag & 0x02))) {	 
			ofs = fp->obj.objsize;
		}
		ifptr = fp->fptr;
		fp->fptr = nsect = 0;
		if (ofs) {
			bcs = (DWORD)fs->csize * ((UINT)512);	 
			if (ifptr > 0 &&
				(ofs - 1) / bcs >= (ifptr - 1) / bcs) {	 
				fp->fptr = (ifptr - 1) & ~(FSIZE_t)(bcs - 1);	 
				ofs -= fp->fptr;
				clst = fp->clust;
			} else {									 
				clst = fp->obj.sclust;					 

				if (clst == 0) {						 
					clst = create_chain(&fp->obj, 0);
					if (clst == 1) { fp->err = (BYTE)(FR_INT_ERR); return FR_INT_ERR; };
					if (clst == 0xFFFFFFFF) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
					fp->obj.sclust = clst;
				}

				fp->clust = clst;
			}
			if (clst != 0) {
				while (ofs > bcs) {						 
					ofs -= bcs; fp->fptr += bcs;

					if (fp->flag & 0x02) {			 
						if (0 && fp->fptr > fp->obj.objsize) {	 
							fp->obj.objsize = fp->fptr;
							fp->flag |= 0x40;
						}
						clst = create_chain(&fp->obj, clst);	 
						if (clst == 0) {				 
							ofs = 0; break;
						}
					} else

					{
						clst = get_fat(&fp->obj, clst);	 
					}
					if (clst == 0xFFFFFFFF) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
					if (clst <= 1 || clst >= fs->n_fatent) { fp->err = (BYTE)(FR_INT_ERR); return FR_INT_ERR; };
					fp->clust = clst;
				}
				fp->fptr += ofs;
				if (ofs % ((UINT)512)) {
					nsect = clust2sect(fs, clst);	 
					if (!nsect) { fp->err = (BYTE)(FR_INT_ERR); return FR_INT_ERR; };
					nsect += (DWORD)(ofs / ((UINT)512));
				}
			}
		}
		if (!0 && fp->fptr > fp->obj.objsize) {		 
			fp->obj.objsize = fp->fptr;
			fp->flag |= 0x40;
		}
		if (fp->fptr % ((UINT)512) && nsect != fp->sect) {	 


			if (fp->flag & 0x80) {			 
				if (disk_write(fs->drv, fp->buf, fp->sect, 1) != RES_OK) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };
				fp->flag &= (BYTE)~0x80;
			}

			if (disk_read(fs->drv, fp->buf, nsect, 1) != RES_OK) { fp->err = (BYTE)(FR_DISK_ERR); return FR_DISK_ERR; };	 

			fp->sect = nsect;
		}
	}

	return res;
}




 
 
 

FRESULT f_opendir (
	DIR* dp,			 
	const TCHAR* path	 
)
{
	FRESULT res;
	FATFS *fs;
	_FDID *obj;
	


	if (!dp) return FR_INVALID_OBJECT;

	 
	obj = &dp->obj;
	res = find_volume(&path, &fs, 0);
	if (res == FR_OK) {
		obj->fs = fs;
		;
		res = follow_path(dp, path);			 
		if (res == FR_OK) {						 
			if (!(dp->fn[11] & 0x80)) {	 
				if (obj->attr & 0x10) {		 
#line 4120 "..\\FatFS\\src\\ff.c"
					{
						obj->sclust = ld_clust(fs, dp->dir);	 
					}
				} else {						 
					res = FR_NO_PATH;
				}
			}
			if (res == FR_OK) {
				obj->id = fs->id;
				res = dir_sdi(dp, 0);			 
#line 4140 "..\\FatFS\\src\\ff.c"
			}
		}
		;
		if (res == FR_NO_FILE) res = FR_NO_PATH;
	}
	if (res != FR_OK) obj->fs = 0;		 

	return res;
}




 
 
 

FRESULT f_closedir (
	DIR *dp		 
)
{
	FRESULT res;
	FATFS *fs;


	res = validate(&dp->obj, &fs);			 
	if (res == FR_OK) {
#line 4173 "..\\FatFS\\src\\ff.c"
		{
			dp->obj.fs = 0;			 
		}



	}
	return res;
}




 
 
 

FRESULT f_readdir (
	DIR* dp,			 
	FILINFO* fno		 
)
{
	FRESULT res;
	FATFS *fs;
	


	res = validate(&dp->obj, &fs);	 
	if (res == FR_OK) {
		if (!fno) {
			res = dir_sdi(dp, 0);			 
		} else {
			;
			res = dir_read(dp, 0);			 
			if (res == FR_NO_FILE) res = FR_OK;	 
			if (res == FR_OK) {				 
				get_fileinfo(dp, fno);		 
				res = dir_next(dp, 0);		 
				if (res == FR_NO_FILE) res = FR_OK;	 
			}
			;
		}
	}
	return res;
}



#line 4270 "..\\FatFS\\src\\ff.c"




 
 
 

FRESULT f_stat (
	const TCHAR* path,	 
	FILINFO* fno		 
)
{
	FRESULT res;
	DIR dj;
	


	 
	res = find_volume(&path, &dj.obj.fs, 0);
	if (res == FR_OK) {
		;
		res = follow_path(&dj, path);	 
		if (res == FR_OK) {				 
			if (dj.fn[11] & 0x80) {	 
				res = FR_INVALID_NAME;
			} else {							 
				if (fno) get_fileinfo(&dj, fno);
			}
		}
		;
	}

	return res;
}




 
 
 

FRESULT f_getfree (
	const TCHAR* path,	 
	DWORD* nclst,		 
	FATFS** fatfs		 
)
{
	FRESULT res;
	FATFS *fs;
	DWORD nfree, clst, sect, stat;
	UINT i;
	BYTE *p;
	_FDID obj;


	 
	res = find_volume(&path, &fs, 0);
	if (res == FR_OK) {
		*fatfs = fs;				 
		 
		if (fs->free_clst <= fs->n_fatent - 2) {
			*nclst = fs->free_clst;
		} else {
			 
			nfree = 0;
			if (fs->fs_type == 1) {	 
				clst = 2; obj.fs = fs;
				do {
					stat = get_fat(&obj, clst);
					if (stat == 0xFFFFFFFF) { res = FR_DISK_ERR; break; }
					if (stat == 1) { res = FR_INT_ERR; break; }
					if (stat == 0) nfree++;
				} while (++clst < fs->n_fatent);
			} else {
#line 4364 "..\\FatFS\\src\\ff.c"
				{	 
					clst = fs->n_fatent; sect = fs->fatbase;
					i = 0; p = 0;
					do {
						if (i == 0) {
							res = move_window(fs, sect++);
							if (res != FR_OK) break;
							p = fs->win;
							i = ((UINT)512);
						}
						if (fs->fs_type == 2) {
							if (ld_word(p) == 0) nfree++;
							p += 2; i -= 2;
						} else {
							if ((ld_dword(p) & 0x0FFFFFFF) == 0) nfree++;
							p += 4; i -= 4;
						}
					} while (--clst);
				}
			}
			*nclst = nfree;			 
			fs->free_clst = nfree;	 
			fs->fsi_flag |= 1;		 
		}
	}

	return res;
}




 
 
 

FRESULT f_truncate (
	FIL* fp		 
)
{
	FRESULT res;
	FATFS *fs;
	DWORD ncl;


	res = validate(&fp->obj, &fs);	 
	if (res != FR_OK || (res = (FRESULT)fp->err) != FR_OK) return res;
	if (!(fp->flag & 0x02)) return FR_DENIED;	 

	if (fp->obj.objsize > fp->fptr) {
		if (fp->fptr == 0) {	 
			res = remove_chain(&fp->obj, fp->obj.sclust, 0);
			fp->obj.sclust = 0;
		} else {				 
			ncl = get_fat(&fp->obj, fp->clust);
			res = FR_OK;
			if (ncl == 0xFFFFFFFF) res = FR_DISK_ERR;
			if (ncl == 1) res = FR_INT_ERR;
			if (res == FR_OK && ncl < fs->n_fatent) {
				res = remove_chain(&fp->obj, ncl, fp->clust);
			}
		}
		fp->obj.objsize = fp->fptr;	 
		fp->flag |= 0x40;

		if (res == FR_OK && (fp->flag & 0x80)) {
			if (disk_write(fs->drv, fp->buf, fp->sect, 1) != RES_OK) {
				res = FR_DISK_ERR;
			} else {
				fp->flag &= (BYTE)~0x80;
			}
		}

		if (res != FR_OK) { fp->err = (BYTE)(res); return res; };
	}

	return res;
}




 
 
 

FRESULT f_unlink (
	const TCHAR* path		 
)
{
	FRESULT res;
	DIR dj, sdj;
	DWORD dclst = 0;
	FATFS *fs;



	


	 
	res = find_volume(&path, &fs, 0x02);
	dj.obj.fs = fs;
	if (res == FR_OK) {
		;
		res = follow_path(&dj, path);		 
		if (0 && res == FR_OK && (dj.fn[11] & 0x20)) {
			res = FR_INVALID_NAME;			 
		}



		if (res == FR_OK) {					 
			if (dj.fn[11] & 0x80) {
				res = FR_INVALID_NAME;		 
			} else {
				if (dj.obj.attr & 0x01) {
					res = FR_DENIED;		 
				}
			}
			if (res == FR_OK) {
#line 4493 "..\\FatFS\\src\\ff.c"
				{
					dclst = ld_clust(fs, dj.dir);
				}
				if (dj.obj.attr & 0x10) {			 





					{
						sdj.obj.fs = fs;						 
						sdj.obj.sclust = dclst;
#line 4511 "..\\FatFS\\src\\ff.c"
						res = dir_sdi(&sdj, 0);
						if (res == FR_OK) {
							res = dir_read(&sdj, 0);			 
							if (res == FR_OK) res = FR_DENIED;	 
							if (res == FR_NO_FILE) res = FR_OK;	 
						}
					}
				}
			}
			if (res == FR_OK) {
				res = dir_remove(&dj);			 
				if (res == FR_OK && dclst) {	 



					res = remove_chain(&dj.obj, dclst, 0);

				}
				if (res == FR_OK) res = sync_fs(fs);
			}
		}
		;
	}

	return res;
}




 
 
 

FRESULT f_mkdir (
	const TCHAR* path		 
)
{
	FRESULT res;
	DIR dj;
	FATFS *fs;
	BYTE *dir;
	UINT n;
	DWORD dsc, dcl, pcl, tm;
	


	 
	res = find_volume(&path, &fs, 0x02);
	dj.obj.fs = fs;
	if (res == FR_OK) {
		;
		res = follow_path(&dj, path);			 
		if (res == FR_OK) res = FR_EXIST;		 
		if (0 && res == FR_NO_FILE && (dj.fn[11] & 0x20)) {
			res = FR_INVALID_NAME;
		}
		if (res == FR_NO_FILE) {				 
			dcl = create_chain(&dj.obj, 0);		 
			dj.obj.objsize = (DWORD)fs->csize * ((UINT)512);
			res = FR_OK;
			if (dcl == 0) res = FR_DENIED;		 
			if (dcl == 1) res = FR_INT_ERR;
			if (dcl == 0xFFFFFFFF) res = FR_DISK_ERR;
			if (res == FR_OK) res = sync_window(fs);	 
			tm = get_fattime();
			if (res == FR_OK) {					 
				dsc = clust2sect(fs, dcl);
				dir = fs->win;
				mem_set(dir, 0, ((UINT)512));
				if (!0 || fs->fs_type != 4) {
					mem_set(dir + 0, ' ', 11);	 
					dir[0] = '.';
					dir[11] = 0x10;
					st_dword(dir + 22, tm);
					st_clust(fs, dir, dcl);
					mem_cpy(dir + 32, dir, 32); 	 
					dir[32 + 1] = '.'; pcl = dj.obj.sclust;
					if (fs->fs_type == 3 && pcl == fs->dirbase) pcl = 0;
					st_clust(fs, dir + 32, pcl);
				}
				for (n = fs->csize; n; n--) {	 
					fs->winsect = dsc++;
					fs->wflag = 1;
					res = sync_window(fs);
					if (res != FR_OK) break;
					mem_set(dir, 0, ((UINT)512));
				}
			}
			if (res == FR_OK) res = dir_register(&dj);	 
			if (res == FR_OK) {
#line 4613 "..\\FatFS\\src\\ff.c"
				{
					dir = dj.dir;
					st_dword(dir + 22, tm);	 
					st_clust(fs, dir, dcl);				 
					dir[11] = 0x10;				 
					fs->wflag = 1;
				}
				if (res == FR_OK) res = sync_fs(fs);
			} else {
				remove_chain(&dj.obj, dcl, 0);		 
			}
		}
		;
	}

	return res;
}




 
 
 

FRESULT f_rename (
	const TCHAR* path_old,	 
	const TCHAR* path_new	 
)
{
	FRESULT res;
	DIR djo, djn;
	FATFS *fs;
	BYTE buf[0 ? 32 * 2 : 24], *dir;
	DWORD dw;
	


	get_ldnumber(&path_new);						 
	res = find_volume(&path_old, &fs, 0x02);	 
	if (res == FR_OK) {
		djo.obj.fs = fs;
		;
		res = follow_path(&djo, path_old);		 
		if (res == FR_OK && (djo.fn[11] & (0x20 | 0x80))) res = FR_INVALID_NAME;	 



		if (res == FR_OK) {						 
#line 4687 "..\\FatFS\\src\\ff.c"
			{	 
				mem_cpy(buf, djo.dir + 11, 21);	 
				mem_cpy(&djn, &djo, sizeof (DIR));		 
				res = follow_path(&djn, path_new);		 
				if (res == FR_OK) {						 
					res = (djn.obj.sclust == djo.obj.sclust && djn.dptr == djo.dptr) ? FR_NO_FILE : FR_EXIST;
				}
				if (res == FR_NO_FILE) { 				 
					res = dir_register(&djn);			 
					if (res == FR_OK) {
						dir = djn.dir;					 
						mem_cpy(dir + 13, buf + 2, 19);
						dir[11] = buf[0] | 0x20;
						fs->wflag = 1;
						if ((dir[11] & 0x10) && djo.obj.sclust != djn.obj.sclust) {	 
							dw = clust2sect(fs, ld_clust(fs, dir));
							if (!dw) {
								res = FR_INT_ERR;
							} else {
 
								res = move_window(fs, dw);
								dir = fs->win + 32 * 1;	 
								if (res == FR_OK && dir[1] == '.') {
									st_clust(fs, dir, djn.obj.sclust);
									fs->wflag = 1;
								}
							}
						}
					}
				}
			}
			if (res == FR_OK) {
				res = dir_remove(&djo);		 
				if (res == FR_OK) {
					res = sync_fs(fs);
				}
			}
 
		}
		;
	}

	return res;
}








#line 4826 "..\\FatFS\\src\\ff.c"



#line 5042 "..\\FatFS\\src\\ff.c"



#line 5132 "..\\FatFS\\src\\ff.c"



#line 5203 "..\\FatFS\\src\\ff.c"



#line 5730 "..\\FatFS\\src\\ff.c"




