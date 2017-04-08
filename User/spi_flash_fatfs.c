/*
*********************************************************************************************************
*
*	模块名称 : SPI Flash的Fat文件系统演示模块。
*	文件名称 : demo_spi_flash_fatfs.c
*	版    本 : V1.0
*	说    明 : 该例程移植FatFS文件系统（版本 R0.10b），演示如何创建文件、读取文件、创建目录和删除文件
*			   并测试了文件读写速度。
*
*	修改记录 :
*		版本号   日期        作者     说明
*		V1.0    2014-06-15  armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "ff.h"			/* FatFS文件系统模块*/
#include "spi_flash_fatfs.h"

/* 用于测试读写速度 */
#define TEST_FILE_LEN			(2*1024*1024)	/* 用于测试的文件长度 */
#define BUF_SIZE				(4*1024)		/* 每次读写SD卡的最大数据长度 */
uint8_t g_TestBuf[BUF_SIZE];

/* FatFs API的返回值 */
static const char * FR_Table[]=
{
	"FR_OK：成功",				                             /* (0) Succeeded */
	"FR_DISK_ERR：底层硬件错误",			                 /* (1) A hard error occurred in the low level disk I/O layer */
	"FR_INT_ERR：断言失败",				                     /* (2) Assertion failed */
	"FR_NOT_READY：物理驱动没有工作",			             /* (3) The physical drive cannot work */
	"FR_NO_FILE：文件不存在",				                 /* (4) Could not find the file */
	"FR_NO_PATH：路径不存在",				                 /* (5) Could not find the path */
	"FR_INVALID_NAME：无效文件名",		                     /* (6) The path name format is invalid */
	"FR_DENIED：由于禁止访问或者目录已满访问被拒绝",         /* (7) Access denied due to prohibited access or directory full */
	"FR_EXIST：文件已经存在",			                     /* (8) Access denied due to prohibited access */
	"FR_INVALID_OBJECT：文件或者目录对象无效",		         /* (9) The file/directory object is invalid */
	"FR_WRITE_PROTECTED：物理驱动被写保护",		             /* (10) The physical drive is write protected */
	"FR_INVALID_DRIVE：逻辑驱动号无效",		                 /* (11) The logical drive number is invalid */
	"FR_NOT_ENABLED：卷中无工作区",			                 /* (12) The volume has no work area */
	"FR_NO_FILESYSTEM：没有有效的FAT卷",		             /* (13) There is no valid FAT volume */
	"FR_MKFS_ABORTED：由于参数错误f_mkfs()被终止",	         /* (14) The f_mkfs() aborted due to any parameter error */
	"FR_TIMEOUT：在规定的时间内无法获得访问卷的许可",		 /* (15) Could not get a grant to access the volume within defined period */
	"FR_LOCKED：由于文件共享策略操作被拒绝",				 /* (16) The operation is rejected according to the file sharing policy */
	"FR_NOT_ENOUGH_CORE：无法分配长文件名工作区",		     /* (17) LFN working buffer could not be allocated */
	"FR_TOO_MANY_OPEN_FILES：当前打开的文件数大于_FS_SHARE", /* (18) Number of open files > _FS_SHARE */
	"FR_INVALID_PARAMETER：参数无效"	                     /* (19) Given parameter is invalid */
};

/*
********************************************************************************
  * @brief  Receive byte from sender
  * @param  c: Character
  * @param  timeout: Timeout
  * @retval 0: Byte received
  *        -1: Timeout
********************************************************************************
*/
int32_t Recv_Byte (COM_PORT_E _ucPort, uint8_t *_pByte, uint32_t timeout)
{
#if USING_FIFO_EN == 1
    while (timeout-- > 0)
    {
        if(comGetChar(_ucPort, _pByte) == 1)
        {
            return 0;
        }
    }
#else
#if 1
    while (timeout-- > 0)
    {
        if (SerialKeyPressed(_ucPort, _pByte) == 1)
        {
            return 0;
        }
    }
#else
    while(!SerialKeyPressed(_ucPort, _pByte)) {};
    return 0;
#endif
#endif

    return -1;
}

/*
********************************************************************************
  * @brief  Send a byte
  * @param  c: Character
  * @retval 0: Byte sent
********************************************************************************
*/
uint32_t Send_Byte (COM_PORT_E _ucPort, uint8_t _pByte)
{
#if USING_FIFO_EN == 1
    comSendChar(_ucPort, _pByte);
#else
    SerialPutChar(_ucPort, _pByte);
#endif

    return 0;
}


/*
*********************************************************************************************************
*	函 数 名: DispMenu
*	功能说明: 显示操作提示菜单
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void DispMenu(void)
{
	printf("\r\n------------------------------------------------\r\n");
	printf("第一次使用请选择命令0进行SPI Flash格式化\r\n");
	printf("请选择操作命令:\r\n");
	printf("0 - 对SPI_Flash进行文件系统格式化\r\n");
	printf("1 - 显示根目录下的文件列表\r\n");
	printf("2 - 创建armfly.txt,并读出文件的内容\r\n");
	printf("3 - 创建目录\r\n");
	printf("4 - 删除文件和目录\r\n");
	printf("5 - 读写文件速度测试\r\n");
    printf("6 - 获取磁盘信息\r\n");
    printf("7 - 使用Ymodem协议下载文件\r\n");
    printf("8 - 使用Ymodem协议上传文件\r\n");
    printf("9 - 升级Firmware到指定的地址\r\n");
}

/*
*********************************************************************************************************
*	函 数 名: MountFS
*	功能说明: 文件系统格式化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void MountFS(FATFS *fs, uint8_t opt)
{
    /* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;

	/* 挂载文件系统 */
	result = f_mount(fs, "0:", opt);
	if (result != FR_OK)
	{
		printf("%s文件系统失败 (%s)\r\n",(fs)? "挂载" : "卸载"  ,FR_Table[result]);
	}
	//else
	//{
	//	printf("%s文件系统成功 (%s)\r\n",(fs)? "挂载" : "卸载" ,FR_Table[result]);
	//}
}


/*
*********************************************************************************************************
*	函 数 名: FileFormat
*	功能说明: 文件系统格式化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void FileFormat(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS fs;

	/* 挂载文件系统 */
    MountFS(&fs, 0);

	/* 第一次使用必须进行格式化 */
	result = f_mkfs("0:",0,0);
	if (result != FR_OK)
	{
		printf("格式化失败 (%s)\r\n", FR_Table[result]);
	}
	else
	{
		printf("格式化成功 (%s)\r\n", FR_Table[result]);
	}

	/* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: ViewRootDir
*	功能说明: 显示SD卡根目录下的文件名
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ViewRootDir(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS fs;
	DIR DirInf;
	FILINFO FileInf;
	uint32_t cnt = 0;
	char lfname[256];

	/* 挂载文件系统 */
	MountFS(&fs, 0);

	/* 打开根文件夹 */
	result = f_opendir(&DirInf, "0:/"); /* 如果不带参数，则从当前目录开始 */
	if (result != FR_OK)
	{
		printf("打开根目录失败 (%s)\r\n", FR_Table[result]);
		return;
	}

	/* 读取当前文件夹下的文件和目录 */
	FileInf.lfname = lfname;
	FileInf.lfsize = 256;

	printf("属性        |  文件大小 | 文件名\r\n");
	for (cnt = 0; ;cnt++)
	{
		result = f_readdir(&DirInf,&FileInf); 		/* 读取目录项，索引会自动下移 */
		if (result != FR_OK || FileInf.fname[0] == 0)
		{
			break;
		}

		if (FileInf.fname[0] == '.')
		{
			continue;
		}

		/* 判断是文件还是子目录 */
		if (FileInf.fattrib & AM_DIR)
		{
			printf("(0x%02d)目录  ", FileInf.fattrib);
		}
		else
		{
			printf("(0x%02d)文件  ", FileInf.fattrib);
		}

		/* 打印文件大小, 最大4G */
		printf(" %10d", FileInf.fsize);

		//printf("  %s |", FileInf.fname);	/* 短文件名 */

		printf("  %s\r\n", (char *)FileInf.lfname);	/* 长文件名 */
	}

	/* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: CreateNewFile
*	功能说明: 在SD卡创建一个新文件，文件内容填写“www.armfly.com”
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void CreateNewFile(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS fs;
	FIL file;
	DIR DirInf;
	uint32_t bw;

 	/* 挂载文件系统 */
	MountFS(&fs, 0);

	/* 打开根文件夹 */
	result = f_opendir(&DirInf, "0:/"); /* 如果不带参数，则从当前目录开始 */
	if (result != FR_OK)
	{
		printf("打开根目录失败 (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* 打开文件 */
	result = f_open(&file, "armfly.txt", FA_CREATE_ALWAYS | FA_WRITE);

	/* 写一串数据 */
	result = f_write(&file, "FatFS Write Demo \r\n www.armfly.com \r\n", 34, &bw);
	if (result == FR_OK)
	{
		printf("armfly.txt 文件写入成功\r\n");
	}
	else
	{
		printf("armfly.txt 文件写入失败\r\n");
	}

	/* 关闭文件*/
	f_close(&file);

	/* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: ReadFileData
*	功能说明: 读取文件armfly.txt前128个字符，并打印到串口
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ReadFileData(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS fs;
	FIL file;
	DIR DirInf;
	uint32_t bw;
	char buf[128];

 	/* 挂载文件系统 */
	MountFS(&fs, 0);

	/* 打开根文件夹 */
	result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
	if (result != FR_OK)
	{
		printf("打开根目录失败(%s)\r\n",  FR_Table[result]);
		return;
	}

	/* 打开文件 */
	result = f_open(&file, "armfly.txt", FA_OPEN_EXISTING | FA_READ);
	if (result !=  FR_OK)
	{
		printf("Don't Find File : armfly.txt\r\n");
		return;
	}

	/* 读取文件 */
	result = f_read(&file, &buf, sizeof(buf) - 1, &bw);
	if (bw > 0)
	{
		buf[bw] = 0;
		printf("\r\narmfly.txt 文件内容 : \r\n%s\r\n", buf);
	}
	else
	{
		printf("\r\narmfly.txt 文件内容 : \r\n");
	}

	/* 关闭文件*/
	f_close(&file);

	/* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: CreateDir
*	功能说明: 在SD卡根目录创建Dir1和Dir2目录，在Dir1目录下创建子目录Dir1_1
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void CreateDir(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS fs;

 	MountFS(&fs, 0);

	/* 创建目录/Dir1 */
	result = f_mkdir("/Dir1");
	if (result == FR_OK)
	{
		printf("f_mkdir Dir1 Ok\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir1 目录已经存在(%s)\r\n",  FR_Table[result]);
	}
	else
	{
		printf("f_mkdir Dir1 失败 (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* 创建目录/Dir2 */
	result = f_mkdir("/Dir2");
	if (result == FR_OK)
	{
		printf("f_mkdir Dir2 Ok\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir2 目录已经存在(%s)\r\n",  FR_Table[result]);
	}
	else
	{
		printf("f_mkdir Dir2 失败 (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* 创建子目录 /Dir1/Dir1_1	   注意：创建子目录Dir1_1时，必须先创建好Dir1 */
	result = f_mkdir("/Dir1/Dir1_1"); /* */
	if (result == FR_OK)
	{
		printf("f_mkdir Dir1_1 成功\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir1_1 目录已经存在 (%s)\r\n",  FR_Table[result]);
	}
	else
	{
		printf("f_mkdir Dir1_1 失败 (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: DeleteDirFile
*	功能说明: 删除SD卡根目录下的 armfly.txt 文件和 Dir1，Dir2 目录
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void DeleteDirFile(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS fs;
	char FileName[13];
	uint8_t i;

 	/* 挂载文件系统 */
	MountFS(&fs, 0);

	#if 0
	/* 打开根文件夹 */
	result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
	if (result != FR_OK)
	{
		printf("打开根目录失败(%s)\r\n",  FR_Table[result]);
		return;
	}
	#endif

	/* 删除目录/Dir1 【因为还存在目录非空（存在子目录)，所以这次删除会失败】*/
	result = f_unlink("/Dir1");
	if (result == FR_OK)
	{
		printf("删除目录Dir1成功\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("没有发现文件或目录 :%s\r\n", "/Dir1");
	}
	else
	{
		printf("删除Dir1失败(错误代码 = %s) 文件只读或目录非空\r\n",  FR_Table[result]);
	}

	/* 先删除目录/Dir1/Dir1_1 */
	result = f_unlink("/Dir1/Dir1_1");
	if (result == FR_OK)
	{
		printf("删除子目录/Dir1/Dir1_1成功\r\n");
	}
	else if ((result == FR_NO_FILE) || (result == FR_NO_PATH))
	{
		printf("没有发现文件或目录 :%s\r\n", "/Dir1/Dir1_1");
	}
	else
	{
		printf("删除子目录/Dir1/Dir1_1失败(错误代码 = %s) 文件只读或目录非空\r\n",  FR_Table[result]);
	}

	/* 先删除目录/Dir1 */
	result = f_unlink("/Dir1");
	if (result == FR_OK)
	{
		printf("删除目录Dir1成功\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("没有发现文件或目录 :%s\r\n", "/Dir1");
	}
	else
	{
		printf("删除Dir1失败(错误代码 = %s) 文件只读或目录非空\r\n",  FR_Table[result]);
	}

	/* 删除目录/Dir2 */
	result = f_unlink("/Dir2");
	if (result == FR_OK)
	{
		printf("删除目录 Dir2 成功\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("没有发现文件或目录 :%s\r\n", "/Dir2");
	}
	else
	{
		printf("删除Dir2 失败(错误代码 = %s) 文件只读或目录非空\r\n",  FR_Table[result]);
	}

	/* 删除文件 armfly.txt */
	result = f_unlink("armfly.txt");
	if (result == FR_OK)
	{
		printf("删除文件 armfly.txt 成功\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("没有发现文件或目录 :%s\r\n", "armfly.txt");
	}
	else
	{
		printf("删除armfly.txt失败(错误代码 = %s) 文件只读或目录非空\r\n",  FR_Table[result]);
	}

	/* 删除文件 speed1.txt */
	for (i = 0; i < 20; i++)
	{
		sprintf(FileName, "Speed%02d.txt", i);		/* 每写1次，序号递增 */
		result = f_unlink(FileName);
		if (result == FR_OK)
		{
			printf("删除文件%s成功\r\n", FileName);
		}
		else if (result == FR_NO_FILE)
		{
			printf("没有发现文件:%s\r\n", FileName);
		}
		else
		{
			printf("删除%s文件失败(错误代码 = %d) 文件只读或目录非空\r\n", FileName, result);
		}
	}

	/* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: WriteFileTest
*	功能说明: 测试文件读写速度
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void WriteFileTest(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS fs;
	FIL file;
	DIR DirInf;
	uint32_t bw;
	uint32_t i,k;
	uint32_t runtime1,runtime2,timelen;
	uint8_t err = 0;
	char TestFileName[13];
	static uint8_t s_ucTestSn = 0;

	for (i = 0; i < sizeof(g_TestBuf); i++)
	{
		g_TestBuf[i] = (i / 512) + '0';
	}

  	/* 挂载文件系统 */
	MountFS(&fs, 0);

	/* 打开根文件夹 */
	result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
	if (result != FR_OK)
	{
		printf("打开根目录失败 (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* 打开文件 */
	sprintf(TestFileName, "Speed%02d.txt", s_ucTestSn++);		/* 每写1次，序号递增 */
	result = f_open(&file, TestFileName, FA_CREATE_ALWAYS | FA_WRITE);

	/* 写一串数据 */
	printf("开始写文件%s %dKB ...\r\n", TestFileName, TEST_FILE_LEN / 1024);
	runtime1 = bsp_GetRunTime();	/* 读取系统运行时间 */
	for (i = 0; i < TEST_FILE_LEN / BUF_SIZE; i++)
	{
		result = f_write(&file, g_TestBuf, sizeof(g_TestBuf), &bw);
		if (result == FR_OK)
		{
			if (((i + 1) % 8) == 0)
			{
				printf(".");
			}
		}
		else
		{
			err = 1;
			printf("%s文件写失败\r\n", TestFileName);
			break;
		}
	}
	runtime2 = bsp_GetRunTime();	/* 读取系统运行时间 */

	if (err == 0)
	{
		timelen = (runtime2 - runtime1);
		printf("\r\n  写耗时 : %dms   平均写速度 : %dB/S (%dKB/S)\r\n",
			timelen,
			(TEST_FILE_LEN * 1000) / timelen,
			((TEST_FILE_LEN / 1024) * 1000) / timelen);
	}

	f_close(&file);		/* 关闭文件*/


	/* 开始读文件测试 */
	result = f_open(&file, TestFileName, FA_OPEN_EXISTING | FA_READ);
	if (result !=  FR_OK)
	{
		printf("没有找到文件: %s\r\n", TestFileName);
		return;
	}

	printf("开始读文件 %dKB ...\r\n", TEST_FILE_LEN / 1024);
	runtime1 = bsp_GetRunTime();	/* 读取系统运行时间 */
	for (i = 0; i < TEST_FILE_LEN / BUF_SIZE; i++)
	{
		result = f_read(&file, g_TestBuf, sizeof(g_TestBuf), &bw);
		if (result == FR_OK)
		{
			if (((i + 1) % 8) == 0)
			{
				printf(".");
			}

			/* 比较写入的数据是否正确，此语句会导致读卡速度结果降低到 3.5MBytes/S */
			for (k = 0; k < sizeof(g_TestBuf); k++)
			{
				if (g_TestBuf[k] != (k / 512) + '0')
				{
				  	err = 1;
					printf("Speed1.txt 文件读成功，但是数据出错\r\n");
					break;
				}
			}
			if (err == 1)
			{
				break;
			}
		}
		else
		{
			err = 1;
			printf("Speed1.txt 文件读失败\r\n");
			break;
		}
	}
	runtime2 = bsp_GetRunTime();	/* 读取系统运行时间 */

	if (err == 0)
	{
		timelen = (runtime2 - runtime1);
		printf("\r\n  读耗时 : %dms   平均读速度 : %dB/S (%dKB/S)\r\n", timelen,
			(TEST_FILE_LEN * 1000) / timelen, ((TEST_FILE_LEN / 1024) * 1000) / timelen);
	}

	/* 关闭文件*/
	f_close(&file);

	/* 卸载文件系统 */
	MountFS(NULL, 0);
}

void GetDiskInfo(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect;

	/* 挂载文件系统 */
	MountFS(fs, 0);

    /* Get volume information and free clusters of drive 1 */
    result = f_getfree("0:", &fre_clust, &fs);
    if (result != FR_OK)
	{
		printf("获取磁盘信息失败(%s)\r\n",  FR_Table[result]);
	}

    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;

    /* Print the free space (assuming 512 bytes/sector) */
    printf("fs_type:%d drv:%d csize:%d n_fats:%d id:%d\r\n\
            ssize:%d last_clust:%d free_clust:%d fsize:%d\r\n\
            volbase:%d fatbase:%d dirbase:%d datbase:%d\r\n",
        fs->fs_type,		/* FAT sub-type (0:Not mounted) */
	    fs->drv,			/* Physical drive number */
	    fs->csize,			/* Sectors per cluster (1,2,4...128) */
	    fs->n_fats,			/* Number of FAT copies (1 or 2) */
	    fs->id,				/* File system mount ID */
	    fs->ssize,			/* Bytes per sector (512, 1024, 2048 or 4096) */
	    fs->last_clust,		/* Last allocated cluster */
	    fs->free_clust,		/* Number of free clusters */
	    fs->fsize,			/* Sectors per FAT */
	    fs->volbase,		/* Volume start sector */
	    fs->fatbase,		/* FAT start sector */
	    fs->dirbase,		/* Root directory start sector (FAT32:Cluster#) */
	    fs->database		/* Data start sector */
    );
    printf("%10lu KiB Total.\r\n%10lu KiB Available.\r\n",
           tot_sect / 2, fre_sect / 2);

    /* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: DownloadFile
*	功能说明: FatFS文件系统演示主程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void DownloadFile(void)
{
    /* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	FRESULT result;
	FATFS fs;
	FIL RecvFile;
	DIR DirInf;
	uint32_t ByteWrite;
    uint16_t i = 0;

    char ErrorNum=0;            //错误计数
    char YmodemState=0;
    //uint8_t Timeout_TryCount = 0;
    uint16_t NextPackNum = 0;
    unsigned char TempChar=0;   //接收数据
    int CrcValue=0;
    int PacketLen=0;

    //int count=0;
    COM_PORT_E _ucPort=COM1;
    char FileName[128] = {'\0'};

    int32_t FileSize = 0;
    uint8_t Transfer_End = 0;
    char target_path[32]="0:/";
    uint8_t UserBuf[PACKET_1K_SIZE + PACKET_OVERHEAD] = {0};

	/* 挂载文件系统 */
	MountFS(&fs, 0);
    //Send_Byte(COM1, MODEM_C);
    //Recv_Byte(COM1, &i, NAK_TIMEOUT);
    //printf("recv data:%c \r\n",i);

    /* 打开根文件夹 */
    //printf("Please input the path:");
    //scanf("%s",target_path);
    //printf("Open director: %s\r\n",target_path);
	result = f_opendir(&DirInf, target_path); /* 如果不带参数，则从当前目录开始 */
	if (result != FR_OK)
	{
		printf("打开根目录失败 (%s)\r\n",  FR_Table[result]);
		return;
	}

#if 1

    while(1) {
        //vTaskDelay(50);
        bsp_DelayMS(50);
        if(Transfer_End != 0) break;
        switch(YmodemState) {
            case YMODEM_START: //通信起始阶段
                printf("YMODEM_START\r\n");
                Send_Byte(_ucPort, MODEM_C);                //发起始信号
                //每次等待0.2s钟，发生超时重发“C”
                if(Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT) == 0) {
                    if(TempChar==MODEM_SOH )
                    {
                        Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                        if(TempChar != 0x00) { //不是00序号。
                            Transfer_End = 0x03 ;
                            continue;
                        }
                        Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                        if ( TempChar != 0xFF ) {     //不是00序号补码。
                            Transfer_End = 0x03 ;
                            continue;
                        }
                        for ( i=0; i<128; i++ )
                        {                                //接收数据包0，共128字节
                            Recv_Byte(_ucPort, (UserBuf+i), NAK_TIMEOUT);
                        }
                        Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                        Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);  //丢弃CRC校验，暂时不想实现。

                        Send_Byte(_ucPort, MODEM_ACK);             //发送确认信号。
                        YmodemState = YMODEM_DATATRANS;  //状态切换到数据传输状态
                        for ( i=0; i<128; i++ )
                        {                        //接收数据包0，共128字节
                            FileName[i]=0;
                        }
                        for ( i=0; (i<128)&&(UserBuf[i]); i++ )
                        {                          //接收数据包0，共128字节
                            FileName[i]=UserBuf[i];
                        }
                        Str2Int((UserBuf+i+1), &FileSize);
                        //Serial_PutString(_ucPort, FileName);
                        //Serial_PutString(_ucPort, &FileSize);
                        result = f_open( &RecvFile, (const char *)FileName,
                                        FA_READ | FA_WRITE| FA_CREATE_NEW );
                        NextPackNum = 1;
                        //printf("FileName: %s FileSize:%d \r\n",FileName, FileSize);
                        //printf("YMODEM_DATATRANS\r\n");
                        Send_Byte(_ucPort, MODEM_C);   //再发一个C，正式启动数据传输
                    }
                }
                break;
            case YMODEM_DATATRANS:        //数据传输阶段
                Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                switch(TempChar)
                {
                    case MODEM_EOT:
                        YmodemState = YMODEM_ENDOFTRANS;
                        Send_Byte(_ucPort, MODEM_ACK);
                        continue;
                    case MODEM_SOH:
                        PacketLen = 128;
                        break;
                    case MODEM_STX:
                        PacketLen = 1024;
                        break;
                    case USER_ABORT1:
                    case USER_ABORT2:
                    case MODEM_CAN:
                        Send_Byte(_ucPort, MODEM_CAN);
                        Send_Byte(_ucPort, MODEM_CAN);
                        f_unlink(FileName);
                        printf("User cancel transfer data \r\n");
                        Transfer_End = 0x02 ;
                        continue;
                    default:
                        printf("Unknow StartCode: %d packSeq: %d\r\n",TempChar,NextPackNum);
                        continue;
                }  //end of switch (StartChar)
                Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                //if(NextPackNum != TempChar) {
                  //  Transfer_End = 0x03 ;
                    //ErrorNum++;
                    //printf("NextPackNum: %d TempChar:%d\r\n",NextPackNum,TempChar);
                    //continue;
                //}
                Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                for(i = 0; i < PacketLen; i++)
                {      //接收整个数据包
                    Recv_Byte(_ucPort, (UserBuf+i), NAK_TIMEOUT);
                }
                //CRC桥验
                Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                CrcValue = TempChar;
                CrcValue <<= 8;
                Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                CrcValue |= TempChar;

                if(Cal_CRC16(UserBuf, PacketLen) != CrcValue)
                {
                    ErrorNum++;;
                    printf("CrcValue: %d CRC:%d packsize: %d\r\n",
                        CrcValue,Cal_CRC16(UserBuf,PacketLen),PacketLen);
                }

                if(ErrorNum==0)
                {
                    result = f_write ( &RecvFile, (void *)UserBuf,
                        PacketLen, &ByteWrite );//将接收到得数据写入文件
                    NextPackNum++;
                    Send_Byte(_ucPort, MODEM_ACK);
                    //printf("%c",(PacketLen == 128)? '#' : '@');
                }
                else
                {
                    Send_Byte(_ucPort, MODEM_NAK);//接收发现错误，要求重发。
                    printf("Err(%d)\r\n",ErrorNum);
                    ErrorNum--;
                }
                break;
            case YMODEM_ENDOFTRANS:   //结束传输阶段
                printf("YMODEM_ENDOFTRANS\r\n");
                Send_Byte(_ucPort, MODEM_C);
                Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);//接收起始字符。
                if(TempChar==MODEM_SOH)
                {
                    Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                    if(TempChar != 0x00) { //不是00序号。
                        Transfer_End = 0x03 ;
                        continue;
                    }
                    Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                    if ( TempChar != 0xFF ) {     //不是00序号补码。
                        Transfer_End = 0x03 ;
                        continue;
                    }
                    for(i=0;i<128;i++)
                    {
                        Recv_Byte(_ucPort, (UserBuf+i), NAK_TIMEOUT);
                    }
                    //CRC桥验
                    Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                    CrcValue = TempChar;
                    CrcValue <<= 8;
                    Recv_Byte(_ucPort, &TempChar, NAK_TIMEOUT);
                    CrcValue |= TempChar;

					if(Cal_CRC16(UserBuf,128)!=CrcValue)
                    {
                        ErrorNum+=1;
                    }
                    if(ErrorNum==0)
                    {
                        f_lseek ( &RecvFile, FileSize );
				        f_truncate ( &RecvFile );
				        f_close( &RecvFile );
                        Send_Byte(_ucPort, MODEM_ACK);
                        Transfer_End = 0x04 ;
                        continue; ;
                    }
                    else Send_Byte(_ucPort, MODEM_NAK);//接收发现错误，要求重发。
                    break;
                }
                //Transfer_End = 0x04 ;

            default:
                Transfer_End = 0x01 ;
                break ;
        }
    }
#endif
    /* 卸载文件系统 */
    printf("download file end \r\n");
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: UploadFile
*	功能说明: FatFS文件系统演示主程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void UploadFile(void)
{
    /* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	//FRESULT result;
	FATFS fs;

	/* 挂载文件系统 */
	MountFS(&fs, 0);

    printf("\r\nWaiting for the file to be sent ... (press 'a' to abort)\n\r");


    /* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: DownloadFile
*	功能说明: FatFS文件系统演示主程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void UpdateFirmware(void)
{
    /* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
	//FRESULT result;
	FATFS fs;

	/* 挂载文件系统 */
	MountFS(&fs, 0);

    printf("\r\nWaiting for the file to be sent ... (press 'a' to abort)\n\r");


    /* 卸载文件系统 */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	函 数 名: DemoFatFS
*	功能说明: FatFS文件系统演示主程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void DemoFatFS(void)
{
	uint8_t cmd;

	/* 打印命令列表，用户可以通过串口操作指令 */
	DispMenu();
	//bsp_StartAutoTimer(1, 100);
	while(1)
	{
		bsp_Idle();		/* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */

		//if(bsp_CheckTimer(1))
		//{
		//	bsp_LedToggle(1);
		//}
        vTaskDelay(100);
		if (comGetChar(COM1, &cmd))	/* 从串口读入一个字符(非阻塞方式) */
		{
			printf("\r\n");
			switch (cmd)
			{
				case '0':
					printf("【0 - FileFormat】\r\n");
					FileFormat();		/* 显示SD卡根目录下的文件名 */
					break;

				case '1':
					printf("【1 - ViewRootDir】\r\n");
					ViewRootDir();		/* 显示SD卡根目录下的文件名 */
					break;

				case '2':
					printf("【2 - CreateNewFile】\r\n");
					CreateNewFile();		/* 创建一个新文件,写入一个字符串 */
					printf("【2 - ReadFileData】\r\n");
					ReadFileData();		/* 读取根目录下armfly.txt的内容 */
                    break;

				case '3':
					printf("【3 - CreateDir】\r\n");
					CreateDir();		/* 创建目录 */
					break;

				case '4':
					printf("【4 - DeleteDirFile】\r\n");
					DeleteDirFile();	/* 删除目录和文件 */
					break;

				case '5':
					printf("【5 - TestSpeed】\r\n");
					WriteFileTest();	/* 速度测试 */
					break;

                case '6':
                    printf("【6 - DiskInfo】\r\n");
                    GetDiskInfo();
                    break;

                case '7':
                    printf("【7 - Download File】\r\n");
                    DownloadFile();
                    break;

                case '8':
                    printf("【8 - Upoad File】\r\n");
                    UploadFile();
                    break;

                case '9':
                    printf("【9 - Update Firmware】\r\n");
                    UpdateFirmware();
                    break;
                case 0x0D:
                    DispMenu();
					break;
				default:
					//DispMenu();
					break;
			}
		}
	}
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
