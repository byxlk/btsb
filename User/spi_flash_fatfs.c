/*
*********************************************************************************************************
*
*	ģ������ : SPI Flash��Fat�ļ�ϵͳ��ʾģ�顣
*	�ļ����� : demo_spi_flash_fatfs.c
*	��    �� : V1.0
*	˵    �� : ��������ֲFatFS�ļ�ϵͳ���汾 R0.10b������ʾ��δ����ļ�����ȡ�ļ�������Ŀ¼��ɾ���ļ�
*			   ���������ļ���д�ٶȡ�
*
*	�޸ļ�¼ :
*		�汾��   ����        ����     ˵��
*		V1.0    2014-06-15  armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "ff.h"			/* FatFS�ļ�ϵͳģ��*/
#include "spi_flash_fatfs.h"

/* ���ڲ��Զ�д�ٶ� */
#define TEST_FILE_LEN			(2*1024*1024)	/* ���ڲ��Ե��ļ����� */
#define BUF_SIZE				(4*1024)		/* ÿ�ζ�дSD����������ݳ��� */
uint8_t g_TestBuf[BUF_SIZE];

/* FatFs API�ķ���ֵ */
static const char * FR_Table[]=
{
	"FR_OK���ɹ�",				                             /* (0) Succeeded */
	"FR_DISK_ERR���ײ�Ӳ������",			                 /* (1) A hard error occurred in the low level disk I/O layer */
	"FR_INT_ERR������ʧ��",				                     /* (2) Assertion failed */
	"FR_NOT_READY����������û�й���",			             /* (3) The physical drive cannot work */
	"FR_NO_FILE���ļ�������",				                 /* (4) Could not find the file */
	"FR_NO_PATH��·��������",				                 /* (5) Could not find the path */
	"FR_INVALID_NAME����Ч�ļ���",		                     /* (6) The path name format is invalid */
	"FR_DENIED�����ڽ�ֹ���ʻ���Ŀ¼�������ʱ��ܾ�",         /* (7) Access denied due to prohibited access or directory full */
	"FR_EXIST���ļ��Ѿ�����",			                     /* (8) Access denied due to prohibited access */
	"FR_INVALID_OBJECT���ļ�����Ŀ¼������Ч",		         /* (9) The file/directory object is invalid */
	"FR_WRITE_PROTECTED������������д����",		             /* (10) The physical drive is write protected */
	"FR_INVALID_DRIVE���߼���������Ч",		                 /* (11) The logical drive number is invalid */
	"FR_NOT_ENABLED�������޹�����",			                 /* (12) The volume has no work area */
	"FR_NO_FILESYSTEM��û����Ч��FAT��",		             /* (13) There is no valid FAT volume */
	"FR_MKFS_ABORTED�����ڲ�������f_mkfs()����ֹ",	         /* (14) The f_mkfs() aborted due to any parameter error */
	"FR_TIMEOUT���ڹ涨��ʱ�����޷���÷��ʾ�����",		 /* (15) Could not get a grant to access the volume within defined period */
	"FR_LOCKED�������ļ�������Բ������ܾ�",				 /* (16) The operation is rejected according to the file sharing policy */
	"FR_NOT_ENOUGH_CORE���޷����䳤�ļ���������",		     /* (17) LFN working buffer could not be allocated */
	"FR_TOO_MANY_OPEN_FILES����ǰ�򿪵��ļ�������_FS_SHARE", /* (18) Number of open files > _FS_SHARE */
	"FR_INVALID_PARAMETER��������Ч"	                     /* (19) Given parameter is invalid */
};

/*
*********************************************************************************************************
*	�� �� ��: DispMenu
*	����˵��: ��ʾ������ʾ�˵�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void DispMenu(void)
{
	printf("\r\n------------------------------------------------\r\n");
	printf("��һ��ʹ����ѡ������0����SPI Flash��ʽ��\r\n");
	printf("��ѡ���������:\r\n");
	printf("0 - ��SPI_Flash�����ļ�ϵͳ��ʽ��\r\n");
	printf("1 - ��ʾ��Ŀ¼�µ��ļ��б�\r\n");
	printf("2 - ����armfly.txt,�������ļ�������\r\n");
	printf("3 - ����Ŀ¼\r\n");
	printf("4 - ɾ���ļ���Ŀ¼\r\n");
	printf("5 - ��д�ļ��ٶȲ���\r\n");
    printf("6 - ��ȡ������Ϣ\r\n");
    printf("7 - ʹ��YmodemЭ�������ļ�\r\n");
    printf("8 - ʹ��YmodemЭ���ϴ��ļ�\r\n");
    printf("9 - ����Firmware��ָ���ĵ�ַ\r\n");
}

/*
*********************************************************************************************************
*	�� �� ��: MountFS
*	����˵��: �ļ�ϵͳ��ʽ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MountFS(FATFS *fs, uint8_t opt)
{
    /* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;

	/* �����ļ�ϵͳ */
	result = f_mount(fs, "0:", opt);
	if (result != FR_OK)
	{
		printf("%s�ļ�ϵͳʧ�� (%s)\r\n",(fs)? "����" : "ж��"  ,FR_Table[result]);
	}
	//else
	//{
	//	printf("%s�ļ�ϵͳ�ɹ� (%s)\r\n",(fs)? "����" : "ж��" ,FR_Table[result]);
	//}
}


/*
*********************************************************************************************************
*	�� �� ��: FileFormat
*	����˵��: �ļ�ϵͳ��ʽ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void FileFormat(void)
{
	/* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;
	FATFS fs;

	/* �����ļ�ϵͳ */
    MountFS(&fs, 0);

	/* ��һ��ʹ�ñ�����и�ʽ�� */
	result = f_mkfs("0:",0,0);
	if (result != FR_OK)
	{
		printf("��ʽ��ʧ�� (%s)\r\n", FR_Table[result]);
	}
	else
	{
		printf("��ʽ���ɹ� (%s)\r\n", FR_Table[result]);
	}

	/* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: ViewRootDir
*	����˵��: ��ʾSD����Ŀ¼�µ��ļ���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ViewRootDir(void)
{
	/* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;
	FATFS fs;
	DIR DirInf;
	FILINFO FileInf;
	uint32_t cnt = 0;
	char lfname[256];

	/* �����ļ�ϵͳ */
	MountFS(&fs, 0);

	/* �򿪸��ļ��� */
	result = f_opendir(&DirInf, "0:/"); /* ���������������ӵ�ǰĿ¼��ʼ */
	if (result != FR_OK)
	{
		printf("�򿪸�Ŀ¼ʧ�� (%s)\r\n", FR_Table[result]);
		return;
	}

	/* ��ȡ��ǰ�ļ����µ��ļ���Ŀ¼ */
	FileInf.lfname = lfname;
	FileInf.lfsize = 256;

	printf("����        |  �ļ���С | ���ļ��� | ���ļ���\r\n");
	for (cnt = 0; ;cnt++)
	{
		result = f_readdir(&DirInf,&FileInf); 		/* ��ȡĿ¼��������Զ����� */
		if (result != FR_OK || FileInf.fname[0] == 0)
		{
			break;
		}

		if (FileInf.fname[0] == '.')
		{
			continue;
		}

		/* �ж����ļ�������Ŀ¼ */
		if (FileInf.fattrib & AM_DIR)
		{
			printf("(0x%02d)Ŀ¼  ", FileInf.fattrib);
		}
		else
		{
			printf("(0x%02d)�ļ�  ", FileInf.fattrib);
		}

		/* ��ӡ�ļ���С, ���4G */
		printf(" %10d", FileInf.fsize);

		printf("  %s |", FileInf.fname);	/* ���ļ��� */

		printf("  %s\r\n", (char *)FileInf.lfname);	/* ���ļ��� */
	}

	/* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: CreateNewFile
*	����˵��: ��SD������һ�����ļ����ļ�������д��www.armfly.com��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void CreateNewFile(void)
{
	/* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;
	FATFS fs;
	FIL file;
	DIR DirInf;
	uint32_t bw;

 	/* �����ļ�ϵͳ */
	MountFS(&fs, 0);

	/* �򿪸��ļ��� */
	result = f_opendir(&DirInf, "0:/"); /* ���������������ӵ�ǰĿ¼��ʼ */
	if (result != FR_OK)
	{
		printf("�򿪸�Ŀ¼ʧ�� (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* ���ļ� */
	result = f_open(&file, "armfly.txt", FA_CREATE_ALWAYS | FA_WRITE);

	/* дһ������ */
	result = f_write(&file, "FatFS Write Demo \r\n www.armfly.com \r\n", 34, &bw);
	if (result == FR_OK)
	{
		printf("armfly.txt �ļ�д��ɹ�\r\n");
	}
	else
	{
		printf("armfly.txt �ļ�д��ʧ��\r\n");
	}

	/* �ر��ļ�*/
	f_close(&file);

	/* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: ReadFileData
*	����˵��: ��ȡ�ļ�armfly.txtǰ128���ַ�������ӡ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ReadFileData(void)
{
	/* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;
	FATFS fs;
	FIL file;
	DIR DirInf;
	uint32_t bw;
	char buf[128];

 	/* �����ļ�ϵͳ */
	MountFS(&fs, 0);

	/* �򿪸��ļ��� */
	result = f_opendir(&DirInf, "/"); /* ���������������ӵ�ǰĿ¼��ʼ */
	if (result != FR_OK)
	{
		printf("�򿪸�Ŀ¼ʧ��(%s)\r\n",  FR_Table[result]);
		return;
	}

	/* ���ļ� */
	result = f_open(&file, "armfly.txt", FA_OPEN_EXISTING | FA_READ);
	if (result !=  FR_OK)
	{
		printf("Don't Find File : armfly.txt\r\n");
		return;
	}

	/* ��ȡ�ļ� */
	result = f_read(&file, &buf, sizeof(buf) - 1, &bw);
	if (bw > 0)
	{
		buf[bw] = 0;
		printf("\r\narmfly.txt �ļ����� : \r\n%s\r\n", buf);
	}
	else
	{
		printf("\r\narmfly.txt �ļ����� : \r\n");
	}

	/* �ر��ļ�*/
	f_close(&file);

	/* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: CreateDir
*	����˵��: ��SD����Ŀ¼����Dir1��Dir2Ŀ¼����Dir1Ŀ¼�´�����Ŀ¼Dir1_1
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void CreateDir(void)
{
	/* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;
	FATFS fs;

 	MountFS(&fs, 0);

	/* ����Ŀ¼/Dir1 */
	result = f_mkdir("/Dir1");
	if (result == FR_OK)
	{
		printf("f_mkdir Dir1 Ok\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir1 Ŀ¼�Ѿ�����(%s)\r\n",  FR_Table[result]);
	}
	else
	{
		printf("f_mkdir Dir1 ʧ�� (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* ����Ŀ¼/Dir2 */
	result = f_mkdir("/Dir2");
	if (result == FR_OK)
	{
		printf("f_mkdir Dir2 Ok\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir2 Ŀ¼�Ѿ�����(%s)\r\n",  FR_Table[result]);
	}
	else
	{
		printf("f_mkdir Dir2 ʧ�� (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* ������Ŀ¼ /Dir1/Dir1_1	   ע�⣺������Ŀ¼Dir1_1ʱ�������ȴ�����Dir1 */
	result = f_mkdir("/Dir1/Dir1_1"); /* */
	if (result == FR_OK)
	{
		printf("f_mkdir Dir1_1 �ɹ�\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir1_1 Ŀ¼�Ѿ����� (%s)\r\n",  FR_Table[result]);
	}
	else
	{
		printf("f_mkdir Dir1_1 ʧ�� (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: DeleteDirFile
*	����˵��: ɾ��SD����Ŀ¼�µ� armfly.txt �ļ��� Dir1��Dir2 Ŀ¼
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void DeleteDirFile(void)
{
	/* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;
	FATFS fs;
	char FileName[13];
	uint8_t i;

 	/* �����ļ�ϵͳ */
	MountFS(&fs, 0);

	#if 0
	/* �򿪸��ļ��� */
	result = f_opendir(&DirInf, "/"); /* ���������������ӵ�ǰĿ¼��ʼ */
	if (result != FR_OK)
	{
		printf("�򿪸�Ŀ¼ʧ��(%s)\r\n",  FR_Table[result]);
		return;
	}
	#endif

	/* ɾ��Ŀ¼/Dir1 ����Ϊ������Ŀ¼�ǿգ�������Ŀ¼)���������ɾ����ʧ�ܡ�*/
	result = f_unlink("/Dir1");
	if (result == FR_OK)
	{
		printf("ɾ��Ŀ¼Dir1�ɹ�\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("û�з����ļ���Ŀ¼ :%s\r\n", "/Dir1");
	}
	else
	{
		printf("ɾ��Dir1ʧ��(������� = %s) �ļ�ֻ����Ŀ¼�ǿ�\r\n",  FR_Table[result]);
	}

	/* ��ɾ��Ŀ¼/Dir1/Dir1_1 */
	result = f_unlink("/Dir1/Dir1_1");
	if (result == FR_OK)
	{
		printf("ɾ����Ŀ¼/Dir1/Dir1_1�ɹ�\r\n");
	}
	else if ((result == FR_NO_FILE) || (result == FR_NO_PATH))
	{
		printf("û�з����ļ���Ŀ¼ :%s\r\n", "/Dir1/Dir1_1");
	}
	else
	{
		printf("ɾ����Ŀ¼/Dir1/Dir1_1ʧ��(������� = %s) �ļ�ֻ����Ŀ¼�ǿ�\r\n",  FR_Table[result]);
	}

	/* ��ɾ��Ŀ¼/Dir1 */
	result = f_unlink("/Dir1");
	if (result == FR_OK)
	{
		printf("ɾ��Ŀ¼Dir1�ɹ�\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("û�з����ļ���Ŀ¼ :%s\r\n", "/Dir1");
	}
	else
	{
		printf("ɾ��Dir1ʧ��(������� = %s) �ļ�ֻ����Ŀ¼�ǿ�\r\n",  FR_Table[result]);
	}

	/* ɾ��Ŀ¼/Dir2 */
	result = f_unlink("/Dir2");
	if (result == FR_OK)
	{
		printf("ɾ��Ŀ¼ Dir2 �ɹ�\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("û�з����ļ���Ŀ¼ :%s\r\n", "/Dir2");
	}
	else
	{
		printf("ɾ��Dir2 ʧ��(������� = %s) �ļ�ֻ����Ŀ¼�ǿ�\r\n",  FR_Table[result]);
	}

	/* ɾ���ļ� armfly.txt */
	result = f_unlink("armfly.txt");
	if (result == FR_OK)
	{
		printf("ɾ���ļ� armfly.txt �ɹ�\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("û�з����ļ���Ŀ¼ :%s\r\n", "armfly.txt");
	}
	else
	{
		printf("ɾ��armfly.txtʧ��(������� = %s) �ļ�ֻ����Ŀ¼�ǿ�\r\n",  FR_Table[result]);
	}

	/* ɾ���ļ� speed1.txt */
	for (i = 0; i < 20; i++)
	{
		sprintf(FileName, "Speed%02d.txt", i);		/* ÿд1�Σ���ŵ��� */
		result = f_unlink(FileName);
		if (result == FR_OK)
		{
			printf("ɾ���ļ�%s�ɹ�\r\n", FileName);
		}
		else if (result == FR_NO_FILE)
		{
			printf("û�з����ļ�:%s\r\n", FileName);
		}
		else
		{
			printf("ɾ��%s�ļ�ʧ��(������� = %d) �ļ�ֻ����Ŀ¼�ǿ�\r\n", FileName, result);
		}
	}

	/* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: WriteFileTest
*	����˵��: �����ļ���д�ٶ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void WriteFileTest(void)
{
	/* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
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

  	/* �����ļ�ϵͳ */
	MountFS(&fs, 0);

	/* �򿪸��ļ��� */
	result = f_opendir(&DirInf, "/"); /* ���������������ӵ�ǰĿ¼��ʼ */
	if (result != FR_OK)
	{
		printf("�򿪸�Ŀ¼ʧ�� (%s)\r\n",  FR_Table[result]);
		return;
	}

	/* ���ļ� */
	sprintf(TestFileName, "Speed%02d.txt", s_ucTestSn++);		/* ÿд1�Σ���ŵ��� */
	result = f_open(&file, TestFileName, FA_CREATE_ALWAYS | FA_WRITE);

	/* дһ������ */
	printf("��ʼд�ļ�%s %dKB ...\r\n", TestFileName, TEST_FILE_LEN / 1024);
	runtime1 = bsp_GetRunTime();	/* ��ȡϵͳ����ʱ�� */
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
			printf("%s�ļ�дʧ��\r\n", TestFileName);
			break;
		}
	}
	runtime2 = bsp_GetRunTime();	/* ��ȡϵͳ����ʱ�� */

	if (err == 0)
	{
		timelen = (runtime2 - runtime1);
		printf("\r\n  д��ʱ : %dms   ƽ��д�ٶ� : %dB/S (%dKB/S)\r\n",
			timelen,
			(TEST_FILE_LEN * 1000) / timelen,
			((TEST_FILE_LEN / 1024) * 1000) / timelen);
	}

	f_close(&file);		/* �ر��ļ�*/


	/* ��ʼ���ļ����� */
	result = f_open(&file, TestFileName, FA_OPEN_EXISTING | FA_READ);
	if (result !=  FR_OK)
	{
		printf("û���ҵ��ļ�: %s\r\n", TestFileName);
		return;
	}

	printf("��ʼ���ļ� %dKB ...\r\n", TEST_FILE_LEN / 1024);
	runtime1 = bsp_GetRunTime();	/* ��ȡϵͳ����ʱ�� */
	for (i = 0; i < TEST_FILE_LEN / BUF_SIZE; i++)
	{
		result = f_read(&file, g_TestBuf, sizeof(g_TestBuf), &bw);
		if (result == FR_OK)
		{
			if (((i + 1) % 8) == 0)
			{
				printf(".");
			}

			/* �Ƚ�д��������Ƿ���ȷ�������ᵼ�¶����ٶȽ�����͵� 3.5MBytes/S */
			for (k = 0; k < sizeof(g_TestBuf); k++)
			{
				if (g_TestBuf[k] != (k / 512) + '0')
				{
				  	err = 1;
					printf("Speed1.txt �ļ����ɹ����������ݳ���\r\n");
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
			printf("Speed1.txt �ļ���ʧ��\r\n");
			break;
		}
	}
	runtime2 = bsp_GetRunTime();	/* ��ȡϵͳ����ʱ�� */

	if (err == 0)
	{
		timelen = (runtime2 - runtime1);
		printf("\r\n  ����ʱ : %dms   ƽ�����ٶ� : %dB/S (%dKB/S)\r\n", timelen,
			(TEST_FILE_LEN * 1000) / timelen, ((TEST_FILE_LEN / 1024) * 1000) / timelen);
	}

	/* �ر��ļ�*/
	f_close(&file);

	/* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

void GetDiskInfo(void)
{
	/* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;
	FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect;

	/* �����ļ�ϵͳ */
	MountFS(fs, 0);

    /* Get volume information and free clusters of drive 1 */
    result = f_getfree("0:", &fre_clust, &fs);
    if (result != FR_OK)
	{
		printf("��ȡ������Ϣʧ��(%s)\r\n",  FR_Table[result]);
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

    /* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: DownloadFile
*	����˵��: FatFS�ļ�ϵͳ��ʾ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DownloadFile(void)
{
    /* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	FRESULT result;
	FATFS fs;
	FIL file;
	DIR DirInf;
	uint32_t bw;
    uint8_t i = 0;
    uint8_t transfer_end_flag = 0;
    int8_t target_path[128]="0:/";
    int8_t target_filename[128] = "armfly.txt";
    uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD] = {0};

	/* �����ļ�ϵͳ */
	MountFS(&fs, 0);
    //Send_Byte(COM1, MODEM_C);
    //Recv_Byte(COM1, &i, NAK_TIMEOUT);
    //printf("recv data:%c \r\n",i);
#if 1
    /* �򿪸��ļ��� */
    //printf("Please input the path:");
    //scanf("%s",target_path);
    //printf("Open director: %s\r\n",target_path);
	result = f_opendir(&DirInf, target_path); /* ���������������ӵ�ǰĿ¼��ʼ */
	if (result != FR_OK)
	{
		printf("�򿪸�Ŀ¼ʧ�� (%s)\r\n",  FR_Table[result]);
		return;
	}

    while(1)
    {
        if(transfer_end_flag == 2)// Terminna transfer data
            break;

        switch(Ymodem_Receive(COM1,packet_data))
        {
            case RECV_STATUS_SOH_DAT_OK:
                if(transfer_end_flag == 1)
                {
                    for(i = 0; i < PACKET_SIZE; i++)
                    {
                        if(packet_data[i] != 0)
                        {
                            transfer_end_flag = 0;
                            break;
                        }
                        else
                            transfer_end_flag = 2;
                    }
                }
                if(transfer_end_flag == 2)
                    break;
                /* ��ȡ�����ļ������� */
                if (packet_data[PACKET_HEADER] != 0)
                {
                    for(i = 0; i < PACKET_SIZE; i++)
                    {
                        if(packet_data[i] != ' ')
                            target_filename[i] = packet_data[i];
                        else
                        {
                            printf("Recive file name: %s \r\n",target_filename);
                            break;
                        }
                    }
                }
    	        /* ���ļ� */
    	        result = f_open(&file, target_filename, FA_CREATE_ALWAYS | FA_WRITE);
                if (result != FR_OK)
    	        {
    		        printf("(%s)�ļ���ʧ��(%s)\r\n",target_filename, FR_Table[result]);
    	        }
                break;
            case RECV_STATUS_STX_DAT_OK:
                /* дһ������ */
    	        result = f_write(&file, packet_data, 34, &bw);
    	        if (result != FR_OK)
    	        {
    		        printf("(%s)�ļ�д��ʧ��(%s)\r\n",target_filename, FR_Table[result]);
    	        }
                break;
            case RECV_STATUS_TRANSFER_END:
                transfer_end_flag = 1;
                /* �ر��ļ�*/
                if(transfer_end_flag == 1)
                {
	                f_close(&file);
                    transfer_end_flag = 0;
                }
                break;
        }

    }
#endif
    /* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: UploadFile
*	����˵��: FatFS�ļ�ϵͳ��ʾ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void UploadFile(void)
{
    /* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	//FRESULT result;
	FATFS fs;

	/* �����ļ�ϵͳ */
	MountFS(&fs, 0);

    printf("\r\nWaiting for the file to be sent ... (press 'a' to abort)\n\r");


    /* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: DownloadFile
*	����˵��: FatFS�ļ�ϵͳ��ʾ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void UpdateFirmware(void)
{
    /* ������ʹ�õľֲ�����ռ�ý϶࣬���޸������ļ�����֤��ջ�ռ乻�� */
	//FRESULT result;
	FATFS fs;

	/* �����ļ�ϵͳ */
	MountFS(&fs, 0);

    printf("\r\nWaiting for the file to be sent ... (press 'a' to abort)\n\r");


    /* ж���ļ�ϵͳ */
	MountFS(NULL, 0);
}

/*
*********************************************************************************************************
*	�� �� ��: DemoFatFS
*	����˵��: FatFS�ļ�ϵͳ��ʾ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DemoFatFS(void)
{
	uint8_t cmd;

	/* ��ӡ�����б��û�����ͨ�����ڲ���ָ�� */
	DispMenu();
	//bsp_StartAutoTimer(1, 100);
	while(1)
	{
		bsp_Idle();		/* ���������bsp.c�ļ����û������޸��������ʵ��CPU���ߺ�ι�� */

		//if(bsp_CheckTimer(1))
		//{
		//	bsp_LedToggle(1);
		//}
        vTaskDelay(100);
		if (comGetChar(COM1, &cmd))	/* �Ӵ��ڶ���һ���ַ�(��������ʽ) */
		{
			printf("\r\n");
			switch (cmd)
			{
				case '0':
					printf("��0 - FileFormat��\r\n");
					FileFormat();		/* ��ʾSD����Ŀ¼�µ��ļ��� */
					break;

				case '1':
					printf("��1 - ViewRootDir��\r\n");
					ViewRootDir();		/* ��ʾSD����Ŀ¼�µ��ļ��� */
					break;

				case '2':
					printf("��2 - CreateNewFile��\r\n");
					CreateNewFile();		/* ����һ�����ļ�,д��һ���ַ��� */
					printf("��2 - ReadFileData��\r\n");
					ReadFileData();		/* ��ȡ��Ŀ¼��armfly.txt������ */
                    break;

				case '3':
					printf("��3 - CreateDir��\r\n");
					CreateDir();		/* ����Ŀ¼ */
					break;

				case '4':
					printf("��4 - DeleteDirFile��\r\n");
					DeleteDirFile();	/* ɾ��Ŀ¼���ļ� */
					break;

				case '5':
					printf("��5 - TestSpeed��\r\n");
					WriteFileTest();	/* �ٶȲ��� */
					break;

                case '6':
                    printf("��6 - DiskInfo��\r\n");
                    GetDiskInfo();
                    break;

                case '7':
                    printf("��7 - Download File��\r\n");
                    DownloadFile();
                    break;

                case '8':
                    printf("��8 - Upoad File��\r\n");
                    UploadFile();
                    break;

                case '9':
                    printf("��9 - Update Firmware��\r\n");
                    UpdateFirmware();
                    break;

				default:
					DispMenu();
					break;
			}
		}
	}
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
