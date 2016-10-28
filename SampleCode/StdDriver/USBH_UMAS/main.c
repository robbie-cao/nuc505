/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 14/06/11 6:55p $
 * @brief    Use USB Host core driver, USB mass storage driver, and FATFS 
 *           file system to show a disk access shell interface
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "diskio.h"
#include "ff.h"
#include "usbh_core.h"
#include "usbh_umas.h"


static UINT blen = 16*1024;
volatile UINT Timer = 0;        /* Performance timer (1kHz increment) */
DWORD acc_size;             /* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;
FATFS FatFs[_VOLUMES];      /* File system object for logical drive */
char Line[256];             /* Console input buffer */
#if _USE_LFN
char Lfname[512];
#endif

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[1024] ;       /* Working buffer */
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[1024] ;       /* Working buffer */
#endif

void Delay(uint32_t delayCnt)
{
    while(delayCnt--) {
        __NOP();
        __NOP();
    }
}


/*--------------------------------------------------------------------------*/
/* Monitor                                                                  */

/*----------------------------------------------*/
/* Get a value of the string                    */
/*----------------------------------------------*/
/*  "123 -5   0x3ff 0b1111 0377  w "
        ^                           1st call returns 123 and next ptr
           ^                        2nd call returns -5 and next ptr
                   ^                3rd call returns 1023 and next ptr
                          ^         4th call returns 15 and next ptr
                               ^    5th call returns 255 and next ptr
                                  ^ 6th call fails and returns 0
*/

int xatoi (         /* 0:Failed, 1:Successful */
    TCHAR **str,    /* Pointer to pointer to the string */
    long *res       /* Pointer to a variable to store the value */
)
{
    unsigned long val;
    unsigned char r, s = 0;
    TCHAR c;


    *res = 0;
    while ((c = **str) == ' ') (*str)++;    /* Skip leading spaces */

    if (c == '-') {     /* negative? */
        s = 1;
        c = *(++(*str));
    }

    if (c == '0') {
        c = *(++(*str));
        switch (c) {
        case 'x':       /* hexadecimal */
            r = 16;
            c = *(++(*str));
            break;
        case 'b':       /* binary */
            r = 2;
            c = *(++(*str));
            break;
        default:
            if (c <= ' ') return 1; /* single zero */
            if (c < '0' || c > '9') return 0;   /* invalid char */
            r = 8;      /* octal */
        }
    } else {
        if (c < '0' || c > '9') return 0;   /* EOL or invalid char */
        r = 10;         /* decimal */
    }

    val = 0;
    while (c > ' ') {
        if (c >= 'a') c -= 0x20;
        c -= '0';
        if (c >= 17) {
            c -= 7;
            if (c <= 9) return 0;   /* invalid char */
        }
        if (c >= r) return 0;       /* invalid char for current radix */
        val = val * r + c;
        c = *(++(*str));
    }
    if (s) val = 0 - val;           /* apply sign if needed */

    *res = val;
    return 1;
}


/*----------------------------------------------*/
/* Dump a block of byte array                   */

void put_dump (
    const unsigned char* buff,  /* Pointer to the byte array to be dumped */
    unsigned long addr,         /* Heading address value */
    int cnt                     /* Number of bytes to be dumped */
)
{
    int i;


    printf(_T("%08lX "), addr);

    for (i = 0; i < cnt; i++)
        printf(_T(" %02X"), buff[i]);

    putchar(' ');
    for (i = 0; i < cnt; i++)
        putchar((TCHAR)((buff[i] >= ' ' && buff[i] <= '~') ? buff[i] : '.'));

    putchar('\n');
}


/*--------------------------------------------------------------------------*/
/* Monitor                                                                  */
/*--------------------------------------------------------------------------*/

static
FRESULT scan_files (
    char* path      /* Pointer to the path name working buffer */
)
{
    DIR dirs;
    FRESULT res;
    BYTE i;
    char *fn;


    if ((res = f_opendir(&dirs, path)) == FR_OK) {
        i = strlen(path);
        while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
            if (_FS_RPATH && Finfo.fname[0] == '.') continue;
#if _USE_LFN
            fn = *Finfo.lfname ? Finfo.lfname : Finfo.fname;
#else
            fn = Finfo.fname;
#endif
            if (Finfo.fattrib & AM_DIR) {
                acc_dirs++;
                *(path+i) = '/';
                strcpy(path+i+1, fn);
                res = scan_files(path);
                *(path+i) = '\0';
                if (res != FR_OK) break;
            } else {
                /*              printf("%s/%s\n", path, fn); */
                acc_files++;
                acc_size += Finfo.fsize;
            }
        }
    }

    return res;
}



void put_rc (FRESULT rc)
{
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
    //FRESULT i;
    uint32_t i;
    for (i = 0; (i != (UINT)rc) && *p; i++) {
        while(*p++) ;
    }
    printf(_T("rc=%u FR_%s\n"), (UINT)rc, p);
}

/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/

void get_line (char *buff, int len)
{
    TCHAR c;
    int idx = 0;
//  DWORD dw;


    for (;;) {
        c = getchar();
        putchar(c);
        if (c == '\r') break;
        if ((c == '\b') && idx) idx--;
        if ((c >= ' ') && (idx < len - 1)) buff[idx++] = c;
    }
    buff[idx] = 0;

    putchar('\n');

}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
}

#define HOST_LIKE_PORT1_0						0x10
#define HOST_LIKE_PORT1_1						0x20	
#define HOST_LIKE_PORT2_0						0x00
#define HOST_LIKE_PORT1_DISABLE			0xFF
#define HOST_LIKE_PORT2_DISABLE			0xFF

void USB_PortInit(uint32_t u32Port1, uint32_t u32Port2)
{
		SYS->WAKEUP = SYS->WAKEUP | SYS_WAKEUP_USBHWF_Msk;
		switch(u32Port1) 
		{//port 1
				case HOST_LIKE_PORT1_DISABLE:
						printf("USB host like port 1 Disable\n");
						break;
				case HOST_LIKE_PORT1_0:		
						SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk|SYS_GPB_MFPH_PB13MFP_Msk)) | (0x2 << SYS_GPB_MFPH_PB12MFP_Pos) | (0x2 << SYS_GPB_MFPH_PB13MFP_Pos);														
						printf("USB host like port 1 from GPB12 & GPB13\n");
						break;				
				case HOST_LIKE_PORT1_1:								
						SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk|SYS_GPB_MFPH_PB15MFP_Msk)) | (0x1 << SYS_GPB_MFPH_PB14MFP_Pos) | (0x1 << SYS_GPB_MFPH_PB15MFP_Pos);																
						printf("USB host like port 1 from GPB14 & GPB15\n");
						break;	
		}	
		switch(u32Port2)	
		{//port 2						
				case HOST_LIKE_PORT2_DISABLE:
						printf("USB host like port 2 Disable\n");
						break;		
				case HOST_LIKE_PORT2_0:			
						SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC13MFP_Msk|SYS_GPC_MFPH_PC14MFP_Msk)) | (0x1 << SYS_GPC_MFPH_PC13MFP_Pos) | (0x1 << SYS_GPC_MFPH_PC14MFP_Pos);				
						printf("USB host like port 2 from GPC13 & GPC14\n");
						break;
		}		
		USBH->HcMiscControl = (USBH->HcMiscControl & ~(USBH_HcMiscControl_DPRT1_Msk | USBH_HcMiscControl_DPRT2_Msk)) | ((u32Port1 & 0x01) << USBH_HcMiscControl_DPRT1_Pos) | ((u32Port2 & 0x01) << USBH_HcMiscControl_DPRT2_Pos);

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

		/* USB Host Clock Source MUST be multiple of 48MHz */
    CLK_SetCoreClock(96000000);

    /* Set PCLK divider */
    CLK_SetModuleClock(PCLK_MODULE, NULL, 1);	
	
    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

	  /* Enable USB Host IP clock */
    CLK_EnableModuleClock(USBH_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(USBH_MODULE, CLK_USBH_SRC_PLL, CLK_GetPLLClockFreq() / 48000000 - 1);	
		
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


static FIL file1, file2;        /* File objects */

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    char *ptr, *ptr2;
    long p1, p2, p3;
    BYTE *buf;
    FATFS *fs;              /* Pointer to file system object */
    FRESULT res;
		char Item;
		uint32_t u32UsbhPort1 = HOST_LIKE_PORT1_DISABLE;
    DIR dir;                /* Directory object */
    UINT s1, s2, cnt;
    static const BYTE ft[] = {0, 12, 16, 32};
    DWORD ofs = 0, sect = 0;

		NVIC_DisableIRQ((IRQn_Type)1);

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+-----------------------------------------------+\n");
    printf("|                                               |\n");
    printf("|     USB Host Mass Storage sample program      |\n");
    printf("|                                               |\n");
    printf("+-----------------------------------------------+\n");
	
	
		do
		{
				printf("============================================================================================\n");
				printf("Please select the USB host port 1 through GPIO\n");
				printf("[A] GPB12 and GPB13\n");
				printf("[B] GPB14 and GPB15\n");
				printf("[C] Disable\n");
				printf("============================================================================================\n");

				scanf("%c",&Item);
				switch(Item)
				{
						case 'A': 	
                                                case 'a': 
											u32UsbhPort1 = HOST_LIKE_PORT1_0;	
											goto next;
						case 'B': 		
                                                case 'b': 
											u32UsbhPort1 = HOST_LIKE_PORT1_1;	
											goto next;
						case 'C': 
                                                case 'c': 
											goto next;	    		
    				
				}
		}while(1);	
next:
		do
		{
				printf("============================================================================================\n");
				printf("Please select the USB host port 2 through GPIO\n");
				printf("[A] GPC13 and GPC14\n");
				printf("[B] Disable\n");
				printf("============================================================================================\n");
				scanf("%c",&Item);
				switch(Item)
				{
						case 'A': 	
                                                case 'a': 	
										USB_PortInit(u32UsbhPort1, HOST_LIKE_PORT2_0);							
										goto start;
						case 'B': 		
                                                case 'b':   
										USB_PortInit(u32UsbhPort1, HOST_LIKE_PORT2_DISABLE);	
										goto start;		
				}
		}while(1);			
				
start:	

    USBH_Open();

    USBH_MassInit();

    Delay(0x1000);
    USBH_ProcessHubEvents();

    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    //f_mount(0, &FatFs[0]);  // for FATFS v0.09
    //Register work area to the default drive
    f_mount(&FatFs[0], "", 0);  // for FATFS v0.11
		
    for (;;) {
    	
        if (USBH_ProcessHubEvents())
        {
					//put_rc(f_mount(0, &FatFs[0]));  // for FATFS v0.09
          put_rc(f_mount(&FatFs[0], "", 0));  // for FATFS v0.11
				}
    	
        printf(_T(">"));
        ptr = Line;
        get_line(ptr, sizeof(Line));
        switch (*ptr++) {

        case 'q' :  /* Exit program */
            return 0;

        case '1' :  /* USB re-init */
            printf("USBH_Close() ----------\n");
            USBH_Close();
            printf("USBH_Open()  ----------\n");
            USBH_Open();
            USBH_MassInit();
            Delay(0x500000);
            USBH_ProcessHubEvents();
            printf("\n\nUSBH port status: 0x%x, 0x%x\n", USBH->HcRhPortStatus1, USBH->HcRhPortStatus2);
            break;

        case '2' :  /* USB suspend */
            USBH_Suspend();
            printf("\n\nUSBH port status: 0x%x, 0x%x\n", USBH->HcRhPortStatus1, USBH->HcRhPortStatus2);
            break;

        case '3' :  /* USB resume */
            USBH_Resume();
            printf("\n\nUSBH port status: 0x%x, 0x%x\n", USBH->HcRhPortStatus1, USBH->HcRhPortStatus2);
            break;

        case 'd' :
            switch (*ptr++) {
            case 'd' :  /* dd [<lba>] - Dump sector */
                if (!xatoi(&ptr, &p2)) p2 = sect;
                res = (FRESULT)disk_read(0, Buff, p2, 1);
                if (res) {
                    printf("rc=%d\n", (WORD)res);
                    break;
                }
                sect = p2 + 1;
                printf("Sector:%lu\n", p2);
                for (buf=(unsigned char*)Buff, ofs = 0; ofs < 0x200; buf+=16, ofs+=16)
                    put_dump(buf, ofs, 16);
                break;

            case 'i' :  /* di - Initialize disk */
                printf("rc=%d\n", (WORD)disk_initialize(0));
                break;


            }
            break;

        case 'b' :
            switch (*ptr++) {
            case 'd' :  /* bd <addr> - Dump R/W buffer */
                if (!xatoi(&ptr, &p1)) break;
                for (ptr=(char*)&Buff[p1], ofs = p1, cnt = 32; cnt; cnt--, ptr+=16, ofs+=16)
                    put_dump((BYTE*)ptr, ofs, 16);
                break;

            case 'e' :  /* be <addr> [<data>] ... - Edit R/W buffer */
                if (!xatoi(&ptr, &p1)) break;
                if (xatoi(&ptr, &p2)) {
                    do {
                        Buff[p1++] = (BYTE)p2;
                    } while (xatoi(&ptr, &p2));
                    break;
                }
                for (;;) {
                    printf("%04X %02X-", (WORD)p1, Buff[p1]);
                    get_line(Line, sizeof(Line));
                    ptr = Line;
                    if (*ptr == '.') break;
                    if (*ptr < ' ') {
                        p1++;
                        continue;
                    }
                    if (xatoi(&ptr, &p2))
                        Buff[p1++] = (BYTE)p2;
                    else
                        printf("???\n");
                }
                break;

            case 'r' :  /* br <sector> [<n>] - Read disk into R/W buffer */
                if (!xatoi(&ptr, &p2)) break;
                if (!xatoi(&ptr, &p3)) p3 = 1;
                printf("rc=%u\n", disk_read(0, Buff, p2, p3));
                break;

            case 'w' :  /* bw <sector> [<n>] - Write R/W buffer into disk */
                if (!xatoi(&ptr, &p2)) break;
                if (!xatoi(&ptr, &p3)) p3 = 1;
                printf("rc=%u\n", disk_write(0, Buff, p2, p3));
                break;

            case 'f' :  /* bf <n> - Fill working buffer */
                if (!xatoi(&ptr, &p1)) break;
                memset(Buff, (int)p1, sizeof(Buff));
                break;

            }
            break;



        case 'f' :
            switch (*ptr++) {
            case 'i' :  /* fi - Force initialized the logical drive */
                //put_rc(f_mount(0, &FatFs[0]));  // for FATFS v0.09
                put_rc(f_mount(&FatFs[0], "", 0));  // for FATFS v0.11						
                break;

            case 's' :  /* fs - Show logical drive status */
                res = f_getfree("", (DWORD*)&p2, &fs);
                if (res) {
                    put_rc(res);
                    break;
                }
                printf("FAT type = FAT%u\nBytes/Cluster = %lu\nNumber of FATs = %u\n"
                       "Root DIR entries = %u\nSectors/FAT = %lu\nNumber of clusters = %lu\n"
                       "FAT start (lba) = %lu\nDIR start (lba,clustor) = %lu\nData start (lba) = %lu\n\n...",
                       ft[fs->fs_type & 3], fs->csize * 512UL, fs->n_fats,
                       fs->n_rootdir, fs->fsize, fs->n_fatent - 2,
                       fs->fatbase, fs->dirbase, fs->database
                      );
                acc_size = acc_files = acc_dirs = 0;
#if _USE_LFN
                Finfo.lfname = Lfname;
                Finfo.lfsize = sizeof(Lfname);
#endif
                res = scan_files(ptr);
                if (res) {
                    put_rc(res);
                    break;
                }
                printf("\r%u files, %lu bytes.\n%u folders.\n"
                       "%lu KB total disk space.\n%lu KB available.\n",
                       acc_files, acc_size, acc_dirs,
                       (fs->n_fatent - 2) * (fs->csize / 2), p2 * (fs->csize / 2)
                      );
                break;
            case 'l' :  /* fl [<path>] - Directory listing */
                while (*ptr == ' ') ptr++;
                res = f_opendir(&dir, ptr);
                if (res) {
                    put_rc(res);
                    break;
                }
                p1 = s1 = s2 = 0;
                for(;;) {
                    res = f_readdir(&dir, &Finfo);
                    if ((res != FR_OK) || !Finfo.fname[0]) break;
                    if (Finfo.fattrib & AM_DIR) {
                        s2++;
                    } else {
                        s1++;
                        p1 += Finfo.fsize;
                    }
                    printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s",
                           (Finfo.fattrib & AM_DIR) ? 'D' : '-',
                           (Finfo.fattrib & AM_RDO) ? 'R' : '-',
                           (Finfo.fattrib & AM_HID) ? 'H' : '-',
                           (Finfo.fattrib & AM_SYS) ? 'S' : '-',
                           (Finfo.fattrib & AM_ARC) ? 'A' : '-',
                           (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
                           (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63, Finfo.fsize, Finfo.fname);
#if _USE_LFN
                    for (p2 = strlen(Finfo.fname); p2 < 14; p2++)
                        putchar(' ');
                    printf("%s\n", Lfname);
#else
                    putchar('\n');
#endif
                }
                printf("%4u File(s),%10lu bytes total\n%4u Dir(s)", s1, p1, s2);
                if (f_getfree(ptr, (DWORD*)&p1, &fs) == FR_OK)
                    printf(", %10lu bytes free\n", p1 * fs->csize * 512);
                break;


            case 'o' :  /* fo <mode> <file> - Open a file */
                if (!xatoi(&ptr, &p1)) break;
                while (*ptr == ' ') ptr++;
                put_rc(f_open(&file1, ptr, (BYTE)p1));
                break;

            case 'c' :  /* fc - Close a file */
                put_rc(f_close(&file1));
                break;

            case 'e' :  /* fe - Seek file pointer */
                if (!xatoi(&ptr, &p1)) break;
                res = f_lseek(&file1, p1);
                put_rc(res);
                if (res == FR_OK)
                    printf("fptr=%lu(0x%lX)\n", file1.fptr, file1.fptr);
                break;

            case 'd' :  /* fd <len> - read and dump file from current fp */
                if (!xatoi(&ptr, &p1)) break;
                ofs = file1.fptr;
                while (p1) {
                    if ((UINT)p1 >= 16) {
                        cnt = 16;
                        p1 -= 16;
                    } else                {
                        cnt = p1;
                        p1 = 0;
                    }
                    res = f_read(&file1, Buff, cnt, &cnt);
                    if (res != FR_OK) {
                        put_rc(res);
                        break;
                    }
                    if (!cnt) break;
                    put_dump(Buff, ofs, cnt);
                    ofs += 16;
                }
                break;

            case 'r' :  /* fr <len> - read file */
                if (!xatoi(&ptr, &p1)) break;
                p2 = 0;
                Timer = 0;
                while (p1) {
                    if ((UINT)p1 >= blen) {
                        cnt = blen;
                        p1 -= blen;
                    } else {
                        cnt = p1;
                        p1 = 0;
                    }
                    res = f_read(&file1, Buff, cnt, &s2);
                    if (res != FR_OK) {
                        put_rc(res);
                        break;
                    }
                    p2 += s2;
                    if (cnt != s2) break;
                }
                printf("%lu bytes read with %lu kB/sec.\n", p2, p2 / Timer);
                break;

            case 'w' :  /* fw <len> <val> - write file */
                if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
                memset(Buff, (BYTE)p2, blen);
                p2 = 0;
                Timer = 0;
                while (p1) {
                    if ((UINT)p1 >= blen) {
                        cnt = blen;
                        p1 -= blen;
                    } else {
                        cnt = p1;
                        p1 = 0;
                    }
                    res = f_write(&file1, Buff, cnt, &s2);
                    if (res != FR_OK) {
                        put_rc(res);
                        break;
                    }
                    p2 += s2;
                    if (cnt != s2) break;
                }
                printf("%lu bytes written with %lu kB/sec.\n", p2, p2 / Timer);
                break;

            case 'n' :  /* fn <old_name> <new_name> - Change file/dir name */
                while (*ptr == ' ') ptr++;
                ptr2 = strchr(ptr, ' ');
                if (!ptr2) break;
                *ptr2++ = 0;
                while (*ptr2 == ' ') ptr2++;
                put_rc(f_rename(ptr, ptr2));
                break;

            case 'u' :  /* fu <name> - Unlink a file or dir */
                while (*ptr == ' ') ptr++;
                put_rc(f_unlink(ptr));
                break;

            case 'v' :  /* fv - Truncate file */
                put_rc(f_truncate(&file1));
                break;

            case 'k' :  /* fk <name> - Create a directory */
                while (*ptr == ' ') ptr++;
                put_rc(f_mkdir(ptr));
                break;

            case 'a' :  /* fa <atrr> <mask> <name> - Change file/dir attribute */
                if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
                while (*ptr == ' ') ptr++;
                put_rc(f_chmod(ptr, p1, p2));
                break;

            case 't' :  /* ft <year> <month> <day> <hour> <min> <sec> <name> - Change timestamp */
                if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                Finfo.fdate = (WORD)(((p1 - 1980) << 9) | ((p2 & 15) << 5) | (p3 & 31));
                if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                Finfo.ftime = (WORD)(((p1 & 31) << 11) | ((p1 & 63) << 5) | ((p1 >> 1) & 31));
                put_rc(f_utime(ptr, &Finfo));
                break;

            case 'x' : /* fx <src_name> <dst_name> - Copy file */
                while (*ptr == ' ') ptr++;
                ptr2 = strchr(ptr, ' ');
                if (!ptr2) break;
                *ptr2++ = 0;
                while (*ptr2 == ' ') ptr2++;
                printf("Opening \"%s\"", ptr);
                res = f_open(&file1, ptr, FA_OPEN_EXISTING | FA_READ);
                putchar('\n');
                if (res) {
                    put_rc(res);
                    break;
                }
                printf("Creating \"%s\"", ptr2);
                res = f_open(&file2, ptr2, FA_CREATE_ALWAYS | FA_WRITE);
                putchar('\n');
                if (res) {
                    put_rc(res);
                    f_close(&file1);
                    break;
                }
                printf("Copying...");
                p1 = 0;
                for (;;) {
                    res = f_read(&file1, Buff, sizeof(Buff), &s1);
                    if (res || s1 == 0) break;   /* error or eof */
                    res = f_write(&file2, Buff, s1, &s2);
                    p1 += s2;
                    if (res || s2 < s1) break;   /* error or disk full */
                }
                printf("\n%lu bytes copied.\n", p1);
                f_close(&file1);
                f_close(&file2);
                break;
#if _FS_RPATH
            case 'g' :  /* fg <path> - Change current directory */
                while (*ptr == ' ') ptr++;
                put_rc(f_chdir(ptr));
                break;

            case 'j' :  /* fj <drive#> - Change current drive */
                if (xatoi(&ptr, &p1)) {
                    put_rc(f_chdrive((BYTE)p1));
                }
                break;
#endif
#if _USE_MKFS
            case 'm' :  /* fm <partition rule> <sect/clust> - Create file system */
                if (!xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                printf("The memory card will be formatted. Are you sure? (Y/n)=");
                get_line(ptr, sizeof(Line));
                if (*ptr == 'Y')
                    put_rc(f_mkfs(0, (BYTE)p2, (WORD)p3));
                break;
#endif
            case 'z' :  /* fz [<rw size>] - Change R/W length for fr/fw/fx command */
                if (xatoi(&ptr, &p1) && p1 >= 1 && (size_t)p1 <= sizeof(Buff))
                    blen = p1;
                printf("blen=%u\n", blen);
                break;
            }
            break;
        case '?':       /* Show usage */
            printf(
                _T("dd [<lba>] - Dump sector\n")
                _T("di - Initialize disk\n")
                //_T("ds <pd#> - Show disk status\n")
                _T("\n")
                _T("bd <ofs> - Dump working buffer\n")
                _T("be <ofs> [<data>] ... - Edit working buffer\n")
                _T("br <pd#> <sect> [<num>] - Read disk into working buffer\n")
                _T("bw <pd#> <sect> [<num>] - Write working buffer into disk\n")
                _T("bf <val> - Fill working buffer\n")
                _T("\n")
                _T("fi - Force initialized the logical drive\n")
                _T("fs - Show volume status\n")
                _T("fl [<path>] - Show a directory\n")
                _T("fo <mode> <file> - Open a file\n")
                _T("fc - Close the file\n")
                _T("fe <ofs> - Move fp in normal seek\n")
                //_T("fE <ofs> - Move fp in fast seek or Create link table\n")
                _T("fd <len> - Read and dump the file\n")
                _T("fr <len> - Read the file\n")
                _T("fw <len> <val> - Write to the file\n")
                _T("fn <object name> <new name> - Rename an object\n")
                _T("fu <object name> - Unlink an object\n")
                _T("fv - Truncate the file at current fp\n")
                _T("fk <dir name> - Create a directory\n")
                _T("fa <atrr> <mask> <object name> - Change object attribute\n")
                _T("ft <year> <month> <day> <hour> <min> <sec> <object name> - Change timestamp of an object\n")
                _T("fx <src file> <dst file> - Copy a file\n")
                _T("fg <path> - Change current directory\n")
                _T("fj <ld#> - Change current drive\n")
                _T("fm <ld#> <rule> <cluster size> - Create file system\n")
                _T("\n")
            );
            break;
        }
    }
    //while(1);
}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
