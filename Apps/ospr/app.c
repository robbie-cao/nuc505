#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
#include "partest.h"
#include "app.h"

#include "NUC505Series.h"
#include "usrprog_ovly_tab.h"
#include "config.h"
#include "mad.h"
#include "ff.h"
#include "diskio.h"
#include "amrenc.h"
#include "TypeDefine.h"
#include "Mp3.h"


#define AMR_FILE    "0:\\test.amr"
uint8_t PcmRxBuff[2][FSTLVL_BUFF_LEN] = {0};
__align(4) uint8_t EncBuff[SECLVL_BUFF_LEN];
FIL             fs;
//=======================================================================================//
#define MP3_FILE    "0:\\36.mp3"
__align(4) UINT8 g_au8Mp3WorkBuf[MP3_DECODE_WORK_BUF_SIZE];		// work buffer of MP3 decoder
__align(4) UINT8 g_au8Mp3TempBuf[MP3_DECODE_TEMP_BUF_SIZE];		// temp buffer of MP3 decoder, it can be freed as it's not in running decode API

__align(4) int16_t g_aiTxBuf[TX_SAMPLE_COUNT];

__align(4) INT16 g_aiWorkingBuf[MP3APP_PCM_BUF_SIZE * 2];	// Stoe decoded frames of PCm sampels in this ring buffer for output

int32_t g_i32offset;

#define mp3FileObject fs

struct AudioInfoObject audioInfo;

extern FATFS FatFs[_VOLUMES];               /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
extern BYTE Buff[16] ;                   /* Working buffer */
#endif

#ifdef __ARMCC_VERSION
extern __align(32) BYTE Buff[16] ;       /* Working buffer */
#endif
//============================================================================================//
#define pollqTASK_DELAY		( ( portTickType ) 3000 / portTICK_RATE_MS )
#define pollqPRODUCER_DELAY		( ( portTickType ) 200 / portTICK_RATE_MS )
#define pollqCONSUMER_DELAY		( pollqPRODUCER_DELAY - ( portTickType ) ( 20 / portTICK_RATE_MS ) )
#define mainAPP_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )

uint32_t x=0,y=0;

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vAMRTask, pvParameters );
static portTASK_FUNCTION_PROTO( vMP3Task, pvParameters );
static portTASK_FUNCTION_PROTO( vSWITCHTask, pvParameters );

/*-----------------------------------------------------------*/
xTaskHandle AMR,MP3,SWITCH;
void vStartAPPTasks( unsigned portBASE_TYPE uxPriority )
{
    /* Spawn the task. */
    xTaskCreate( vSWITCHTask, ( signed char * ) "SWITCH", ( unsigned short ) 120, NULL, uxPriority, &SWITCH );
#if 1
    xTaskCreate( vAMRTask, ( signed char * ) "AMR", ( unsigned short ) 120, NULL, uxPriority, &AMR );
#endif

    xTaskCreate( vMP3Task, ( signed char * ) "MP3", ( unsigned short ) 1000, NULL, uxPriority, &MP3 );

    vTaskSuspendAll();
    load_overlay(ovly_A);
    load_overlay(ovly_B);
    xTaskResumeAll();
    //vTaskSuspend(MP3);
    vTaskSuspend(SWITCH);
}

static portTASK_FUNCTION( vSWITCHTask, pvParameters )
{

    /* The parameters are not used. */
    ( void ) pvParameters;
    printf("%vSWITCHTask\n");
    //printf("T%s\n", (char *)pvParameters);

    for(;;)
    {
        if (((x%2)==1)&&(y==1))
        {
            vTaskSuspendAll();
            load_overlay(ovly_B);
            TIMER_Delay(TIMER1, 300000);
            xTaskResumeAll();
            vTaskSuspend(AMR);
            y=0;
            vTaskResume(MP3);
        }
        else if (((x%2)==0)&&(y==1))
        {
            vTaskSuspendAll();
            load_overlay(ovly_A);
            TIMER_Delay(TIMER1, 300000);
            xTaskResumeAll();
            vTaskSuspend(MP3);
            y=0;
            vTaskResume(AMR);
        }
        vTaskDelay( pollqTASK_DELAY );
    }
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

/*-----------------------------------------------------------*/

#if defined ( __CC_ARM )
#pragma arm section code="overlay_a"
#elif defined (__ICCARM__)
#pragma default_function_attributes = @ "overlay_a"
#endif

void amr_recorder_th(char *Filename)
{
    FRESULT res;
    UINT s2;

    //    /* Configure FATFS */
    //    printf("rc=%d\n", (WORD)disk_initialize(0));
    //    disk_read(0, Buff, 2, 1);
    //    //f_mount(0, &FatFs[0]);  // for FATFS v0.09
    //	// Register work area to the default drive
    //    f_mount(&FatFs[0], "", 0);  // for FATFS v0.11

    res = f_open(&fs, Filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("Open file error \r\n");
        return;
    }

    /* write AMR header to file */
    f_write(&fs, &AMR_ENC_AMR_FILE_ID, sizeof(AMR_ENC_AMR_FILE_ID) - 1, &s2);
    f_sync(&fs);
}
uint32_t record_len;
void amr_recorder_bh(int Seconds)
{
    volatile uint8_t PcmRxBuffFull[2] = {0};


    int play_started = 0;
    uint32_t WaitingPcmBuf = 0;
    int encsize;
    uint32_t realwrite;

    I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
    I2S_SET_RX_TH_LEVEL(I2S, I2S_FIFO_RX_LEVEL_WORD_16);

    I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &PcmRxBuff[0]);                     // Tx Start Address
    I2S_SET_TXDMA_THADDR(I2S, (uint32_t) &PcmRxBuff[0][FSTLVL_BUFF_LEN-4]);  // Tx Threshold Address
    I2S_SET_TXDMA_EADDR( I2S, (uint32_t) &PcmRxBuff[1][FSTLVL_BUFF_LEN-4]);  // Tx End Address

    I2S_SET_RXDMA_STADDR(I2S, (uint32_t) &PcmRxBuff[0]);                     // Rx Start Address
    I2S_SET_RXDMA_THADDR(I2S, (uint32_t) &PcmRxBuff[0][FSTLVL_BUFF_LEN-4]);  // Rx Threshold Address
    I2S_SET_RXDMA_EADDR( I2S, (uint32_t) &PcmRxBuff[1][FSTLVL_BUFF_LEN-4]);  // Rx End Address

    /* Start to record. */
    I2S_ENABLE_RXDMA(I2S);
    I2S_ENABLE_RX(I2S);

    /* Enable related interrupts */
    //    NVIC_EnableIRQ(I2S_IRQn);
    I2S_EnableInt(I2S, (I2S_IEN_RDMATIEN_Msk|I2S_IEN_RDMAEIEN_Msk));

    record_len = 0;

    while(1)
    {
        uint32_t u32I2SIntFlag;

        u32I2SIntFlag = I2S_GET_INT_FLAG(I2S, (I2S_STATUS_RDMATIF_Msk | I2S_STATUS_RDMAEIF_Msk));

        /* Copy RX data to TX buffer */
        if (u32I2SIntFlag & I2S_STATUS_RDMATIF_Msk)
        {
            I2S_CLR_INT_FLAG(I2S, I2S_STATUS_RDMATIF_Msk);
            PcmRxBuffFull[0] = TRUE;
        }
        else if (u32I2SIntFlag & I2S_STATUS_RDMAEIF_Msk)
        {
            I2S_CLR_INT_FLAG(I2S, I2S_STATUS_RDMAEIF_Msk);
            PcmRxBuffFull[1] = TRUE;
        }
        if (PcmRxBuffFull[WaitingPcmBuf] == TRUE)
        {
            if (!play_started)
            {
                /* Start playback after the first buffer has been ready. */
                I2S_ENABLE_TXDMA(I2S);
                I2S_ENABLE_TX(I2S);
                play_started = 1;
            }

            /* Encode a AMR frame */
            amrEncode(7, 0, (short *)(PcmRxBuff[WaitingPcmBuf]), (short *)(EncBuff), &encsize);//+EncBuffHead
            PcmRxBuffFull[WaitingPcmBuf] = FALSE;

            record_len += encsize;

            WaitingPcmBuf = 1-WaitingPcmBuf;
            f_write(&fs, EncBuff, encsize, &realwrite);//+EncBuffTail

        }

        /* check record time */
        if (record_len >= Seconds * 50 * AMR_ENC_MAX_PACKET_SIZE)
            /* break out if time is up */
            break;
    }
    f_close(&fs);

    printf( "\n### AMR recorded ###\n" );
}

void demo_LineIn()
{
    uint32_t i;

    // Setting Right Line In Channel
    SYS->GPD_MFPL  = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk) ) | SYS_GPD_MFPL_PD4MFP_RLINEIN;
    SYS_SetSharedPinType(SYS_PORT_D, 4, 0, 0);

    /* IIC Configure Step without PLL: */
    /* Add MCLK(256*Fs) in. */

    I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);	// Mute headphone of Left channel
    I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);	// Mute headphone of Right channel
    I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x0F);	//Mute the ADC Left channel volume
    I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x0F);	//Mute the ADC Right channel volume
    I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x0F);	//Mute the ADC Side tone volume

    I2S_SET_INTERNAL_CODEC(I2S, 0x02, 0xC0);	//Set CODEC slave

    I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x80);	//Digital Part Enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0F, 0xF0);	//Enable Analog Part
    I2S_SET_INTERNAL_CODEC(I2S, 0x0E, 0x00);	//ADC input select Line in

    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF3);	//Analog Part Enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0D, 0x31);	//Biasing enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xE3);
    for (i=0; i < 15; i++)	//Delay 1.5s~2.5s
        TIMER_Delay(TIMER1, 100000);
    I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
    I2S_SET_INTERNAL_CODEC(I2S, 0x00, 0xD0);	//ADC digital enabled
    TIMER_Delay(TIMER1, 100000);	//Delay 100mS

    I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x06);	//Un-mute Headphone and set volume
    I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x06);	//Un-mute Headphone and set volume
    I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x08);	//Un-Mute the ADC Left channel volume
    I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x08);	//Un-Mute the ADC Right channel volume
    //	I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

    /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */

    printf("[OK]\n");
}

static portTASK_FUNCTION( vAMRTask, pvParameters )
{

    /* The parameters are not used. */
    ( void ) pvParameters;
    printf("%vAMRTask\n");
    //	printf("T%s\n", (char *)pvParameters);

    for(;;)
    {

        printf("APP1: AMR is running on SRAM.\n");

#if 0
        I2S_Open(I2S, I2S_MODE_MASTER, 8000, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);

        // Open MCLK
        I2S_EnableMCLK(I2S, 8000 * 256);

        /* Init AMR codec */
        amrInitEncode();
        /* Create AMR file */
        amr_recorder_th(AMR_FILE);

        /* Init one of three input source */
        demo_LineIn();
        //	demo_MIC0();
        //	demo_MIC1();

        /* record 20 seconds then close file */
        taskENTER_CRITICAL();
        amr_recorder_bh(20);
        taskEXIT_CRITICAL();

        /* Clean AMR codec */
        amrFinishEncode();
#endif

        x++;
        y=1;
        vTaskDelay( pollqTASK_DELAY );
    }
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

#if defined ( __CC_ARM )
#pragma arm section code
#elif defined (__ICCARM__)
#pragma default_function_attributes =
#endif

#if defined ( __CC_ARM )
#pragma arm section code="overlay_b"
#elif defined (__ICCARM__)
#pragma default_function_attributes = @ "overlay_b"
#endif

void demo_InternalDAC(void)
{
    uint32_t i;

    /* IIC Configure Step without PLL: */
    /* Add MCLK(256*Fs) in. */

    I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);	// Mute headphone of Left channel
    I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);	// Mute headphone of Right channel
    I2S_SET_INTERNAL_CODEC(I2S, 0x02, 0xC0);	// Set CODEC slave
    I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x80);	//Digital Part Enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF3);	//Analog Part Enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0D, 0x31);	//Biasing enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xE3);
    for (i=0; i < 15; i++)	//Delay 1.5s~2.5s
        TIMER_Delay(TIMER1, 100000);
    I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
    TIMER_Delay(TIMER1, 100);	//Delay 100uS
    I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x12);	//Un-mute Headphone and set volume
    I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x12);	//Un-mute Headphone and set volume

    /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
}

void Mp3App_SampleRateCallback(UINT16 u16SamplRate)
{
#if 0
    if (audioInfo.mp3SampleRate == 0)
    {
        if ( u16SamplRate % 11025 )
            CLK_SET_APLL(CLK_APLL_49152031);
        else
            CLK_SET_APLL(CLK_APLL_45158425);

        if (u16SamplRate == 8000)
            CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 1);
        else
            CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);

        I2S_Open(I2S, I2S_MODE_MASTER, u16SamplRate, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
        I2S_EnableMCLK(I2S, u16SamplRate*256);

        I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);

        I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &g_aiTxBuf[0]);								// Tx Start Address
        I2S_SET_TXDMA_EADDR( I2S, (uint32_t) &g_aiTxBuf[TX_SAMPLE_COUNT-2]);	// Tx End Address

        demo_InternalDAC();

        // Clear Interrupt Status
        I2S_CLR_INT_FLAG(I2S, I2S_STATUS_LZCIF_Msk|I2S_STATUS_RZCIF_Msk|I2S_STATUS_TXOVIF_Msk|I2S_STATUS_TXUDIF_Msk|I2S_STATUS_RXOVIF_Msk|I2S_STATUS_RXUDIF_Msk|I2S_STATUS_TDMATIF_Msk|I2S_STATUS_TDMAEIF_Msk|I2S_STATUS_RDMATIF_Msk|I2S_STATUS_RDMAEIF_Msk);

        printf("%d\n", u16SamplRate);
    }
    else
    {
        printf("change sampel rate of DAC\n");
    }

    audioInfo.mp3SampleRate = u16SamplRate;
#else
    return;
#endif
}

UINT32 Mp3App_ResLdrGetMp3DataFromSPI(void *pDesAddr, UINT32 u32Position, UINT32 u32ByteNum)
{
    size_t   sizetReturnSize;
    if (mp3FileObject.fptr != (u32Position + g_i32offset))
    {
        if (f_lseek(&mp3FileObject,(u32Position + g_i32offset)) != FR_OK)
            return 0;
    }
    if (f_read(&mp3FileObject, pDesAddr, u32ByteNum, &sizetReturnSize) == FR_OK)
        return sizetReturnSize;
    else
        return 0;
}

// Parse MP3 header and get some informations
int32_t MP3_ParseHeaderInfo(TCHAR * u8MP3FileName)
{
    FRESULT res;
    //	uint32_t i=0;
    int32_t i32Result;
    size_t          ReturnSize;

    res = f_open(&mp3FileObject, (void *)u8MP3FileName, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK) {
        printf("file is opened!!\r\n");
        audioInfo.playFileSize = mp3FileObject.fsize;

        /* FIXME we just to seek mp3 data (1st frame) */
        res = f_read(&mp3FileObject, (char *)(&g_au8Mp3WorkBuf[0]), 10, &ReturnSize);
        if((res != FR_OK) || f_eof(&mp3FileObject)) {
            printf("Stop !(%x)\n\r", res);
            return -1;
        }
        g_i32offset = 0;
        if ( (g_au8Mp3WorkBuf[0] == 'I') && (g_au8Mp3WorkBuf[1] == 'D') && (g_au8Mp3WorkBuf[2] == '3') )
        {
            g_i32offset = (g_i32offset << 7) | (g_au8Mp3WorkBuf[6] & 0x7F);
            g_i32offset = (g_i32offset << 7) | (g_au8Mp3WorkBuf[7] & 0x7F);
            g_i32offset = (g_i32offset << 7) | (g_au8Mp3WorkBuf[8] & 0x7F);
            g_i32offset = (g_i32offset << 7) | (g_au8Mp3WorkBuf[9] & 0x7F);
            g_i32offset += 10;
        }

        //			while(i++<=500)
        audioInfo.mp3SampleRate=0;
        //			i = f_size(&mp3FileObject);
        //			f_lseek(&mp3FileObject, i/2);
        //		f_lseek(&mp3FileObject, g_i32offset + ((audioInfo.playFileSize - g_i32offset) / 2));
        f_lseek(&mp3FileObject, g_i32offset);
        //			while (audioInfo.mp3SampleRate==0)
        //			{
        res = f_read(&mp3FileObject, (char *)(&g_au8Mp3WorkBuf[0]), 2048, &ReturnSize);
        //				printf("%d %d\n",FILE_IO_BUFFER_SIZE,ReturnSize);
        if ((res != FR_OK) && (audioInfo.mp3SampleRate==0))
        {
            f_close(&mp3FileObject);
            printf("Open File Error\r\n");
            return -1;
        }
        //parsing MP3 header
        i32Result = mp3CountV1L3Headers((unsigned char *)(&g_au8Mp3WorkBuf[0]), ReturnSize);
        if ( i32Result == 0 )
        {
            printf("not a MP3 file! %d\n",i32Result);
            f_close(&mp3FileObject);
            return -1;
        }
        printf("%d byte(s) to the 1st frame.\n", g_i32offset);
        //			}
#if 0
        audioInfo.mp3SampleRate=0;
        while (audioInfo.mp3SampleRate==0)
        {
            res = f_read(&mp3FileObject, (char *)(&g_au8Mp3WorkBuf[0]), FILE_IO_BUFFER_SIZE, &ReturnSize);
            if ((res != FR_OK) && (audioInfo.mp3SampleRate==0))
            {
                f_close(&mp3FileObject);
                printf("Open File Error\r\n");
                return -1;
            }
            //parsing MP3 header
            mp3CountV1L3Headers((unsigned char *)(&g_au8Mp3WorkBuf[0]), ReturnSize);
        }
#endif
    } else {
        f_close(&mp3FileObject);
        printf("Open File Error\r\n");
        return -1;
    }
    //    f_lseek(&mp3FileObject, g_i32offset);

    printf("====[MP3 Info]======\r\n");
    printf("FileSize = %d\r\n", audioInfo.playFileSize);
    printf("SampleRate = %d\r\n", audioInfo.mp3SampleRate);
    printf("BitRate = %d\r\n", audioInfo.mp3BitRate);
    printf("Channel = %d\r\n", audioInfo.mp3Channel);
    printf("PlayTime = %d\r\n", audioInfo.mp3PlayTime);
    printf("=====================\r\n");

    return g_i32offset;
}

BOOL Mp3App_StartDecode(TCHAR * u8MP3FileName)
{
    if ( MP3_ParseHeaderInfo( u8MP3FileName ) < 0 )
    {
        printf("Open file error!\n");
        return FALSE;
    }

    if ( audioInfo.mp3SampleRate % 11025 )
    {
        CLK_SET_APLL(CLK_APLL_49152031);
        if ( audioInfo.mp3SampleRate == 8000 )
            CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 1);
    }
    else
        CLK_SET_APLL(CLK_APLL_45158425);

    I2S_Open(I2S, I2S_MODE_MASTER, audioInfo.mp3SampleRate, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
    I2S_EnableMCLK(I2S, audioInfo.mp3SampleRate*256);

    I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);

    I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &g_aiTxBuf[0]);								// Tx Start Address
    I2S_SET_TXDMA_EADDR( I2S, (uint32_t) &g_aiTxBuf[TX_SAMPLE_COUNT-2]);	// Tx End Address

    demo_InternalDAC();

    // Clear Interrupt Status
    I2S_CLR_INT_FLAG(I2S, I2S_STATUS_LZCIF_Msk|I2S_STATUS_RZCIF_Msk|I2S_STATUS_TXOVIF_Msk|I2S_STATUS_TXUDIF_Msk|I2S_STATUS_RXOVIF_Msk|I2S_STATUS_RXUDIF_Msk|I2S_STATUS_TDMATIF_Msk|I2S_STATUS_TDMAEIF_Msk|I2S_STATUS_RDMATIF_Msk|I2S_STATUS_RDMAEIF_Msk);

    // Mp3 decoder initiates work buffer and returns sample rate.
    if ( (Mp3_DecodeInitiate(g_au8Mp3WorkBuf, g_au8Mp3TempBuf, 0, Mp3App_ResLdrGetMp3DataFromSPI )) == 0 )
        return FALSE;

    Mp3_EnableSampleRateCallback(g_au8Mp3WorkBuf,Mp3App_SampleRateCallback);

    return TRUE;
}

void Mp3App_ResetBuffer(void)
{
    memset(g_aiTxBuf,0,sizeof(g_aiTxBuf));
    memset(g_aiWorkingBuf,0,sizeof(g_aiWorkingBuf));
    memset(&audioInfo,0,sizeof(audioInfo));
    memset(g_au8Mp3WorkBuf,0,sizeof(g_au8Mp3WorkBuf));
}

static portTASK_FUNCTION( vMP3Task, pvParameters )
{
    /* The parameters are not used. */
    ( void ) pvParameters;
    printf("%vMP3Task\n");
    //printf("T%s\n", (char *)pvParameters);

    for(;;)
    {
        uint32_t u32TxIdx, u32TxBufFlag, u32TxDmaEnabled;
        int32_t i32TxDmaCurAddr, i32TxDmaLastAddr, i32TxBufCurAddr, i32TxBufLastAddr, i32TxDiff, i32TxDmaDiff, i32TxBufDiff, i32TxDmaCur, i32TxBufCur, i32TxBufStartAddr;
        uint32_t u32i;
        TCHAR * u8MP3FileName=MP3_FILE;

        printf("APP2: MP3 is running on SRAM.\n");
        u32TxIdx = 0;
        i32TxDmaCur = 0;
        i32TxBufCur = 0;
        u32TxDmaEnabled = 0;
        u32TxBufFlag = 0;

        i32TxBufStartAddr = i32TxDmaLastAddr = i32TxDmaCurAddr = i32TxBufLastAddr = i32TxBufCurAddr = (int32_t)&g_aiTxBuf[u32TxIdx];

        Mp3App_ResetBuffer();

        if (Mp3App_StartDecode(u8MP3FileName) == FALSE)
        {
            goto _app_end_;
        }

        while ( u32TxDmaEnabled == 0 )
        {
            if (Mp3_DecodeProcess(g_au8Mp3WorkBuf,g_au8Mp3TempBuf,g_aiWorkingBuf,Mp3App_ResLdrGetMp3DataFromSPI,NULL) != MP3_DECODE_SAMPLE_PER_FRAME*2)
            {
                goto _app_end_;
            }

            // copy working buffer to I2S TX DMA buffer
            for ( u32i = 0; u32i < (MP3APP_PCM_BUF_SIZE * 2); u32i++ )
            {
                if ( u32TxIdx >= TX_SAMPLE_COUNT )
                {
                    printf("I2S TX buffer overflow! %d samples\n", u32TxIdx);
                    goto _app_end_;
                }

                g_aiTxBuf[u32TxIdx++] = g_aiWorkingBuf[u32i];
            }

            // I2S TX DMA enable condition
            if ( u32TxIdx >= (MP3APP_PCM_BUF_SIZE * 2 * 10) )
            {
                printf( "%d samples decoded and filled!\n", u32TxIdx );
                printf( "Start playing...\n" );

                u32TxDmaEnabled = 1;
                I2S_ENABLE_TXDMA(I2S);
                I2S_ENABLE_TX(I2S);
            }
        }

        while ( 1 )
        {
            i32TxDmaLastAddr = i32TxDmaCurAddr;
            i32TxDmaCurAddr = I2S_GET_TXDMA_CADDR(I2S);
            i32TxDmaDiff = i32TxDmaCurAddr - i32TxDmaLastAddr;
            if ( i32TxDmaDiff > 0 )
            {
                if ( I2S_GET_INT_FLAG(I2S, I2S_STATUS_TDMAEIF_Msk) & I2S_STATUS_TDMAEIF_Msk )
                {
                    I2S_CLR_INT_FLAG(I2S, I2S_STATUS_TDMAEIF_Msk);
                    if ( (i32TxDmaCurAddr - i32TxBufStartAddr + 4) != TX_SAMPLE_SIZE )
                    {
                        printf("A%d %x %x %x\n",i32TxDmaDiff,i32TxBufStartAddr,i32TxDmaCurAddr,i32TxDmaLastAddr);
                        break;
                    }
                }
            }
            else if ( i32TxDmaDiff < 0 )
            {
                if ( I2S_GET_INT_FLAG(I2S, I2S_STATUS_TDMAEIF_Msk) & I2S_STATUS_TDMAEIF_Msk )
                {
                    I2S_CLR_INT_FLAG(I2S, I2S_STATUS_TDMAEIF_Msk);
                }
                else
                {
                    if ( (i32TxDmaLastAddr - i32TxBufStartAddr + 4) != TX_SAMPLE_SIZE )
                    {
                        printf("B%d %x %x %x\n",i32TxDmaDiff,i32TxBufStartAddr,i32TxDmaCurAddr,i32TxDmaLastAddr);
                        break;
                    }
                }
                i32TxDmaDiff = TX_SAMPLE_SIZE + i32TxDmaDiff;
            }
            i32TxDmaCur += i32TxDmaDiff;

            i32TxBufLastAddr = i32TxBufCurAddr;
            i32TxBufCurAddr = (int32_t)&g_aiTxBuf[u32TxIdx];
            i32TxBufDiff = i32TxBufCurAddr - i32TxBufLastAddr;
            if ( i32TxBufDiff > 0 )
            {
                if ( u32TxBufFlag )
                {
                    u32TxBufFlag = 0;
                    printf("C%d %x %x %x\n",i32TxBufDiff,i32TxBufStartAddr,i32TxBufCurAddr,i32TxBufLastAddr);
                    break;
                }
            }
            else if ( i32TxBufDiff < 0 )
            {
                if ( u32TxBufFlag )
                {
                    u32TxBufFlag = 0;
                }
                else
                {
                    printf("D%d %x %x %x\n",i32TxBufDiff,i32TxBufStartAddr,i32TxBufCurAddr,i32TxBufLastAddr);
                    break;
                }
                i32TxBufDiff = TX_SAMPLE_SIZE + i32TxBufDiff;
            }
            i32TxBufCur += i32TxBufDiff;

            if ( (i32TxDmaCur >= TX_SAMPLE_SIZE) && (i32TxBufCur >= TX_SAMPLE_SIZE) )
            {
                i32TxDmaCur -= TX_SAMPLE_SIZE;
                i32TxBufCur -= TX_SAMPLE_SIZE;
            }
            i32TxDiff = i32TxBufCur - i32TxDmaCur;
            if ( i32TxDiff > 0 )
            {
                if ( i32TxDiff > TX_SAMPLE_SIZE )
                {
                    printf("E%d\n",i32TxDiff);
                    break;
                }
            }
            else if ( i32TxDiff < 0 )
            {
                printf("F%d\n",i32TxDiff);
                break;
            }

            if ( (TX_SAMPLE_SIZE - i32TxDiff) >= (MP3APP_PCM_BUF_SIZE * 2 * 2) )
            {
                if (Mp3_DecodeProcess(g_au8Mp3WorkBuf,g_au8Mp3TempBuf,g_aiWorkingBuf,Mp3App_ResLdrGetMp3DataFromSPI,NULL) != MP3_DECODE_SAMPLE_PER_FRAME*2)
                {
                    break;
                }

                // copy working buffer to I2S TX DMA buffer
                for ( u32i = 0; u32i < (MP3APP_PCM_BUF_SIZE * 2); u32i++ )
                {
                    if ( u32TxIdx >= TX_SAMPLE_COUNT )
                    {
                        u32TxIdx = 0;
                        u32TxBufFlag = 1;
                    }

                    g_aiTxBuf[u32TxIdx++] = g_aiWorkingBuf[u32i];
                }
            }
        }

_app_end_:

        I2S_DISABLE_TX(I2S);
        I2S_DISABLE_TXDMA(I2S);

        f_close(&mp3FileObject);

        printf("Stop ...\n");
        x++;
        y=1;
        vTaskDelay( pollqTASK_DELAY );
    }
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

#if defined ( __CC_ARM )
#pragma arm section code
#elif defined (__ICCARM__)
#pragma default_function_attributes =
#endif
