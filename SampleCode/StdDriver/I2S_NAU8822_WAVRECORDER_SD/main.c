/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 14 $
 * $Date: 15/09/01 2:40p $
 * @brief    A WAV file recorder demo using NAU8822 audio codec used to record WAV file and stored in SD card.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "ringbuff.h"

FATFS FatFs[_VOLUMES];      /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[1024] ;       /* Working buffer */
#endif

#ifdef __ICCARM__
#pragma data_alignment=32
uint8_t PcmRxBuff[2][FSTLVL_BUFF_LEN] = {0};
#endif

#ifdef __ICCARM__
#if SUPPORT_24BIT
#pragma data_alignment=32
uint8_t PcmRxBuff_24[(sizeof (PcmRxBuff[0]) / 4 * 3) * NUM_24BIT_BUFF];
#endif  // #if SUPPORT_24BIT
#endif

#ifdef __ICCARM__
#pragma data_alignment=32
uint8_t ringbuff[SECLVL_BUFF_LEN];
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[1024] ;       /* Working buffer */

__align(32) uint8_t PcmRxBuff[2][FSTLVL_BUFF_LEN] = {0};
#if SUPPORT_24BIT
__align(32) uint8_t PcmRxBuff_24[(sizeof (PcmRxBuff[0]) / 4 * 3) * NUM_24BIT_BUFF];
#endif  // #if SUPPORT_24BIT
__align(32) uint8_t ringbuff[SECLVL_BUFF_LEN];
#endif

int volatile nBuffOverRun = 0;  // Count of buffer overrun.
struct RingBuff audio_rb;

struct WaveRecorderParam {
    uint16_t    NumChannels;
    uint32_t    SampleRate;
    uint16_t    BitsPerSample;
    uint32_t    Duration;       // In secs.
} static wave_recorder_param;

struct WaveHeader {
    uint32_t    RIFFChunkID;     // "RIFF"
    uint32_t    RIFFChunkSize;
    uint32_t    WAVEID;         // "WAVE"
    
    uint32_t    fmtXChunkID;     // "fmt "
    uint32_t    fmtXChunkSize;
    uint16_t    FormatTag;
    uint16_t    NumChannels;
    uint32_t    SampleRate;
    uint32_t    ByteRate;
    uint16_t    BlockAlign;
    uint16_t    BitsPerSample;
    
    uint16_t    ExtSize;
    uint16_t    ValidBitsPerSample;
    uint32_t    ChannelMask;
    uint8_t     SubFormat[16];
    
    uint32_t    dataChunkID;    // "data"
    uint32_t    dataChunkSize;
} static wave_header;

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void I2C0_Init(void);
void I2S_Init(void);
void demo_LineIn(void);
void demo_MIC(void);
void SD0_Init(void);
void config_wave_recorder(void);
void wave_recorder_th(void);
void wave_recorder_bh(void);

#define WAV_FILE2    "0:\\test.wav"

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

        /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

#ifdef __ICCARM__
        #pragma section = "VECTOR2"
        extern uint32_t __Vectors[];
        extern uint32_t __Vectors_Size[];
        
        printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", __section_begin("VECTOR2"));
        memcpy((void *) __section_begin("VECTOR2"), (void *) __Vectors, (unsigned int) __Vectors_Size);
        SCB->VTOR = (uint32_t) __section_begin("VECTOR2");
#endif
#ifdef __ARMCC_VERSION
        /* Relocate vector table in SRAM for fast interrupt handling. */
    {
        extern uint32_t __Vectors[];
        extern uint32_t __Vectors_Size[];
        extern uint32_t Image$$ER_VECTOR2$$ZI$$Base[];
    
        printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling, size = 0x%x.\n", Image$$ER_VECTOR2$$ZI$$Base, (unsigned int) __Vectors_Size);
        memcpy((void *) Image$$ER_VECTOR2$$ZI$$Base, (void *) __Vectors, (unsigned int) __Vectors_Size);
        SCB->VTOR = (uint32_t) Image$$ER_VECTOR2$$ZI$$Base;
    }
#endif

    /* FIXME: configure wave recorder parameters here. */
    wave_recorder_param.NumChannels = 2;
    wave_recorder_param.SampleRate = 22050;
    wave_recorder_param.BitsPerSample = 16;
    wave_recorder_param.Duration = 30;
    
		/* Init I2C0, IP clock and multi-function I/O */
		I2C0_Init();
		
        /* Init I2S, IP clock and multi-function I/O */
        I2S_Init();
        
    printf("+-----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (external CODEC)        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate %d Hz\n", wave_recorder_param.SampleRate);
    printf("      Word width %d bits\n", wave_recorder_param.BitsPerSample);
    printf("      %s mode\n", wave_recorder_param.NumChannels == 1 ? "Mono" : "Stereo");
    printf("      I2S format\n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PC11)\n      I2S_BCLK(PC12)\n      I2S_MCLK(PC8)\n");
    printf("      I2S_DI (PC9)\n      I2S_DO (PC10)\n\n");
    printf("  NOTE: Need head-phone and line-in for external CODEC.\n");
    
    I2S_Open(I2S, 
        I2S_MODE_MASTER, 
        wave_recorder_param.SampleRate, 
        wave_recorder_param.BitsPerSample == 8 ? I2S_DATABIT_8 : 
            wave_recorder_param.BitsPerSample == 16 ? I2S_DATABIT_16 : 
            wave_recorder_param.BitsPerSample == 24 ? I2S_DATABIT_24 : I2S_DATABIT_32,
        wave_recorder_param.NumChannels == 1 ? I2S_MONO : I2S_STEREO, 
        I2S_FORMAT_I2S, 
        I2S_DISABLE_INTERNAL_CODEC);

        // Open MCLK
        I2S_EnableMCLK(I2S, wave_recorder_param.SampleRate * 256);
        
      I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
      I2S_SET_RX_TH_LEVEL(I2S, I2S_FIFO_RX_LEVEL_WORD_16);
        
        I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &PcmRxBuff[0]);                                // Tx Start Address
        I2S_SET_TXDMA_THADDR(I2S, (uint32_t) &PcmRxBuff[0][FSTLVL_BUFF_LEN-4]);  // Tx Threshold Address
        I2S_SET_TXDMA_EADDR( I2S, (uint32_t) &PcmRxBuff[1][FSTLVL_BUFF_LEN-4]);  // Tx End Address

        I2S_SET_RXDMA_STADDR(I2S, (uint32_t) &PcmRxBuff[0]);                                // Rx Start Address
        I2S_SET_RXDMA_THADDR(I2S, (uint32_t) &PcmRxBuff[0][FSTLVL_BUFF_LEN-4]);  // Rx Threshold Address
        I2S_SET_RXDMA_EADDR( I2S, (uint32_t) &PcmRxBuff[1][FSTLVL_BUFF_LEN-4]);  // Rx End Address

        wave_recorder_th();
        
        demo_LineIn();
//				demo_MIC();

        wave_recorder_bh();
        
        while(1);
}

/* Parameter passing from wave_recorder_th() to wave_recorder_bh(). */
struct WaveRecorderHalf {
    uint32_t u32NumBuffRun;     // Number of buffer runs to record.
    uint16_t u16BlockAlignHw;
    FIL file2;
} wav_rec_hf;

void wave_recorder_th(void)
{
    FRESULT res;
    UINT s2;
        
    wave_header.RIFFChunkID = 0x46464952;   // "RIFF"
    wave_header.WAVEID = 0x45564157;   // "WAVE"
    wave_header.fmtXChunkID = 0x20746D66;   // "fmt "
    
    wave_header.FormatTag = 0xFFFE; // WAVE_FORMAT_EXTENSIBLE
    wave_header.NumChannels = wave_recorder_param.NumChannels;
    wave_header.SampleRate = wave_recorder_param.SampleRate;
    wave_header.BitsPerSample = wave_recorder_param.BitsPerSample;
    wave_header.BlockAlign = wave_recorder_param.NumChannels * wave_header.BitsPerSample / 8;
    wave_header.ByteRate = wave_recorder_param.SampleRate * wave_header.BlockAlign;
    wav_rec_hf.u16BlockAlignHw = (wave_recorder_param.BitsPerSample == 24) ? (wave_header.BlockAlign + wave_recorder_param.NumChannels) : wave_header.BlockAlign;
    
    wave_header.ExtSize = 22;
    wave_header.ValidBitsPerSample = wave_recorder_param.BitsPerSample;
    wave_header.ChannelMask = (wave_recorder_param.NumChannels == 1) ? 0x04 : 0x03;
    *((uint16_t *) wave_header.SubFormat) = 1;  // WAVE_FORMAT_PCM
    {
        static const uint8_t ext_sig[] = {
            0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 
            0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71
        };
        memcpy(wave_header.SubFormat + 2, ext_sig, 14);
    }
        
    wave_header.dataChunkID = 0x61746164;   // "data"
    wav_rec_hf.u32NumBuffRun = (wave_recorder_param.SampleRate * wave_recorder_param.Duration * (wav_rec_hf.u16BlockAlignHw) + (sizeof (PcmRxBuff[0]) - 1)) / sizeof (PcmRxBuff[0]); // Round up if not integral.
    
    wave_header.dataChunkSize = (sizeof (PcmRxBuff[0]) / wav_rec_hf.u16BlockAlignHw * wave_header.BlockAlign) * wav_rec_hf.u32NumBuffRun;
    wave_header.fmtXChunkSize = 16 + 2 + wave_header.ExtSize;
    wave_header.RIFFChunkSize = 4 + 8 + wave_header.fmtXChunkSize + 8 + wave_header.dataChunkSize;
    
    /* Configure FATFS */
    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    //f_mount(0, &FatFs[0]);  // for FATFS v0.09
		// Register work area to the default drive
    f_mount(&FatFs[0], "", 0);  // for FATFS v0.11

        res = f_open(&wav_rec_hf.file2, WAV_FILE2, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("Open file error \r\n");
        return;
    }
    
    /* Preallocation for performance. */
    {
        f_lseek(&wav_rec_hf.file2, sizeof (wave_header) + sizeof (PcmRxBuff[0]) / wav_rec_hf.u16BlockAlignHw * wave_header.BlockAlign * wav_rec_hf.u32NumBuffRun - 1);
        
        f_write(&wav_rec_hf.file2, &wave_header, 1, &s2);
        f_lseek(&wav_rec_hf.file2, 0);
    }
    
    f_write(&wav_rec_hf.file2, &wave_header, sizeof (wave_header), &s2);
    f_sync(&wav_rec_hf.file2);
}

void wave_recorder_bh(void)
{
    //FRESULT res;
    uint32_t i=0;
    UINT s2;
    int play_started = 0;
    
    rb_init(&audio_rb, ringbuff, sizeof (ringbuff));
    
    /* Start to record. */
    I2S_ENABLE_RXDMA(I2S);
    I2S_ENABLE_RX(I2S);
    
    NVIC_EnableIRQ(I2S_IRQn);
    I2S_EnableInt(I2S, (I2S_IEN_RDMATIEN_Msk|I2S_IEN_RDMAEIEN_Msk));
                
    while (i < wav_rec_hf.u32NumBuffRun) {
        int empty;
        void *next_rd_p;
        unsigned next_rd_cap;
        int tmp;
        
        /* Wait until ring buffer is not empty. */
        do {
            NVIC_DisableIRQ(I2S_IRQn);
            empty = rb_empty(&audio_rb);
            NVIC_EnableIRQ(I2S_IRQn);
        }
        while (empty);
        
        if (! play_started) {
            /* Start playback after the first buffer has been ready. */
            I2S_ENABLE_TXDMA(I2S);
            I2S_ENABLE_TX(I2S);
            play_started = 1;
        }
        
        NVIC_DisableIRQ(I2S_IRQn);
        rb_next_read(&audio_rb, &next_rd_p, &next_rd_cap);
        NVIC_EnableIRQ(I2S_IRQn);
        
        /* One whole write will lock 2nd level buffer. So write step by step. */
        if (next_rd_cap > MAX_WRITE_STRIDE) {
            next_rd_cap = MAX_WRITE_STRIDE;
        }
        
        if (wave_recorder_param.BitsPerSample == 24) {  // 24-bit.
#if SUPPORT_24BIT
            void *next_rd_p2 = next_rd_p;
            unsigned next_rd_cap2 = next_rd_cap;
            
            uint8_t *buff_24_ind = PcmRxBuff_24;
            uint8_t *buff_24_end = PcmRxBuff_24 + sizeof (PcmRxBuff_24);
            
            while (next_rd_cap2) {
                uint8_t *buff_32_ind = (uint8_t *) next_rd_p2;
                uint8_t *buff_32_end = buff_32_ind + sizeof (PcmRxBuff[0]);
                
                while (buff_32_ind != buff_32_end) {
                    *buff_24_ind ++ = *buff_32_ind ++;
                    *buff_24_ind ++ = *buff_32_ind ++;
                    *buff_24_ind ++ = *buff_32_ind ++;
                    *buff_32_ind ++;
                }
                NVIC_DisableIRQ(I2S_IRQn);
                rb_read_done(&audio_rb, sizeof (PcmRxBuff[0]));
                NVIC_EnableIRQ(I2S_IRQn);
                next_rd_p2 = (uint8_t *) next_rd_p2 + sizeof (PcmRxBuff[0]);
                next_rd_cap2 -= sizeof (PcmRxBuff[0]);
                
                if (! next_rd_cap2 || buff_24_ind == buff_24_end) {
                    f_write(&wav_rec_hf.file2, PcmRxBuff_24, buff_24_ind - PcmRxBuff_24, &s2);
                    buff_24_ind = PcmRxBuff_24;
                }
            }
#else
            printf("24-bit sample size not supported!\n");
            break;
#endif  // #if SUPPORT_24BIT
        }
        else {  // 8-, 16, 32-bit.
            f_write(&wav_rec_hf.file2, next_rd_p, next_rd_cap, &s2);
            
            NVIC_DisableIRQ(I2S_IRQn);
            rb_read_done(&audio_rb, next_rd_cap);
            NVIC_EnableIRQ(I2S_IRQn);
        }
        
        /* Check buffer overrun. */
        NVIC_DisableIRQ(I2S_IRQn);
        tmp = nBuffOverRun;
        nBuffOverRun = 0;
        NVIC_EnableIRQ(I2S_IRQn);
        if (tmp) {
            printf("Buffer overrun: %d\n", tmp);
        }
        
        tmp = next_rd_cap / sizeof (PcmRxBuff[0]);
#if 0
        while (tmp >= 8) {
            tmp -= 8;
            i += 8;
            printf("........");
        }
        while (tmp --) {
            i ++;
            printf(".");
        }
#else
        i += tmp;
#endif
    }
        
    f_close(&wav_rec_hf.file2);
        
    printf( "\n### wave recorded ###\n" );
}

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0)) {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    } else {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }

	/* To avoid the synchronization issue between system and APB clock domain */
    u32Status = I2C_GET_STATUS(I2C0);
}

uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[2];
uint8_t g_u8RxData;
uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted and prepare SLA+W */
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != 1) {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_SI);
        }
    } else if (u32Status == 0x10) {             /* Repeat START has been transmitted and prepare SLA+R */
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x01);  /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x40) {             /* SLA+R has been transmitted and ACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);
    } else if (u32Status == 0x50) {             /* DATA has been received and NACK has been returned */
        g_u8RxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x58) {             /* DATA has been received and NACK has been returned */
        g_u8RxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
				g_u8EndFlag = 1;
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted */
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != 2) {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
        }
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void NAU8822_WriteData(char addr, unsigned short data)
{
		g_au8TxData[0] = ((addr << 1)  | (data >> 8));		//addr(7bit) + data(first bit)
		g_au8TxData[1] = (char)(data & 0x00FF);			//data(8bit)

		g_u8DataLen = 0;
		g_u8EndFlag = 0;

		/* I2C as master sends START signal */
		I2C_SET_CONTROL_REG(I2C0, I2C_STA);

		/* Wait I2C Tx Finish */
		while (g_u8EndFlag == 0);
		g_u8EndFlag = 0;
}

void NAU8822_ReadData(char addr)
{
		g_au8TxData[0] = (addr << 1); 					//addr(7bit) + 0 (lsb)

		g_u8DataLen = 0;
		g_u8EndFlag = 0;

		/* I2C as master sends START signal */
		I2C_SET_CONTROL_REG(I2C0, I2C_STA);

		/* Wait I2C Tx Finish */
		while (g_u8EndFlag == 0);
		g_u8EndFlag = 0;
}

void I2C_Delay(int count)
{
	volatile unsigned int i;
	for (i = 0; i < count ; i++);
}

void demo_LineIn()
{
		printf("  NOTE: Need head-phone for external CODEC.\n");
		printf("  NOTE: Need Line-In for external CODEC.\n");

		/* I2C function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
		g_u8DeviceAddr = 0x1A;
		
		NAU8822_WriteData(0, 0x000);   /* Reset all registers */
		I2C_Delay(0x1000);
		
		//input source is Line-In
		NAU8822_WriteData(1, 0x01F);//R1 MICBIASEN ABIASEN IOBUFEN REFIMP
		NAU8822_WriteData(4, 0x070);//R4 select audio format(I2S format) and word length (32bits)
		NAU8822_WriteData(5, 0x000);//R5 companding ctrl
		NAU8822_WriteData(6, 0x000);//R6 clock ctrl at slave mode
		NAU8822_WriteData(35, 0x000);//R35 disable noise gate
		NAU8822_WriteData(44, 0x044);//R44 Line-In
		NAU8822_WriteData(47, 0x050);//R47 Left ADC boost
		NAU8822_WriteData(48, 0x050);//R48 Right ADC boost
		NAU8822_WriteData(2, 0x1BF);//R2 Power Management 2
		NAU8822_WriteData(3, 0x00F);//R3 Power Management 3
		NAU8822_WriteData(10, 0x000);//R10 DAC control (softmute disable, oversample select 64x (lowest power) )
		NAU8822_WriteData(50, 0x001);//R50 Left mixer
		NAU8822_WriteData(51, 0x001);//R51 Right mixer
		NAU8822_WriteData(49, 0x002);//R49 Output control

		printf("I2C write NAU8822 OK\n");
		
		s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
		
		NAU8822_ReadData(1);//R1 MICBIASEN ABIASEN IOBUFEN REFIMP
		NAU8822_ReadData(4);//R4 select audio format(I2S format) and word length (32bits)
		NAU8822_ReadData(5);//R5 companding ctrl
		NAU8822_ReadData(6);//R6 clock ctrl at slave mode
		NAU8822_ReadData(35);//R35 disable noise gate
		NAU8822_ReadData(44);//R44 Line-In
		NAU8822_ReadData(47);//R47 Left ADC boost
		NAU8822_ReadData(48);//R48 Right ADC boost
		NAU8822_ReadData(2);//R2 Power Management 2
		NAU8822_ReadData(3);//R3 Power Management 3
		NAU8822_ReadData(10);//R10 DAC control (softmute disable, oversample select 64x (lowest power) )
		NAU8822_ReadData(50);//R50 Left mixer
		NAU8822_ReadData(51);//R51 Right mixer
		NAU8822_ReadData(49);//R49 Output control
		
		printf("I2C read NAU8822 OK at g_u8RxData\n");

    printf("[OK]\n");
}

void demo_MIC(void)
{
	printf("  NOTE: Need head-phone for external CODEC.\n");
		printf("  NOTE: Need 2 mic for external CODEC.\n");

		/* I2C function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
		g_u8DeviceAddr = 0x1A;
		
		NAU8822_WriteData(0, 0x000);   /* Reset all registers */
		I2C_Delay(0x1000);
		
		//input source is MIC
		NAU8822_WriteData(1, 0x01F);//R1 MICBIASEN ABIASEN IOBUFEN REFIMP
		NAU8822_WriteData(4, 0x070);//R4 select audio format(I2S format) and word length (32bits)
		NAU8822_WriteData(5, 0x000);//R5 companding ctrl
		NAU8822_WriteData(6, 0x000);//R6 clock ctrl at slave mode
		NAU8822_WriteData(35, 0x000);//R35 disable noise gate
		NAU8822_WriteData(45, 0x13F);//R45 Left input PGA gain
		NAU8822_WriteData(46, 0x13F);//R46 Right input PGA gain
		NAU8822_WriteData(44, 0x033);//R44 MIC
		NAU8822_WriteData(47, 0x000);//R47 Left ADC boost
		NAU8822_WriteData(48, 0x000);//R48 Right ADC boost
		NAU8822_WriteData(2, 0x1BF);//R2 Power Management 2
		NAU8822_WriteData(3, 0x00F);//R3 Power Management 3
		NAU8822_WriteData(10, 0x000);//R10 DAC control (softmute disable, oversample select 64x (lowest power) )
		NAU8822_WriteData(45, 0x13F);//R45 Left input PGA gain
		NAU8822_WriteData(46, 0x13F);//R46 Right input PGA gain
		NAU8822_WriteData(50, 0x001);//R50 Left mixer
		NAU8822_WriteData(51, 0x001);//R51 Right mixer
		NAU8822_WriteData(49, 0x002);//R49 Output control

		printf("I2C write NAU8822 OK\n");
		
		s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
		
		NAU8822_ReadData(1);//R1 MICBIASEN ABIASEN IOBUFEN REFIMP
		NAU8822_ReadData(4);//R4 select audio format(I2S format) and word length (32bits)
		NAU8822_ReadData(5);//R5 companding ctrl
		NAU8822_ReadData(6);//R6 clock ctrl at slave mode
		NAU8822_ReadData(35);//R35 disable noise gate
		NAU8822_ReadData(45);//R45 Left input PGA gain
		NAU8822_ReadData(46);//R46 Right input PGA gain
		NAU8822_ReadData(44);//R44 MIC
		NAU8822_ReadData(47);//R47 Left ADC boost
		NAU8822_ReadData(48);//R48 Right ADC boost
		NAU8822_ReadData(2);//R2 Power Management 2
		NAU8822_ReadData(3);//R3 Power Management 3
		NAU8822_ReadData(10);//R10 DAC control (softmute disable, oversample select 64x (lowest power) )
		NAU8822_ReadData(45);//R45 Left input PGA gain
		NAU8822_ReadData(46);//R46 Right input PGA gain
		NAU8822_ReadData(50);//R50 Left mixer
		NAU8822_ReadData(51);//R51 Right mixer
		NAU8822_ReadData(49);//R49 Output control
		
		printf("I2C read NAU8822 OK at g_u8RxData\n");
	 
	 printf("[OK]\n");
}

void SYS_Init(void)
{

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    //SYS_UnlockReg();
     
    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

//    CLK_SetCoreClock(FREQ_96MHZ);
        CLK_SetCoreClock(100000000);
    
        /* PCLK divider */
        CLK_SetModuleClock(PCLK_MODULE, NULL, 1);
        
    /* Lock protected registers */
    //SYS_LockReg();

        //--- Initial SD0 multi-function pin
    SD0_Init();
}

void UART0_Init(void)
{
        /* Enable UART0 Module clock */
    CLK_EnableModuleClock(UART0_MODULE);
        /* UART0 module clock from EXT */
        CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    /* Reset IP */
    SYS_ResetModule(UART0_RST);    
    /* Configure UART0 and set UART0 Baud-rate */
        UART_Open(UART0, 115200);
        /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
        SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD; 
        SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD; 
    
}

void I2C0_Init(void)
{
		/* Enable I2C0 Module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
		/* Reset IP */
    SYS_ResetModule(I2C0_RST);    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPA14,GPA15 multi-function pins for I2C0 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA14MFP_Msk) ) | SYS_GPA_MFPH_PA14MFP_I2C0_SCL;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk) ) | SYS_GPA_MFPH_PA15MFP_I2C0_SDA;
		
		/* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);
		
		/* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
		
		I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2S_Init(void)
{
        /* Enable I2S Module clock */
    CLK_EnableModuleClock(I2S_MODULE);
        /* I2S module clock from APLL */
    
    if (wave_recorder_param.SampleRate % 11025) {
        // APLL = 49152031Hz
        CLK_SET_APLL(CLK_APLL_49152031);
        //CLK_SysTickDelay(500);    //delay 5 seconds
        // I2S = 49152031Hz / (0+1) = 49152031Hz for 8k, 12k, 16k, 24k, 32k, 48k, and 96k sampling rate
        CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);
    }
    else {
        // APLL = 45158425Hz
        CLK_SET_APLL(CLK_APLL_45158425);
        //CLK_SysTickDelay(500);    //delay 5 seconds
        // I2S = 45158425Hz / (0+1) = 45158425Hz for 11025, 22050, and 44100 sampling rate
        CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);
    }
    
    /* Reset IP */
    SYS_ResetModule(I2S_RST);    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for I2S */
        // GPC[8]  = MCLK
        // GPC[9]  = DIN
        // GPC[10] = DOUT
        // GPC[11] = LRCLK
        // GPC[12] = BCLK
        SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk) ) | SYS_GPC_MFPH_PC8MFP_I2S_MCLK;  
        SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk) ) | SYS_GPC_MFPH_PC9MFP_I2S_DIN;   
        SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk) ) | SYS_GPC_MFPH_PC10MFP_I2S_DOUT;    
        SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk) ) | SYS_GPC_MFPH_PC11MFP_I2S_LRCLK;   
        SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk) ) | SYS_GPC_MFPH_PC12MFP_I2S_BCLK;    
    
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

//---------------------------------------------------------
//--- Initial SD0 multi-function GPIO pin
//
// NUC505 support 3 groups of GPIO pins and SD sockets for same one SD port.
// Please select ONLY ONE configuration from them.
// 1. SD-A socket on daughter board + default SD0_Init(). (Default)
// 2. SD-B socket on main board + short JP3 and JP4
//    + define compile flag "SDH_GPIO_GB" in SD0_Init().
//    (Note: this configuration conflict with UART1)
// 3. SD-C socket on main board + short JP3 and JP2
//    + define compile flag "SDH_GPIO_GA" in SD0_Init()
//    (Note: this configuration conflict with UART0)
//---------------------------------------------------------
void SD0_Init(void)
{
#ifdef SDH_GPIO_GA
    // The group A are GPA10~11, GPA13~15, GPB0~1
    // Conflict with UART0
    // printf("SD_Open(): Configure GPIO group A as SDH pins.\n");
    SYS->GPA_MFPH &= (~0x77707700);
    SYS->GPA_MFPH |=   0x44404400;
    SYS->GPA_MFPH &= (~0x00000077);
    SYS->GPB_MFPL |=   0x00000044;

#elif defined SDH_GPIO_GB
    // The group B are GPB2~3, GPB5~9
    // Conflict with UART1
    // printf("SD_Open(): Configure GPIO group B as SDH pins.\n");
    SYS->GPB_MFPL &= (~0x77707700);
    SYS->GPB_MFPL |=   0x44404400;
    SYS->GPB_MFPH &= (~0x00000077);
    SYS->GPB_MFPH |=   0x00000044;

#elif defined SDH_GPIO_G_48PIN
    // The group 48PIN are GPB0~3, GPB5~7 for NUC505 48PIN chip
    // Conflict with both UART0 and UART1
    // printf("SD_Open(): Configure special GPIO as SDH pins for 48 pins NUC505 chip.\n");
    SYS->GPB_MFPL &= (~0x77707777);
    SYS->GPB_MFPL |=   0x44404444;

#else   // default for defined SDH_GPIO_GC
    // The group C are GPC0~2, GPC4~7
    // printf("SD_Open(): Configure GPIO group C as SDH pins.\n");
    SYS->GPC_MFPL &= (~0x77770777);
    SYS->GPC_MFPL |=   0x11110111;
#endif
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
