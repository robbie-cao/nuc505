/**************************************************************************//**
 * @file     main.c
 * @version  V5.2
 * $Revision: 10 $
 * $Date: 15/09/30 08:23 p.m. $ 
 * @brief    NUC505 Series Global Control and Clock Control Driver Sample Code
 *					
 * 1.	Demonstrate delay function by systick.
 * 2.	Demonstrate core clock switching.
 * 3.	Demonstrate how to enable module clock and set module clock divider.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "usbd_audio.h"

#ifdef __HID__
#include "Keypad.h"
#ifdef __ICCARM__
#pragma data_alignment=4
	uint8_t buf[8];
#else
__align(4) uint8_t buf[8];
#endif
void HID_UpdateKbData(void);
extern uint8_t volatile g_u8EPCReady;
#endif

//int32_t g_volume_flag;
void InternalCODEC_Setup(int8_t i8PlayLVol, int8_t i8PlayRVol);
extern uint32_t g_ubsd_rx_flag;

extern volatile uint8_t bPlayVolumeLAdjust;
extern volatile uint8_t bPlayVolumeRAdjust;
extern volatile uint8_t bSampleRateAdjust;
extern volatile uint8_t bPlayMuteAdjust;

extern volatile uint32_t g_usbd_SampleRate;
extern volatile uint32_t g_usbd_SampleBitRate;
extern uint8_t  g_usbd_PlayMute;
extern int16_t  g_usbd_PlayVolumeL;
extern int16_t  g_usbd_PlayVolumeR;

/* FIXME */
extern volatile uint32_t g_u32Temp1;
extern volatile uint32_t g_u32Temp2;

/* FIXME */
uint32_t g_u32Temp3;
uint32_t g_u32Temp4;
//uint32_t g_u32Temp5;
uint32_t g_u32Temp6;
uint32_t g_u32Temp7;
uint32_t g_u32Temp8;
uint32_t g_u32Temp9;

/* FIXME */
volatile uint32_t g_u32DecreaseAPLL;
volatile uint32_t g_u32CurrentAPLL;
volatile uint32_t g_u32IncreaseAPLL;
volatile uint32_t g_u32RefAPLL;

extern volatile uint8_t g_u8AudioPlaying;
//volatile int32_t g_i32AudioPlayingCnt;

extern volatile uint32_t g_u32Flag1;
extern volatile uint32_t g_u32Flag2;
extern volatile uint32_t g_u32Flag3;

#ifdef __HID__
static volatile uint32_t s_u32TimeOutCounter;

#define EVB_VKEYC00BIT       (0x0001)		/* GPIO C0 */
#define EVB_VKEYC01BIT       (0x0002)		/* GPIO C1 */
#define EVB_VKEYC02BIT       (0x0004)		/* GPIO C2 */
#define EVB_VKEYC03BIT       (0x0008)		/* GPIO C3 */
#define EVB_VKEYC04BIT       (0x0010)		/* GPIO C4 */

/*
   GPIO KEY has three states, falling, pressiong, and releasing.

   postfix _F means falling
   postfix _P means pressing
   postfix _R means releasing
*/
#define APP_KEYPAD_C00_F         (0x00)
#define APP_KEYPAD_C00_P         (0x01)
#define APP_KEYPAD_C00_R         (0x02)
#define APP_KEYPAD_C01_F         (0x10)
#define APP_KEYPAD_C01_P         (0x11)
#define APP_KEYPAD_C01_R         (0x12)
#define APP_KEYPAD_C02_F         (0x20)
#define APP_KEYPAD_C02_P         (0x21)
#define APP_KEYPAD_C02_R         (0x22)
#define APP_KEYPAD_C03_F         (0x30)
#define APP_KEYPAD_C03_P         (0x31)
#define APP_KEYPAD_C03_R         (0x32)
#define APP_KEYPAD_C04_F         (0x40)
#define APP_KEYPAD_C04_P         (0x41)
#define APP_KEYPAD_C04_R         (0x42)

void
App_KeypadEventHandler(
	uint32_t u32Param
)
{
#ifdef __KEYBOARD__	
	switch (u32Param) {	
	case APP_KEYPAD_C00_F:		/* falling */
//		printf("C00_F\n");
		break;
	case APP_KEYPAD_C00_P:		/* pressing */
//		printf("C00_P\n");
		break;
	case APP_KEYPAD_C00_R:		/* releasing */
		buf[2] = 0x04;	/* Key A */
//		printf("C00_R\n");
		break;
	case APP_KEYPAD_C01_F:		/* falling */
//		printf("C01_F\n");
		break;
	case APP_KEYPAD_C01_P:		/* pressing */
//		printf("C01_P\n");
		break;
	case APP_KEYPAD_C01_R:		/* releasing */
		buf[2] = 0x05;	/* Key B */
//		printf("C01_R\n");
		break;
	case APP_KEYPAD_C02_F:		/* falling */
//		printf("C02_F\n");
		break;
	case APP_KEYPAD_C02_P:		/* pressing */
//		printf("C02_P\n");
		break;
	case APP_KEYPAD_C02_R:		/* releasing */
		buf[2] = 0x06;	/* Key C */
//		printf("C02_R\n");
		break;
	case APP_KEYPAD_C03_F:		/* falling */
//		printf("C03_F\n");
		break;
	case APP_KEYPAD_C03_P:		/* pressing */
//		printf("C03_P\n");
		break;
	case APP_KEYPAD_C03_R:		/* releasing */
		buf[2] = 0x07;	/* Key D */
//		printf("C03_R\n");
		break;
	case APP_KEYPAD_C04_F:		/* falling */
//		printf("C04_F\n");
		break;
	case APP_KEYPAD_C04_P:		/* pressing */
//		printf("C04_P\n");
		break;
	case APP_KEYPAD_C04_R:		/* releasing */
		buf[2] = 0x08;	/* Key E */
//		printf("C04_R\n");
		break;
	default:
		break;
	}
#elif defined __MEDIAKEY__
	buf[0] = 0;
	buf[1] = 0;
	switch (u32Param) {
	case APP_KEYPAD_C00_F:		/* falling */
//		printf("C00_F\n");
		break;
	case APP_KEYPAD_C00_P:		/* pressing */
//		printf("C00_P\n");
		break;
	case APP_KEYPAD_C00_R:		/* releasing */
		buf[1] |= HID_CTRL_PAUSE;	/* play/pause */
//		printf("C00_R\n");
		break;
	case APP_KEYPAD_C01_F:		/* falling */
//		printf("C01_F\n");
		break;
	case APP_KEYPAD_C01_P:		/* pressing */
//		printf("C01_P\n");
		break;
	case APP_KEYPAD_C01_R:		/* releasing */
		buf[1] |= HID_CTRL_NEXT;	/* Next */
//		printf("C01_R\n");
		break;
	case APP_KEYPAD_C02_F:		/* falling */
//		printf("C02_F\n");
		break;
	case APP_KEYPAD_C02_P:		/* pressing */
//		printf("C02_P\n");
		break;
	case APP_KEYPAD_C02_R:		/* releasing */
		buf[1] |= HID_CTRL_PREVIOUS;	/* Previous */
//		printf("C02_R\n");
		break;
	case APP_KEYPAD_C03_F:		/* falling */
//		printf("C03_F\n");
		break;
	case APP_KEYPAD_C03_P:		/* pressing */
		buf[0] |= HID_CTRL_VOLUME_INC;
//		printf("C03_P\n");
		break;
	case APP_KEYPAD_C03_R:		/* releasing */
		buf[0] |= HID_CTRL_VOLUME_INC;	/* Vol+ */
//		printf("C03_R\n");
		break;
	case APP_KEYPAD_C04_F:		/* falling */
//		printf("C04_F\n");
		break;
	case APP_KEYPAD_C04_P:		/* pressing */
		buf[0] |= HID_CTRL_VOLUME_DEC;
//		printf("C04_P\n");
		break;
	case APP_KEYPAD_C04_R:		/* releasing */
		buf[0] |= HID_CTRL_VOLUME_DEC;	/* Vol- */
//		printf("C04_R\n");
		break;
	default:
		break;
	}
#endif
}

static const S_KEYPAD_TGR_HANDLER g_asKeypadTgrHandler[] = {
	{App_KeypadEventHandler, APP_KEYPAD_C00_F,           EVB_VKEYC00BIT, KEYPAD_GPIOC, KEYPAD_FALLING},
	{App_KeypadEventHandler, APP_KEYPAD_C00_P,           EVB_VKEYC00BIT, KEYPAD_GPIOC, KEYPAD_PRESSING},
	{App_KeypadEventHandler, APP_KEYPAD_C00_R,           EVB_VKEYC00BIT, KEYPAD_GPIOC, KEYPAD_RISING},
	{App_KeypadEventHandler, APP_KEYPAD_C01_F,           EVB_VKEYC01BIT, KEYPAD_GPIOC, KEYPAD_FALLING},
	{App_KeypadEventHandler, APP_KEYPAD_C01_P,           EVB_VKEYC01BIT, KEYPAD_GPIOC, KEYPAD_PRESSING},
	{App_KeypadEventHandler, APP_KEYPAD_C01_R,           EVB_VKEYC01BIT, KEYPAD_GPIOC, KEYPAD_RISING},
	{App_KeypadEventHandler, APP_KEYPAD_C02_F,           EVB_VKEYC02BIT, KEYPAD_GPIOC, KEYPAD_FALLING},
	{App_KeypadEventHandler, APP_KEYPAD_C02_P,           EVB_VKEYC02BIT, KEYPAD_GPIOC, KEYPAD_PRESSING},
	{App_KeypadEventHandler, APP_KEYPAD_C02_R,           EVB_VKEYC02BIT, KEYPAD_GPIOC, KEYPAD_RISING},
	{App_KeypadEventHandler, APP_KEYPAD_C03_F,           EVB_VKEYC03BIT, KEYPAD_GPIOC, KEYPAD_FALLING},
	{App_KeypadEventHandler, APP_KEYPAD_C03_P,           EVB_VKEYC03BIT, KEYPAD_GPIOC, KEYPAD_PRESSING},
	{App_KeypadEventHandler, APP_KEYPAD_C03_R,           EVB_VKEYC03BIT, KEYPAD_GPIOC, KEYPAD_RISING},
	{App_KeypadEventHandler, APP_KEYPAD_C04_F,           EVB_VKEYC04BIT, KEYPAD_GPIOC, KEYPAD_FALLING},
	{App_KeypadEventHandler, APP_KEYPAD_C04_P,           EVB_VKEYC04BIT, KEYPAD_GPIOC, KEYPAD_PRESSING},
	{App_KeypadEventHandler, APP_KEYPAD_C04_R,           EVB_VKEYC04BIT, KEYPAD_GPIOC, KEYPAD_RISING},

	{0,0,0,0,0}         // ended by all zero
};

void TMR0_IRQHandler(void)
{
	// clear timer interrupt flag
	TIMER_ClearIntFlag(TIMER0);

	s_u32TimeOutCounter++;
	Keypad_TgrDecDebounceCounter();

	/* To avoid the synchronization issue between system and APB clock domain */
	TIMER_GetIntFlag(TIMER0);
}

void KEY_Init(void)
{
	/* Enable IP clock */
  CLK_EnableModuleClock(TMR0_MODULE);

  /* Select IP clock source */
  CLK_SetModuleClock(TMR0_MODULE, CLK_TMR0_SRC_EXT, 0);

	SYS->GPC_MFPL  = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk) );
	SYS->GPC_MFPL  = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk) );
	SYS->GPC_MFPL  = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk) );
	SYS->GPC_MFPL  = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk) );
	SYS->GPC_MFPL  = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk) );
	
	GPIO_SetMode(PC, (BIT4 |BIT3 |BIT2 |BIT1 |BIT0), GPIO_MODE_INPUT);
  GPIO_SetPullMode(PC, (BIT4 |BIT3 |BIT2 |BIT1 |BIT0), GPIO_PULL_UP_EN);
	
	// Debounce 8 time unit, pressing event interval 80 time unit
  Keypad_InitKeypad(8,40);
	
	Keypad_InitTgr(
        NULL,
        NULL,
				(EVB_VKEYC04BIT |EVB_VKEYC03BIT |EVB_VKEYC02BIT |EVB_VKEYC01BIT |EVB_VKEYC00BIT),
        g_asKeypadTgrHandler);

  // Set timer frequency
  TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100);

  // Enable timer interrupt
  TIMER_EnableInt(TIMER0);
  NVIC_EnableIRQ(TMR0_IRQn);

  // Start Timer 0
  TIMER_Start(TIMER0);
}
#endif

void SYS_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/   
    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;        

		/* Configure System Core Clock */
		CLK_SetCoreClock(FREQ_96MHZ);
	
    /* Set PCLK divider */
    CLK_SetModuleClock(PCLK_MODULE, NULL, 1);
	
    /* Update System Core Clock */
    SystemCoreClockUpdate();
    
      /* Enable USB IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select USB IP clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_USBD_SRC_EXT, 0);          
}

void I2S_Init(uint32_t u32SR, uint32_t u32SBR)
{
	uint32_t u32Temp1 = 0, u32Temp2 = 0, u32Temp3;
	
		/* Enable I2S Module clock */
    CLK_EnableModuleClock(I2S_MODULE);
		/* I2S module clock from APLL */

	if ( u32SR % 11025 )
	{
		// APLL = 49152031Hz
		CLK_SET_APLL(CLK_APLL_49152031);
		
		if ( u32SR == 8000 )
			// I2S = 49152031Hz / (1+1) for 8k sampling rate
			CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 1);
		else
			// I2S = 49152031Hz / (0+1) for 12k, 16k, 24k, 32k, 48k, and 96k sampling rate
			CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);

	g_u32DecreaseAPLL = CLK_APLL_49142857;
	g_u32CurrentAPLL = CLK_APLL_49152031;
	g_u32IncreaseAPLL = 0x844D542;
	g_u32RefAPLL = g_u32CurrentAPLL;
	
	/* FIXME */
	if ( u32SBR == 24 )
	{
	/* for 8000, 16000, 32000, 24000, 48000, and 96000 2 channels 24-bit */
	u32Temp1 = 5760; /* 5760 bytes = 96000 Hz * 2 channels * 3 bytes / 1000 ms * 10 ms */
	u32Temp2 = u32SR * 2 * 3 / 1000;
	}
	else if ( u32SBR == 16 )
	{
	/* for 8000, 16000, 32000, 24000, 48000, and 96000 2 channels 16-bit */
	u32Temp1 = 3840; /* 3840 bytes = 96000 Hz * 2 channels * 2 bytes / 1000 ms * 10 ms */
	u32Temp2 = u32SR * 2 * 2 / 1000;
	}
	else
	{
		printf( "Error! Bit rate not supported %d\n", u32SBR );
	}
	g_u32Temp3 = u32Temp1 / u32Temp2;
	g_u32Temp3--;

	/* FIXME */
	g_u32Temp1 = u32Temp2;
	g_u32Temp2 = u32Temp2;
	
	}
	else
	{
		// APLL = 45158425Hz
		CLK_SET_APLL(CLK_APLL_45158425);
		
		// I2S = 45158425Hz / (0+1) for 11025k, 22050k, and 44100k sampling rate
		CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);
	
		g_u32DecreaseAPLL = CLK_APLL_45142857;
		g_u32CurrentAPLL = CLK_APLL_45158425;
		g_u32IncreaseAPLL = 0xE04D382;
		g_u32RefAPLL = g_u32CurrentAPLL;
		
		/* FIXME */
		if ( u32SBR == 24 )
		{
		/* for 11025, 22050 and 44100 2 channels 24-bit */
		u32Temp1 = 2646; /* 2646 bytes = 44100 Hz * 2 channels * 3 bytes / 1000 ms * 10 ms */
		u32Temp2 = u32SR * 2 * 3 / 1000;
		}
		else if ( u32SBR == 16 )
		{
		/* for 11025, 22050 and 44100 2 channels 16-bit */
		u32Temp1 = 1764; /* 1764 bytes = 44100 Hz * 2 channels * 2 bytes / 1000 ms * 10 ms */
		u32Temp2 = u32SR * 2 * 2 / 1000;
		}
		else
		{
			printf( "Error! Bit rate not supported %d\n", u32SBR );
		}
		g_u32Temp3 = u32Temp1 / u32Temp2;
		u32Temp3 = u32Temp2 * g_u32Temp3;
		u32Temp3 = (u32Temp1 - u32Temp3) + u32Temp2;
		g_u32Temp3--;

		/* FIXME */
		g_u32Temp1 = u32Temp2;
		g_u32Temp2 = u32Temp3;
	}

	/* FIXME TX start condition */
	if ( u32SBR == 24 )
	{
	g_u32Temp4 = (u32Temp2 * 3) / 3; /* (3ms, byte size) / 3 = sample count */
	}
	else if ( u32SBR == 16 )
	{
	g_u32Temp4 = (u32Temp2 * 3) / 2; /* (3ms, byte size) / 2 = sample count */
	}
	else
	{
		printf( "Error! Bit rate not supported %d\n", u32SBR );
	}

	/* FIXME TX noise detect and try to fix condition */
	g_u32Temp6 = g_u32Temp1 * 2;
	g_u32Temp7 = RING_BUF_SMPL_SZ - g_u32Temp6;

	/* FIXME RX start condition */
	g_u32Temp8 = g_u32Temp1 * 5;
	
	/* FIXME RX noise detect */
	g_u32Temp9 = RING_BUF_SMPL_SZ - g_u32Temp1;

    /* Reset IP */
    SYS_ResetModule(I2S_RST);    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for SPI0 */
		// GPC[8]  = MCLK
		// GPC[9]  = DIN
		// GPC[10] = DOUT
		// GPC[11] = LRCLK
		// GPC[12] = BCLK
//		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk) ) | SYS_GPC_MFPH_PC8MFP_I2S_MCLK;	
//		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk) ) | SYS_GPC_MFPH_PC9MFP_I2S_DIN;	
//		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk) ) | SYS_GPC_MFPH_PC10MFP_I2S_DOUT;	
//		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk) ) | SYS_GPC_MFPH_PC11MFP_I2S_LRCLK;	
//		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk) ) | SYS_GPC_MFPH_PC12MFP_I2S_BCLK;	
	
}

void UART0_Init(void)
{
		/* Enable UART1 Module clock */
    CLK_EnableModuleClock(UART0_MODULE);
	
		/* UART1 module clock from EXT */
		CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
	
    /* Reset IP */
    SYS_ResetModule(UART0_RST);    
	
    /* Configure UART0 and set UART0 Baudrate */
		UART_Open(UART0, 115200);
	
		/*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
		SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;	
		SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;	
	
}

void UART1_Init(void)
{
		/* Enable UART1 Module clock */
    CLK_EnableModuleClock(UART1_MODULE);
	
		/* UART1 module clock from EXT */
		CLK_SetModuleClock(UART1_MODULE, CLK_UART1_SRC_EXT, 0);
	
    /* Reset IP */
    SYS_ResetModule(UART1_RST);    
	
    /* Configure UART1 and set UART1 Baudrate */
		UART_Open(UART1, 115200);
	
		/*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART1 RXD and TXD */
		SYS->GPA_MFPH  = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA8MFP_Msk) ) | SYS_GPA_MFPH_PA8MFP_UART1_TXD;	
		SYS->GPA_MFPH  = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA9MFP_Msk) ) | SYS_GPA_MFPH_PA9MFP_UART1_RXD;	
	
}

int main(void)
{
		uint32_t u32CurSR, u32CurSBR, u32LastTime = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
	
    /* Init UART to 115200-8n1 for print message */
    UART0_Init();   
//    UART1_Init();

	
		printf("NUC505 USB UAC - Speaker");
#ifdef __HID__
		printf(" + HID");
#ifdef __KEYBOARD__
		printf(" - Keyboard");
#elif defined __MEDIAKEY__	
		printf(" - Mediakey");
#endif
#endif	
		printf("\n");
#if defined (__ICCARM__)
        #pragma section = "VECTOR2"              
        
				extern uint32_t __Vectors[];
				extern uint32_t __Vectors_Size[];	
             
				//printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", __section_begin("VECTOR2"));
				memcpy((void *) __section_begin("VECTOR2"), (void *) __Vectors, (unsigned int) __Vectors_Size);
				SCB->VTOR = (uint32_t) __section_begin("VECTOR2");
        
#else 
	/* Relocate vector table in SRAM for fast interrupt handling. */
    {
        extern uint32_t __Vectors[];
        extern uint32_t __Vectors_Size[];
        extern uint32_t Image$$ER_VECTOR2$$ZI$$Base[];
    
//        printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", Image$$ER_VECTOR2$$ZI$$Base);
        memcpy((void *) Image$$ER_VECTOR2$$ZI$$Base, (void *) __Vectors, (unsigned int) __Vectors_Size);
        SCB->VTOR = (uint32_t) Image$$ER_VECTOR2$$ZI$$Base;
    }
#endif 		

		/* Init I2S, IP clock and multi-function I/O */
		I2S_Init(PLAY_DEF_RATE, PLAY_BIT_RATE);
		
#ifdef __HID__
	/* Init KEY */
	KEY_Init();
#endif

		/* FIXME: support stereo and 16-bit/24-bit only for now */
		if ( PLAY_BIT_RATE == 24 )
		I2S_Open(I2S, I2S_MODE_MASTER, PLAY_DEF_RATE, I2S_DATABIT_32, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
		else if ( PLAY_BIT_RATE == 16 )
		I2S_Open(I2S, I2S_MODE_MASTER, PLAY_DEF_RATE, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
		else
			printf( "Error! Bit rate not supported %d\n", PLAY_BIT_RATE );
	u32CurSR = PLAY_DEF_RATE;
		u32CurSBR = PLAY_BIT_RATE;
		I2S_EnableMCLK(I2S, PLAY_DEF_RATE*256);
		
		I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
		
		InternalCODEC_Setup(g_usbd_PlayVolumeL, g_usbd_PlayVolumeR);
//	printf("1 %d %d %d\n", u32CurSR, g_usbd_SampleRate, bSampleRateAdjust);
	DMA_Init();

    USBD_Open(&gsInfo, UAC_ClassRequest, UAC_SetInterface);
		
    /* Endpoint configuration */
    UAC_Init();
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();
    while(1)
		{
				if(g_ubsd_rx_flag && (USBD->EP[EPA].EPDATCNT == 0))
				{
						EPB_Handler();
						g_ubsd_rx_flag = 0;
					u32LastTime = s_u32TimeOutCounter;
				}
				
				if (USBD_IS_ATTACHED()) {
						/* USB Plug In */
						USBD_ENABLE_USB();
				} else {
						UAC_DeviceDisable();    /* stop play */
						g_ubsd_rx_flag = 0;
//					printf("da\n");
				}			
#if 1
				/* FIXME workaound for pause noise in WinXP media player and short volume adjustment sound noise in Win7 */
				if ( g_u8AudioPlaying )
				{
					if ( s_u32TimeOutCounter - u32LastTime > 1 )
					{
						UAC_DeviceDisable();    /* stop play */
						printf("t\n");
					}
//					printf("%d\n", g_i32AudioPlayingCnt);
//					if ( g_i32AudioPlayingCnt-- < 0 )
//					{
//						UAC_DeviceDisable(UAC_SPEAKER);    /* stop play */
//						printf("c%d\n", g_i32AudioPlayingCnt);
//						g_i32AudioPlayingCnt = 1000;
//					}
				}
				else
				{
					if (g_u32Flag1)
					{
						if (USBD->CEPINTSTS & USBD_CEPINTSTS_RXPKIF_Msk)
						{
							USBD_CtrlOUT((uint32_t)&g_usbd_PlayVolumeL, gUsbCmd.wLength);
							bPlayVolumeLAdjust = TRUE;
							g_u32Flag1 = 0;
							/* Status stage */
							USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
							USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
							USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
						}
					}

					if (g_u32Flag2)
					{
						if (USBD->CEPINTSTS & USBD_CEPINTSTS_RXPKIF_Msk)
						{
							USBD_CtrlOUT((uint32_t)&g_usbd_PlayVolumeR, gUsbCmd.wLength);
							bPlayVolumeRAdjust = TRUE;
							g_u32Flag2 = 0;
							/* Status stage */
							USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
							USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
							USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
						}
					}

					if (g_u32Flag3)
					{
						if (USBD->CEPINTSTS & USBD_CEPINTSTS_RXPKIF_Msk)
						{
							USBD_CtrlOUT((uint32_t)&g_usbd_PlayMute, gUsbCmd.wLength);
							bPlayMuteAdjust = TRUE;
							g_u32Flag3 = 0;
							/* Status stage */
							USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
							USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
							USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
						}
					}
				}
#endif

		if (bSampleRateAdjust)
		{
			bSampleRateAdjust = FALSE;
			printf("2a %d %d %d %d %d\n", u32CurSR, g_usbd_SampleRate, bSampleRateAdjust, u32CurSBR, g_usbd_SampleBitRate);
			if ( (u32CurSR != g_usbd_SampleRate) || (u32CurSBR != g_usbd_SampleBitRate))
			{
//				printf("2b %d %d %d %d %d\n", u32CurSR, g_usbd_SampleRate, bSampleRateAdjust, u32CurSBR, g_usbd_SampleBitRate);
				u32CurSR = g_usbd_SampleRate;
				u32CurSBR = g_usbd_SampleBitRate;
				printf("2c %d %d %d %d %d\n", u32CurSR, g_usbd_SampleRate, bSampleRateAdjust, u32CurSBR, g_usbd_SampleBitRate);
				/* Init I2S, IP clock and multi-function I/O */
				I2S_Init(g_usbd_SampleRate, g_usbd_SampleBitRate);

				/* FIXME: support stereo and 16-bit/24-bit only for now */
				if ( g_usbd_SampleBitRate == 24 )
				I2S_Open(I2S, I2S_MODE_MASTER, g_usbd_SampleRate, I2S_DATABIT_32, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
				else if ( g_usbd_SampleBitRate == 16 )
				I2S_Open(I2S, I2S_MODE_MASTER, g_usbd_SampleRate, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
				else
					printf( "Error! Bit rate not supported %d\n", g_usbd_SampleBitRate );
				I2S_EnableMCLK(I2S, g_usbd_SampleRate*256);

				I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
				
				/* FIXME: need to ramp up or ramp down to avoid pop-noise */
				InternalCODEC_Setup(g_usbd_PlayVolumeL, g_usbd_PlayVolumeR);
//				printf("2 %d %d %d\n", u32CurSR, g_usbd_SampleRate, bSampleRateAdjust);
				DMA_Init();

			}
		}

#ifdef __HID__				
				HID_UpdateKbData();
#endif							

		}
}

#ifdef __HID__	
void HID_UpdateKbData(void)
{
#if 0
    uint32_t key = 0xF;
    static uint32_t preKey;
#endif
    int32_t n;
		int32_t volatile i;
    n = 8;
    if(g_u8EPCReady)
    {
				if(USBD->EP[EPC].EPDATCNT)
				{
								USBD->EP[EPC].EPRSPCTL = USB_EP_RSPCTL_SHORTTXEN;	
					g_u8EPCReady = 0;
					return;
					
				}
#ifdef __KEYBOARD__			
#if 0
        // PC0_PIN, A
        // PC1_PIN, B
        // PC2_PIN, C
        // PC3_PIN, D
				// PC4_PIN, E
        key = !PC0_PIN | (!PC1_PIN << 1) | (!PC2_PIN << 1) | (!PC3_PIN << 1) | (!PC4_PIN << 1);

        if(key == 0)
        {
            for(i = 0; i < n; i++)
                buf[i] = 0;

            if(key != preKey)
            {
                preKey = key;
                g_u8EPCReady = 0;
                /* Trigger to note key release */
								/* Set transfer length and trigger IN transfer */
					
								while(1) 
								{
										if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
												break;

										if (!USBD_IS_ATTACHED())
												break;
								}
								USBD_SET_DMA_READ(HID_IN_EP_NUM);
								USBD_SET_DMA_ADDR((uint32_t)&buf[0]);
								USBD_SET_DMA_LEN(8);
								USBD_ENABLE_DMA();	
								while(1) 
								{
										if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
												break;

										if (!USBD_IS_ATTACHED())
												break;
								}		
            }				
        }
        else
        {
            preKey = key;   
						if(!PC0_PIN)
                buf[2] = 0x04;	/* Key A */
            else if(!PC1_PIN)
                buf[2] = 0x05;	/* Key B */
            else if(!PC2_PIN)
                buf[2] = 0x06;	/* Key C */	
						else if(!PC3_PIN)
                buf[2] = 0x07;	/* Key D */
            else if(!PC4_PIN)
                buf[2] = 0x08;	/* Key E */			
						else if(!PC5_PIN)
                buf[2] = 0x09;	/* Key F */

            g_u8EPCReady = 0;
						/* Set transfer length and trigger IN transfer */	
						while(1) 
						{
								if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
										break;

								if (!USBD_IS_ATTACHED())
										break;
						}
						USBD_SET_DMA_READ(HID_IN_EP_NUM);
						USBD_SET_DMA_ADDR((uint32_t)&buf[0]);
						USBD_SET_DMA_LEN(8);
						USBD_ENABLE_DMA();	
						while(1) 
						{
								if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
										break;

								if (!USBD_IS_ATTACHED())
										break;
						}			
				}
#else
for(i = 0; i < n; i++)
	buf[i] = 0;
g_u8EPCReady = 0;
Keypad_ScanTgr();			/* GPIO KEY scan */
/* Set transfer length and trigger IN transfer */	
while (1)
{
	if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
		break;

	if (!USBD_IS_ATTACHED())
		break;
}
USBD_SET_DMA_READ(HID_IN_EP_NUM);
USBD_SET_DMA_ADDR((uint32_t)&buf[0]);
USBD_SET_DMA_LEN(8);
USBD_ENABLE_DMA();
while (1)
{
	if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
		break;

	if (!USBD_IS_ATTACHED())
		break;
}
#endif
#elif defined __MEDIAKEY__	
#if 0

        // PC0_PIN, play/pause
        // PC1_PIN, Next
        // PC2_PIN, Previous
        // PC3_PIN, Vol+
        // PC4_PIN, Vol-
        key = !PC0_PIN | (!PC1_PIN << 1) | (!PC2_PIN << 1) | (!PC3_PIN << 1) | (!PC4_PIN << 1);

        if(key == 0)
        {
            for(i = 0; i < n; i++)
                buf[i] = 0;

            if(key != preKey)
            {
                preKey = key;
                g_u8EPCReady = 0;
                /* Trigger to note key release */
								/* Set transfer length and trigger IN transfer */
								while(1) 
								{
										if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
												break;

										if (!USBD_IS_ATTACHED())
												break;
								}
								USBD_SET_DMA_READ(HID_IN_EP_NUM);
								USBD_SET_DMA_ADDR((uint32_t)&buf[0]);
								USBD_SET_DMA_LEN(8);
								USBD_ENABLE_DMA();	
								while(1) 
								{
										if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
												break;

										if (!USBD_IS_ATTACHED())
												break;
								}	
            }				
        }
        else
        {
            // Don't repeat key when it is media key
            if(preKey != key)
            {
                preKey = key;
                buf[0] = 0;
                buf[1] = 0;
                if(!PC0_PIN)
                    buf[1] |= HID_CTRL_PAUSE;
                else if(!PC1_PIN)
                    buf[1] |= HID_CTRL_NEXT;
                else if(!PC2_PIN)
                    buf[1] |= HID_CTRL_PREVIOUS;
                else if(!PC3_PIN)
                    buf[0] |= HID_CTRL_VOLUME_INC;
                else if(!PC4_PIN)
                    buf[0] |= HID_CTRL_VOLUME_DEC;

                g_u8EPCReady = 0;
								/* Set transfer length and trigger IN transfer */
								while(1) 
								{
										if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
												break;

										if (!USBD_IS_ATTACHED())
												break;
								}
								USBD_SET_DMA_READ(HID_IN_EP_NUM);
								USBD_SET_DMA_ADDR((uint32_t)&buf[0]);
								USBD_SET_DMA_LEN(8);
								USBD_ENABLE_DMA();	
								while(1) 
								{
										if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
												break;

										if (!USBD_IS_ATTACHED())
												break;
								}		
            }
        }
#else
for(i = 0; i < n; i++)
    buf[i] = 0;
g_u8EPCReady = 0;
Keypad_ScanTgr();
/* Set transfer length and trigger IN transfer */
while(1) 
{
		if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
				break;

		if (!USBD_IS_ATTACHED())
				break;
}
USBD_SET_DMA_READ(HID_IN_EP_NUM);
USBD_SET_DMA_ADDR((uint32_t)&buf[0]);
USBD_SET_DMA_LEN(8);
USBD_ENABLE_DMA();	
while(1) 
{
		if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
				break;

		if (!USBD_IS_ATTACHED())
				break;
}	

#endif		
#endif				
    }
}
#endif


void SysTick_Handler(void)
{
		
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Internal CODEC Setting                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void demo_LineIn(int8_t i8PlayLVol, int8_t i8PlayRVol)
{
	uint32_t i;

		// Setting Right Line In Channel
		SYS->GPD_MFPL  = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk) ) | SYS_GPD_MFPL_PD4MFP_RLINEIN; /* FIXME: for BSP .003 */
		/* SYS->GPD_MFPL  = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk) ) | SYS_GPD_MFPL_PD4MFP_RIGHT_LINE_IN; FIXME: for BSP .002 */
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
			CLK_SysTickDelay(100000);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
		I2S_SET_INTERNAL_CODEC(I2S, 0x00, 0xD0);	//ADC digital enabled
		CLK_SysTickDelay(100000);	//Delay 100mS

	if (i8PlayLVol <= 0 )
	{
		i8PlayLVol = 31;	// Analog Mute
	}
	else
	{
		i8PlayLVol--;
		if (i8PlayLVol > 30)	i8PlayLVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
		i8PlayLVol = 30 - i8PlayLVol;
	}
	if (i8PlayRVol <= 0 )
	{
		i8PlayRVol = 31;	// Analog Mute
	}
	else
	{
		i8PlayRVol--;
		if (i8PlayRVol > 30)	i8PlayRVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
		i8PlayRVol = 30 - i8PlayRVol;
	}

		I2S_SET_INTERNAL_CODEC(I2S, 0x08, i8PlayLVol);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, i8PlayRVol);	//Un-mute Headphone and set volume
//		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

    /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
}

void demo_MIC0(int8_t i8PlayLVol, int8_t i8PlayRVol)
{
	uint32_t i;

		/* IIC Configure Step without PLL: */
		/* Add MCLK(256*Fs) in. */

		I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);	// Mute headphone of Left channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);	// Mute headphone of Right channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x0F);	//Mute the ADC Left channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x0F);	//Mute the ADC Right channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x0F);	//Mute the ADC Side tone volume

		I2S_SET_INTERNAL_CODEC(I2S, 0x02, 0xC0);	//Set CODEC slave

		I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x80);	//Digital Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0F, 0xC0);	//Enable Analog Part
		I2S_SET_INTERNAL_CODEC(I2S, 0x0E, 0x02);	//ADC input select MIC0

		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF3);	//Analog Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0D, 0x31);	//Biasing enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xE3);
		for (i=0; i < 15; i++)	//Delay 1.5s~2.5s
			CLK_SysTickDelay(100000);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
		I2S_SET_INTERNAL_CODEC(I2S, 0x00, 0xD0);	//ADC digital enabled
		CLK_SysTickDelay(100000);	//Delay 100mS

	if (i8PlayLVol <= 0 )
	{
		i8PlayLVol = 31;	// Analog Mute
	}
	else
	{
		i8PlayLVol--;
		if (i8PlayLVol > 30)	i8PlayLVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
		i8PlayLVol = 30 - i8PlayLVol;
	}
	if (i8PlayRVol <= 0 )
	{
		i8PlayRVol = 31;	// Analog Mute
	}
	else
	{
		i8PlayRVol--;
		if (i8PlayRVol > 30)	i8PlayRVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
		i8PlayRVol = 30 - i8PlayRVol;
	}

		I2S_SET_INTERNAL_CODEC(I2S, 0x08, i8PlayLVol);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, i8PlayRVol);	//Un-mute Headphone and set volume

//		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

   /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
}

void demo_MIC1(int8_t i8PlayLVol, int8_t i8PlayRVol)
{
	uint32_t i;

		SYS->GPD_MFPL  = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD3MFP_Msk) ) | SYS_GPD_MFPL_PD3MFP_MIC1_N;
		SYS->GPD_MFPL  = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD2MFP_Msk) ) | SYS_GPD_MFPL_PD2MFP_MIC1_P;
		SYS_SetSharedPinType(SYS_PORT_D, 2, 0, 0);
		SYS_SetSharedPinType(SYS_PORT_D, 3, 0, 0);

		/* IIC Configure Step without PLL: */
		/* Add MCLK(256*Fs) in. */

		I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);	// Mute headphone of Left channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);	// Mute headphone of Right channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x0F);	//Mute the ADC Left channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x0F);	//Mute the ADC Right channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x0F);	//Mute the ADC Side tone volume

		I2S_SET_INTERNAL_CODEC(I2S, 0x02, 0xC0);	//Set CODEC slave

		I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x80);	//Digital Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0F, 0xC0);	//Enable Analog Part
		I2S_SET_INTERNAL_CODEC(I2S, 0x0E, 0x06);	//ADC input select MIC1

		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF3);	//Analog Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0D, 0x31);	//Biasing enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xE3);
		for (i=0; i < 15; i++)	//Delay 1.5s~2.5s
			CLK_SysTickDelay(100000);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
		I2S_SET_INTERNAL_CODEC(I2S, 0x00, 0xD0);	//ADC digital enabled
		CLK_SysTickDelay(100000);	//Delay 100mS

	if (i8PlayLVol <= 0 )
	{
		i8PlayLVol = 31;	// Analog Mute
	}
	else
	{
		i8PlayLVol--;
		if (i8PlayLVol > 30)	i8PlayLVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
		i8PlayLVol = 30 - i8PlayLVol;
	}
	if (i8PlayRVol <= 0 )
	{
		i8PlayRVol = 31;	// Analog Mute
	}
	else
	{
		i8PlayRVol--;
		if (i8PlayRVol > 30)	i8PlayRVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
		i8PlayRVol = 30 - i8PlayRVol;
	}

		I2S_SET_INTERNAL_CODEC(I2S, 0x08, i8PlayLVol);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, i8PlayRVol);	//Un-mute Headphone and set volume

//		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

  /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
}

void InternalCODEC_Setup(int8_t i8PlayLVol, int8_t i8PlayRVol)
{
		demo_LineIn(i8PlayLVol, i8PlayRVol);
//		demo_MIC0(i8PlayLVol, i8PlayRVol);
///		demo_MIC1(i8PlayLVol, i8PlayRVol);

//    printf("Internal CODEC init [OK]\n");
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
