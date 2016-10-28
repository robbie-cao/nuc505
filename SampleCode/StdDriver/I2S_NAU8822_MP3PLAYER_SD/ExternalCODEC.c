/**************************************************************************//**
 * @file     ExternalCODEC.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/05/29 1:14p $
 * @brief    NUC505 I2S Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

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

void demo_stereo_LineIn(void)
{
		printf("  NOTE: Need head-phone for external CODEC.\n");

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

/*---------------------------------------------------------------------------------------------------------*/
/*  External CODEC Settings                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void ExternalCODEC_Setup(void)
{
	printf("\nConfigure External CODEC ...");
	
	demo_stereo_LineIn();

	printf("[OK]\n");
}
