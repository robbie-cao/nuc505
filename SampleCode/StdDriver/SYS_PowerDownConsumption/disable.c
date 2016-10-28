/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 14/05/29 1:14p $
 * @brief    NUC505 General Purpose I/O Driver Sample Code
 *           Connect PB.10 to VSS will wake up system form Power-down mode
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"
#include "adc.h"
#include "gpio.h"
#include "i2s.h"
#include "usbh_core.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}
void USB_Device_Phy_Disable(void)
{
    CLK_SetModuleClock(USBD_MODULE, CLK_USBD_SRC_EXT, 0);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_SysTickDelay(1000);
    USBD_DISABLE_PHY();
    USBD_DISABLE_USB();
    CLK_DisableModuleClock(USBD_MODULE);
}

void USB_Host_Disable(void)
{
    /* USB Host transceiver standby  */
    CLK_SetModuleClock(USBH_MODULE, CLK_USBH_SRC_EXT, 1);	/* Only for access USBH register */
    CLK_EnableModuleClock(USBH_MODULE);
    CLK_SysTickDelay(10);
    USBH_Suspend();
    CLK_DisableModuleClock(USBH_MODULE);
}

void ADC_Disable(void)
{
    ADC_T* adc=0;
    CLK_SetModuleClock(ADC_MODULE, CLK_ADC_SRC_EXT, 1);
    CLK_EnableModuleClock(ADC_MODULE);
    CLK_SysTickDelay(10);
    ADC_Close(adc);
    CLK_SysTickDelay(10);
    //CLK_DisableModuleClock(ADC_MODULE);
}

void I2S_Disable(void)
{
    CLK->APBCLK = CLK->APBCLK | 0x4000;			/* Enable I2S clock */
    CLK_EnableModuleClock(I2S_MODULE);
    CLK_SysTickDelay(10);
    I2S_Open(I2S, I2S_MODE_MASTER, 48000, I2S_DATABIT_32, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xFF);			// DAC Power Off
    I2S_SET_INTERNAL_CODEC(I2S, 0x0F, 0xFF);			// ADC Power Off

    //I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);			// DAC Power On
    //I2S_SET_INTERNAL_CODEC(I2S, 0x0F, 0xE0);			// ADC Power On
    CLK_SysTickDelay(10);
    //CLK_DisableModuleClock(I2S_MODULE);
}

#define PA_ALL_MASK	(BIT15|BIT14|BIT13|BIT12|BIT11|BIT10|BIT9|BIT8|BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)
#define PB_ALL_MASK	(BIT15|BIT14|BIT13|BIT12|BIT11|BIT10|BIT9|BIT8|BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)
#define PC_ALL_MASK	(BIT14|BIT13|BIT12|BIT11|BIT10|BIT9|BIT8|BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)
#define PD_ALL_MASK	(BIT4|BIT3|BIT2|BIT1|BIT0)

void GPIO_Path(void)
{
    uint32_t pin;
    uint32_t idx;

    GPIO_SetMode(PA, PA_ALL_MASK, GPIO_MODE_INPUT);
    GPIO_SetPullMode(PA, PA_ALL_MASK, GPIO_PULL_UP_EN);
    GPIO_SET_OUT_DATA(PA, 0xFFFF);
    pin = GPIO_GET_IN_DATA(PA);
    for(idx=0x1; idx<0x10000; idx=idx<<1)
    {
        if((pin&idx)==0) //pin is 0	==>
        {
            uint32_t u32Pin;
            GPIO_SetMode(PA, idx, GPIO_MODE_OUTPUT);//Set to output mode
            u32Pin = GPIO_GET_IN_DATA(PA);
            GPIO_SET_OUT_DATA(PA, u32Pin&(~idx));
            GPIO_SetPullMode(PA, idx, GPIO_PULL_DISABLE);	//Disable pull enabl
        }
    }

    GPIO_SetMode(PB, PB_ALL_MASK, GPIO_MODE_INPUT);
    GPIO_SetPullMode(PB, PB_ALL_MASK, GPIO_PULL_UP_EN);
    GPIO_SET_OUT_DATA(PB, 0xFFFF);
    pin = GPIO_GET_IN_DATA(PB);
    for(idx=0x1; idx<0x10000; idx=idx<<1)
    {
        if((pin&idx)==0) //pin is 0	==>
        {
            uint32_t u32Pin;
            GPIO_SetMode(PB, idx, GPIO_MODE_OUTPUT);//Set to output mode
            u32Pin = GPIO_GET_IN_DATA(PB);
            GPIO_SET_OUT_DATA(PB, u32Pin&(~idx));
            GPIO_SetPullMode(PB, idx, GPIO_PULL_DISABLE);	//Disable pull enabl
        }
    }



    GPIO_SetMode(PC, PC_ALL_MASK, GPIO_MODE_INPUT);
    GPIO_SetPullMode(PC, PC_ALL_MASK, GPIO_PULL_UP_EN);
    GPIO_SET_OUT_DATA(PC, 0x7FFF);


    pin = GPIO_GET_IN_DATA(PC);
    for(idx=0x1; idx<0x8000; idx=idx<<1)
    {
        if((pin&idx)==0) //pin is 0	==>
        {
            uint32_t u32Pin;
            GPIO_SetMode(PC, idx, GPIO_MODE_OUTPUT);//Set to output mode
            u32Pin = GPIO_GET_IN_DATA(PC);
            GPIO_SET_OUT_DATA(PC, u32Pin&(~idx));
            GPIO_SetPullMode(PC, idx, GPIO_PULL_DISABLE);	//Disable pull enabl
        }
    }
    GPIO_SetMode(PD, PD_ALL_MASK, GPIO_MODE_INPUT);
    GPIO_SetPullMode(PD, PD_ALL_MASK, GPIO_PULL_UP_EN);
    GPIO_SET_OUT_DATA(PD, 0x1F);
    pin = GPIO_GET_IN_DATA(PD);
    for(idx=0x1; idx<0x20; idx=idx<<1)
    {
        if((pin&idx)==0) //pin is 0	==>
        {
            uint32_t u32Pin;
            GPIO_SetMode(PD, idx, GPIO_MODE_OUTPUT);//Set to output mode
            u32Pin = GPIO_GET_IN_DATA(PD);
            GPIO_SET_OUT_DATA(PD, u32Pin&(~idx));
            GPIO_SetPullMode(PD, idx, GPIO_PULL_DISABLE);	//Disable pull enabl
        }
    }
}

void LVD_Disable(void)
{
    //SYS->LVDCTL &= (~0x4);
    SYS_DISABLE_LVR();
    SYS_DISABLE_POR();
}
/*---------------------------------------------------------------------------------------------------------*/
/* Power Conspumtion                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void DisableIPs(void)
{
    volatile uint32_t i;

    /* Power consumption measure */
    CLK_SET_AHBCLK(0xFFFFFFFF);
    CLK_SET_APBCLK(0xFFFFFFFF);

    /* USB Device phy suspend */
    USB_Device_Phy_Disable();

    /* USB Host Transceiver */
    USB_Host_Disable();

    /* ADC */
    ADC_Disable();

    /* External DAC */
    I2S_Disable();

    /* LVD */
    LVD_Disable();


    /* GPIO */
    GPIO_Path();

    /* Core clock from external clock */
    CLK_SetHCLK(CLK_HCLK_SRC_EXT, 0);
    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* APLL power down */
    CLK_APLL_ENABLE_POWERDOWN();
    /* PLL power down */
    //CLK_PLL_ENABLE_POWERDOWN();

    /* Internal embedded SPI Flash MISO/MOSI pins pull enable */
    SYS_SET_EMBEDDED_SPIFLASH_PULL(0x12);

    /* Pre-scalar counter from 0 ~ 0xFFFF */
    CLK_SET_WAKEUP_PRESCALAR	(0x1000);

    /* Wake up time is about 45ms */
    CLK_ENABLE_WAKEUP_PRESCALAR();
    SPIM_ENABLE_IO_MODE(SPIM, SPIM_CTL0_BITMODE_STAN, 0);
    /* Enter to Power-down mode */
    PowerDownFunction();
    SPIM_ENABLE_DMM_MODE(SPIM, SPIM_CTL0_CMDCODE_FAST_READ, 0);

    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

    printf("System waken-up done.\n\n");
    /* Wake up */

    CLK_SetCoreClock(96000000);
    /* Update System Core Clock */
    SystemCoreClockUpdate();
}
