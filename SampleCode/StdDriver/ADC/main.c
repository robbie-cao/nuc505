/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/24 4:08p $
 * @brief    NUC505 Series Global Control and Clock Control Driver Sample Code
 *
 * 1.	Demonstrate ADC conversion from channel 0.
 * 2.	Demonstrate analog keypad detection from channel 2
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"


volatile uint8_t u8ADF;

void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    // Get ADC conversion finish interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADF_INT);

    if(u32Flag & ADC_ADF_INT)
    {
        u8ADF = 1;
        ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    }
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADF_INT); /* To avoid the the synchronization issue between system and APB clock domain */
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;


    /* Enable IP clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Enable ADC clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select IP clock source */
    /* UART1 clock source = XIN */
    CLK->CLKDIV3 &= ~(CLK_CLKDIV3_UART1DIV_Msk | CLK_CLKDIV3_UART1SEL_Msk);

    /* Set PCLK Divider */
    CLK_SetModuleClock(PCLK_MODULE, 0, 1);		/* PCLK divider = 1 */

#if 0	/* if running code in SRAM, the system clock can up to 100MHz */
    CLK_SetCoreClock(96000000);
#else	/* if running code in SPIM, programmer needs to take care of SPIM clock. Please reference the SPIM_SPIROM example code */
    CLK_SetCoreClock(48000000);
#endif
    /* Update System Core Clock */
    SystemCoreClockUpdate();

}

void UART0_Init(void)
{
    /* Enable UART0 Module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* UART0 module clock from EXT */
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

void ADC_Init(void)
{
    /* Enable ADC Module clock */
    CLK_EnableModuleClock(ADC_MODULE);
    /* ADC module clock from EXT */
    CLK_SetModuleClock(ADC_MODULE, CLK_ADC_SRC_EXT, 4);		/* ADC clock = 12M/(4+1). Conversion rate = ADC clock/16 */
    /* Reset IP */
    SYS_ResetModule(ADC_RST);
    /* Configure UART0 and set UART0 Baudrate */
    ADC_Open(ADC, NULL, NULL, ADC_CH_0_MASK);

    // Enable ADC ADC_IF interrupt
    ADC_EnableInt(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for AIN0 ~ AIN7 */
    SYS->GPA_MFPL  = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk) ) | SYS_GPA_MFPL_PA0MFP_ADC_CH0;
    SYS->GPA_MFPL  = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk) ) | SYS_GPA_MFPL_PA1MFP_ADC_CH1;
}

uint16_t u16ScanCodeTbl[]= {1522/*SW1*/, 1741/*SW2*/, 2442/*SW3*/, 2820/*SW4*/, 3056/*SW5*/, 3220/*SW6*/,		/* Single key*/
                            1012/*SW1+SW2*/, 1216/*SW1+SW3*/, 1303/*SW1+SW4*/, 1352/*SW1+SW5*/, 1385/*SW1+SW5*/	/* Compound key*/
                           };
uint16_t u16MapTbl[] = {1, (1<<1), (1<<2), (1<<3), (1<<4), (1<<5),
                        (1|(1<<1)), (1|(1<<2)), (1|(1<<3)), (1|(1<<4)), (1|(1<<5))
                       };

uint16_t keymap(uint16_t u16ScanCode)
{
    uint16_t u16Max, u16Min;
    uint16_t i;
    if(u16ScanCode<(u16ScanCodeTbl[0]-50))
        u16Max=u16ScanCode+16, u16Min=u16ScanCode-16;	/* compound key */
    else
        u16Max=u16ScanCode+30, u16Min=u16ScanCode-30;	/* single key */
    for(i=0; i<sizeof(u16ScanCodeTbl)/sizeof(u16ScanCodeTbl[0]); i=i+1)
    {
        if((u16ScanCodeTbl[i]>u16Min) &&	(u16ScanCodeTbl[i]<u16Max))
            return u16MapTbl[i];
    }
    return 0;
}


int main(void)
{
    uint16_t u16Data, u16LastKey=0, t=0;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART0_Init();
    ADC_Init();

    printf("+----------------------------------------------------------------+\n");
    printf("|           NUC505 Series ADC Conversion Sample Code             |\n");
    printf("+----------------------------------------------------------------+\n");
    printf("CPU clock\t\t\t\t%dHz\n", SystemCoreClock);

    SYS_SetSharedPinType(SYS_PORT_A, SYS_PIN_0, 0, 0);
    SYS_SetSharedPinType(SYS_PORT_A, SYS_PIN_2, 0, 0);
    for(t=0; t<10; t=t+1)
    {
        u8ADF = 0;
        ADC_EnableHWTrigger(ADC, ADC_CH_0_MASK, 0x10);
        while(u8ADF==0);
        u16Data = ADC_GET_CONVERSION_DATA(ADC, NULL);
        printf("ADC channel 0 Conversion Data = %d\n", u16Data);
    }


    printf("+----------------------------------------------------------------+\n");
    printf("|           NUC505 Series Analog Keypad Sample Code              |\n");
    printf("+----------------------------------------------------------------+\n");
    printf("+   Keypad needs to use channel 2 for pressing detection			   +\n");
    printf("+   Please adjust JP1 to disconnect ADC1 and connect ADC2		     +\n");

    ADC_DisableInt(ADC, ADC_KEY_INT);
    do
    {
        while( ADC_GET_INT_FLAG(ADC, ADC_KEY_INT)!=0 )
        {
            ADC_CLR_INT_FLAG(ADC, ADC_KEY_INT);
            u8ADF = 0;
            ADC_EnableHWTrigger(ADC, ADC_CH_2_MASK, 0x20);
            while(u8ADF==0);
            ADC_CLR_INT_FLAG(ADC, ADC_KEY_INT);
            if(ADC_GET_INT_FLAG(ADC, ADC_KEY_INT)!=0) /* Double Confirm Keypad Still Pressing */
            {
                u16Data = ADC_GET_CONVERSION_DATA(ADC, NULL);
                u16Data = keymap(u16Data);
                if((u16LastKey != u16Data) && (u16Data!=0)) /* Debouncing */
                {
                    u16LastKey = u16Data;
                    continue;
                }
                if(u16Data!=0)
                    printf("Key code = %d\n", u16Data);
            }
            else
            {
                u16LastKey = 0;
            }
            if(u16Data==33)
                break;
        }

    }
    while(u16Data!=33);  /* SW1+SW6*/
    printf("Demo done\n");
    while(1);
}




