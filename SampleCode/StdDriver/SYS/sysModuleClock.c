
/**************************************************************************//**
 * @file     sysModuleClock.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/24 4:08p $ $
 * @brief    NUC505 Series Global Control and Clock Control Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"



typedef struct tagModule
{
    uint32_t u32ModuleName;
    uint32_t u32SrcClk;
} S_MODCLK;

S_MODCLK s_ClkArray[]=
{

    /* Don't not set following three clock divider if run code between SPIM or SRAM */
//	SRAM23_MODULE, 0,
//	ROM_MODULE,	0,				/* 0 means no divider and select field in clock controller. */
//	SPIM_MODULE,	0,			/* 0 means no divider and select field in clock controller. */

    SDH_MODULE, CLK_SDH_SRC_PLL,
    USBD_MODULE, CLK_USBD_SRC_PLL,
    USBH_MODULE, CLK_USBH_SRC_PLL,
    TMR0_MODULE, CLK_TMR0_SRC_EXT, //CLK_TMR0_SRC_EXT
    TMR1_MODULE, CLK_TMR1_SRC_EXT,
    TMR2_MODULE, CLK_TMR2_SRC_EXT,
    TMR3_MODULE, CLK_TMR3_SRC_EXT,
    WDT_MODULE,	CLK_WDT_SRC_EXT,
    PWM_MODULE, CLK_PWM_SRC_PLL,
    I2C0_MODULE,	0,			/* 0 means no divider and select field in clock controller. */
    I2C1_MODULE,	0,			/* 0 means no divider and select field in clock controller. */
    RTC_MODULE,	0,			/* 0 means no divider and select field in clock controller. */

    SPI0_MODULE,		CLK_SPI0_SRC_PLL,		/* Only clock src */
    SPI1_MODULE,		CLK_SPI1_SRC_PLL,		/* Only clock src */

    /* Don't not set following UART0 clock divider if print message from UART0 */
    //UART0_MODULE,		CLK_UART0_SRC_PLL,

    UART1_MODULE,		CLK_UART1_SRC_PLL,
    UART2_MODULE,		CLK_UART2_SRC_PLL,

    I2S_MODULE,		CLK_I2S_SRC_APLL,
    ADC_MODULE,		CLK_ADC_SRC_PLL,

};


void demo_ModuleClock(void)
{
    uint32_t j, i, t;

    printf("Set Module Clock Divider\n");
    t = sizeof(s_ClkArray)/sizeof(s_ClkArray[0]);
    for(j=0; j<t; j=j+1)
    {
        uint32_t DivMsk= MODULE_CLKDIV_Msk(s_ClkArray[j].u32ModuleName);
        for(i=0; i<=DivMsk; i=i+1)
        {
            CLK_EnableModuleClock(s_ClkArray[j].u32ModuleName);
            CLK_SetModuleClock(s_ClkArray[j].u32ModuleName, s_ClkArray[j].u32SrcClk, i);
            CLK_DisableModuleClock(s_ClkArray[j].u32ModuleName);
        }
        printf("Set Module Clock Divider Item = %d\n", j);
    }
}
