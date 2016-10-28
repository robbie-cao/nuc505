/**************************************************************************//**
 * @file     sysHclkSwitch.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/24 4:08p $
 * @brief    NUC505 Series Global Control and Clock Control Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"

uint32_t s_HCLKArray[]=
{
    1000000,
    2000000,
    3000000,
    4000000,
    6000000,
    12000000,
    48000000,
    64000000,
    72000000,
    84000000,
    96000000,
    120000000   /* Violation of max. HCLK rule (Max. HCLK¡@= 100MHz) */
};

void demo_SysHclkSwitch(void)
{
    uint32_t j, t;
    t = sizeof(s_HCLKArray)/sizeof(s_HCLKArray[0]);

    /* Set PCLK divider to 1 (/2) */
    CLK_SetModuleClock(PCLK_MODULE,
                       NULL, 	/* PCLK source is always from HCLK */
                       1);			/* PCLK divider */
    for(j=0; j<t; j=j+1)
    {
        printf("===============================================\n");
        printf("Want to set HCLK = %d\n",s_HCLKArray[j]);
        printf("HCLK value = %d\n",CLK_SetCoreClock(s_HCLKArray[j]));
        printf("PCLK = %d\n", CLK_GetPCLKFreq());
        printf("PLL = %d\n", CLK_GetPLLClockFreq());

    }
}
