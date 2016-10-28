/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 14/06/26 7:49p $
 * @brief    Demonstrate how to call ARM CMSIS DSP library to calculate FFT.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"
#include "arm_math.h"


#define TEST_LENGTH_SAMPLES 2048

/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES / 2];

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Reference index at which max energy of bin occurs */
uint32_t refIndex = 213, testIndex = 0;


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK_SetCoreClock(96000000);
    /* Update System Core Clock */
    SystemCoreClockUpdate();

		// Set APB clock as 1/2 HCLK
		CLK_SetModuleClock(PCLK_MODULE, NULL, 1);
	
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

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


int main()
{
    arm_cfft_radix4_instance_f32 S;
    float32_t maxValue;

    SYS_Init();
    UART0_Init();

    /*
        This sample code is used to show how to use ARM Cortex-M4 DSP library to calculate FFT.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|        NUC505 DSP FFT Sample Code      |\n");
    printf("+----------------------------------------+\n");

    /* Initialize the CFFT/CIFFT module */
    arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse);

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_radix4_f32(&S, testInput_f32_10khz);

    /* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
    arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);

    /* Calculates maxValue and returns corresponding BIN value */
    arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);

    if(testIndex !=  refIndex)
    {
        printf("ERROR: FFT calculation result fail!\n");
    }
    else
    {
        printf("FFT calculation test ok!\n");
    }

    while(SYS->PDID);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
