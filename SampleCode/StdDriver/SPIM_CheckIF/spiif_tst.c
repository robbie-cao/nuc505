/**************************************************************************//**
 * @file        spiif_tst.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       Do SPIM I/O mode test.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NUC505Series.h"
#include "spiflash_drv.h"

#define TEST_IP         "SPIM"
#define TEST_ITEM       "SPI I/F Test"

#define SPI_BUS_CLK     60000000

struct SPIIF {
    uint32_t        id;
    const char *    name;
};

void TestSPIIF(void)
{   
    /* Actual SPI bus clock */
    printf("[%s][%s]SPI bus clock\t%dHz\n", TEST_IP, TEST_ITEM, SPIM_GetBusClock(SPIM));
    
    do {
        struct SPIIF spiif_arr[] = {
            {SPIM_CTL1_IFSEL_MCP, "MCP"},
            {SPIM_CTL1_IFSEL_MCP64, "MCP64"},
            {SPIM_CTL1_IFSEL_GPIO, "GPIO"},
        };
        struct SPIIF *spiif_ind = spiif_arr;
        struct SPIIF *spiif_end = spiif_arr + sizeof (spiif_arr) / sizeof (spiif_arr[0]);
        
        printf("[%s][%s]Check which SPI I/F SPI Flash is attached to\n", TEST_IP, TEST_ITEM);
        for (; spiif_ind != spiif_end; spiif_ind ++) {
            SPIM_Open(SPIM, 0, SPI_BUS_CLK);
            SPIM->CTL1 = (SPIM->CTL1 & ~SPIM_CTL1_IFSEL_Msk) | spiif_ind->id;   // Change SPI I/F.
            
            if (spiif_ind->id == SPIM_CTL1_IFSEL_GPIO) {
                /* Configure multi-function pins for SPIM, Slave I/F=GPIO. */
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk | (1 << SYS_GPA_MFPH_PA8MFP_Pos);    // SPIM_SS
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA9MFP_Msk | (1 << SYS_GPA_MFPH_PA9MFP_Pos);    // SPIM_SCLK
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA10MFP_Msk | (1 << SYS_GPA_MFPH_PA10MFP_Pos);  // SPIM_D0
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA11MFP_Msk | (1 << SYS_GPA_MFPH_PA11MFP_Pos);  // SPIM_D1
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA12MFP_Msk | (1 << SYS_GPA_MFPH_PA12MFP_Pos);  // SPIM_D2
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA13MFP_Msk | (1 << SYS_GPA_MFPH_PA13MFP_Pos);  // SPIM_D3
            }
            
            {
                /* Check if SPI Flash is attached by reading its JEDEC ID. */
                uint32_t u32JedecID = SPIFlash_ReadJedecID();
                printf("[%s][%s]I/F %s (JEDEC ID=0x%08x)\t%s\n", TEST_IP, TEST_ITEM, spiif_ind->name, 
                    u32JedecID, (u32JedecID == 0 || u32JedecID == 0xFFFFFF) ? "NONE" : "READY");
            }
            
            if (spiif_ind->id == SPIM_CTL1_IFSEL_GPIO) {
                /* Configure multi-function pins for GPIO. */
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk | (0 << SYS_GPA_MFPH_PA8MFP_Pos);    // GPIOA[8]
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA9MFP_Msk | (0 << SYS_GPA_MFPH_PA9MFP_Pos);    // GPIOA[9]
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA10MFP_Msk | (0 << SYS_GPA_MFPH_PA10MFP_Pos);  // GPIOA[10]
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA11MFP_Msk | (0 << SYS_GPA_MFPH_PA11MFP_Pos);  // GPIOA[11]
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA12MFP_Msk | (0 << SYS_GPA_MFPH_PA12MFP_Pos);  // GPIOA[12]
                SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA13MFP_Msk | (0 << SYS_GPA_MFPH_PA13MFP_Pos);  // GPIOA[13]
            }
            
            SPIM_Close(SPIM);
        }
    }
    while (0);
}   






