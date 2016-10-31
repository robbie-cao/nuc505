/**************************************************************************//**
 * @file        ovlymgr.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       Overlay utilities
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "ovlymgr.h"

//#define LOAD_OVERLAY_BY_SPIM_DMA_READ

#ifdef LOAD_OVERLAY_BY_SPIM_DMA_READ

#include "NUC505Series.h"

/* Get unmapped SRAM address, which is required in SPIM DMM/DMA mode. */
#define _SRAM_UNMAP_ADDR(x)       (((uint32_t) (x)) & 0x000FFFFF | 0x20000000)

/*
 * Wait for SPIM DMM to become idle.
 */

void TIMER0_Delay(TIMER_T *timer, uint32_t u32Usec)
{
    uint32_t delay = 3;

    // Clear current timer configuration
    timer->CTL = 0;
    timer->EXTCTL = 0;

    timer->CMP = (uint32_t)u32Usec * 12;
    timer->CTL = TIMER_CTL_CNTEN_Msk; // one shot mode

    // When system clock is faster than timer clock, it is possible timer active bit cannot set in time while we check it.
    // And the while loop below return immediately, so put a tiny delay here allowing timer start counting and raise active flag.
    for(; delay > 0; delay--) {
        __NOP();
    }

    //    while(timer->CTL & TIMER_CTL_ACTSTS_Msk);

}
static void WaitSPIMDMMIdle_Begin(void)
{
    /* SPIM H/W may still be in operation due to in DMM mode, delay at least 250 peripheral cycles (SPI bus cycles). */
    uint32_t u32Divider = ((SPIM->CTL1 & SPIM_CTL1_DIVIDER_Msk) >> SPIM_CTL1_DIVIDER_Pos);
    uint32_t u32SPIBusClock = u32Divider ? SystemCoreClock / (u32Divider * 2) : SystemCoreClock;
    uint32_t u32Delay = 250 * 1000000 / u32SPIBusClock;

    //    SysTick->LOAD = u32Delay * CyclesPerUs;
    //    SysTick->VAL  =  (0x00);
    //    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    TIMER0_Delay(TIMER0,u32Delay);
}

/*
 * Wait for SPIM DMM to become idle.
 */
static void WaitSPIMDMMIdle_End(void)
{
    //    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);   // Wait for down-count to zero.
    while(TIMER0->CTL & TIMER_CTL_ACTSTS_Msk);
}
#endif  // #ifdef LOAD_OVERLAY_BY_SPIM_DMA_READ

void load_overlay(struct ovly *ovly_new)
{
    if (! ovly_new ||
            ovly_new == ovly_new->ovly_reg_containing->ovly_loaded) {
        return;
    }

    printf("Loading overlay: EXEC=0x%08x, LOAD=0x%08x, LENGTH=%d ...\n", ovly_new->exec_ro_base, ovly_new->load_ro_base, (unsigned) ovly_new->ro_length);

#ifdef LOAD_OVERLAY_BY_SPIM_DMA_READ
    WaitSPIMDMMIdle_Begin();    // NOTE: Don't call out to routine located in SPI Flash until WaitSPIMDMMIdle_End() get called.
#endif  // #ifdef LOAD_OVERLAY_BY_SPIM_DMA_READ

#ifdef LOAD_OVERLAY_BY_SPIM_DMA_READ
    {
        uint32_t u32ReadCmdCode = SPIM->CTL0 & SPIM_CTL0_CMDCODE_Msk;
        int is4ByteAddr = (SPIM->CTL0 & SPIM_CTL0_B4ADDREN_Msk) ? 1 : 0;

        WaitSPIMDMMIdle_End();

        SPIM_ENABLE_DMA_MODE(SPIM, 0, u32ReadCmdCode, is4ByteAddr);             // Switch to DMA Read mode.
        SPIM->SRAMADDR = (uint32_t) _SRAM_UNMAP_ADDR(ovly_new->exec_ro_base);   // SRAM address.
        SPIM->DMATBCNT = (uint32_t) _SRAM_UNMAP_ADDR(ovly_new->ro_length);      // Transfer length.
        SPIM->FADDR = (uint32_t) ovly_new->load_ro_base;                        // Flash address.
        SPIM_TRIGGER(SPIM);                                                     // Go.
        while (SPIM_IS_BUSY(SPIM));                                             // Wait for ready.

        SPIM_ENABLE_DMM_MODE(SPIM, u32ReadCmdCode, is4ByteAddr);                // Switch back to DMM mode.
    }
#else
    memcpy(ovly_new->exec_ro_base, ovly_new->load_ro_base, (unsigned) ovly_new->ro_length);
#endif  // #ifdef LOAD_OVERLAY_BY_SPIM_DMA_READ

    /* Flush cache here. Could skip because ARM Cortex-M doesn't support cache. */
    // FlushCache();

    ovly_new->ovly_reg_containing->ovly_loaded = ovly_new;
    printf("Loading overlay: EXEC=0x%08x, LOAD=0x%08x, LENGTH=%d DONE\n", ovly_new->exec_ro_base, ovly_new->load_ro_base, (unsigned) ovly_new->ro_length);

}
