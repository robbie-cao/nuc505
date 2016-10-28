/**************************************************************************//**
 * @file        usrprog_ovly_tab.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       User program overlay table
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "usrprog_ovly_tab.h"

/* Define overlay table, which must be consistent with your linker script file. */

/* FIXME: Define your overlays here. */
#if defined ( __CC_ARM )
DEFINE_OVERLAY(ER_OVERLAY_A, ovly_A, ovly_reg_1)
DEFINE_OVERLAY(ER_OVERLAY_B, ovly_B, ovly_reg_1)

#elif defined (__ICCARM__)
#pragma section = "overlay_a"
#pragma section = "overlay_a_init"
#pragma section = "overlay_b"
#pragma section = "overlay_b_init"
DEFINE_OVERLAY(overlay_a, ovly_A, ovly_reg_1)
DEFINE_OVERLAY(overlay_b, ovly_B, ovly_reg_1)
#endif

/* FIXME: Define your overlay regions here. */
DEFINE_2OVERLAY_REGION(ovly_reg_1, ovly_A, ovly_B)
