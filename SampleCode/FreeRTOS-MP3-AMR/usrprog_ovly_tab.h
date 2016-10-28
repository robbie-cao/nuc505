/**************************************************************************//**
 * @file        usrprog_ovly_tab.h
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       User program overlay table
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "ovlymgr.h"
 
#ifdef __cplusplus
extern "C" {
#endif

#ifndef __USRPROG_OVLY_TAB_H__
#define __USRPROG_OVLY_TAB_H__

/* FIXME: Declare C-struct overlays which are consistent with your overlay table. */
extern struct ovly *ovly_A;
extern struct ovly *ovly_B;

/* FIXME: Declare C-struct overlay regions which must be consistent with your overlay table. */
extern struct ovly_reg *ovly_reg_1;

#endif /* #ifndef __USRPROG_OVLY_TAB_H__ */

#ifdef __cplusplus
}
#endif
