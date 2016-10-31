/**************************************************************************//**
 * @file        ovlymgr.h
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       Overlay utilities
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __OVLY_UTIL_H__
#define __OVLY_UTIL_H__

struct ovly;
struct ovly_reg;

/* Structure to hold overlay information */
struct ovly
{
    struct ovly_reg *   ovly_reg_containing;
    void *              load_ro_base;
	void *              exec_ro_base;
#if defined ( __CC_ARM )
    void *              ro_length;  // Declare as pointer type instead of integer type to match type of linker-generated symbol.
                                    // Its actual meaning is still integer.
#elif defined (__ICCARM__)
    unsigned            ro_length;
#endif
};

/* Structure to hold overlay region information */
struct ovly_reg
{
    struct ovly *       ovlys_contained[8];
    struct ovly *       ovly_loaded;
};

#if defined ( __CC_ARM )
/**
  * @brief                  Define C-struct overlay which is consistent with linker script file.
  * @param[in]  OVERLAY     Overlay execution region defined in linker file.
  * @param[in]  OVLY        C-struct overlay to be defined.
  * @param[in]  OVLY_REG    C-struct overlay region recording which overlays are located in it. OVLY is one of them.
  * @return                 None.
  * \hideinitializer
  */
#define DEFINE_OVERLAY(ER_OVERLAY, OVLY, OVLY_REG) \
static struct ovly _##OVLY; \
static struct ovly_reg _##OVLY_REG; \
extern unsigned Load$$##ER_OVERLAY##$$Base[]; \
extern unsigned Image$$##ER_OVERLAY##$$Base[]; \
extern unsigned Image$$##ER_OVERLAY##$$Length[]; \
struct ovly _##OVLY = { \
    &_##OVLY_REG, \
    Load$$##ER_OVERLAY##$$Base, \
    Image$$##ER_OVERLAY##$$Base, \
    Image$$##ER_OVERLAY##$$Length \
}; \
struct ovly *OVLY = &_##OVLY;

#elif defined (__ICCARM__)
/**
  * @brief                      Define C-struct overlay which is consistent with linker script file.
  * @param[in]  OVERLAY         Overlay section.
  * @param[in]  OVLY            C-struct overlay to be defined.
  * @param[in]  OVLY_REG        C-struct overlay region recording which overlays are located in it. OVLY is one of them.
  * @return                     None.
  * \hideinitializer
  */
#define DEFINE_OVERLAY(OVERLAY, OVLY, OVLY_REG) \
static struct ovly _##OVLY; \
static struct ovly_reg _##OVLY_REG; \
struct ovly _##OVLY = { \
    &_##OVLY_REG, \
    __section_begin(#OVERLAY "_init"), \
    __section_begin(#OVERLAY), \
    __section_size(#OVERLAY "_init") \
}; \
struct ovly *OVLY = &_##OVLY;

#endif

/**
  * @brief                  Define C-struct overlay region which is consistent with linker script file.
  * @param[in]  OVLY_REG    C-struct overlay region in which there is 1 overlay located.
  * @param[in]  OVLY_1      C-struct overlay which is located in OVLY_REG.
  * @return                 None.
  * \hideinitializer
  */
#define DEFINE_1OVERLAY_REGION(OVLY_REG, OVLY_1) \
struct ovly_reg _##OVLY_REG = { \
    {&_##OVLY_1, NULL}, \
    NULL \
}; \
struct ovly_reg *OVLY_REG = &_##OVLY_REG;

/**
  * @brief                  Define C-struct overlay region which is consistent with linker script file.
  * @param[in]  OVLY_REG    C-struct overlay region in which there are 2 overlays located.
  * @param[in]  OVLY_1      C-struct overlay which is located in OVLY_REG.
  * @param[in]  OVLY_2      C-struct overlay which is located in OVLY_REG.
  * @return                 None.
  * \hideinitializer
  */
#define DEFINE_2OVERLAY_REGION(OVLY_REG, OVLY_1, OVLY_2) \
struct ovly_reg _##OVLY_REG = { \
    {&_##OVLY_1, &_##OVLY_2, NULL}, \
    NULL \
}; \
struct ovly_reg *OVLY_REG = &_##OVLY_REG;

/**
  * @brief                  Define C-struct overlay region which is consistent with linker script file.
  * @param[in]  OVLY_REG    C-struct overlay region in which there are 3 overlays located.
  * @param[in]  OVLY_1      C-struct overlay which is located in OVLY_REG.
  * @param[in]  OVLY_2      C-struct overlay which is located in OVLY_REG.
  * @param[in]  OVLY_3      C-struct overlay which is located in OVLY_REG.
  * @return                 None.
  * \hideinitializer
  */
#define DEFINE_3OVERLAY_REGION(OVLY_REG, OVLY_1, OVLY_2, OVLY_3) \
struct ovly_reg _##OVLY_REG = { \
    {&_##OVLY_1, &_##OVLY_2, &_##OVLY_3, NULL}, \
    NULL \
}; \
struct ovly_reg *OVLY_REG = &_##OVLY_REG;

void load_overlay(struct ovly *ovly_new);

#endif /* #ifndef __OVLY_UTIL_H__ */

#ifdef __cplusplus
}
#endif
