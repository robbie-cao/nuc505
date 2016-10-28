/**************************************************************************//**
 * @file     Keypad.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/01/28 10:19a $
 * @brief    Keypad (Direct Key) via GPIO interface to control.
 *
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NUC505Series.h"
#include "gpio.h"
#include "Keypad.h"

typedef struct {
    void*               pInGPIO;            // Debounced input GPIO port
    void*               pOutGPIO;           // Debounced output GPIO port
    S_KEYPAD_KEY_HANDLER* psKeyHandler;        // event handler of key matrix
    uint16_t*             pu16PinValueArray;   // Pin value buffer that is provided by application(Buffer size is output pin number x 2 bytes)
    uint16_t              u16InPinMaskA;       // Input pin mask of GPIOA
    uint16_t              u16InPinMaskB;       // Input pin mask of GPIOB
    uint16_t              u16OutPinMaskA;      // Output pin mask of GPIOA
    uint16_t              u16OutPinMaskB;      // Output pin mask of GPIOB
    uint16_t              u16OutPinDebMask;    // Pin mask of debunded output pin
    uint16_t              u16PinValueDiff;     // The changed pin value
    uint8_t               u8KeyMode;           // 0: Falling, 1: Rising, 2: Pressing
    uint8_t               u8KeyPinValueIdx;    // Indx of of pin value buffer
} S_KEYPAD_KEY_CTRL_HANDLE;

typedef struct {
    void*               pGPIO;              // Debounced GPIO port
    S_KEYPAD_TGR_HANDLER* psTgrHandler;        // Event handler of direct trigger
    uint16_t              u16PinMaskA;         // Pin mask of GPIOA
    uint16_t              u16PinMaskB;         // Pin mask of GPIOB
    uint16_t              u16PinMaskC;         // Pin mask of GPIOC
    uint16_t              u16PinValueA;        // Previous pin value of GPIOA to compare the changed pin
    uint16_t              u16PinValueB;        // Previous pin value of GPIOB to compare the changed pin
    uint16_t              u16PinValueC;        // Previous pin value of GPIOC to compare the changed pin
    uint16_t              u16PinValueDiff;     // The changed pin value
    uint8_t               u8TgrMode;           // 0: Falling, 1: Rising, 2: Pressing
} S_KEYPAD_TGR_CTRL_HANDLE;

static S_KEYPAD_KEY_CTRL_HANDLE s_sKeypadKeyCtrlHandler;
static S_KEYPAD_TGR_CTRL_HANDLE s_sKeypadTgrCtrlHandler;

// Counter of Debounce time
static volatile uint16_t        s_u16TgrDebounceCounter;
// Counter of Debounce time
static volatile uint16_t        s_u16KeyDebounceCounter;

// Debounce time count to be reloaded.
uint16_t g_u16DebounceCount;
// Pressing events interval counter to e reloaded
uint16_t g_u16PressingCount;

/**
  * @brief  1. Initial keypad.
  *         2. Set Debounce and pressing events interval count.
  *         3. Time unit is decided by application.
  *         4. Application should initiated a timer to call ScanKeypad() periodically.
  * @param  u16DebounceCount is the debounce count for change pin status.
  * @param  u16PressingCount is the interval count between 2 pressing events.
  * @return None
  */
void Keypad_InitKeypad( uint16_t u16DebounceCount, uint16_t u16PressingCount )
{
    // Config debounce variable.
    GPIO_SET_DEBOUNCE_TIME(NULL, GPIO_DBCTL_DBCLKSEL_128);

    g_u16DebounceCount = u16DebounceCount;
    g_u16PressingCount = u16PressingCount;
}

/**
  * @brief  1. Initial key matix pins of input and output ports.
  *         2. Internal function.
  * @param  pInGPIO is the address of GPIO to be input port of key matrix.
  * @param  u16InKeyPinMask is the mask of input pins of key matrix.
  * @param  pOutGPIO is the address of GPIO to be output port of key matrix.
  * @param  u16OutKeyPinMask is the mask of output pins of key matrix.
  */
void Keypad_InitKeyMatrixPort(void* pInGPIO,uint16_t u16InKeyPinMask,void* pOutGPIO,uint16_t u16OutKeyPinMask)
{
    uint16_t u16Mask = 1;
    GPIO_PA_T *pIn = (GPIO_PA_T *)pInGPIO;
    GPIO_PA_T *pOut = (GPIO_PA_T *)pOutGPIO;

    if (u16InKeyPinMask && u16OutKeyPinMask) {
        while( u16Mask > 0 ) {
            if (u16OutKeyPinMask & u16Mask) {
                // Set tested output port pin to 0, keep other output pin to 1, keep other pins as orignal
                GPIO_SET_OUT_DATA(pOut,((pOut->DOUT)&~u16Mask));
                // Get input pins value of the tested output pin
                s_sKeypadKeyCtrlHandler.pu16PinValueArray[s_sKeypadKeyCtrlHandler.u8KeyPinValueIdx++] = GPIO_GET_IN_DATA(pIn)&u16InKeyPinMask;
                // Reset output pin value to high, but keep other pin as original
                GPIO_SET_OUT_DATA(pOut,((pOut->DOUT)|u16Mask));
            }
            u16Mask <<= 1;
        }
    }
}

/**
  * @brief  Initial key matix pins, and event handlers.
  * @param  u16InPinMaskA is the mask of GPIOA to be input pins of key matrix.
  * @param  u16InPinMaskB is the mask of GPIOB to be input pins of key matrix.
  * @param  u16OutPinMaskA is the mask of GPIOA to be output pins of key matrix.
  * @param  u16OutPinMaskB is the mask of GPIOB to be output pins of key matrix.
  * @param  psKeyHandler is the matrix key event handler array.
  * @param  pu16KeyPinValueBuf is to provided an array to be pin value buffer by application.
  *         Array size is "output pin number".
  *         This buffer must exist as keypad scaning.
  */
void Keypad_InitKey(uint16_t u16InPinMaskA,uint16_t u16InPinMaskB,
                    uint16_t u16OutPinMaskA,uint16_t u16OutPinMaskB,
                    const S_KEYPAD_KEY_HANDLER* psKeyHandler,uint16_t* pu16KeyPinValueBuf)
{
    s_sKeypadKeyCtrlHandler.u16InPinMaskA = u16InPinMaskA;
    s_sKeypadKeyCtrlHandler.u16InPinMaskB = u16InPinMaskB;
    s_sKeypadKeyCtrlHandler.u16OutPinMaskA = u16OutPinMaskA;
    s_sKeypadKeyCtrlHandler.u16OutPinMaskB = u16OutPinMaskB;
    s_sKeypadKeyCtrlHandler.psKeyHandler = (S_KEYPAD_KEY_HANDLER*)psKeyHandler;
    s_sKeypadKeyCtrlHandler.pu16PinValueArray = pu16KeyPinValueBuf;

    // Get default pin value
    s_sKeypadKeyCtrlHandler.u8KeyPinValueIdx = 0;
    Keypad_InitKeyMatrixPort(PA,s_sKeypadKeyCtrlHandler.u16InPinMaskA,PA,s_sKeypadKeyCtrlHandler.u16OutPinMaskA);
    Keypad_InitKeyMatrixPort(PA,s_sKeypadKeyCtrlHandler.u16InPinMaskA,PB,s_sKeypadKeyCtrlHandler.u16OutPinMaskB);
    Keypad_InitKeyMatrixPort(PB,s_sKeypadKeyCtrlHandler.u16InPinMaskB,PA,s_sKeypadKeyCtrlHandler.u16OutPinMaskA);
    Keypad_InitKeyMatrixPort(PB,s_sKeypadKeyCtrlHandler.u16InPinMaskB,PB,s_sKeypadKeyCtrlHandler.u16OutPinMaskB);
}

/**
  * @brief  Reset static variable for matrix key.
  */
void Keypad_ResetKey(void)
{
    s_sKeypadKeyCtrlHandler.pInGPIO = NULL;
    s_sKeypadKeyCtrlHandler.pOutGPIO = NULL;
    s_sKeypadKeyCtrlHandler.u16OutPinDebMask = 0;
    s_sKeypadKeyCtrlHandler.u16PinValueDiff = 0;
    s_u16KeyDebounceCounter = 0;
}

/**
  * @brief  1. Scan input pins of a tested output pin of key matrix.
  *         2. Call callback function as debounce time up.
  *         3. Internal function.
  * @param  pInGPIO is the address of GPIOA or GPIOB to be input port of key matrix.
  * @param  u16InPinMask is the mask of input pins of key matrix
  * @param  pOutGPIO is the address of GPIOA ot GPIOB to be output port of key matrix.
  * @param  u16OutPinMask is the mask of output pins of key matrix.
  */
void Keypad_ScanKeyPort(void* pInGPIO,uint16_t u16InPinMask,void* pOutGPIO,uint16_t u16OutPinMask)
{
    uint16_t u16Mask = 1,u16CurPinValue,u16Diff;
    GPIO_PA_T *pIn = (GPIO_PA_T *)pInGPIO;
    GPIO_PA_T *pOut = (GPIO_PA_T *)pOutGPIO;

    if(u16OutPinMask && u16InPinMask) {
        while( u16Mask > 0 ) {
            if (u16OutPinMask & u16Mask) {
                // Set tested output port pin to 0, keep other output pin to 1, keep other pins as orignal
                GPIO_SET_OUT_DATA(pOut,((pOut->DOUT)&~u16Mask));
                // Get input pins value of the tested output pin
                u16CurPinValue = GPIO_GET_IN_DATA(pIn)&u16InPinMask;
                // Diff with previous pin value
                u16Diff = u16CurPinValue ^ s_sKeypadKeyCtrlHandler.pu16PinValueArray[s_sKeypadKeyCtrlHandler.u8KeyPinValueIdx];
                // Reset output pin value to high, but keep other pin as original
                GPIO_SET_OUT_DATA(pOut,((pOut->DOUT)|u16Mask));
                // Key is not changed, return
                if (u16Diff == 0) {
                    s_sKeypadKeyCtrlHandler.u8KeyPinValueIdx++;
                    u16Mask <<= 1;
                    continue ;
                }
                // Key changed, check in debounce state.
                if ( s_sKeypadKeyCtrlHandler.u8KeyMode != KEYPAD_PRESSING &&
                        s_sKeypadKeyCtrlHandler.pInGPIO == pInGPIO         &&
                        s_sKeypadKeyCtrlHandler.pOutGPIO == pOutGPIO       &&
                        s_sKeypadKeyCtrlHandler.u16OutPinDebMask == u16Mask  &&
                        s_sKeypadKeyCtrlHandler.u16PinValueDiff == u16Diff   ) {
                    // Same Key changed during Debounce time is counting.
                    // It may be a noise. Clear Debounce status.
                    s_sKeypadKeyCtrlHandler.pInGPIO = NULL;
                    s_sKeypadKeyCtrlHandler.pu16PinValueArray[s_sKeypadKeyCtrlHandler.u8KeyPinValueIdx++] = u16CurPinValue;
                    u16Mask <<= 1;
                    continue ;
                }
                // start Debounce
                s_sKeypadKeyCtrlHandler.pInGPIO = pInGPIO;
                s_sKeypadKeyCtrlHandler.pOutGPIO = pOutGPIO;
                s_sKeypadKeyCtrlHandler.u16OutPinDebMask = u16Mask;
                s_sKeypadKeyCtrlHandler.pu16PinValueArray[s_sKeypadKeyCtrlHandler.u8KeyPinValueIdx] = u16CurPinValue;
                s_sKeypadKeyCtrlHandler.u16PinValueDiff = u16Diff;
                s_u16KeyDebounceCounter = g_u16DebounceCount;
                s_sKeypadKeyCtrlHandler.u8KeyMode = (u16Diff & u16CurPinValue) ? KEYPAD_RISING : KEYPAD_FALLING;
                s_sKeypadKeyCtrlHandler.u8KeyPinValueIdx++;
            }
            u16Mask <<= 1;
        }
    }
}

/**
  * @brief  1. Call the keypad evant handler for key matrix.
  *         2. Internal function.
  * @return 0: handler is called, 1: no handler meet the input parameters.
  */
uint8_t Keypad_KeyMapHandler(void)
{
    uint8_t u8InPort = (s_sKeypadKeyCtrlHandler.pInGPIO == PA)? 0 : 1;
    uint8_t u8OutPort = (s_sKeypadKeyCtrlHandler.pOutGPIO == PA)? 0 : 1;
    uint8_t u8i;

    for (u8i=0 ; s_sKeypadKeyCtrlHandler.psKeyHandler[u8i].m_pfnKey_Callback != NULL ; u8i++) {
        if (s_sKeypadKeyCtrlHandler.psKeyHandler[u8i].m_u8Port_I == u8InPort                                 &&
                s_sKeypadKeyCtrlHandler.psKeyHandler[u8i].m_u16Pin_I == s_sKeypadKeyCtrlHandler.u16PinValueDiff  &&
                s_sKeypadKeyCtrlHandler.psKeyHandler[u8i].m_u8Port_O == u8OutPort                                &&
                s_sKeypadKeyCtrlHandler.psKeyHandler[u8i].m_u16Pin_O == s_sKeypadKeyCtrlHandler.u16OutPinDebMask &&
                s_sKeypadKeyCtrlHandler.psKeyHandler[u8i].m_u8Mode == s_sKeypadKeyCtrlHandler.u8KeyMode          ) {
            s_sKeypadKeyCtrlHandler.psKeyHandler[u8i].m_pfnKey_Callback(s_sKeypadKeyCtrlHandler.psKeyHandler[u8i].m_u32Param);
            return 0;
        }
    }
    return 1;
}

/**
  * @brief  1.This function scan GPIOA or GPIOB pins for matrix key operation.
  *         2.It calls event handler if Debounce is timer up.
  * @note   1.This API can be called anywhere to scan key matrix status,
  *           and should be called timely to prevent status loss.
  *         2.But should be careful to call it in ISR to prevent block other ISR
  *           if event handler will take too much CPU time
  * @return 0: key event does not happen or event handler is called,
  *         1: event happen but handler is not found.
  */
uint8_t Keypad_ScanKey(void)
{
    uint8_t u8MapResult = 0;

    s_sKeypadKeyCtrlHandler.u8KeyPinValueIdx = 0;
    Keypad_ScanKeyPort(PA, s_sKeypadKeyCtrlHandler.u16InPinMaskA, PA, s_sKeypadKeyCtrlHandler.u16OutPinMaskA);
    Keypad_ScanKeyPort(PA, s_sKeypadKeyCtrlHandler.u16InPinMaskA, PB, s_sKeypadKeyCtrlHandler.u16OutPinMaskB);
    Keypad_ScanKeyPort(PB, s_sKeypadKeyCtrlHandler.u16InPinMaskB, PA, s_sKeypadKeyCtrlHandler.u16OutPinMaskA);
    Keypad_ScanKeyPort(PB, s_sKeypadKeyCtrlHandler.u16InPinMaskB, PB, s_sKeypadKeyCtrlHandler.u16OutPinMaskB);

    if (s_sKeypadKeyCtrlHandler.pInGPIO != NULL && s_u16KeyDebounceCounter == 0) {
        // Debounce time up, call callback function
        u8MapResult = Keypad_KeyMapHandler();

        switch(s_sKeypadKeyCtrlHandler.u8KeyMode) {
        // Falling trigger, start debounce for pressing
        case KEYPAD_FALLING:
            s_sKeypadKeyCtrlHandler.u8KeyMode = KEYPAD_PRESSING;
            s_u16KeyDebounceCounter = g_u16PressingCount;
            break;
        // Rising trigger. reset debounce
        case KEYPAD_RISING:
            s_sKeypadKeyCtrlHandler.pInGPIO = NULL;
            break;
        // Pressing trigger. set pressing counter.
        case KEYPAD_PRESSING:
            s_u16KeyDebounceCounter = g_u16PressingCount;
            break;
        }
        return u8MapResult;
    }
    return 0;
}

/**
  * @brief  This API is called in timer interrupt to decrease Debounce counter periodically.
  * @return 1: Decrease Debounce counter (in Debounce),
  *         0: not decrease Debounce counter (not in Debounce).
  */
uint8_t Keypad_KeyDecDebounceCounter(void)
{
    if (s_sKeypadKeyCtrlHandler.pInGPIO != NULL && s_u16KeyDebounceCounter>0 ) {
        s_u16KeyDebounceCounter--;
        return 1;
    }
    return 0;
}

/**
  * @brief  1. Initiate direct trigger pins.
  *         2. Set trigger event handlers.
  * @param  u16PinMaskA is the mask of PGIOA to be input pins.
  * @param  u16PinMaskB is the mask of PGIOB to be input pins.
  * @param  u16PinMaskC is the mask of PGIOC to be input pins.
  * @param  psTgrHandler is the direct trigger event handler array.
  */
void Keypad_InitTgr(uint16_t u16PinMaskA,uint16_t u16PinMaskB,uint16_t u16PinMaskC,const S_KEYPAD_TGR_HANDLER* psTgrHandler)
{
    s_sKeypadTgrCtrlHandler.u16PinMaskA = u16PinMaskA;
    s_sKeypadTgrCtrlHandler.u16PinMaskB = u16PinMaskB;
    s_sKeypadTgrCtrlHandler.u16PinMaskC = u16PinMaskC;
    s_sKeypadTgrCtrlHandler.u16PinValueA = GPIO_GET_IN_DATA(PA)&u16PinMaskA;
    s_sKeypadTgrCtrlHandler.u16PinValueB = GPIO_GET_IN_DATA(PB)&u16PinMaskB;
    s_sKeypadTgrCtrlHandler.u16PinValueC = GPIO_GET_IN_DATA(PC)&u16PinMaskC;
    s_sKeypadTgrCtrlHandler.psTgrHandler = (S_KEYPAD_TGR_HANDLER*)psTgrHandler;
}

/**
  * @brief  Reset static variable for trigger key.
  */
void Keypad_ResetTgr(void)
{
    s_sKeypadTgrCtrlHandler.pGPIO = NULL;
    s_sKeypadTgrCtrlHandler.u16PinValueDiff = 0;
    s_u16TgrDebounceCounter = 0;
}

/**
  * @brief  1.This function is called to scan GPIO pins for direct trigger operation.
  *         2.Internal function.
  * @param  pGPIO is the address of GPIO to be input port of direct trigger.
  * @param  u16PinMask is the mask of input pins to be input port of direct trigger.
  * @param  pu16PinValue is the pin value of the masked input pins.
  */
void Keypad_ScanTgrPort(void* pGPIO,uint16_t u16PinMask,uint16_t* pu16PinValue)
{
    uint16_t u16CurPinValue, u16Diff;
    GPIO_PA_T *p = (GPIO_PA_T *)pGPIO;
    // Check parameter
    if (u16PinMask == 0 || pGPIO == NULL )
        return;
    // Read current pin value
    u16CurPinValue = GPIO_GET_IN_DATA(p)&u16PinMask;
    // Compare with previous pin value
    u16Diff = u16CurPinValue ^ *pu16PinValue;
    // Tgr key isn't changed.
    if (u16Diff == 0)
        return ;
    // Trigger pin is in Debounce duration, or into starting debounce state.
    if ( s_sKeypadTgrCtrlHandler.pGPIO == pGPIO              &&
            s_sKeypadTgrCtrlHandler.u8TgrMode != KEYPAD_PRESSING &&
            u16Diff == s_sKeypadTgrCtrlHandler.u16PinValueDiff   ) {
        // same tgr pin changed. It may be a noise. Clear Debounce status.
        s_sKeypadTgrCtrlHandler.pGPIO = NULL;
        *pu16PinValue = u16CurPinValue;
    } else {
        s_sKeypadTgrCtrlHandler.pGPIO = pGPIO;
        *pu16PinValue = u16CurPinValue;
        s_sKeypadTgrCtrlHandler.u16PinValueDiff = u16Diff;
        s_u16TgrDebounceCounter = g_u16DebounceCount;
        s_sKeypadTgrCtrlHandler.u8TgrMode = (s_sKeypadTgrCtrlHandler.u16PinValueDiff & *pu16PinValue) ? KEYPAD_RISING : KEYPAD_FALLING;
    }
}

/**
  * @brief  1.Call the keypad evant handler for direct trigger.
  *         2.Internal function.
  * @return 1: handler is called, 0: no handler meet the input parameters.
  */
uint8_t Keypad_TgrMapHandler(void)
{
    uint8_t u8Port;
    uint8_t u8i;

    u8Port =  GPIO_GET_OFFSET((uint32_t)s_sKeypadTgrCtrlHandler.pGPIO);

    for (u8i=0 ; s_sKeypadTgrCtrlHandler.psTgrHandler[u8i].m_pfnTgr_Callback != NULL ; u8i++) {
        if (s_sKeypadTgrCtrlHandler.psTgrHandler[u8i].m_u8Port == u8Port                                  &&
                s_sKeypadTgrCtrlHandler.psTgrHandler[u8i].m_u16Pin == s_sKeypadTgrCtrlHandler.u16PinValueDiff &&
                s_sKeypadTgrCtrlHandler.psTgrHandler[u8i].m_u8Mode == s_sKeypadTgrCtrlHandler.u8TgrMode       ) {
            s_sKeypadTgrCtrlHandler.psTgrHandler[u8i].m_pfnTgr_Callback(s_sKeypadTgrCtrlHandler.psTgrHandler[u8i].m_u32Param);
            return 1;
        }
    }
    return 0;
}

/**
  * @brief  1.This function scan GPIOA , GPIOB or GPIOC pins for direct trigger operation.
  *         2.It calls event handler if Debounce is timer up.
  * @note   1.This API can be called anywhere to scan trigger status,
  *           and should be called timely to prevent status loss.
  *         2.But should be careful to call it in ISR to prevent block other ISR
  *           if event handler will take too much CPU time
  * @return 0: trigger event does not happen or event handler is called,
  *         1: event happen but handler is not found.
  */
uint8_t Keypad_ScanTgr(void)
{
    uint8_t u8MapResult = 0;

    Keypad_ScanTgrPort(PA, s_sKeypadTgrCtrlHandler.u16PinMaskA, &s_sKeypadTgrCtrlHandler.u16PinValueA);
    Keypad_ScanTgrPort(PB, s_sKeypadTgrCtrlHandler.u16PinMaskB, &s_sKeypadTgrCtrlHandler.u16PinValueB);
    Keypad_ScanTgrPort(PC, s_sKeypadTgrCtrlHandler.u16PinMaskC, &s_sKeypadTgrCtrlHandler.u16PinValueC);

    if (s_sKeypadTgrCtrlHandler.pGPIO != NULL && s_u16TgrDebounceCounter == 0) {
        // Debounce time up
        u8MapResult = Keypad_TgrMapHandler();

        switch(s_sKeypadTgrCtrlHandler.u8TgrMode) {
        // Falling trigger, start debounce for pressing
        case KEYPAD_FALLING:
            s_sKeypadTgrCtrlHandler.u8TgrMode = KEYPAD_PRESSING;
            s_u16TgrDebounceCounter = g_u16PressingCount;
            break;
        // Rising trigger. reset debounce
        case KEYPAD_RISING:
            s_sKeypadTgrCtrlHandler.pGPIO = NULL;
            break;
        // Pressing trigger. set pressing counter.
        case KEYPAD_PRESSING:
            s_u16TgrDebounceCounter = g_u16PressingCount;
            break;
        }
        return u8MapResult;
    }
    return 1;
}

/**
  * @brief  This API is called in timer interrupt to decrease Debounce counter periodically.
  * @return 1: Decrease Debounce counter (in Debounce),
  *         0: not decrease Debounce counter (not in Debounce).
  */
uint8_t Keypad_TgrDecDebounceCounter(void)
{
    if (s_sKeypadTgrCtrlHandler.pGPIO != NULL && s_u16TgrDebounceCounter > 0) {
        s_u16TgrDebounceCounter--;
        return 1;
    }
    return 0;
}
