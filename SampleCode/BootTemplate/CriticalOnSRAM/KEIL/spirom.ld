; Copyright (c) Nuvoton Technology Corp. All rights reserved.

LR_ROM      0x00000000  0x00200000  ; 2MB (SPI FLash)
{
	ER_STARTUP +0
    {
        startup_nuc505Series.o(RESET, +First)
	}
    
    ER_RO +0
    {
		*(+RO)
	}
    
    ; Relocate vector table in SRAM for fast interrupt handling.
    ER_VECTOR2      0x20000000  EMPTY   0x00000400
    {
    }
    
    ; Critical code in SRAM for fast execution. Loaded by ARM C library at startup.
    ER_FASTCODE_INIT    0x20000400
    {
        clk.o(+RO); CLK_SetCoreClock() may take a long time if it is run on SPI Flash.
    }
    
    ER_RW   +0
	{
		*(+RW)
	}
    
    ; Critical code in SRAM for fast execution. Loaded by user.
    ER_FASTCODE_UNINIT  +0  OVERLAY
    {
        *(fastcode)
    }
    
    
}

LR_RAM      0x20010000  0x00010000
{	
	ER_ZI +0
	{
		*(+ZI)
	}
}
