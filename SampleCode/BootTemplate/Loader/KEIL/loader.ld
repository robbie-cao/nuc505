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
    
    ; Reserve SRAM to place RAM configuration image.
    ER_RAMIMG   0x20000000  UNINIT
    {
        *(ramimg)
    }
    
    ; Critical code in SRAM for fast execution. Loaded by ARM C library at startup.
    ER_FASTCODE_INIT    +0
    {
        clk.o(+RO); CLK_SetCoreClock() may take a long time if it is run on SPI Flash.
    }
    
    ER_RW   +0
	{
		*(+RW)
	}
    
    ER_ZI +0
	{
		*(+ZI)
	}
}
