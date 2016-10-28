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
    ER_CLKSWITCH   0x20001000  0x000f000
    {
		main.o
        CLK.o		
		disable.o
	}
	
    ER_RW           +0
	{
		*(+RW)
	}
}

LR_RAM      0x20010000  0x00010000
{	
	ER_ZI +0
	{
		*(+ZI)
	}
}
