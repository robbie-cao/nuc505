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
}

LR_RAM      0x20000000  0x00020000
{	
	ER_CLKSWITCH   +0
    {
        CLK.o
	}
	ER_ZI +0
	{
		*(+ZI)
	}
	ER_RW           +0
	{
		*(+RW)
	}
}

