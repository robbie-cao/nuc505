; Copyright (c) Nuvoton Technology Corp. All rights reserved.

LR_ROM      0x00000000  0x00200000  ; 2MB (SPI FLash)
{
	ER_STARTUP +0
    {
        startup_nuc505Series.o(RESET, +First)
	}
    
    ER_MTPSIG 0x00000100 FIXED  ; MTP signature is fixed at 0x0000_0100.
    {
        main.o(mtpsig)
    }
    
    ER_RO +0
    {
		*(+RO)
	}
    
    ER_SPIMCODE     0x20000000  0x00010000
    {
        clk.o(+RO); CLK_SetCoreClock() may take a long time if it is run on SPI Flash. Locate this function on SRAM for speed.
        *(spimcode)
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
