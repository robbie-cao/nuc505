; Copyright (c) Nuvoton Technology Corp. All rights reserved.

LR_ROM      0x00000000  0x00100000  ; 1MB (SPI FLash)
{
	ER_STARTUP +0
    {
        startup_nuc505Series.o(RESET, +First)
	}

    ER_RO +0
    {
		*(+RO)
	}
	ER_RW 0x20000000
	{
		*(+RW)
	}
}

LR_ROM_2      0x00100000  0x00100000  ; 1MB (SPI FLash)
{
    ER_STARTUP +0
    {
        startup_nuc505Series.o(RESET, +First)
    }

    ER_RO +0
    {
        *(+RO)
    }
    ER_RW 0x20000000
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
