; Copyright (c) Nuvoton Technology Corp. All rights reserved.

LR_ROM      0x00000000  0x00200000  ; 2MB (SPI FLash)
{
	ER_STARTUP +0
    {
        startup_nuc505Series.o(RESET, +First)
	}
    
    ;ER_MTPSIG 0x00000100 FIXED  ; MTP signature is fixed at 0x0000_0100.
    ;{
    ;    main.o(mtpsig)
    ;}
    
    ER_RO +0
	{
		*(+RO)
	}
    
    ;ER_SPIMCODE     0x1FF00000  0x00011000
	ER_SPIMCODE     0x1FF00000  0x00020000
    {
		clk.o(+RO)	; CLK_SetCoreClock() may take a long time if it is run on SPI Flash. Locate this function on SRAM for speed.
        nuc505_isr.o(+RO)	; put heavy loading code to SRAM
		ff.o(+RO)	; put heavy loading code to SRAM
		layer3.o(+RO)
		layer12.o(+RO)
		synth.o(+RO)
		huffman.o(+RO)
		frame.o(+RO)
		bit.o(+RO)
		stream.o(+RO)
		timer.o(+RO)
		diskio.o(+RO)
		mp3.o(+RO)
    ;    *(spimcode)
	}
    
    ER_RW           +0
	{
		*(+RW, +ZI)
	}
}

;LR_RAM      0x20011000  0x0000F000
;{	
;	ER_ZI +0
;	{
;		*(+ZI)
;	}
;}
