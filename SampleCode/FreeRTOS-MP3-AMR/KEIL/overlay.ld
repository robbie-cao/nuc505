; Copyright (c) Nuvoton Technology Corp. All rights reserved.


LR_ROM      0x00000000
{
	ER_STARTUP +0
    {
        startup_nuc505Series.o(RESET, +First)   ; vector table
        *(InRoot$$Sections)                     ; library init
        ; If neither (+ input_section_attr) nor (input_section_pattern) is specified, the default is +RO.
        startup_nuc505Series.o                  ; startup
        system_nuc505Series.o(i.SystemInit)    
	}
    
    ER_RO       +0
    {
		*(+RO)
	}
    
    ; Relocate vector table in SRAM for fast interrupt handling.
    ER_VECTOR2  0x20000000  EMPTY   0x000000D0
    {
    }
    
    ; Critical code located in SRAM. Loaded by ARM C library at startup.
    ER_FASTCODE_INIT +0
    {
        clk.o(+RO)          ; CLK_SetCoreClock() may take a long time if it is run on SPI Flash.
        ovlymgr.o(+RO)    ; Overlay manager itself must locate in SRAM because it will be responsible for loading code 
                            ; through SPIM DMA Read.
		;main.o(+Ro)
		;tasks.o(+Ro)
		;timers.o(+Ro)
		;list.o(+Ro)
		;queue.o(+Ro)
		;port.o(+Ro)
		;heap_2.o(+Ro)
		;QueueSet.o(+Ro)
		;PollQ.o(+Ro)
		app.o(+Ro)
		sd.o(+Ro)
		diskio.o(+Ro)
		ff.o(+Ro)
		*mc_w.l(+Ro)	;for memcpy
		Timer.o(+Ro)
    }
    
    ER_RW       +0
	{
		*(+RW)
	}
    
    ER_ZI       +0
    {
        *(+ZI)
    }
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ER_OVERLAY_A +0  OVERLAY NOCOMPRESS
    {
		*(overlay_a)
		*amr_codec.lib(+Ro)
    }
    ER_OVERLAY_B +0  OVERLAY NOCOMPRESS
    {
        *(overlay_b)
		*Mp3_M4_Keil.lib(+Ro)
    }
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
    ; Serve to mark end of used SRAM.
    ER_INDICATOR_END  +0  EMPTY   0 {}
}
ScatterAssert(LoadLimit(LR_ROM) <= 0x00200000)
ScatterAssert(ImageLimit(ER_INDICATOR_END) <= 0x20020000)
