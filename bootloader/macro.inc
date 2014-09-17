;******************************************************************************
;*                                                                            *
;*   Filename:     macro.inc                                                  *
;*   Date:         28 July 2012                                               *
;*   File Version: 0.1                                                        *
;*                                                                            *
;*   Author:       Benoit Frigon                                              *
;*   Email:        <bfrigon@gmail.com>                                        *
;*                                                                            *
;******************************************************************************

;*******************************************************************************
;
; SET_OSCCON : Set OSCCON based on OSC_TYPE and CLOCK settings
;
;*******************************************************************************
SET_OSCCON macro
osc SET 0

if CLOCK == 8000000
osc SET 0x70
endif

if (CLOCK == 4000000)
osc SET 0x60		; 4MHz int. osc,
endif
		
if (CLOCK == 2000000)
osc SET 0x50		; 2MHz int. osc,		
endif
		
if (CLOCK == 1000000)
osc SET 0x40		; 1MHz int. osc,				
endif
		
if (CLOCK == 500000)
osc SET 0x30		; 500KHz int. osc,				
endif
				
if (CLOCK == 250000)
osc SET 0x20		; 250KHz int. osc,				
endif
		
if (CLOCK == 125000)
osc SET 0x10		; 125KHz int. osc,				
endif

if (OSC_TYPE == 0x02 && osc == 0 && CLOCK != 31000)
		error "Invalid clock setting for internal oscillator"
endif

		movlw	osc+OSC_TYPE
		movwf	OSCCON

		endm



;*******************************************************************************
;
; SET_TBLPTR : 
;
;*******************************************************************************

SET_TBLPTR macro address
		
		movlw		(address & 0xFF)
		movwf		TBLPTRL
		
if (address > 0xFF)
		movlw		(address & 0xFF00) >> 8
		movwf		TBLPTRH
else
		clrf		TBLPTRH		
endif

if (address > 0xFFFF)
		movlw		(address & 0xFF0000) >> 16
		movwf		TBLPTRU
else
		clrf		TBLPTRU		
endif

		endm
		
		

