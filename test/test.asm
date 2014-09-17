;******************************************************************************
;*                                                                            *
;*   Filename:     test.asm                                                   *
;*   Date:         02 July 2004                                               *
;*   File Version: 0.1                                                        *
;*                                                                            *
;*   Author:       Benoit Frigon                                              *
;*   Email:        <bfrigon@gmail.com>                                        *
;*                                                                            *
;******************************************************************************
include <p18f2520.inc>
LIST P=18F2520


	CONFIG	OSC=INTIO67, PWRT=ON, WDT=OFF, WRTB=ON, PBADEN=OFF
	CONFIG	LVP=ON, MCLRE=ON


		org	0x0400							; Reset vector
		bra Init							; Jump to main code

		org	0x0408							; High priority vector
		bra 	$		
		
		org	0x0418							; Low priority vector
		bra 	$
		

		org 0x0430
Init:		
		clrf	OSCTUNE						
		movlw	0x72						; 8MHz int. osc, 
		movwf	OSCCON

		bcf		INTCON,			GIE			; Disable interrupts

											; ---------------------------------
											; Initialize peripherals
											; ---------------------------------											
		clrf	ADCON0						; Disable A/D
		movlw	0x0F
		movwf	ADCON1
		
		bsf		TRISB,			TRISB1		; Set button port as input
		bcf		PORTB,			RB1
		bcf		TRISC,			TRISC2		; Set status led port as output
		bsf		LATC,			LATC2		; Turn on led		


loop:
		btfsc	PORTB,			RB1
		bra		led_off
		
		bsf		LATC,			LATC2
		bra		loop
		
led_off		
		bcf		LATC,			LATC2
		bra		loop		


	
		
		bra		$
		
		


		org 0x4000
		bra		$

		org 0x4100
		bra		$

		org 0x4220
		bra		$

		org 0x43F0
		bra		$

		org 0x5000
		bra		$
		
		
		org 0x8300
		bra		$		
		
		
		org 0x200000
			de  "test"
		
		org 0x400000
			db	0x01,0x02, 0x03
			
		org 0x400010
			db 0x10		
		
	END

