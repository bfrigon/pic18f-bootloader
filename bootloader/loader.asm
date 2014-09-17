;******************************************************************************
;*                                                                            *
;*   Filename:     loader.asm                                                 *
;*   Date:         28 July 2012                                               *
;*   File Version: 0.1                                                        *
;*                                                                            *
;*   Author:       Benoit Frigon                                              *
;*   Email:        <bfrigon@gmail.com>                                        *
;*                                                                            *
;******************************************************************************
include <p18f2520.inc>
include "macro.inc"

#define PROJECT_PVR

STATUS_BLOCK_OK				equ		0x01

ERROR_UNIMPLEMENTED			equ		0xA0
ERROR_PROTECTION			equ		0xF0
ERROR_BUFFER_OVERFLOW		equ		0xF1
ERROR_INVALID_LOC			equ		0xF2
ERROR_BAD_CHECKSUM         	equ		0xF3
ERROR_WRITE_FAILED			equ		0xF5


;==============================================================================
;==============================================================================
;
;							  Bootloader config
;
;==============================================================================
;==============================================================================
#define READCMD_ENABLED

#define OSC_TYPE				0x01
#define CLOCK					8000000
#define BAUD_RATE				38400

#define PROGRAM_MEMORY_SIZE		0x8000
#define EEPROM_SIZE				0x100
#define	ADDR_USERCODE_START		0x400

#define	RELOC_IVT_RESET			ADDR_USERCODE_START
#define RELOC_IVT_HIGH			ADDR_USERCODE_START+0x8
#define RELOC_IVT_LOW			ADDR_USERCODE_START+0x18

#define TRIS_USART_RX			TRISC, TRISC7
#define TRIS_USART_TX			TRISC, TRISC6

#define PIN_BTN					PORTB, RB0
#define TRIS_BTN				TRISB, TRISB0	


;--------------------------------------------------
; Specific projects config
;--------------------------------------------------

; PBX lcd
;----------------------
#ifdef PROJECT_PBX
	#define LTCH_LED				LATC, LATC2
	#define TRIS_LED				TRISC, TRISC2
#endif


; PVR lcd
;----------------------
#ifdef PROJECT_PVR
	#define LTCH_LED_A				LATA, LATA1
	#define TRIS_LED_A				TRISA, TRISA1
	#define LTCH_LED				LATA, LATA0
	#define TRIS_LED				TRISA, TRISA0
	
	#define PIN_BTN_P				PORTC, RC2
	#define TRIS_BTN_P				TRISC, TRISC2
#endif



;==============================================================================
;==============================================================================
;
;							 Configuration bits
;
;==============================================================================
;==============================================================================
	CONFIG 	OSC=INTIO67
	CONFIG 	PWRT=ON
	CONFIG 	WDT=OFF
	CONFIG 	PBADEN=OFF
	CONFIG 	LVP=OFF
	CONFIG 	MCLRE=ON
	CONFIG 	DEBUG=OFF
	CONFIG	STVREN=ON
	




;==============================================================================
;==============================================================================
;
;								    Data
;
;==============================================================================
;==============================================================================
	CBLOCK	0x000
		d1, d2, d3							; Delay counters
		t1, t2								; Temporary values
		i, i2								; Iteration counters
		checksum							; Block checksum
		length								; Block length
		addr:3								; Block address
		buffer:64							; Block data
	ENDC



;==============================================================================
;==============================================================================
;
;								     IVT
;
;==============================================================================
;==============================================================================
		org		0x0000						
		bra Init							; Jump to bootloader init

		org		0x0008						; Jump to user high priority vector
		goto	RELOC_IVT_HIGH          	

		org		0x0018						; Jump to user low priority vector
		goto	RELOC_IVT_LOW



;==============================================================================
;==============================================================================
;
;								 Entry point
;
;==============================================================================
;==============================================================================
		org		0x0020
Init:
											; ---------------------------------
											; Initialize CPU
											; ---------------------------------											
		clrf	OSCTUNE						
		SET_OSCCON
		
		bcf		INTCON,	GIE					; Disable interrupts

											; ---------------------------------
											; Initialize peripherals
											; ---------------------------------											
		clrf	ADCON0						; Disable A/D
		movlw	0x0F
		movwf	ADCON1
		
#ifdef PROJECT_PBX		
		bsf		TRIS_BTN					; Set button port as input
		bcf		PIN_BTN
		bcf		TRIS_LED					; Set status led port as output
		bsf		LTCH_LED					; Turn on led
#endif		

#ifdef PROJECT_PVR
		bcf		TRIS_BTN_P					
		bsf		PIN_BTN_P

		bsf		TRIS_BTN					; Set button port as input
		bcf		PIN_BTN

		bcf 	TRIS_LED_A					; Set status led port as output
		bsf		LTCH_LED_A					; Turn on led
		bcf		TRIS_LED
		bcf		LTCH_LED
#endif

		
		movlw	D'88'        
		cpfseq	0x00
		bra		no_bl_entry
		
		movlw	D'44'
		cpfseq	0x01
		bra		no_bl_entry
		
		bra		bootloader_main

no_bl_entry:
		
											; ---------------------------------
											; 
											; ---------------------------------											
		movlw	D'15'						; 15 x 100 ms delay (1.5s)
		rcall	Delay

		btfsc	PIN_BTN						; If the button is pressed, enter 
		bra		bootloader_main				; bootloader


check_valid_usercode:

		SET_TBLPTR(RELOC_IVT_RESET)			; Set TBLPTR to user reset vector

		clrf	EECON1
		bsf		EECON1,	EEPGD

		tblrd*+								; Read location
		movf	TABLAT, W

		xorlw	0xFF						; Check if location is programmed
		bz		bootloader_main				; if not, start bootloader
	
		goto	RELOC_IVT_RESET				; if so, jump to user reset vector


;******************************************************************************
; Bootloader main
;******************************************************************************
bootloader_main:
											; ---------------------------------
											; Make led flash 3 times to 
											; acknowledge bootloader entry
											; ---------------------------------
		movlw	d'6'						; Set iteration counter to 6        
		movwf	i							;

loop_flash:
		btg		LTCH_LED, 0					; Toggle led state
		
		movlw	D'2'						; 200ms delay
		rcall	Delay

		decfsz	i, F
		bra		loop_flash


		clrf	addr+0
		clrf	addr+1
		clrf	addr+2

											; ---------------------------------
											; Initialize USART module
											; ---------------------------------											
		bcf		TRIS_USART_TX				; Set TX pin as output
		bsf		TRIS_USART_RX				; Set RX pin as input

		clrf	BAUDCON						; TX idle high
		bsf		BAUDCON, BRG16				; 16-bit baud rate generator

		movlw	D'12'						; 38400 bps 
		movwf	SPBRG						; 8Mhz/(16(12+1)) = 38461.538461538
		clrf	SPBRGH						; (+0.2% error)

		bcf		PIE1, RCIE					; Disable Receive interrupt

		clrf	TXSTA						; 8-bit, async mode, low speed
		bsf		TXSTA, TXEN					; Enable transmitter
		
		clrf	RCSTA						; 8-bit
		bsf		RCSTA, CREN					; Continous receive enabled
		bsf		RCSTA, SPEN					; Enable serial port
		
loop_commands:
		rcall	UsartRead
		movwf	t1
		
		movlw	'@'
		xorwf	t1,	W
		bz		command_address
		
		movlw	'+'
		xorwf	t1,	W
		bz		command_data
		
		movlw	'!'
		xorwf	t1,	W
		bz		command_done

#ifdef	READCMD_ENABLED		
		movlw	'<'
		xorwf	t1, W
		bz		command_read
#endif ; READCMD_ENABLED

		movlw	'?'
		xorwf	t1, W
		bz		command_ping
		
		
		bra		loop_commands				; Ignore all other characters


;------------------------------------------------------------------------------
; Command : Ping
;------------------------------------------------------------------------------		
command_ping:
		movlw	STATUS_BLOCK_OK
		rcall	UsartWriteStatus

		bra		loop_commands
			
;------------------------------------------------------------------------------
; Command : Address
;------------------------------------------------------------------------------		
command_address:

		clrf	checksum
				
		lfsr	FSR0, addr+0

		movlw	D'3'
		movwf	i

loop_read_addr:		
		rcall	UsartReadHex
		movwf	POSTINC0
		rcall	AddChecksum
		
		decfsz	i, F
		bra		loop_read_addr
		
		rcall	ValidateChecksum

		movlw	STATUS_BLOCK_OK				; Sent OK status
		rcall	UsartWriteStatus
		
		bra		loop_commands


;------------------------------------------------------------------------------
; Command : EOF
;------------------------------------------------------------------------------
command_done:
		movlw	STATUS_BLOCK_OK
		rcall	UsartWriteStatus

											; ---------------------------------
											; Make led flash 2 times to 
											; acknowledge end of file
											; ---------------------------------
		movlw	d'4'						; Set iteration counter to 4
		movwf	i							;

loop_flash2:
		btg		LTCH_LED, 		0			; Toggle led state

		movlw	D'7'						; 700ms delay
		rcall	Delay

		decfsz	i, 				F
		bra		loop_flash2		
		
		reset


;------------------------------------------------------------------------------
; Command : Read
;------------------------------------------------------------------------------
#ifdef	READCMD_ENABLED		
command_read:
		movlw	STATUS_BLOCK_OK
		rcall	UsartWriteStatus


		movff	addr+0, TBLPTRU
		movff	addr+1, TBLPTRH
		movff	addr+2, TBLPTRL

		btfsc	addr+0, 5
		bsf		EECON1,	CFGS
		btfss	addr+0, 4
		bcf		EECON1,	CFGS

		movlw	D'64'
		movwf	i
		
loop_read:		
		tblrd	*+
		movf	TABLAT,	W
		rcall	UsartWriteHex
		
		decfsz	i, F
		bra		loop_read
		
		rcall	GotoNextBlock

		bra		loop_commands

#endif ; READCMD_ENABLED


;------------------------------------------------------------------------------
; Command : Write data
;------------------------------------------------------------------------------
command_data:
		clrf	checksum
		
		rcall	UsartReadHex

		movwf	length
		movwf	i
		rcall	AddChecksum
		
		movlw	D'65'
		cpfslt	length
		bra		error_buffer_overflow
		
		
		lfsr	FSR0, buffer+0

loop_read_data:
		rcall	UsartReadHex
		movwf	POSTINC0
		rcall	AddChecksum
		
		decfsz	i, F
		bra		loop_read_data
		
		rcall	ValidateChecksum
		
		btfsc	addr+0, 6
		bra		write_data_mem
		
		movlw	0x30
		xorwf	addr+0, W
		bz		write_config_mem
		
		; ** Fall through **
	
;------------------------------------------------------------------------------
; Write code memory handler 
; (0x000000-0x0007FF)
;------------------------------------------------------------------------------
write_code_mem:

		tstfsz	addr+0
		bra		no_bb_check
						
		movf	addr+1, W
		andlw	0xFC
		bz		error_protection
no_bb_check:

		movlw	0x80
		cpfslt	addr+1
		bra		error_invalid_loc

		movff	addr+0, TBLPTRU
		movff	addr+1, TBLPTRH
		movff	addr+2, TBLPTRL
		lfsr	FSR0, buffer
		
		;movlw	b'10010100'
		bsf		EECON1, EEPGD
		bcf		EECON1,	CFGS
		bsf		EECON1, FREE
		bsf		EECON1, WREN
		
		rcall	MemoryWrite
		tblrd	*-

		movff	length, i
	
loop_write_code:
		movf	POSTINC0, W
		movwf	TABLAT
		tblwt	+*
		
		decf	i, F						; 
		
		movf	i, W						; Check if counter is at the
		andlw	0x1F						; end of a 32 byte boundary

		bnz		loop_write_code				; If not, continue filling table
		
		;movlw	b'10000100'					; Write table
		bsf		EECON1, EEPGD
		bcf		EECON1, FREE
		bsf		EECON1, WREN

		rcall	MemoryWrite
		
		tstfsz	i
		bra		loop_write_code
		
		bcf		EECON1, WREN				; Disable writes
		
		movff	length, i
		movf	POSTDEC0, W
		
loop_check_code:
		tblrd	*-
		movf	TABLAT, W
		xorwf	POSTDEC0, W
		bnz		error_write_failed
		
		decfsz	i, F
		bra 	loop_check_code
		
		rcall	GotoNextBlock


		movlw	STATUS_BLOCK_OK				; Sent OK status
		rcall	UsartWriteStatus
		
		bra		loop_commands		




;------------------------------------------------------------------------------
; Write data memory handler 
; (0x400000-0x4000FF)
;------------------------------------------------------------------------------
write_data_mem:

		clrf	EECON1
		bsf		EECON1, WREN
		
		movff	addr+2, EEADR
		lfsr	FSR0, buffer
		movff	length, i
		
loop_write_data:
		movff	POSTINC0, EEDATA
		rcall	MemoryWrite
		
wait_wr:
		btfss	PIR2, EEIF
		bra		wait_wr

		

		incf	EEADR, F
		btfsc	STATUS, C
		bra		error_invalid_loc
		
		decfsz	i, F
		bra 	loop_write_data

		bcf		EECON1, WREN
		
		movff	addr+2, EEADR
		lfsr	FSR0, buffer
		movff	length, i

loop_check_data:
		bsf		EECON1, RD

		movf	EEDATA, W
		xorwf	POSTINC0, W
		bnz		error_write_failed

		incf	EEADR, F

		decfsz	i, F
		bra		loop_check_data

		movlw	STATUS_BLOCK_OK				; Sent OK status
		rcall	UsartWriteStatus

		bra		loop_commands

;------------------------------------------------------------------------------
; Write code memory handler 
; (0x000000-0x0007FF), (0x200000-0x200008)
;------------------------------------------------------------------------------
write_config_mem:		

		movlw	ERROR_UNIMPLEMENTED			; Unimplemented
		rcall	UsartWriteStatus
		
		bra		loop_commands
		
				


;******************************************************************************
; Error F0 : Attempt to write over protected memory location
;******************************************************************************
error_protection:
		movlw	ERROR_PROTECTION			; Send protection fault error code
		rcall	UsartWriteStatus
		bra		bootloader_main				; Restart bootloader



;******************************************************************************
; Error F2 : Invalid memory location
;******************************************************************************
error_invalid_loc:
		movlw	ERROR_INVALID_LOC      		; Send invalid location error code
		rcall	UsartWriteStatus
		bra		bootloader_main				; Restart bootloader



;******************************************************************************
; Error F1 : Buffer overflow, record length > 32
;******************************************************************************
error_buffer_overflow:
		movlw	ERROR_BUFFER_OVERFLOW		; Send buffer overflow error code
		rcall	UsartWriteStatus			
		bra		bootloader_main				; Restart bootloader


;******************************************************************************
; Error F3 : Record checksum invalid
;******************************************************************************
error_checksum:
		movlw	ERROR_BAD_CHECKSUM			; Send invalid checksum error code
		rcall	UsartWriteStatus
		bra		bootloader_main				; Restart bootloader


;******************************************************************************
; Error F5 : Write failed
;******************************************************************************
error_write_failed:
		movlw	ERROR_WRITE_FAILED			; Send unexpected record error
		rcall	UsartWriteStatus
		bra		bootloader_main				; Restart bootloader




;==============================================================================
;==============================================================================
;
;								Subroutines
;
;==============================================================================
;==============================================================================

;******************************************************************************
; Delay: ~100 ms loop @ 8Mhz
;******************************************************************************
Delay:
		movwf	d3

delay_loop:
		clrf	d1
		movlw	0x70
		movwf	d2

		decfsz  d1, 			F
		goto 	$+6
		decfsz	d2, 			F
		goto 	$-8
		
		decfsz	d3, 			F
		bra 	delay_loop		
		return


;******************************************************************************
; AddChecksum : Calculate the checksum for the byte in W
;******************************************************************************
AddChecksum:
		xorlw	0xFF						; Calculate checksum of WREG
		addlw	0x01
		addwf	checksum,		F			; Add to record checksum	
		return



;******************************************************************************
; NibbleToHex : Nibble (4 bits) to hexadecimal string 
;******************************************************************************
NibbleToHex:
		andlw	0x0F
		movwf	t1
		movlw	D'10'
		cpfslt	t1
		bra		alpha_1

		movlw	D'48'
		addwf	t1,				W
		return
alpha_1:		
		movlw	D'55'
		addwf	t1,				W
		return		

		
;******************************************************************************
; HexToNibble : Hexadecimal string to nibble
;******************************************************************************
HexToNibble:
		movwf	t1
		
		movlw	D'58'
		cpfslt	t1
		bra 	alpha_2
		
		movlw	D'48'
		subwf	t1,				W
		return		
		
alpha_2:
		movlw	D'55'
		subwf	t1,				W
		return


;******************************************************************************
; WriteStatus : Write status code to USART using the folowing format : 
;				S[code]
;******************************************************************************
UsartWriteStatus:
		movwf	t2

		movlw	'S'
		rcall	UsartWrite
		
		movf	t2,				W
		rcall	UsartWriteHex
		return


;******************************************************************************
; Read byte from USART
;******************************************************************************
UsartRead:
		btfss	PIR1,			RCIF
		bra		UsartRead
		movf	RCREG,			W
		return


;******************************************************************************
; UsartReadHex:	
;******************************************************************************
UsartReadHex:
		rcall	UsartRead					; Read upper nibble
		rcall	HexToNibble
		movwf	t2
		swapf	t2,				F
		
		rcall	UsartRead					; Read lower nibble
		rcall	HexToNibble		
		addwf	t2,				W

		return


;******************************************************************************
; UsartWrite
;******************************************************************************
UsartWrite:
		btfss	PIR1,			TXIF		; Wait for xmit buffer to empty
		bra		UsartWrite
		movwf	TXREG						; Send the byte
		return


;******************************************************************************
; UsartWriteHex
;******************************************************************************
UsartWriteHex:
		movwf	t2

		swapf	t2,				W			
		rcall	NibbleToHex					
		rcall	UsartWrite					; Write upper nibble

		movf	t2,				W
		rcall	NibbleToHex
		rcall	UsartWrite					; Write lower nibble
		return


;******************************************************************************
; ValidateChecksum
;******************************************************************************
ValidateChecksum:
		rcall	UsartReadHex
		
		xorwf	checksum,		W
		bnz		error_checksum

		return


;******************************************************************************
; MemoryWrite
;******************************************************************************
		goto	0x0000
MemoryWrite:
		bcf		PIR2, EEIF

		movlw 	0x55
		movwf 	EECON2
		movlw 	0xAA
		movwf 	EECON2
		bsf		EECON1,				WR			; WRITE
		nop
		nop

		return


;******************************************************************************
; GotoNextBlock
;******************************************************************************
GotoNextBlock:
		bcf		STATUS, C

		movlw	0x40						; Increment address by 64 bytes
		addwf	addr+2,	F
		btfsc	STATUS,	C			
		incf	addr+1,	F
		btfsc	STATUS,	C
		incf	addr+0,	F 

		return



;==============================================================================
		END
;******************************************************************************
