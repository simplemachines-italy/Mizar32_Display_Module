;***********************************************************
; Programma editato da Antonio Cingolani per Simplemachines
; per la scheda Display_module del Progetto Mizar
; Display_module.asm V. 1.0
; Pic 16f84 quarzo da 3,68 Mhz
;
; For better view in your text editor select 'Keep Tabs'
; and set Tabs sizes with 7
;************************************************************

                PROCESSOR       16F84
                RADIX           DEC
                INCLUDE         "P16f84.INC"
                ;ERRORLEVEL      -302
								__CONFIG        0x3FF1

;Linee di controllo dell'LCD

LCD_RS          equ     2       ;Register Select on portb
LCD_E           equ     4       ;Lcd Enable on port portb
LCD_RW          equ     3       ;Read o write selection on portb

;LCD data line bus

LCD_DB4         equ     0       ;LCD data line DB4
LCD_DB5         equ     1       ;LCD data line DB5
LCD_DB6         equ     2       ;LCD data line DB6
LCD_DB7         equ     3       ;LCD data line DB7


SET_EN        	MACRO
                bsf  PORTB,LCD_E
              	ENDM

CLEAR_EN      	MACRO
                bcf  PORTB,LCD_E
              	ENDM

EN_STROBE				MACRO
                SET_EN
                nop
                CLEAR_EN
              	ENDM

sda		equ	0	;PortB bit 0 is sda pin
scl		equ	1	;PortB bit 1 is scl pin
i2c		equ	PORTB	;
I2CTRIS	equ	TRISB

;Used PIC memory ORG 0x0C

tmpLcdRegister	equ	0x0c	;Two locations reserved for LCD register
msDelayCounter	equ	0x0e   ;Two locations reserved for delay register
general		equ	0x10  ; general pourpose register
time_out		equ	0x11	;Counter register for time out i2c comunications

flag			equ	0x12	;|Register used for bit flag allocation
;Initializes bit-flag in register-flag-----------------------------------------------------|
button			equ	0	;|0 = no event on switch - 1 = event on switch		|
switch_bit		equ	1	;|0 = do not send the value of switch down on i2c	|
					;|1 = send the value of switch down on i2c		|
read			equ	2	;|1 = read in master operation				|				
write			equ	2	;|0 = write in master operation				|
istr_lcd		equ	3	;|0 = instruction to send to display			|
data_lcd		equ	3	;|1 = data to send to dislpay				|
intflag		equ	4	;|0 = no interrupt occurred 1 = interrupt occurred	|
avr32			equ	5	;|0 = send i2c word to eeprom 1 =send i2c word to avr32	|
;------------------------------------------------------------------------------------------|

i2c_data		equ	0x13	;Save location for i2c data byte.
i2c_bit		equ	0x14	;bit counter in i2c byte transfert
addr			equ	0x15	;address to write or to read in i2c operation
addr_slave		equ	0x16	;the phisical address to identificate the slave
nb_data		equ	0x17	;data number's to write (o read) in I2C operation
bump			equ	0x18	;register used for switch's antibump

start_buffer		equ	0x2c	;Start buffer for i2c comunication
					;buffer deep is 0x2c-0x4f, then we have 36 byte
					;reserved

avr32_addr		equ	b'11110010'	;Avr32 phisical address

;i2c_addr_read		equ	0xaf	;24lc32 hardware address for read operation
;i2c_addr_write	equ	0xae	;24lc32 hardware address for write operation

;*******************************************************************************
;Memory address to write on LCD
;*******************************************************************************
;The address on I2C memory where are wrote all information


     ;Start program

	ORG	0x0000
	goto	Start

	org	0x0004	;Start interrupt routine

Interrupt_routine
;If we are here, sda pin is gone low. Look for scl pin; If scl=1 a start 
;condiction is TRUE

	bcf	STATUS,RP0      	;Select bank of memory 0
	bcf	INTCON,GIE		;Disable all interrupt
	bsf	flag,intflag		;interrupt occurred

	btfss	PORTB,scl
	goto	uscita_interrupt	;If no start condiction go out
	call	start_bit_ok	

	btfsc	STATUS,C		;error on i2c comunication?
	goto	uscita_interrupt	;yes

	movlw	B'11110110'		;command?
	subwf	i2c_data,0
	
	btfsc	STATUS,Z		
	goto	command_recive_routine	;yes... command

	movlw	b'11110010'		;data?
	subwf	i2c_data,0
	
	btfsc	STATUS,Z		
	goto	data_recive_routine	;yes... data

uscita_interrupt				

	bcf	INTCON,INTF
	retfie	

command_recive_routine

	bcf	flag,istr_lcd
	goto	recive_and_store

data_recive_routine

	bsf	flag,istr_lcd

recive_and_store

	clrf	nb_data
	call	send_ack

	call	start_bit_ok

	btfsc	STATUS,C		;error on i2c comunication?
	goto	uscita_interrupt	;yes
	
	movf	i2c_data,w
	movwf	INDF
	incf	FSR,f

recive_again_and_store

	incf	nb_data,f								;<-|
	btfsc	nb_data,5		;32 byte already recieved?			   |
	goto	memory_full		;yes						   |
											;  |
	call	send_ack								;  |
	call	start_bit_ok								;  |
											;  |
	btfsc	STATUS,C		;error on i2c comunication?			   |
	goto	maybe_stop		;yes						   |
											;  |
	movf	i2c_data,w								;  |
	movwf	INDF									;  |
	incf	FSR,f									;  |
	goto	recive_again_and_store						;  |
											;  |
memory_full										;  |
											;  |
	decf	nb_data,f	;last increment was wrong then decrement nb_data  <-|

maybe_stop
;XXXXinserire controllo di stop guardando il numero di bit ricevuto

	movlw	start_buffer		;Init FSR register for indirect addressing
	movwf	FSR			;for read i2c data recived

	btfss	flag,istr_lcd		;data o istruction?
	goto	send_istruction

send_data

	movf	INDF,w
	incf	FSR,f

	call	LcdSendData
	decfsz	nb_data,f
	goto	send_data
	goto	uscita_interrupt

send_istruction

	movf	INDF,w
	sublw	0	;toggle switch flag?
	btfsc	STATUS,Z
	goto	ctrl_switch_bit

send_istruction_again

	movf	INDF,w
	incf	FSR,f

	call	LcdSendCommand
	decfsz	nb_data,f
	goto	send_istruction_again
	goto	uscita_interrupt

ctrl_switch_bit

	movlw	2
	subwf	nb_data,w
	btfss	STATUS,Z	;correct number of byte? if Ziclear no

	goto	error_command_switch_detected

read_switch_command

	incf	FSR,f
	movf	INDF,w
	
	sublw	0
	btfsc	STATUS,Z	;Switch off?
	goto	clear_switch_bit;yes

	movf	INDF,w
	sublw	1		;Switch on?
	btfsc	STATUS,Z
	goto	set_switch_bit;yes

error_command_switch_detected
	bsf	STATUS,C	;error occurred
	goto	uscita_interrupt

set_switch_bit		;able to send i2c switch number

	bsf	flag,switch_bit
	bcf	STATUS,C	;no error detect
	goto	uscita_interrupt

clear_switch_bit		;no able to send i2c switch number

	bcf	flag,switch_bit
	bcf	STATUS,C	;no error detect
	goto	uscita_interrupt

;******************************************************************************
;Interrupt subroutines
;******************************************************************************
start_bit_ok
;If start condiction i true than execute this code

	movlw	0x08		;init bit counter for i2c comunication
	movwf	i2c_bit

clr_time_out_and_wait

	clrf	time_out	;try 256 time before declare i2c time out comunication

waiting_for_scl_down

	btfss	i2c,scl	;First scl down true?
	goto	first_scl_down

	decf	time_out,1	;time_out reached?
	btfss	STATUS,Z
	goto	waiting_for_scl_down
	goto	i2c_error

first_scl_down

	clrf	time_out	;try 256 time before declare i2c time out comunication
	
waiting_for_scl_up

	btfsc	i2c,scl
	goto	looking_for_sda_value

	decfsz	time_out,f	;time_out reached?
	goto	waiting_for_scl_up
	goto	i2c_error

looking_for_sda_value

	btfss	i2c,sda	;read sda bit and store in i2c_data register
	goto	c_0
c_1
	bsf	STATUS,C
	rlf	i2c_data,f
	goto	check_i2c_bit
c_0	
	bcf	STATUS,C
	rlf	i2c_data,f

check_i2c_bit
	
	decfsz	i2c_bit,f
	goto	clr_time_out_and_wait

i2c_comunication_ok

	bcf	STATUS,C		;comunication ok
	return

i2c_error

	bsf	STATUS,C
	return

send_ack

	clrf	time_out
	bcf	i2c,sda	;prepare pin sda to send ack
	
recheck_scl_pin

	btfsc	i2c,scl	;sda pin must be wrote when scl = 0
	goto	check_time_out

	bsf	STATUS,RP0	;select bank 1
	bcf	TRISB,sda	;send ack
	bcf	STATUS,RP0	;select bank 0
	
	clrf	time_out

recheck_scl_pin_1

	btfss	i2c,scl
	goto	check_time_out1			

	clrf	time_out

recheck_scl_pin_2

	btfsc	i2c,scl	;if scl=0 the ack bit is sent
	goto	check_time_out2

	bsf	STATUS,RP0	;select bank 1
	bsf	TRISB,sda	;set pin sda as input
	bcf	STATUS,RP0	;select bank 0

	bcf	STATUS,C	; no error
	return

check_time_out

	decfsz	time_out,f
	goto	recheck_scl_pin
	goto	i2c_error	

check_time_out1

	decfsz	time_out,f
	goto	recheck_scl_pin_1
	goto	i2c_error	

check_time_out2

	decfsz	time_out,f
	goto	recheck_scl_pin_2
	goto	i2c_error	

;******************************************************************************
;******************************************************************************
;Inizio della MAIN routine
;******************************************************************************
;******************************************************************************
Start  
	bsf	STATUS,RP0	;Select bank of memory 1
	movlw	00010000B	;Config port A all output - RA4 Input for switch 
	movwf	TRISA		;Sx - Dx

	movlw	11100011B	;Config port B - RB5, 6 & 7 for input switch
	movwf	TRISB		;Rb1= Scl (input) Rb0=Sda (input)
	bcf	STATUS,RP0	;Select bank of memory 0
	movlw	start_buffer	;Init FSR register for indirect addressin
	movwf	FSR		;for storage i2c data recived

;Inizializza il display LCD
	call	LcdInit
	call	LcdClear

     	movlw	'M'			
	call	LcdSendData

    	movlw	'i'			
	call	LcdSendData

    	movlw	'z'			
	call	LcdSendData

      	movlw	'a'			
	call	LcdSendData

     	movlw	'r'			
	call	LcdSendData

    	movlw	'3'			
	call	LcdSendData

      	movlw	'2'			
	call	LcdSendData

	movlw	197			;set display position to 2nd line
	call	LcdSendCommand	;6th character

     	movlw	'I'			
	call	LcdSendData

    	movlw	'2'			
	call	LcdSendData

    	movlw	'c'			
	call	LcdSendData

      	movlw	' '			
	call	LcdSendData

     	movlw	'D'			
	call	LcdSendData

    	movlw	'i'			
	call	LcdSendData

      	movlw	's'			
	call	LcdSendData

      	movlw	'p'			
	call	LcdSendData

     	movlw	'l'			
	call	LcdSendData

    	movlw	'a'			
	call	LcdSendData

      	movlw	'y'			
	call	LcdSendData

	bcf	flag,switch_bit	;can't send the i2c switch value

	clrf	bump
	bcf	flag,button	;clear flag for switch up/down

foreverLoop

	call	init_interrupt

antibump

	btfss	flag,button
	goto	check_switch
	
	decfsz	bump,f		;check for antibump's time
	goto	check_switch	

	bcf	flag,button	;clear flag for switch up/down

check_switch

	btfsc	flag,intflag	;interrupt served?
	goto	foreverLoop

	bsf	STATUS,RP0	;Select bank of memory 1
	bsf	TRISB,6	;set RB6 as input
	bcf	TRISB,5	;set RB5 as output
	bcf	STATUS,RP0	;Select bank of memory 0
	bcf	PORTB,5	;for check sx and up

	btfss	PORTA,4	;Sx down?
	goto	SX_down

	btfss	PORTB,7	;Up down?
	goto	UP_down

	btfss	PORTB,6	;Confirm down?
	goto	CONFIRM_down

	bsf	STATUS,RP0	;Select bank of memory 1
	bsf	TRISB,5	;set RB5 as input
	bcf	TRISB,6	;set RB6 as output
	bcf	STATUS,RP0	;Select bank of memory 0
	bcf	PORTB,6	;for check dx and down

	btfss	PORTA,4	;Dx down?
	goto	DX_down

	btfss	PORTB,7	;Down down?
	goto	DOWN_down
	goto	antibump
	
SX_down

	movlw	2
	movwf	addr
	goto	send_switch_to_i2c	

DX_down

	bsf	STATUS,RP0	;Select bank of memory 1
	bsf	TRISB,6	;set RB6 as input to recheck RB6
	bcf	STATUS,RP0	;Select bank of memory 0

	btfss	PORTB,6	;Confirm down?
	goto	CONFIRM_down

	movlw	4
	movwf	addr
	goto	send_switch_to_i2c	

UP_down

	movlw	8
	movwf	addr
	goto	send_switch_to_i2c	

DOWN_down

	bsf	STATUS,RP0	;Select bank of memory 1
	bsf	TRISB,6	;set RB6 as input to recheck RB6
	bcf	STATUS,RP0	;Select bank of memory 0

	btfss	PORTB,6	;Confirm down?
	goto	CONFIRM_down

	movlw	16
	movwf	addr
	goto	send_switch_to_i2c	

CONFIRM_down

	movlw	32
	movwf	addr

send_switch_to_i2c	

	btfsc	flag,button		;antibump active yet?
	goto	prepare_antibump	;yes

	btfss	flag,switch_bit	;can send the i2c switch value?
	goto	prepare_antibump	;no

	bsf	flag,avr32		;prepare i2c comunication 
	movlw	avr32_addr		;avr32 address
	movwf	addr_slave
	call	write_i2c_data	

prepare_antibump

	clrf	bump		;set antibump to 255 time before release
	bsf	flag,button	;set flag button down
	goto	check_switch	;return to check the switch
	

;**********************************************************************
;Soubroutines Start here
;**********************************************************************
init_interrupt

	bcf	flag,intflag
	movlw	start_buffer		;Init FSR register for indirect addressin
	movwf	FSR			;for storage i2c data recived
	bsf	STATUS,RP0      	;Select bank of memory 1
	bcf	OPTION_REG,INTEDG	;Select INT interrupt on falling edge
	bcf	INTCON,INTF
	bcf	STATUS,RP0      	;Select bank of memory 0
	bsf	INTCON,INTE		;Enable INT interrupt
	bsf	INTCON,GIE		;Enable global interrupt
	return

;**********************************************************************
; I2C routines in master operation
;**********************************************************************
write_i2c_data

	bcf	flag,read	;write the selected location
	goto	init_bus_i2c

read_i2c_data			

	bsf	flag,read	;read the selected locations

init_bus_i2c

	call	i2cstart

	movf	addr_slave,w
	call	i2csend

	btfsc	STATUS,C
	goto	master_error_i2c

	movf	addr,w
	call	i2csend

	btfsc	STATUS,C
	goto	master_error_i2c

	btfsc	flag,read	;Read or write operation?
	goto	read_operation

	btfsc	flag,avr32	;write to avr32?
	goto	for_avr32	;yes

write_operation
	
	movf	start_buffer,w	
	call	i2csend

	btfsc	STATUS,C
	goto	master_error_i2c

for_avr32

	call	i2cstop
	goto	exit_i2c_master

read_operation

	call	i2cstart

	movf	addr_slave,w
	call	i2csend

	btfsc	STATUS,C
	goto	master_error_i2c

	call	i2creceive
	call	i2cnoack

exit_i2c_master

       BANKSEL I2CTRIS
       bsf     I2CTRIS, scl    ; scl as input
       bsf     I2CTRIS, sda    ; sda as input
       BANKSEL i2c

	btfsc	flag,read	;Read or write operation?
	goto	exit_read

exit_write

	movlw	5	;wait for writing data to the eeprom
	call	msDelay

exit_read

	bcf	flag,avr32		;prepare i2c comunication 
	bcf	STATUS,C		;set no error
	return

master_error_i2c

	call	i2cstop
	bcf	flag,avr32		;prepare i2c comunication 
	bsf	STATUS,C	;set error
	return

i2cstart                        ; Send a start on the I2C bus

       BANKSEL I2CTRIS
       bcf     I2CTRIS, sda    ; sda as output
       bcf     I2CTRIS, scl    ; scl as output
       BANKSEL i2c

       bsf     PORTB, sda      ; The start condition on the I2C bus
	bsf     PORTB, scl      ; An high to low transition when scl is high
       call    shortdelay
       bcf     PORTB, sda
       call    shortdelay     
       bcf     PORTB, scl        
       call    shortdelay      ; Leave sda and scl low
       return

i2csend         		; Send a byte over the I2C interface, 	
	movwf   i2c_data             ; return 0x00 if ACK	
	movlw   0x08	  
	movwf   i2c_bit             ; i2c_bit is used as a counter
	BANKSEL I2CTRIS
	bcf     I2CTRIS, sda    ; sda as output
	BANKSEL i2c
icloops 
	bcf     i2c, scl    ; Clock low: change of sda allowed
       rlf     i2c_data,f
	bcf     i2c, sda
	btfsc   STATUS, C       ; Test the carry bit
	bsf     i2c, sda              
	call    shortdelay
	bsf     i2c, scl    ; Clock high
       call    shortdelay
       decfsz  i2c_bit,f
       goto    icloops         ; i2cwaitack follows directly

i2cwaitack    

	bcf     i2c, scl    ; Clock low
	bsf     i2c, sda
       BANKSEL I2CTRIS
       bsf     I2CTRIS, sda	; sda as input
       BANKSEL i2c
	clrf	time_out	;init time out 
       call	shortdelay
       bsf    i2c, scl    	; Clock high
       call   shortdelay
rewait
	btfss  i2c, sda    	; sda low means ack
	goto	i2cackok
	decfsz	time_out,f	;timeout?
	goto	rewait
	
	bsf	STATUS,C	;timeout overflow i2c comunication error
	goto	exiti2csend
i2cackok

	bcf	STATUS,C	;i2c comunication ok

exiti2csend

	BANKSEL I2CTRIS
       bcf     I2CTRIS, sda ; sda as output
       BANKSEL i2c         	; Clock is left low
       bcf     i2c, scl
       call    shortdelay
       return           

; This version of i2cwaitack can be a little bit too fast for some i2c devices
; such as some EEPROM's.
; It may be convenient to introduce a timeout mechanism when waiting for the 
; acknowledge.
; Refer to your device datasheet for more details.

; Questa versione di i2cwaitack puo' essere un po' sbrigativa per certi 
; dispositivi i2c come certe EEPROM.
; Puo' essere conveniente introdurre un meccanismo di timeout ed incrementare 
; l'attesa per l'acknowledge fino ad un tempo limite prefissato.
; Si faccia riferimento al datasheet del dispositivo per maggiori dettagli.

i2creceive      
	clrf    i2c_data            ; Receive a byte over the I2C interface
       movlw   0x08
       movwf   i2c_bit            ; i2c_bit is used as a counter
       BANKSEL I2CTRIS
       bsf     I2CTRIS, sda   ; sda as input 
	BANKSEL i2c
icloopr 
	bcf     i2c, scl   ; Clock low: change of sda allowed
       call    shortdelay
	bsf     i2c, scl   ; Clock high
       call    shortdelay
	bcf     STATUS, C      ; Clear the carry 
       rlf     i2c_data,f
	btfsc   i2c, sda   ; Test the bit being received 
       bsf     i2c_data,0          ; Stock the bit read in i2c_data and rotate
       decfsz  i2c_bit,f
       goto    icloopr
	movf    i2c_data,w        
       bcf     i2c, scl   ; Clock is left low
	call    shortdelay
       return

i2csendack    
        BANKSEL I2CTRIS
        bcf     I2CTRIS, sda   ; sda as output
        BANKSEL i2c
        bcf     i2c, scl   ; Clock low: change of sda allowed
        call    shortdelay
        bcf     i2c, sda   ; sda low means ack
        call    shortdelay
        bsf     i2c, scl   ; Clock high
        call    shortdelay
        bcf     i2c, scl   ; Clock is left low
        return
i2cnoack    
        BANKSEL I2CTRIS
        bcf     I2CTRIS, sda   ; sda as output
        BANKSEL i2c
        bcf     i2c, scl   ; Clock low: change of sda allowed
        call    shortdelay
        bsf     i2c, sda   ; sda high means no ack
        call    shortdelay
        bsf     i2c, scl   ; Clock high
        call    shortdelay
        bcf     i2c, scl    
        ;return                 ; Clock is left low

i2cstop                          ; Send a stop on the I2C bus
        BANKSEL I2CTRIS
        bcf     I2CTRIS, sda     ; sda as output
        BANKSEL i2c
        bcf    i2c, scl
        bcf     i2c,sda     ; The stop condition on the bus I2C
        call    shortdelay
        bsf     i2c, scl     ; A low to high transition when scl is high
        call    shortdelay
        bsf     i2c, sda
        call    shortdelay       ; scl and sda lines are left high

        BANKSEL I2CTRIS	     ; put PIC in slave mode	
        bsf     I2CTRIS, sda     ; sda as input
        bsf     I2CTRIS, scl     ; scl as input
        BANKSEL i2c
        return

shortdelay    			; A short delay ;-)

	nop
       nop
       nop
	nop
	nop
	nop
       nop
       nop
	nop
	nop

	return


;**********************************************************************
; Routine di ritardo
;
; W = Ritardo richiesto in ms (con quarzo a 4MHz)
;**********************************************************************

msDelay
                movwf   msDelayCounter+1
                clrf    msDelayCounter+0

                ; Loop interno da circa 1 ms
msDelayLoop
                nop
                decfsz  msDelayCounter+0,F
                goto    msDelayLoop
                nop

                decfsz  msDelayCounter+1,F
                goto    msDelayLoop

                return

;**********************************************************************
; Inizializza il display LCD
; Questa funzione deve essere chiamata prima di ogni altra funzione
; di gestione dell'LCD
;**********************************************************************

LcdInit
                bcf     PORTB,LCD_E     ;Disabilita l'LCD
                bcf     PORTB,LCD_RS    ;Mette l'LCD in modo comando
                bcf     PORTB,LCD_RW    ;abilita modo scrittura sull'LCD

                movlw   250
                call    msDelay

                ; Invia all'LCD la sequenza di reset

                bsf     PORTA,LCD_DB4
                bsf     PORTA,LCD_DB5
                bcf     PORTA,LCD_DB6
                bcf     PORTA,LCD_DB7

                EN_STROBE

                movlw   2
                call    msDelay

                EN_STROBE

                movlw   2
                call    msDelay

                EN_STROBE

                movlw   2
                call    msDelay

								;--------------------------------

                bcf     PORTA,LCD_DB4
                bsf     PORTA,LCD_DB5
                bcf     PORTA,LCD_DB6
                bcf     PORTA,LCD_DB7

                EN_STROBE

                movlw   2
                call    msDelay

                ;Configura il bus dati a 4 bit

                movlw   0x28
                ;movlw   0x2C
                call    LcdSendCommand

                ;Entry mode set, increment, no shift

                movlw   0x06
                ;movlw   0x09
                call    LcdSendCommand

                ;Display ON, Curson OFF, Blink OFF

                movlw   0x0C
                ;movlw   0x07
                call    LcdSendCommand

                movlw   0x01
                call    LcdSendCommand

                return


;**********************************************************************
; Clear LCD
;**********************************************************************

LcdClear
                movlw   0x01
                call    LcdSendCommand

                movlw   2
                call    msDelay

                ;DD RAM address set 1st digit

;               movlw   0x80
;               call    LcdSendCommand

                return

;**********************************************************************
; Locate cursor on LCD
; W = D7-D4 row, D3-D0 col
;**********************************************************************

LcdLocate
                movwf   tmpLcdRegister+0

                movlw   0x80
                movwf   tmpLcdRegister+1

                movf    tmpLcdRegister+0,W
                andlw   0x0F
                iorwf   tmpLcdRegister+1,F

                btfsc   tmpLcdRegister+0,4
                bsf     tmpLcdRegister+1,6

                movf    tmpLcdRegister+1,W
                call    LcdSendCommand

                return


;**********************************************************************
; Send a data to LCD
;**********************************************************************

LcdSendData

	bsf	PORTB,LCD_RS
	goto	LcdSendByte

;**********************************************************************
; Send a command to LCD
;**********************************************************************

LcdSendCommand
	
	bcf	PORTB,LCD_RS
	goto	LcdSendByte

;**********************************************************************
; Send a byte to LCD by 4 bit data bus
;**********************************************************************

LcdSendByte
                ;Save value to send

                movwf   tmpLcdRegister

                ;Invia i quattro bit piu' significativi

                bcf     PORTA,LCD_DB4
                bcf     PORTA,LCD_DB5
                bcf     PORTA,LCD_DB6
                bcf     PORTA,LCD_DB7

                btfsc   tmpLcdRegister,4
                bsf     PORTA,LCD_DB4
                btfsc   tmpLcdRegister,5
                bsf     PORTA,LCD_DB5
                btfsc   tmpLcdRegister,6
                bsf     PORTA,LCD_DB6
                btfsc   tmpLcdRegister,7
                bsf     PORTA,LCD_DB7

                EN_STROBE

                movlw   2
                call    msDelay

                ;Send lower four bits

                bcf     PORTA,LCD_DB4
                bcf     PORTA,LCD_DB5
                bcf     PORTA,LCD_DB6
                bcf     PORTA,LCD_DB7

                btfsc   tmpLcdRegister,0
                bsf     PORTA,LCD_DB4
                btfsc   tmpLcdRegister,1
                bsf     PORTA,LCD_DB5
                btfsc   tmpLcdRegister,2
                bsf     PORTA,LCD_DB6
                btfsc   tmpLcdRegister,3
                bsf     PORTA,LCD_DB7

                EN_STROBE

                movlw   2
                call    msDelay

                return
;**********************************************************************
;Soubroutines End here
;**********************************************************************


                END
