;***********************************************************
; Programma editato da Antonio Cingolani per Simplemachines
; per la scheda Display_module del Progetto Mizar32
; Display_module_beta_1.1.asm V. 1.1
; Pic 16f84 quarzo da 3,68 Mhz
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
general		equ	0x10	;general pourpose register
time_out		equ	0x11	;Counter register for time out i2c comunications

flag			equ	0x12	;|Register used for flag bit allocation
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
addr_slave		equ	0x16	;the phisical address to identify the slave
nb_data		equ	0x17	;data number's to write (o read) in I2C operation
switch			equ	0x18	;here are saved the last switch pressed
lcd_addr_count	equ	0x19	;here are saved the last address counter reading from LCD

start_buffer		equ	0x2c	;Start buffer for i2c comunication
					;buffer deep is 0x2c-0x4f, then we have 36 byte
					;reserved

avr32_addr		equ	b'11110010'	;Avr32 phisical address

;i2c_addr_read	equ	0xaf	;24lc32 hardware address for read operation
;i2c_addr_write	equ	0xae	;24lc32 hardware address for write operation

;*******************************************************************************
;Memory address to write on LCD
;*******************************************************************************


     ;Start program

	ORG	0x0000
	goto	Start

	org	0x0004	;Start interrupt routine

Interrupt_routine
;If we are here, sda pin is gone low. Look for scl pin; If scl=1 a start 
;condiction is TRUE

	bcf	STATUS,RP0      		;Select bank of memory 0
	bcf	INTCON,GIE			;Disable all interrupt
	bsf	flag,intflag			;interrupt occurred
	
	btfss	PORTB,scl
	goto	uscita_interrupt		;If no start condiction go out
	call	start_bit_ok	

	btfsc	STATUS,C			;error on i2c comunication?
	goto	uscita_interrupt		;yes

	movlw	0xF2
	subwf	i2c_data,w
	btfss	STATUS,C
	goto	uscita_interrupt

	call	send_ack

	movlw	0xF6				;command?
	subwf	i2c_data,0
	
	btfsc	STATUS,Z		
	goto	command_recive_routine	;yes... command

	movlw	0xF2				;data?
	subwf	i2c_data,0
	
	btfsc	STATUS,Z		
	goto	data_recive_routine		;yes... data

	movlw	0xF7				;request switch condition?
	subwf	i2c_data,0
	
	btfsc	STATUS,Z		
	goto	request_switch_condition	;yes... request switch condition

	movlw	0xF3				;request for 'Read Busy Flag and Address'?
	subwf	i2c_data,0
	
	btfsc	STATUS,Z		
	goto	read_busy_flag		;yes... request for 'Read Busy Flag and Address'?

uscita_interrupt				

	bcf	INTCON,INTF
	retfie	

command_recive_routine			;0xF6 = Command

	bcf	flag,istr_lcd
	goto	recive_and_store

data_recive_routine				;0xF2 = Data

	bsf	flag,istr_lcd

recive_and_store

	clrf	nb_data
	call	start_bit_ok_but_scl_low

	btfsc	STATUS,C		;error on i2c comunication?
	goto	uscita_interrupt	;yes
	
	movf	i2c_data,w
	movwf	INDF
	incf	FSR,f

recive_again_and_store

	incf	nb_data,f								;<-|
	btfsc	nb_data,5		;32 byte already recieved?			;  |
	goto	memory_full		;yes						;  |
											;  |
	call	send_ack_without_clk_down						;  |
	call	start_bit_ok								;  |
											;  |
	btfsc	STATUS,C		;error on i2c comunication?			;  |
	goto	maybe_stop		;yes						;  |
											;  |
	movf	i2c_data,w								;  |
	movwf	INDF									;  |
	incf	FSR,f									;  |
	goto	recive_again_and_store						;  |
											;  |
memory_full										;  |
											;  |
	decf	nb_data,f	;last increment was wrong then decrement nb_data ;<-|

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
	incf	FSR,f

	call	LcdSendCommand
	decfsz	nb_data,f
	goto	send_istruction
	goto	uscita_interrupt

request_switch_condition		;0xF7

	bcf	STATUS,RP0	;Select bank of memory 0

       movlw    0   ; no buttons pressed yet...
      ; Check SELECT switch with all lines as inputs
       btfss  PORTB,6
       iorlw  32
      ; Check RIGHT and DOWN by setting RB6 as a low output
	bcf	PORTB,6   
	bsf    STATUS,RP0 ; access tris
	bcf    TRISB,6
	bcf    STATUS,RP0 ; access ports
       btfss  PORTA,4  ; RIGHT?
       iorlw  4
       btfss  PORTB,7 ; DOWN?
       iorlw  16
       ; Check LEFT and UP by setting RB5 as a low output
       bcf	PORTB,5
	bsf    STATUS,RP0 ; access tris
       bsf    TRISB,6         ; RB6 is an input again
       bcf    TRISB,5         ; RB5 is a low output
       bcf    STATUS,RP0 ; access ports
       btfss  PORTA,4       ; LEFT?
       iorlw  2
       btfss  PORTB,7       ; UP?
       iorlw  8
       ; Set RB5 back to an input
       bsf    STATUS,RP0
       bsf    TRISB,5
       bcf    STATUS,RP0

send_switch_to_i2c	

	movwf	i2c_data
	call	send_value_on_i2c	
	goto	uscita_interrupt

read_busy_flag			;0xF3

	call	send_ack
here1
	goto	here1
	goto	uscita_interrupt







;XXXX
send_value_on_i2c

	bsf	i2c,sda	;Put SDA line high and output
	bsf	STATUS,RP0	;select bank 1
	bcf	TRISB,sda	;SDA as output
	bcf	STATUS,RP0	;select bank 0
	goto	send_bit_scl_is_low

waiting_for_send

	clrf	time_out	;try 256 time before declare i2c time out comunication

waiting_for_first_send

	btfss	i2c,scl	;First scl down?
	goto	send_bit	;True

	decf	time_out,f	;time_out reached?
	btfss	STATUS,Z
	goto	waiting_for_first_send
	goto	i2c_error

send_bit

	bcf	i2c,scl	;Put SCL line low for clock stretching
	bsf	STATUS,RP0	;select bank 1
	bcf	TRISB,scl	;Hold low scl line
	bcf	STATUS,RP0	;select bank 0

send_bit_scl_is_low

	clrf	time_out	;try 256 time before declare i2c time out comunication
	rlf	i2c_data,f
	bcf	i2c,sda
	btfsc  STATUS, C     ; Test the carry bit
	bsf    i2c,sda

	bsf	STATUS,RP0	;select bank 1
	bsf	TRISB,scl	;Relise scl line
	bcf	STATUS,RP0	;select bank 0

	decfsz	i2c_bit,f
	goto	send_waiting_for_scl_up
	goto	end_send_i2c

send_waiting_for_scl_up

	btfsc	i2c,scl
	goto	waiting_for_send

	decfsz	time_out,f	;time_out reached?
	goto	send_waiting_for_scl_up
	goto	i2c_error

end_send_i2c

	clrf	time_out

wait_for_end_i2c

	btfsc	i2c,scl
	goto	waiting_for_low_scl_again

	decfsz	time_out,f	;time_out reached?
	goto	wait_for_end_i2c
	goto	i2c_error

waiting_for_low_scl_again

	btfss	i2c,scl
	goto	relise_sda_line

	decfsz	time_out,f	;time_out reached?
	goto	waiting_for_low_scl_again
	goto	i2c_error

relise_sda_line

	bsf	STATUS,RP0	;select bank 1
	bsf	TRISB,sda	;Relise SDA line
	bcf	STATUS,RP0	;select bank 0

	movlw	0x08		;init bit counter for i2c comunication
	movwf	i2c_bit

	bcf	STATUS,C	;comunication ok
	return

;******************************************************************************
;Interrupt subroutines
;******************************************************************************
start_bit_ok_but_scl_low

	bsf	STATUS,RP0	;select bank 1
	bsf	TRISB,scl	;Relise scl line
	bcf	STATUS,RP0	;select bank 0

start_bit_ok
;If start condiction i true than execute this code

	clrf	time_out	;try 256 time before declare i2c time out comunication

waiting_for_scl_down

	btfss	i2c,scl	;First scl down true?
	goto	first_scl_down

	decfsz	time_out,f	;time_out reached?
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
	goto	start_bit_ok

i2c_comunication_ok

	movlw	0x08		;init bit counter for i2c comunication
	movwf	i2c_bit
	bcf	STATUS,C		;comunication ok
	return

i2c_error

	movlw	0x08		;init bit counter for i2c comunication
	movwf	i2c_bit
	bsf	STATUS,C
	return

send_ack_without_clk_down

	clrf	time_out
	bcf	i2c,sda	;prepare pin sda to send ack
	
recheck_scl_pin1

	btfsc	i2c,scl	;sda pin must be wrote when scl = 0
	goto	check_time_out12

	bsf	STATUS,RP0	;select bank 1
	bcf	TRISB,sda	;send ack
	bcf	STATUS,RP0	;select bank 0
	
	clrf	time_out

recheck_scl_pin_11

	btfss	i2c,scl
	goto	check_time_out11			

	clrf	time_out

recheck_scl_pin_21

	btfsc	i2c,scl	;if scl=0 the ack bit is sent
	goto	check_time_out21

	bsf	STATUS,RP0	;select bank 1
	bsf	TRISB,sda	;set pin sda as input
	bcf	STATUS,RP0	;select bank 0

	bcf	STATUS,C	; no error
	return

check_time_out12

	decfsz	time_out,f
	goto	recheck_scl_pin1
	goto	i2c_error	

check_time_out11

	decfsz	time_out,f
	goto	recheck_scl_pin_11
	goto	i2c_error	

check_time_out21

	decfsz	time_out,f
	goto	recheck_scl_pin_21
	goto	i2c_error	

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

	bcf	i2c,scl	;force scl low for clock stretching
	bsf	STATUS,RP0	;select bank 1
	bsf	TRISB,sda	;set pin sda as input
	bcf	TRISB,scl	;scl as output for clock stretching
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
	movlw	0xEF
	movwf	PORTA		;Init PortA all out and all High for correct
				;start LCD. Only RA4 must be low when in out mode
				;for switch function
;XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
;	movlw	0x1C		;Rb7, RB6 and RB5 must be low when in out mode
				;LCD line (E, R/W, RS) must be High.SDA and SCL must 
				;be low when in out mode
;XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

	movlw	0x1F		;Rb7, RB6 and RB5 must be low when in out mode
				;LCD line (E, R/W, RS) must be High.
	movwf	PORTB		;For init PortB

	bsf	STATUS,RP0	;Select bank of memory 1
	movlw	00010000B	;Config port A all output - RA4 input for switch 
	movwf	TRISA		;

	movlw	11100011B	;RB6 and RB5 for input switch. RB7 out for switch
	movwf	TRISB		;Rb1= Scl (input) Rb0=Sda (input)
	bcf	STATUS,RP0	;Select bank of memory 0

	movlw	start_buffer	;Init FSR register for indirect addressin
	movwf	FSR		;for storage i2c data recived

	call	LcdInit	;Init LCD
	call	LcdClear	;and clear

foreverLoop

	call	init_interrupt

check_for_interrupt

	btfsc	flag,intflag		;interrupt served?
	goto	foreverLoop		;yes
	goto	check_for_interrupt	;no


;**********************************************************************
;Soubroutines Start here
;**********************************************************************
init_interrupt

	movlw	0x08		;init bit counter for i2c comunication
	movwf	i2c_bit	;
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
