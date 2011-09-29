;***********************************************************
; Program created by Antonio Cingolani for Simplemachines
; for the Display module of the Mizr32 project
;
; Based on Display_module_beta_1.3.asm V. 1.1,
; hacked by Martin Guy in Septempber 2011
;
; The Pic 16f84 clock crystal runs at 3,68 Mhz
;************************************************************

                PROCESSOR       16F84
                RADIX           DEC
                INCLUDE         "p16f84.inc"
                ERRORLEVEL      -302
                                                        __CONFIG        0x3FF1


; if INTERRUPT is defined, we run as as SDA falling edge-triggered interrupt
; routine with the main program doing nothing.  If it is undefined, we run
; everything in the main routine with no interrupts.

;INTERRUPT equ 1

; if STRETCH_ON_SEND is defined, we do clock stretching at a bit-level
; when sending data onto the I2C bus.  STRETCH_ON_SEND can handle higher I2C
; clock rates at present.

STRETCH_ON_SEND equ 1


; The bits of the I/O ports are assigned as follows:
; PORTA
; 7 6 5 4 3 2 1 0
; X X X | | | | |_ LCD_DB4 (Output)
;       | | | |___ LCD_DB5 (Output)
;       | | |_____ LCD_DB6 (Output)
;       | |_______ LCD_DB7 (Output)
;       |_________ Buttons LEFT/RIGHT (OC)
; PORTB
; 7 6 5 4 3 2 1 0
; | | | | | | | |_ SDA (OC)
; | | | | | | |___ SCL (OC)
; | | | | | |_____ LCD_RS (Output: 0=command, 1=data)
; | | | | |_______ LCD_RW (Output: 0=write command/data, 1=read busy flag/data
; | | | |_________ LCD_E  (Output: Strobe high for 250ns to read/write
; | | |___________ Buttons LEFT/UP (OC)
; | |_____________ Buttons RIGHT/DOWN/SELECT (OC)
; |_______________ Buttons UP/DOWN (OC)
;
; X = any value
; OC = Open collector output: driven low or left as an input to float high
;
; The OC pins must always have output value 0 and are switched using TRIS bits.
; Attempts to set/clear bits in the ports using bcf/bsf are dangerous because
; they do read-modify-write: if an OC bit is not driven at that moment and
; is floating high, the bit read and written by bcf/bsf is the current input
; value on that pin, which risks making it a high output when code switches it
; to its low open-collector function.
;
; One option is always to set OC pins low before enabling them as outputs.
; Another is to be sure only ever to write 0 into their PORT bits and never
; use bcf/bsf on PORTs.
; The TRIS bits, instead, can be set/cleared using bcf/bcf, since the value
; read from them always reflects the content that was last written to them.

;LCD control lines on PORTB

LCD_RS          equ     2       ;Register Select on portb
LCD_RW          equ     3       ;Read o write selection on portb
LCD_E           equ     4       ;Lcd Enable on port portb

;LCD data lines on PORTA

LCD_DB4         equ     0       ;LCD data line DB4
LCD_DB5         equ     1       ;LCD data line DB5
LCD_DB6         equ     2       ;LCD data line DB6
LCD_DB7         equ     3       ;LCD data line DB7

sda           equ    0      ;PortB bit 0 is sda pin
scl           equ    1      ;PortB bit 1 is scl pin
i2c           equ    PORTB  ; 

; Bit values returned by the button-reading code
button_LEFT   equ    2
button_RIGHT  equ    4
button_UP     equ    8
button_DOWN   equ    16
button_SELECT equ    32


; PIC memory registers, from 0x0C

tmpLcdRegister       equ    0x0c   ;Two locations reserved for LCD register
DelayCounter         equ    0x0e   ;Two locations reserved for delay register
time_out             equ    0x11   ;Counter register for time out i2c communications

flag                 equ    0x12   ;Register used for flag bit allocation
;Bits in flag register:
istr_lcd             equ    3      ;0 = instruction to send to display
data_lcd             equ    3      ;1 = data to send to dislpay
intflag              equ    4      ;0 = no interrupt occurred
                                   ;1 = an interrupt occurred
;------------------------------------------------------------------------------------------|

i2c_data             equ    0x13   ;Save location for i2c data byte.
i2c_bit              equ    0x14   ;bit counter in i2c byte transfer
nb_data              equ    0x15   ;Number of bytes read from I2C into buffer
                                   ;and number of bytes left to write to LCD
command_is_lcd_data  equ    0x16   ; Bit 0 says whether this was an I2C data
                                   ; message.  If 1, it was a data message;
				   ; If 0 it was a command message.
command_is_i2c_read  equ    0x17   ; Bit 0 is the read/write bit that follows
                                   ; the 7-bit slave address, 1 for read.
start_buffer         equ    0x2c   ;Start of buffer for i2c communication.
                                   ;buffer depth is 0x2c-0x4f, so we have 36
                                   ;bytes maximum (32 are used)

;*******************************************************************************
; Macros to make code more readable
;*******************************************************************************

; An abbreviation for labels that are local to a macro definition
#define L local


; PIC register bank selectors, to access the PORT or the TRIS registers.

select_port_bank    macro
        bcf     STATUS,RP0      ;select bank 0 for the PORT registers
    endm

select_tris_bank    macro
        bsf     STATUS,RP0      ;select bank 1 for the TRIS registers
    endm


; conditionally execute the following single instruction

if_low    macro    addr,pin
        btfss   addr,pin     ; skip if high
    endm

if_high   macro    addr,pin
        btfsc   addr,pin     ; skip if low
    endm


; Loop until a pin (sda/scl) is high/low

; Loop until a pin is high
wait_for_high    macro    addr,pin
 L loop = $
        if_low  addr,pin
            goto loop
    endm

; Loop until a pin is low
wait_for_low    macro    addr,pin
 L loop = $
        if_high  addr,pin
            goto loop
    endm

; Set an open collector output to actively drive a low value.
; This assumes that the PORT bit is already programmed to 0, which should
; be done in Start and never changed.
; We always leave the bank select register pointing at the ports.

drive_low    macro    addr,pin
        select_tris_bank
        bcf     addr,pin
        select_port_bank
    endm

release    macro    addr,pin
        select_tris_bank
        bsf     addr,pin
        select_port_bank
    endm


; Stretch the clock, and stop doing so.

stretch    macro
        drive_low i2c,scl
    endm

unstretch    macro
        release   i2c,scl
    endm


; In the I2C spec, the minimum time between SDA falling and SCL falling in a
; START is 4us, which is slightly longer that the maximum interrupt latency of
; 4 instruction cycles at 921600 ips.
;
; There are several options to get the best performance:
;
; Option 1)
; Do everything in an interrupt routine triggered by falling SDA edge.
;
; We can check SCL in the first instruction of the interrupt routine.
; The pin is sampled on the rising edge of the second oscillator cycle
; of that instruction, which results in a maximum latency from SDA down
; to checking SCL is high of 4 insns * 4 Tosc + Tosc = 17/3686400 seconds
; which is 4.6115 us. This is outside the standard-speed I2C spec of 4us.
; It also reacts to every SDA high-low transition in the data and generates
; a lot of false STARTs because it only checks that SCL is high after the
; falling edge of SDA, not that both SDA and SCL were high in the preceding
; 4.7us.  There are almost certainly data sequences for other devices that
; will match any addresses we may choose to use.
;
; One way to make this work within the I2C spec is to resign ourselves to not
; always seeing the SCL high condition, and also allow an immediate SCL low,
; which would be in the low clock period preceding the first data bit. This
; extends the possible interrupt latency to 8.7us, which is OK for us.
; However, this is even more prone to detecting false START conditions in
; other devices' data.
;
; Option 2)
; Do everything in a main loop and busy-wait to detect the START condition
;
; There are two ways to sample SDA and SCL.
; The one with the highest sampling rate is to sample each one alternately
; using a sequence of btfsc-goto pairs.  If the matching sequence for a
; START is a linear sequence of code, not taking the branches, this samples
; one pin every 2 instructions. At 3.686Mhz we have 921600 ips, each insn
; takes 1.085us and our sampling rate is 2.17us.
; SDA is guaranteed high for 4.7us before it goes low and SCL is high for
; at least 4us after this, which guarantees us at least two samples in each
; of these intervals.
; If one of these states lasts longer than 4.7us, the looping branch
; extends one of the 2-insn intervals to 3 insns.


;******************************************************************************
; Main code section starts
;******************************************************************************

; In interrupt mode, the main program just initialises then loops forever
; and all work is doen in the SDA falling edge interrupt routine.
; In non-interrupt mode we busy-wait for the START condition, do stuff and loop.

    ifdef INTERRUPT

       org    0x0000   ; Main program
       
       call   initialize
       call   init_interrupt
       goto   $                ; do nothing forever


       org    0x0004   ; Interrupt routine

       ; If we are here, sda pin has gone low. Look for scl pin;
       ; If scl=1 a start condition is TRUE

       btfss  PORTB,scl
       goto   uscita_interrupt            ;If no start condition go out

       ; this is the time-critical branch which continues at match_address

   else

       org    0x0000

       call   initialize
       goto   wait_for_start

; In the version without an interrupt routine, instead of leaving the
; interrupt routine, we come back here, reinitialise and wait for a START.

uscita_interrupt
       ; Reset the dirtied variables
       movlw  0x08
       movwf  i2c_bit

       ; Ensure that clock stretching is undone and that SDA is not held low
       bsf    STATUS,RP0           ;select bank 1
       bsf    TRISB,sda            ;Release SDA line
       bsf    TRISB,scl            ;Release SCL line
       bcf    STATUS,RP0           ;select bank 0

       ; and wait for another start condition

wait_for_start
        ; SCL will always be high for 4.7us before a start condition
        ; so we could also check for SCL being high before SDA here,
        ; assuming we always come back here at least 2us before every start
        ; condition.

	; Sampling SCL and SDA alternately, we look to match the sequence:
	; { SDA-hi then SCL-hi } one or more times
	; followed by SDA-low
	; followed by SCL-high

        if_low  i2c,sda
            goto wait_for_start
start_seen_SDA          ; We've seen SDA high
        if_low  i2c,scl
            goto wait_for_start
                        ; We've seen SDA high, SCL high
        if_high i2c,sda
            goto start_seen_SDA
                        ; We've seen SDA high, SCL high, SDA low
        if_low  i2c,scl
            goto wait_for_start
        ; START sequence complete.

	; The timing at this point is between 2 and 4 instruction cycles
	; after the falling edge of SDA.
	; SCL may still be high;
	; in the worst timing case, SCL went low 0.34us ago

   endif

; After a start bit, instead of reading a bytes of slave address and thinking
; about it, we do address-matching on each bit as they are received.
; This lets us react at 100kHz instead of 15kHz.

; match-address macro:
; Reads a data bit from the I2C stream and compares it against the "bit"th
; bit in the given slave address. If it doesn't match, go wait for another start.
; While matching, this takes 2 instruction cycles after the clock goes high.

match_address_bit    macro    addr, bit
        wait_for_low   i2c,scl
        wait_for_high  i2c,scl
        if (addr & (1 << bit))
            ; continue if the bit is one
            btfss   i2c,sda
        else
            ; (addr & (1<<bit)) == 0, so sda should be zero too.
            ; Continue if the bit is zero
            btfsc   i2c,sda
        endif
     ifdef INTERRUPT
	goto rti
     else
        goto wait_for_start
     endif
   endm

; Roll the value on SDA into the least significant bit of a register.
; This can be used to remember boolean flags, like R/W, where you will
; just test bit 0.
; It can also be used to assemble an 8-bit data byte during reception.
; This assumes that the SDA pin is already configured as an input.

roll_sda_into_lsb    macro    location
        wait_for_low  i2c,scl
        wait_for_high i2c,scl
        rrf     i2c,w           ; roll SDA into C, throw away the rest
        rlf     location,f      ; roll C into the bottom of the register
    endm

; Send the acknowledgement and block the I2C bus while we process it.
; To successfully stretch the clock, the slave must hold SCL low within
; 3.45us of the falling edge of the 8th data clock pulse.
;
; There is some slack time here.

send_acknowledge_and_stretch    macro
        wait_for_low  i2c,scl
        drive_low     i2c,sda
        wait_for_high i2c,scl
        ; keep ACK low for the entire duration of the high clock pulse
        wait_for_low  i2c,scl

        select_tris_bank
        bcf     i2c,scl		; stretch the clock
        bsf     i2c,sda		; release SDA
        select_port_bank
   endm

our_address equ 0x7C		; The first of our four 8-bit slave addresses

match_address
        ; The top 6 bits of the slave address should match out address;
        ; These are followed by whether this is a command or data and
        ; whether it is an I2C read or write command.
        match_address_bit  our_address, 7
        match_address_bit  our_address, 6
        match_address_bit  our_address, 5
        match_address_bit  our_address, 4
        match_address_bit  our_address, 3
        match_address_bit  our_address, 2
        ; save the command/data bit of the slave address and the R/W flag
        roll_sda_into_lsb  command_is_lcd_data
        roll_sda_into_lsb  command_is_i2c_read
        send_acknowledge_and_stretch

	; See whether we should read the I2C bus or write to it
	if_high  command_is_i2c_read,0
	    goto  i2c_read_commands
	
	; Commands to receive data from i2c and do something with it
	if_high  command_is_lcd_data,0
	    goto  data_receive_routine
        goto  command_receive_routine

i2c_read_commands
	if_high  command_is_lcd_data,0
	   goto  request_switch_condition
        goto  read_busy_flag

   ifdef INTERRUPT

uscita_interrupt                          

       ; reinitialize the static variables
       movlw  8
       movwf  i2c_bit

       ; Release the SCL line, put low by send_ack
       bsf    STATUS,RP0           ;select bank 1
       bsf    TRISB,sda            ;Release SDA line too
       bsf    TRISB,scl            ;Release scl line
       bcf    STATUS,RP0           ;select bank 0

rti			; fast exit, for when we have modified nothing
       bcf    INTCON,INTF
       retfie 

   endif

command_receive_routine                    ;0xF6 = Command

       bcf    flag,istr_lcd
       goto   receive_and_store

data_receive_routine                       ;0xF2 = Data

       bsf    flag,istr_lcd

receive_and_store

       movlw  start_buffer  ;Init FSR register for indirect addressing
       movwf  FSR           ;for storage of i2c data received
       clrf   nb_data       ;No bytes have been received yet...

       call   start_bit_ok_but_scl_low

       btfsc  STATUS,C             ;error on i2c communication?
       goto   uscita_interrupt     ;yes
       
       movf   i2c_data,w
       movwf  INDF
       incf   FSR,f

receive_again_and_store

       incf   nb_data,f
       btfsc  nb_data,5            ;32 byte already received?
       goto   memory_full          ;yes

       call   send_ack_without_clk_down
       call   start_bit_ok

       btfsc  STATUS,C             ;error on i2c communication?
       goto   maybe_stop           ;yes

       movf   i2c_data,w
       movwf  INDF
       incf   FSR,f
       goto   receive_again_and_store

memory_full

       decf   nb_data,f     ;last increment was wrong so decrement nb_data

maybe_stop
;XXXXinserire controllo di stop guardando il numero di bit ricevuto

       movlw  start_buffer         ;Init FSR register for indirect addressing
       movwf  FSR                  ;for read i2c data received

       btfss  flag,istr_lcd        ;data or istruction?
       goto   send_istruction

send_data

       movf   INDF,w
       incf   FSR,f

       call   LcdSendData
       decfsz nb_data,f
       goto   send_data
       goto   uscita_interrupt

send_istruction

       movf   INDF,w
       incf   FSR,f

       call   LcdSendCommand
       decfsz nb_data,f
       goto   send_istruction
       goto   uscita_interrupt


;-------------------------------------------------------------------------------
; request_switch_condition subroutine
; reads the buttons and sends their current status to the I2C master as a byte
; with the bottom 5 bits each set or clear, depending whether each button is
; held.  This works for up to two of the L R U D buttons with or without Select
; while three of the four held reports all four held.
;-------------------------------------------------------------------------------

; The buttons are connected as follows
;
;          RB5
;          / \
;         L   U
;        /     \
;      RA4     RB7
;        \     /
;         R   D
;          \ /
;          RB6
;           |
;           S
;           |
;          GND
;
; where R[AB]X are port pins and L R U D S are the buttons.
; There is a 10K pull-up resistor on the RB pins.
;
; SELECT is easy to test: set all pins as (high) inputs and test if RB6 is low.
;
; We can also detect any combination of two of L R U and D being held,
; whether or not SELECT is held or not:
;
; RIGHT and DOWN can be checked by setting RB6 as a low output and testing
; RA4 and RB7.  If SELECT is also pressed, this makes no difference as we are
; already holding RB6 low.
;
; LEFT and UP are more tricky because if SELECT and RIGHT are held,
; RA4 will be low anyway. So we can hold RA4 low and test RB5,
; then hold RB7 low and test RB5.
; However, this gives a false positive on LEFT if S, D and U are held
; and a similar false U if S L and R are held. We get round this
; by testing L and U "in both directions".
;
; The only case we cannot detect is if three of L u R and D are held
; since this short-circuits the fourth button at an electrical level,
; so it looks like all four buttons are held.

request_switch_condition           ;0xF7

       bcf    STATUS,RP0    ;Select bank of memory 0
       clrf   PORTB         ; Ensure all RB lines will output 0
       clrf   PORTA         ; Ensure RA4 will output 0.

       ; TRIS bits should already be set with all four lines an inputs.

       movlw    0   ; no buttons pressed yet...

       ; Check SELECT switch with all lines as inputs
       btfss  PORTB,6
       iorlw  button_SELECT

       ; Check RIGHT and DOWN by setting RB6 as a low output
       bsf    STATUS,RP0 ; access tris
       bcf    TRISB,6
       bcf    STATUS,RP0 ; access ports
       btfss  PORTA,4  ; RIGHT?
       iorlw  button_RIGHT
       btfss  PORTB,7 ; DOWN?
       iorlw  button_DOWN

       ; Check LEFT by setting RA4 as a low output and testing RB5
       bsf    STATUS,RP0 ; access tris
       bsf    TRISB,6         ; RB6 is an input again
       bcf    TRISA,4         ; RA4 is a low output
       bcf    STATUS,RP0 ; access ports
       btfsc  PORTB,5        ; LEFT?
       goto   left_button_not_held
       ; now check the other way
       bsf    STATUS,RP0 ; access tris
       bsf    TRISA,4         ; RA4 is an input again
       bcf    TRISB,5         ; RB5 is a low output
       bcf    STATUS,RP0 ; access ports
       btfss  PORTA,4        ; LEFT?
       iorlw  button_LEFT
left_button_not_held

       ; Check UP by setting RB7 low and testing RB5
       bsf    STATUS,RP0 ; access tris
       bsf    TRISA,4         ; RA4 is an input again
       bsf    TRISB,5         ; RB5 is an input again
       bcf    TRISB,7         ; RB7 is a low output
       bcf    STATUS,RP0 ; access ports
       btfsc  PORTB,5        ; UP?
       goto up_button_not_held
       ; now check the other way
       bsf    STATUS,RP0 ; access tris
       bsf    TRISB,7         ; RB7 is an input again
       bcf    TRISB,5         ; RB5 is a low output
       bcf    STATUS,RP0 ; access ports
       btfss  PORTB,7        ; UP?
       iorlw  button_UP
up_button_not_held

       ; Set UP pins back to inputs
       bsf    STATUS,RP0
       bsf    TRISB,7
       bsf    TRISB,5
       bcf    STATUS,RP0

send_switch_to_i2c   

       movwf  i2c_data
       call   i2c_send_byte
       goto   uscita_interrupt

read_busy_flag                     ;0xF3

       call   send_ack
here1
       goto   here1
       goto   uscita_interrupt


;--------------------------------------

; i2c_send_byte subroutine: send a byte of data to the master.
;
; The byte is in register i2c_data and the clock is already held low
; after the acknowledge bit we sent for our slave address.
; I2C data is transmitted with the most significant bit first.

i2c_send_byte

   ; We have two algorithms for sending data. One stretches the clock at
   ; every bit; the other just runs as fast as it can.
   ; Neither succeeds in respecting the I2C spec for a 100kHz system,
   ; but the clock-stretching one is more successful.

   ifdef STRETCH_ON_SEND

   ; This version stretches the low half of the clock on every clock pulse
   ; The critical times is from releasing the clock to holding it again,
   ; If the clock goes high when released, this happens 125ns after the end
   ; of the "bsf scl" insn. We detect that it is high within 2 insns (2us),
   ; which is 2.17us, well within the the minimum of 4us.
   ;
   ; We then have to hold the clock low within 4.7us of the clock going low.
   ; For this, the worst case is when we sample the last moment of the high part
   ; of the clock. It then takes us 3 insn cycles to sample it again, and
   ;   bsf scl; nop; bsf RP0; bcf scl;
   ; to get it low again, which is
   ; 6.75 * 1.085us + 125ns = 7.45us, much longer than the ~ 4.7us we have.
   ; (I can't find a figure to say what the setup time is for a clock stretch)

send_another_bit

       ; output a bit
       bsf    STATUS,RP0           ;select bank 1
       btfsc  i2c_data,7           ; See if data bit to output is 1
       bsf    TRISB,sda            ; Go open-collector for high output if so
       btfss  i2c_data,7           ; See if data bit to output is 0
       bcf    TRISB,sda            ; Output low if so
       bsf    TRISB,scl            ; Release the SCL line
       bcf    STATUS,RP0           ;select bank 0

       wait_for_high  i2c,scl
       wait_for_low   i2c,scl

       ; hold the clock again
       bsf    STATUS,RP0           ;select bank 1
       bcf    TRISB,scl            ; Hold the SCL line again
       bcf    STATUS,RP0           ;select bank 0

       rlf    i2c_data,f	   ; discard tha bit
       decfsz i2c_bit,f		   ; decrease bit count
       goto   send_another_bit

   else

   ; This version just runs as fast as it can, hoping to keep up with the clock.
   ; The critical time is from detecting SCL low to getting SDA high or low.
   ; SCL is sampled on the falling edge of the first Fosc clock pulse of the
   ; first insn of wait_for_low, so the delay is:
   ;   3/4 of btfsx (high SDA); goto taken (2 insn cycles); btfsx (low SDA);
   ;   nop(goto); bsf STATUS; bsf TRISB; btfss i2c_data; bcf TRISB
   ; and the output is valid 125ns after the first rising edge of the next insn.
   ; Total delay is 8.75*1.085us + 125ns = 9.62us,  which is twice the maximum 
   ; of t(LOW)- t(SU:DAT) = 4.7us - 250ns = 4.45us.
  

       ; The first bit is different because we have to release SCL when we
       ; have set the right value on SDA.  For the other 7 bits we just run
       ; as fast as we can. This takes 6 or 7 instructions maximum from
       ; detecting SCL low to getting the data on SDA.

       ; output bit 7
       bsf    STATUS,RP0           ;select bank 1
       btfsc  i2c_data,7           ; See if data bit to output is 1
       bsf    TRISB,sda            ; Go open-collector for high output if so
       btfss  i2c_data,7           ; See if data bit to output is 0
       bcf    TRISB,sda            ; Output low if so
       bsf    TRISB,scl            ; Release the SCL line
       bcf    STATUS,RP0           ;select bank 0
       
       wait_for_high  i2c,scl
       wait_for_low   i2c,scl

i2c_send_bit    macro   bit        
       bsf    STATUS,RP0           ;select bank 1
       bsf    TRISB,sda            ; Go open-collector for high output
       btfss  i2c_data,bit         ; See if data bit to output is 0
       bcf    TRISB,sda            ; Output low if so
       bcf    STATUS,RP0           ;select bank 0
       wait_for_high  i2c,scl
       wait_for_low   i2c,scl
   endm
       
       i2c_send_bit 6
       i2c_send_bit 5
       i2c_send_bit 4
       i2c_send_bit 3
       i2c_send_bit 2
       i2c_send_bit 1
       i2c_send_bit 0
       
       ; hold the clock again
       bsf    STATUS,RP0           ;select bank 1
       bcf    TRISB,scl            ; Hold the SCL line again
       bcf    STATUS,RP0           ;select bank 0

   endif

       bcf    STATUS,C      ;communication ok

       ; We only ever send one byte, so we don't care about the master's
       ; ACK bit or its STOP condition or anything like that.

       return


;******************************************************************************
;Interrupt subroutines
;******************************************************************************
start_bit_ok_but_scl_low

       bsf    STATUS,RP0    ;select bank 1
       bsf    TRISB,scl     ;release scl line
       bcf    STATUS,RP0    ;select bank 0

start_bit_ok
;If start condition is true than execute this code

       clrf   time_out      ;try 256 time before declaring i2c time out

waiting_for_scl_down

       btfss  i2c,scl       ;First scl down true?
       goto   first_scl_down

       decfsz time_out,f    ;time_out reached?
       goto   waiting_for_scl_down
       goto   i2c_error

first_scl_down

       clrf   time_out      ;try 256 times before declaring i2c time out
       
waiting_for_scl_up

       btfsc  i2c,scl
       goto   looking_for_sda_value

       decfsz time_out,f    ;time_out reached?
       goto   waiting_for_scl_up
       goto   i2c_error

looking_for_sda_value

       ; SDA is bit 0 of PORTB, so just roll it in via the carry flag.
       rrf    i2c,w         ;put bit 0 into C
       rlf    i2c_data,f    ;put C into bit 0
       decfsz i2c_bit,f
       goto   start_bit_ok

i2c_communication_ok

       movlw  0x08          ;init bit counter for i2c communication
       movwf  i2c_bit
       bcf    STATUS,C             ;communication ok
       return

i2c_error

       movlw  0x08          ;init bit counter for i2c communication
       movwf  i2c_bit
       bsf    STATUS,C
       return

send_ack_without_clk_down

       clrf   time_out

recheck_scl_pin1

       btfsc  i2c,scl       ;sda pin must be written when scl = 0
       goto   check_time_out12

       bsf    STATUS,RP0    ;select bank 1
       bcf    TRISB,sda     ;send ack
       bcf    STATUS,RP0    ;select bank 0
       
       clrf   time_out

recheck_scl_pin_11

       btfss  i2c,scl
       goto   check_time_out11                   

       clrf   time_out

recheck_scl_pin_21

       btfsc  i2c,scl       ;if scl=0 the ack bit is sent
       goto   check_time_out21

       bsf    STATUS,RP0    ;select bank 1
       bsf    TRISB,sda     ;set pin sda as input
       bcf    STATUS,RP0    ;select bank 0

       bcf    STATUS,C      ; no error
       return

check_time_out12

       decfsz time_out,f
       goto   recheck_scl_pin1
       goto   i2c_error     

check_time_out11

       decfsz time_out,f
       goto   recheck_scl_pin_11
       goto   i2c_error     

check_time_out21

       decfsz time_out,f
       goto   recheck_scl_pin_21
       goto   i2c_error     

send_ack

       clrf   time_out
       
recheck_scl_pin

       btfsc  i2c,scl       ;sda pin must be wrote when scl = 0
       goto   check_time_out

       bsf    STATUS,RP0    ;select bank 1
       bcf    TRISB,sda     ;send ack
       bcf    STATUS,RP0    ;select bank 0
       
       clrf   time_out

recheck_scl_pin_1

       btfss  i2c,scl
       goto   check_time_out1                    

       clrf   time_out

recheck_scl_pin_2

       btfsc  i2c,scl       ;if scl=0 the ack bit is sent
       goto   check_time_out2

       bsf    STATUS,RP0    ;select bank 1
       bsf    TRISB,sda     ;set pin sda as input
       bcf    TRISB,scl     ;scl as output for clock stretching
       bcf    STATUS,RP0    ;select bank 0

       bcf    STATUS,C      ; no error
       return

check_time_out

       decfsz time_out,f
       goto   recheck_scl_pin
       goto   i2c_error     

check_time_out1

       decfsz time_out,f
       goto   recheck_scl_pin_1
       goto   i2c_error     

check_time_out2

       decfsz time_out,f
       goto   recheck_scl_pin_2
       goto   i2c_error     

;******************************************************************************
;******************************************************************************
;Initialize the hardware and our variables
;******************************************************************************
;******************************************************************************

initialize

       movlw  0xEF
       movwf  PORTA         ;Init PortA all out and all High for correct
                            ;start LCD. Only RA4 must be low when in out mode
                            ;for switch function
;XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
;      movlw  0x1C          ;Rb7, RB6 and RB5 must be low when in out mode
                            ;LCD line (E, R/W, RS) must be High.SDA and SCL must 
                            ;be low when in out mode
;XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

       movlw  0x1F          ;Rb7, RB6 and RB5 must be low when in out mode
                            ;LCD line (E, R/W, RS) must be High.
       movwf  PORTB         ;For init PortB

       bsf    STATUS,RP0    ;Select bank of memory 1
       movlw  00010000B     ;Config port A all output - RA4 input for switch 
       movwf  TRISA         ;

       movlw  11100011B     ;RB6 and RB5 for input switch. RB7 out for switch
       movwf  TRISB         ;Rb1= Scl (input) Rb0=Sda (input)
       bcf    STATUS,RP0    ;Select bank of memory 0

       call   LcdInit       ;Init LCD
       call   LcdClear      ;and clear

       ; Pre-initialize the registers used in the interrupt routine
       ; so that it can respond more quickly

       movlw  0x08          ;init bit counter for i2c communication
       movwf  i2c_bit

       return


;**********************************************************************
;Soubroutines Start here
;**********************************************************************
init_interrupt

       bsf    STATUS,RP0           ;Select bank of memory 1
       bcf    OPTION_REG,INTEDG    ;Select INT interrupt on falling edge
       bcf    INTCON,INTF
       bcf    STATUS,RP0           ;Select bank of memory 0
       bsf    INTCON,INTE          ;Enable INT interrupt
       bsf    INTCON,GIE           ;Enable global interrupt
       return

;**********************************************************************
; Delay routines
;**********************************************************************

; msDelay: Delay for W mmilliseconds

; Fosc = 3.6864MHz; instruction time = 4 clock cycles => 921600 instructions/sec
; 1ms = 921.6 insns = 921.6/4 inner loops = 230.4 iterations.
; An extra 4 cycles are taken by the outer loop re-initialization,
; giving a few cycles over 1ms per inner loop.

msDelay
       movwf   DelayCounter+1    ; number of milliseconds to delay for
       movlw   .230              ; cycles of inner loop to get one millisecond

msDelayLoop1
       movwf   DelayCounter+0    ; 1 cycle

       ; Inner loop takes one millisecond
msDelayLoop2
       nop                       ; 1 cycle
       decfsz  DelayCounter+0,F  ; 1 cycle when looping
       goto    msDelayLoop2      ; 2 cycles

       decfsz  DelayCounter+1,F  ; 1 cycle when looping
       goto    msDelayLoop1      ; 2 cycles

       return

; Delay for 39 microseconds.

Delay39us
       ; 921600 * 0.000039 = 35.9 instructions.
       ; Each loop iteration takes 3 cycles and the calling overhead is
       ; CALL (2), MOVLW(1), MOVWF(1), return(2) = 6 insns
       ; so we need to do (36-6)/3 = 10 iterations
       movlw  .10
       movwf  DelayCounter

Delay39usLoop
       decfsz DelayCounter,f       ; 1 cycle
       goto   Delay39usLoop        ; 2 cycles
              
       return


; Delay for 43 microseconds.
Delay43us
       ; 921600 * 0.000043 = 39.6288 instructions.
       ; Each loop iteration takes 3 cycles and the calling overhead is
       ; CALL (2), NOP(1) MOVLW(1), MOVWF(1), return(2) = 7 insns.
       ; (40-7)/3 = 11
       nop
       movlw  .11
       movwf  DelayCounter
Delay43usLoop
       decfsz DelayCounter,f       ; 1 cycle
       goto   Delay43usLoop        ; 2 cycles
              
       return


; Delay for 1.53 milliseconds.
; 921600 * 0.00153 = 1410.048 insns
; 1410/6 = 235.0, with overhead of 6 cycles, we use 234
Delay1_53ms
       movlw  .234
       movwf  DelayCounter
Delay1_53usLoop
       goto   $+1                  ; 2 cycles
       nop                         ; 1 cycle
       decfsz DelayCounter,f         ; 1 cycle
       goto   Delay1_53usLoop      ; 2 cycles
              
       return
              

;**********************************************************************
; Initialize the LCD display
; This function must be called before any other function that drives the LCD
;**********************************************************************

; First, a nice macro to initiate the read/write from/to the LCD.
; As it uses bcf/bsf, it does a read-modify-write of PORTB, which changes
; the output pin value of any pins set as inputs to their current input voltage.
; Make sure PORTB is cleared before returning from routines that use EN_STROBE.

EN_STROBE            MACRO
                bsf  PORTB,LCD_E
                bcf  PORTB,LCD_E
                     ENDM

LcdInit
              ;bcf     PORTB,LCD_E     ;Lower the enable strobe
              ;bcf     PORTB,LCD_RS    ;Put the LCD in command mode
              ;bcf     PORTB,LCD_RW    ;Enable writing mode to the LCD
              clrf    PORTB           ;equivalent

              movlw   30
              call    msDelay

              ; Send the reset sequence to the LCD

              movlw   00000011B
              movwf   PORTA

              EN_STROBE
              call    Delay39us

              EN_STROBE
              call    Delay39us

              EN_STROBE
              call    Delay39us

;--------------------------------

              movlw   00000010B
              movwf   PORTA

              EN_STROBE
              call    Delay39us

              ;Configure the data bus to 4 bits

              movlw   0x28
              call    LcdSendCommand

              ;Entry mode set, increment, no shift

              movlw   0x06
              call    LcdSendCommand

              ;Display ON, Curson OFF, Blink OFF

              movlw   0x0C
              call    LcdSendCommand

              ; Clear display

              movlw   0x01
              call    LcdSendCommand

              ; LcdSendCommand has cleared PORTB for us.

              return


;**********************************************************************
; Clear LCD
;**********************************************************************

LcdClear
              movlw   0x01
              goto    LcdSendCommand    ; tail call

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
; Send a command to LCD
;**********************************************************************

LcdSendCommand

              bcf    PORTB,LCD_RS
              call   LcdSendByte   ; LcdSendByte clears PORTB for us.

              ; 1.53ms delay is necessary for commands (0), 1, 2 and 3
              ; 39us delay is necessary for all the other commands
              movf   tmpLcdRegister,w; Recover value of w (the command)
              andlw  11111100B     ; Sets Z if command was 0 to 3
              btfsc  STATUS,Z      ; Long delay if Z is set
              goto   Delay1_53ms   ; tail call to long delay
              goto   Delay39us     ; tail call to short delay

;**********************************************************************
; Send a datum to the LCD
;**********************************************************************

LcdSendData

              bsf    PORTB,LCD_RS
              call   LcdSendByte   ; LcdSendByte clears PORTB for us.

              ; 43us delay is necessary when writing data
              goto   Delay43us     ; tail call

;**********************************************************************
; Send a byte to LCD by 4 bit data bus
;**********************************************************************

LcdSendByte
              ;Save value to send
              movwf   tmpLcdRegister

              ; Send the four most significant bits
              swapf  tmpLcdRegister,w
              movwf  PORTA
              EN_STROBE

              ; Send the four least significant bits
              movf   tmpLcdRegister,w
              movwf  PORTA
              EN_STROBE

              ; EN_STROBE corrupts the output values of input pins
              ; so reset them to output zero when enabled.
              clrf   PORTB

              return

;**********************************************************************
;Soubroutines End here
;**********************************************************************


              END
