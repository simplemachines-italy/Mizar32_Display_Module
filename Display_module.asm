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

INTERRUPT equ 1


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


;Used PIC memory ORG 0x0C

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
is_lcd_command       equ    0x16   ; Bit 0 says whether this was an I2C command
                                   ; messsage.  If 0, it is a data message
is_lcd_read          equ    0x17   ; Bit 0 is the read/write bit that follows
                                   ; the 7-bit slave address
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

if_low    macro    pin
        btfss   i2c,pin     ; skip if high
    endm

if_high   macro    pin
        btfsc   i2c,pin     ; skip if low
    endm


; Loop until a pin (sda/scl) is high/low

; Loop until a pin is high
wait_for_high    macro    pin
 L loop = $
        if_low  pin
            goto loop
    endm

; Loop until a pin is low
wait_for_low    macro    pin
 L loop = $
        if_high  pin
            goto loop
    endm

; Set an open collector output to actively drive a low value.
; This assumes that the PORT bit is already programmed to 0, which should
; be done in Start and never changed.
; We always leave the bank select register pointing at the ports.

drive_low    macro    pin
        select_tris_bank
        bcf     i2ctris,pin
        select_port_bank
    endm

release    macro    pin
        select_tris_bank
        bsf     i2ctris,pin
        select_port_bank
    endm

; Pause the I2C bus and release it again

start_clock_stretching    macro
        drive_low scl
    endm

stop_clock_stretching    macro
        release scl
    endm

; Roll the value on SDA into the least significant bit of a register.
; This can be used to remember boolean flags, like R/W, where you will
; just test bit 0.
; It can also be used to assemble an 8-bit data byte during reception.
; This assumes that the SDA pin is already configured as an input.

roll_sda_into_lsb    macro    location
        wait_for_low scl
        wait_for_high scl
        rrf     i2cport, w      ; roll SDA into C, throw away the rest
        rlf     location, f     ; roll C into the bottom of the register
    endm

; We just received an address or data byte.
; Send the acknowledgement and block the I2C bus while
; It must be held low by the slave (us) within 3.45 us of the falling edge of
; the 8th data clock pulse.
;
; There is some slack time in here which may be useful to decode the addresses
; and prepare for reception or sending of data.

send_acknowledge_and_stretch    macro
        wait_for_low  scl
        drive_low     sda
        wait_for_high scl
        ; keep ACK low for the duration of the high clock pulse
        wait_for_low  scl
        start_clock_stretching
        release       sda
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
; at least 4us after this, which garantees us at least two samples in each
; of these intervals.
; The looping branch that we do if these states last longer than 4.7us
; extends one of the 2-insn intervals to 3 insn.
;
; Sampling them alternately, we look to match the sequence:
; { SDA-hi then SCL-hi } one or more times
; followed by SDA-low
; followed by SCL-high
; This completes the start condition (done by macro wait_for_start)
; The calling code should then wait for SCL-low, then wait for SCL-high
; then sample the first bit of the slave address


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

   else

       org    0x0000

       call   initialize
       goto   wait_for_start

; In the version without an interrupt routine, instead of leaving the
; interrupt routine, we come back here, reinitialise and wait for a START.

uscita_interrupt
       ; Ensure that clock stretching is undone and that SDA is not held low
       bsf    STATUS,RP0           ;select bank 1
       bsf    TRISB,scl            ;Release scl line
       bsf    TRISB,sda            ;Release SDA line too
       bcf    STATUS,RP0           ;select bank 0
       ; reset the dirtied variables
       movlw  8
       movwf  i2c_bit
       ; and wait for another start condition

wait_for_start
        ; SCL sill always be high for 4.7us before a start condition
        ; so we could also check for SCL being high before SDA here,
        ; assuming we always come back here at least 2us before every start
        ; condition.

        if_low  sda
            goto wait_for_start
start_seen_SDA          ; We've seen SDA high
        if_low  scl
            goto wait_for_start
                        ; We've seen SDA high, SCL high
        if_high sda
            goto start_seen_SDA
                        ; We've seen SDA high, SCL high, SDA low
        if_low  scl
            goto wait_for_start
        ; START sequence complete

   endif

       call   start_bit_ok  

       btfsc  STATUS,C                    ;error on i2c communication?
       goto   uscita_interrupt            ;yes

       ; See if this slave address is for us. We react to 7C-7F,
       ; which we can check for by flipping the top bit and seeing
       ; if the result is > FB

       movf   i2c_data,w
       xorlw  0x80
       sublw  0xFB
       btfsc  STATUS,C
       goto   uscita_interrupt

       call   send_ack

       movlw  0x7C                        ;command?
       subwf  i2c_data,0
       btfsc  STATUS,Z             
       goto   command_receive_routine      ;yes... command

       movlw  0x7E                        ;data?
       subwf  i2c_data,0
       btfsc  STATUS,Z             
       goto   data_receive_routine         ;yes... data

       movlw  0x7F                        ;request switch condition?
       subwf  i2c_data,0
       btfsc  STATUS,Z             
       goto   request_switch_condition    ;yes... request switch condition

       movlw  0x7D                        ;request for 'Read Busy Flag and Address'?
       subwf  i2c_data,0
       btfsc  STATUS,Z             
       goto   read_busy_flag              ;yes... request for 'Read Busy Flag and Address'?

       ; The program counter cannot arrive here if the above tests are correct.
       ; However, if we do, just leave anyway.

   ifdef INTERRUPT

uscita_interrupt                          

       ; Oops, it wasn't for us.  Relese the SCL line, put low by send_ack
       bsf    STATUS,RP0           ;select bank 1
       bsf    TRISB,scl            ;Release scl line
       bsf    TRISB,sda            ;Release SDA line too
       bcf    STATUS,RP0           ;select bank 0

       movlw  8
       movwf  i2c_bit
       bcf    INTCON,INTF
       retfie 
   else
       goto uscita_interrupt
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
       call   send_bit_scl_is_low  
       goto   uscita_interrupt

read_busy_flag                     ;0xF3

       call   send_ack
here1
       goto   here1
       goto   uscita_interrupt


;--------------------------------------

waiting_for_send

       clrf   time_out      ;try 256 times before declaring i2c time out

waiting_for_first_send

       btfss  i2c,scl       ;First scl down?
       goto   send_bit      ;True

       decf   time_out,f    ;time_out reached?
       btfss  STATUS,Z
       goto   waiting_for_first_send
       goto   i2c_error

send_bit

       bcf    i2c,scl       ;Put SCL line low for clock stretching
       bsf    STATUS,RP0    ;select bank 1
       bcf    TRISB,scl     ;Hold low scl line
       bcf    STATUS,RP0    ;select bank 0

send_bit_scl_is_low

       clrf   time_out      ;try 256 times before declaring i2c time out
       rlf    i2c_data,f
       bcf    i2c,sda
       bsf    STATUS,RP0           ;select bank 1
       bsf    TRISB,sda            ;Prepare if C = 1
       btfss  STATUS, C            ;Test the carry bit
       bcf    TRISB,sda            ;If C = 0
       bsf    TRISB,scl            ;Release scl line
       bcf    STATUS,RP0           ;select bank 0

       decfsz i2c_bit,f
       goto   send_waiting_for_scl_up
       goto   end_send_i2c

send_waiting_for_scl_up

       btfsc  i2c,scl
       goto   waiting_for_send

       decfsz time_out,f    ;time_out reached?
       goto   send_waiting_for_scl_up
       goto   i2c_error

end_send_i2c

       clrf   time_out

wait_for_end_i2c

       btfsc  i2c,scl
       goto   waiting_for_low_scl_again

       decfsz time_out,f    ;time_out reached?
       goto   wait_for_end_i2c
       goto   i2c_error

waiting_for_low_scl_again

       btfss  i2c,scl
       goto   release_sda_line

       decfsz time_out,f    ;time_out reached?
       goto   waiting_for_low_scl_again
       goto   i2c_error

release_sda_line

       bsf    STATUS,RP0    ;select bank 1
       bsf    TRISB,sda     ;release SDA line
       bcf    STATUS,RP0    ;select bank 0

       movlw  0x08          ;init bit counter for i2c communication
       movwf  i2c_bit

       bcf    STATUS,C      ;communication ok
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
       bcf    i2c,sda       ;prepare pin sda to send ack

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
       bcf    i2c,sda       ;prepare pin sda to send ack
       
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

       bcf    i2c,scl       ;force scl low for clock stretching
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
; 921600 * 0.000039 = 35.9 instructions.
; Overhead: CALL (2), MOVLW(1), MOVWF(1), return(2) = 6 insns. 30/3 = 10

Delay39us
       movlw  .10
       movwf  DelayCounter

Delay39usLoop
       decfsz DelayCounter,f       ; 1 cycle
       goto   Delay39usLoop        ; 2 cycles
              
       return


; Delay for 43 microseconds.
; 921600 * 0.000043 = 39.6288 instructions.
; Overhead: CALL (2), NOP(1) MOVLW(1), MOVWF(1), return(2) = 7 insns. 33/3 = 11
Delay43us
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
              call   LcdSendByte

              ; 1.53ms delay is necessary for commands (0), 1, 2 and 3
              ; 39us delay is necessary for all the other commands
              movf   tmpLcdRegister,w; Recover value of w (the command)
              andlw  11111100B     ; Sets Z if command was 0 to 3
              movlw  2             ; Set delay: does not affect Z
              btfsc  STATUS,Z      ; Long delay if Z is set
              goto   Delay1_53ms   ; tail call to long delay
              goto   Delay39us     ; tail call to short delay

;**********************************************************************
; Send a datum to the LCD
;**********************************************************************

LcdSendData

              bsf    PORTB,LCD_RS
              call   LcdSendByte
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

              return

;**********************************************************************
;Soubroutines End here
;**********************************************************************


              END
