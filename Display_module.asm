;***********************************************************
; Program created by Antonio Cingolani for Simplemachines
; for the Display module of the Mizar32 project
; hacked by Martin Guy
;
; Display_module.asm v1.6, 3 Oct 2011
;
; The Pic 16F84 clock crystal runs at 3,68 Mhz and performs 920000
; instruction cycles per second, or one instruction cycle every 1.087 usec.
;
; For the LCD panel see http://home.comet.bg/datasheets/LCD/AC-162B.pdf
; and http://embeddedtutorial.com/2010/01/interfacing-lcd-with-8051/
; The LCD panel is powered with 5V (this affects its timing characteristics).
;************************************************************

                PROCESSOR       16F84
                RADIX           DEC
                INCLUDE         "p16f84.inc"
                ERRORLEVEL      -302
                __CONFIG        0x3FF1


; Configuration options

; We use four 8-bit slave addresses, to read and write commands and data.
; 7C: Write I2C data from master to LCD
; 7D: Read command, replies with the RAM address and busy flag register.
; 7E: Write I2C commands from master to LCD
; 7F: Read the buttons and return byte whose bits say which ones are pressed

our_address equ 0x7C		; The first of our four 8-bit slave addresses


; Hardware definitions

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

; I2C lines on PORTB

i2c           equ    PORTB  ; Which port are the I2C lines on?
sda           equ    0      ; PORTB bit 0 is SDA pin
scl           equ    1      ; PORTB bit 1 is SCL pin

; LCD control lines on PORTB

LCD_RS        equ    2      ; Register Select (0 = command, 1 = data)
LCD_RW        equ    3      ; Read or write to LCD (0 = write, 1 = read)
LCD_E         equ    4      ; LCD Enable (pulse high for >230ns to activate)

; LCD data lines on PORTA

LCD_DB4       equ    0      ; LCD data line DB4 is PA0
LCD_DB5       equ    1      ; LCD data line DB5 is PA1
LCD_DB6       equ    2      ; LCD data line DB6 is PA2
LCD_DB7       equ    3      ; LCD data line DB7 is PA3


; PIC memory registers, from 0x0C

tmpLcdRegister       equ    0x0c   ; One location reserved for LCD register
DelayCounter         equ    0x0e   ; Two locations reserved for delay register
i2c_data             equ    0x10   ; Save location for i2c data byte
i2c_bit              equ    0x11   ; Bit counter in i2c byte transfer
command_is_lcd_data  equ    0x12   ; Bit 0 says whether this was an I2C data
                                   ; message.  If 1, it was a data message;
				   ; If 0 it was a command message.
command_is_i2c_read  equ    0x13   ; Bit 0 is the read/write bit that follows
                                   ; the 7-bit slave address, 1 for read.

;*******************************************************************************
; Nice macros
;*******************************************************************************

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


; Loop until a pin of some register is high/low
; Can't use local labels here cos MPLABIDE v.8.76 assembler barfs on them,
; so we rely on knowing that if_low is one instruction.

; Loop until a pin is high
wait_for_high    macro    addr,pin
        if_low  addr,pin
            goto $-1
    endm

; Loop until a pin is low
wait_for_low    macro    addr,pin
        if_high  addr,pin
            goto $-1
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


; In the I2C spec, the minimum time between SDA falling and SCL falling in a
; START is 4us, which is slightly longer that the maximum interrupt latency of
; 4 instruction cycles at 921600 ips.
;
; We do everything in a main loop and busy-wait to detect the START condition
;
; The fastest way to sample SDA and SCL is to sample each one alternately
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

       org    0x0000

       call   initialize
       goto   wait_for_start
	

; In the version without an interrupt routine, instead of leaving the
; interrupt routine, we come back here and wait for a START.

unstretch_and_wait_for_start

       ; Ensure that clock stretching is undone and that SDA is not held low
       select_tris_bank
       bsf    i2c,sda              ;Release SDA line
       bsf    i2c,scl              ;Release SCL line
       select_port_bank

       ; and wait for another start condition

wait_for_start
        ; SCL is always high for 4.7us before a start condition so
        ; we could also check for SCL being high before SDA here,
        ; assuming we always come back here at least 2us before every start
        ; condition. However, the most time-critical part is from when we see
        ; a STOP condition while waiting for the MSB of the next byte,
        ; to when we get back here in time to catch the following START, and
        ; with a scl-high check, the max clock rate is reduced from 54kHz to 48.

	; Sampling SCL and SDA alternately, we look to match the sequence:
	; { SDA-hi then SCL-hi } one or more times
	; followed by SDA-low
	; followed by SCL-high

        if_low  i2c,sda
            goto $-1
start_seen_SDA          ; We've seen SDA high
        if_low  i2c,scl
            goto wait_for_start
        ; We've seen SDA high, SCL high
        if_high i2c,sda
            goto start_seen_SDA
        ; We've seen SDA high, SCL high, SDA low
        if_low  i2c,scl
            goto wait_for_start
        ; We've seen SDA high, SCL high, SDA low, SCL high

        ; START sequence complete.

	; The timing at this point is between 2 and 4 instruction cycles
	; after the falling edge of SDA.
	; SCL may still be high;
	; in the worst timing case, SCL went low 0.34us ago


; After a start bit, instead of reading a bytes of slave address and thinking
; about it, we do address-matching on each bit as they are received.
; This lets us react at 100kHz instead of 15kHz.

; match-address macro:
; Reads a data bit from the I2C stream and compares it against the "bit"th
; bit in the given slave address. If it doesn't match, go wait for another start.
; While matching, this takes 2 instruction cycles after the clock goes high.

match_address_bit    macro    addr,bit
        wait_for_low   i2c,scl
        wait_for_high  i2c,scl
        if (addr & (1 << bit))
	    ; The bit should be high, so quit if sda is low
            if_low   i2c,sda
        else
            ; (addr & (1<<bit)) == 0, so sda should be low.
            ; Quit if it is high.
            if_high   i2c,sda
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

send_ack_and_stretch    macro
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

match_address
        ; The top 6 bits of the slave address should match out address;
        ; These are followed by whether this is a command or data and
        ; whether it is an I2C read or write command.
        match_address_bit  our_address,7
        match_address_bit  our_address,6
        match_address_bit  our_address,5
        match_address_bit  our_address,4
        match_address_bit  our_address,3
        match_address_bit  our_address,2
        ; save the command/data bit of the slave address and the R/W flag
        roll_sda_into_lsb  command_is_lcd_data
        roll_sda_into_lsb  command_is_i2c_read
        send_ack_and_stretch

	; See whether we should read the I2C bus or write to it
	if_low  command_is_i2c_read,0
	   goto  i2c_write_commands

i2c_read_commands
	if_high  command_is_lcd_data,0
	   goto  request_switch_condition
        goto  read_busy_flag


;******************************************************************************
; i2c_write_commands routine:
; Receive data from the I2C master and do something with it.
;******************************************************************************

; The clock is currently stretched in the low period following an ACK and
; the slave address said that the master wants to send us some data.
; One of three things can happen when we unstretch the clock:
; - we get the first bit of an 8-bit data byte
; - we get a STOP condition
; - we get a repeated START condition
;               ______
;       SDA ///X______X///
; DATA:          ____
;	SCL ____|4us |____
;
;                     _____
;       SDA ///X_____|4.7us
; STOP:          __________
;	SCL ____|4us
;
;               ______
;       SDA ///X 4.7us|______X///
; START:         ___________
;	SCL ____|       4us |____
;
; The figures are the minimum possible times according to the I2C spec.
;
; To detect these three cases, we actually use four branches,
; with separate code for data-0 and for data-1.
;
; This version just runs as fast as possible without stretching the clock on
; every bit, and hopes to be fast enough.  Optimization may follow...
;
; Since every command or data consist of a single byte, we just receive
; one byte, stop the clock, act on that byte, then start the clock again
; for the next byte.
; If we get a STOP condition instead, we go back to waiting for a START,
; and if we get a repeated START, we jump into the start-detection code
; at the appropriate point.

i2c_write_commands

	release  i2c,scl
	wait_for_high  i2c,scl
	if_high  i2c,sda
	  goto data_1_or_restart

data_0_or_stop
	if_high  i2c,scl
	  goto data_0_or_stop_part_2

data_0
	; It's a data byte with bit 7 == 0
	bcf  i2c_data,0                 ; D7
	roll_sda_into_lsb i2c_data	; D6
	roll_sda_into_lsb i2c_data	; D5
	roll_sda_into_lsb i2c_data	; D4
	roll_sda_into_lsb i2c_data	; D3
	roll_sda_into_lsb i2c_data	; D2
	roll_sda_into_lsb i2c_data	; D1
	roll_sda_into_lsb i2c_data	; D0
	send_ack_and_stretch
	goto process_i2c_byte

data_0_or_stop_part_2
	if_low i2c,sda	       ; STOP condition?
	  goto data_0_or_stop  ; no
	; If SDA goes low at the same time as SCL goes high, this looks like
	; a STOP condition. Check that SCL is still high for a real STOP
        if_low i2c,scl
          goto data_0
        goto wait_for_start

data_1_or_restart
	if_high  i2c,scl
	  goto data_1_or_restart_part_2

data_1
	; It's a data byte with bit 7 == 1
	bsf  i2c_data,0                 ; D7
	roll_sda_into_lsb i2c_data	; D6
	roll_sda_into_lsb i2c_data	; D5
	roll_sda_into_lsb i2c_data	; D4
	roll_sda_into_lsb i2c_data	; D3
	roll_sda_into_lsb i2c_data	; D2
	roll_sda_into_lsb i2c_data	; D1
	roll_sda_into_lsb i2c_data	; D0
	send_ack_and_stretch
	goto process_i2c_byte

data_1_or_restart_part_2
	if_high  i2c,sda
	  goto data_1_or_restart
	; If SDA goes low at the same time as SCL goes high, this looks like
	; a START condition. Check that SCL is still high to check for real
	; restart.
        if_low i2c,scl
          goto data_1
	; It's a restart
	goto match_address

process_i2c_byte
	; We have received a data/command byte. Process it.
	movf   i2c_data,w
	if_high  command_is_lcd_data,0
	   call  LcdSendData
	if_low   command_is_lcd_data,0
	   call  LcdSendCommand

        ; That's all. Go and see if there is another byte of data for us.
	goto i2c_write_commands


;******************************************************************************
; request_switch_condition subroutine
; reads the buttons and sends their current status to the I2C master as a byte
; with the bottom 5 bits each set or clear, depending whether each button is
; held.  This works for up to two of the L R U D buttons with or without Select
; while three of the four held reports all four held.
;******************************************************************************

; The buttons are connected as follows
;
;          RB5 (LU)
;          / \
;         L   U
;        /     \
; (LR) RA4     RB7 (UD)
;        \     /
;         R   D
;          \ /
;          RB6 (RDS)
;           |
;           S
;           |
;          GND
;
; where R[AB]X are port pins and L R U D S are the buttons.
; There is a 10K pull-up resistor on the RB pins.

; Pin connected to LEFT and RIGHT
LR_port       equ    PORTA
LR_pin        equ    4
#define       LR     LR_port,LR_pin

; Pin connected to LEFT and UP
LU_port       equ    PORTB
LU_pin        equ    5
#define       LU     LU_port,LU_pin

; Pin connected to RIGHT, DOWN and SELECT
RDS_port      equ    PORTB
RDS_pin       equ    6
#define       RDS    RDS_port,RDS_pin

; Pin connected to UP and DOWN
UD_port       equ    PORTB
UD_pin        equ    7
#define       UD     UD_port,UD_pin

; Bit values returned by the button-reading code
button_SELECT equ    1
button_LEFT   equ    2
button_RIGHT  equ    4
button_UP     equ    8
button_DOWN   equ    16

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

; Speed is not an issue here, since all commands are executed with the
; I2C clock line stretched.
; If you're desperate for code size, you can optimize consecutive
;    drive_low foo
;    release   bar
; sequences into
;    select_tris_bank
;      bcf foo	; drive low
;      bsf bar	; release
;    select_port_bank

request_switch_condition

       clrf   PORTB         ; Ensure all RB lines will output 0
       clrf   PORTA         ; Ensure RA4 will output 0.

       ; TRIS bits should already be set with all four lines as inputs.

       movlw    0   ; no buttons pressed yet...

       ; Check SELECT switch with all lines as inputs
       if_low  RDS           ; Is SELECT pressed?
         iorlw button_SELECT ; yes

       ; Check RIGHT and DOWN by setting RB6 as a low output
       drive_low  RDS
       if_low  LR            ; Is RIGHT pressed?
         iorlw button_RIGHT  ; yes
       if_low  UD            ; Is DOWN pressed?
         iorlw button_DOWN   ; yes
       release    RDS

       ; Check LEFT by setting RA4 as a low output and testing RB5
       drive_low  LR
       if_high  LU           ; Is LEFT pressed?
         goto  left_button_not_pressed ; no
       ; now check the other way round
       release  LR
       drive_low  LU
       if_low  LR            ; Is LEFT really pressed?
         iorlw  button_LEFT  ; yes
       release  LU
left_button_not_pressed
       release  LR

       ; Check UP by setting RB7 low and testing RB5
       drive_low  UD
       if_high  LU           ; Is UP pressed?
         goto up_button_not_pressed ; no
       release    UD
       drive_low  LU
       if_low  UD            ; Is UP really pressed?
         iorlw  button_UP    ; yes
       release    LU
up_button_not_pressed
       release  UD

send_data_to_i2c

       movwf  i2c_data
       call   i2c_send_byte
       goto   unstretch_and_wait_for_start

;******************************************************************************
; read_busy_flag: Read the DDRAM address and busy flag from the LCD
; and send them as a byte to the I2C master.
;******************************************************************************

read_busy_flag

       call   LcdReadAddress
       goto   send_data_to_i2c


;******************************************************************************
; i2c_send_byte subroutine: send a byte of data to the master.
;
; The byte is in register i2c_data and the clock is already held low
; after the acknowledge bit we sent for our slave address.
; I2C data is transmitted with the most significant bit first.
;******************************************************************************

i2c_send_byte

   ; This stretches the low half of the clock on every clock pulse
   ;
   ; The critical time is from releasing the clock to holding it again,
   ; If the clock goes high when released, this happens 125ns after the end
   ; of the "bsf scl" insn. We detect that it is high within 2 insns (2us),
   ; which is 2.17us, well within the the minimum of 4us.
   ;
   ; We then have to hold the clock low within 4.7us of the master driving it
   ; low.  The worst case is when we sample the last moment of the high part
   ; of the clock waveform and it takes us 3 insn cycles to sample it again
   ; and
   ;   btfsx scl; nop; bsf RP0; bcf scl;
   ; to hold it low, which is
   ; 6.75 * 1.085us + 125ns = 7.45us, much longer than the ~ 4.7us we have.
   ; (I can't find a figure to say what the setup time is for a clock stretch)
   ;
   ; This version works with an I2C clock rate of 91kHz or less.
   ; I tried a version that just ran as fast as it could without stretching the clock
   ; but that only worked up to 71kHz was 58 bytes larger.


       movlw  0x08          ;init bit counter for i2c communication
       movwf  i2c_bit

send_another_bit

       ; output a bit
       select_tris_bank
       btfsc  i2c_data,7           ; See if data bit to output is 1
       bsf    i2c,sda              ; Go open-collector for high output if so
       btfss  i2c_data,7           ; See if data bit to output is 0
       bcf    i2c,sda              ; Output low if so
       bsf    i2c,scl              ; Release the clock
       select_port_bank

       wait_for_high  i2c,scl
       wait_for_low   i2c,scl

       drive_low  i2c,scl          ; hold the clock again

       rlf    i2c_data,f	   ; discard tha bit
       decfsz i2c_bit,f		   ; decrease bit count
       goto   send_another_bit

       ; We only ever send one byte, so we don't care about the master's
       ; ACK bit or its STOP condition or anything else.
       ; We just go and wait for another START condition.

       return


;******************************************************************************
;******************************************************************************
;Initialize the hardware and our variables
;******************************************************************************
;******************************************************************************

initialize

       select_port_bank
       bcf    INTCON,GIE           ;Disable global interrupt

       movlw  11101111B
       movwf  PORTA         ;Init all LCD control lines High for correct LCD start
			    ;Only RA4 must be low when in out mode for switch function

       movlw  00011100B     ;RB7, RB6 and RB5 must be low when in out mode
                            ;LCD line (E, R/W, RS) must be High.
			    ; SDA and SCL are always low when outputs
       movwf  PORTB         ;For init PortB

       select_tris_bank

       movlw  11110000B     ;Config LCD data lines all output - RA4 input for switch
       movwf  TRISA

       movlw  11100011B     ;RB6 and RB5 for input switch. RB7 out for switch
       movwf  TRISB         ;Rb1=Scl (input) Rb0=Sda (input)

       select_port_bank

       ;Power-on delay of 3ms for LCD panel
       movlw   3
       call    msDelay

       call   LcdInit       ;Init LCD

   ifdef INTERRUPT
       bsf    STATUS,RP0           ;Select bank of memory 1
       bcf    OPTION_REG,INTEDG    ;Select INT interrupt on falling edge
       bcf    STATUS,RP0           ;Select bank of memory 0
       ;bsf    INTCON,INTE          ;Enable INT interrupt
       ;bcf    INTCON,INTF          ;Clear the interrupt if it is active
       ;bsf    INTCON,GIE           ;Enable global interrupt
       movlw  (1<<INTE) | (1<<GIE) | (0<<INTF) ; equivalent
       movwf  INTCON
   endif

       return

; This is the same, except that it is called from an RESET command,
; so the PIC is in a known state (port bank selected, SCL held low)
; and it must not release the SCL line.
reinitialize
       movlw  11101111B
       movwf  PORTA         ;Init all LCD control lines High for correct LCD start
			    ;Only RA4 must be low when in out mode for switch function

       movlw  00011100B     ;RB7, RB6 and RB5 must be low when in out mode
                            ;LCD line (E, R/W, RS) must be High.
			    ; SDA and SCL are always low when outputs
       movwf  PORTB         ;For init PortB

       select_tris_bank

       movlw  11110000B     ;Config LCD data lines all output - RA4 input for switch
       movwf  TRISA

       movlw  11100001B     ;RB6 and RB5 for input switch. RB7 out for switch
       movwf  TRISB         ;Rb1=Scl (held low) Rb0=Sda (input)

       select_port_bank

       goto   LcdInit       ;Init LCD (tail call)



;**********************************************************************
; Delay routines
;**********************************************************************

; msDelay: Delay for W milliseconds

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
Delay1_53msLoop
       goto   $+1                  ; 2 cycles
       nop                         ; 1 cycle
       decfsz DelayCounter,f         ; 1 cycle
       goto   Delay1_53msLoop      ; 2 cycles

       return


;**********************************************************************
; Initialize the LCD display
; This function must be called before any other function that drives the LCD
;**********************************************************************

; First, a nice macro to actuate the read/write from/to the LCD.
; As it uses bcf/bsf, it does a read-modify-write of PORTB, which changes
; the output pin value of any pins set as inputs to their current input voltage.
; Make sure PORTB is cleared before returning from routines that use EN_STROBE.
; At 5V, the minimum strobe high time is 230ns, whereas b[cs]f take 1 cycle
; which is just over 1 us.

EN_STROBE            MACRO
                bsf  PORTB,LCD_E
                bcf  PORTB,LCD_E
                     ENDM

LcdInit
              ;bcf     PORTB,LCD_E     ;Lower the enable strobe
              ;bcf     PORTB,LCD_RS    ;Put the LCD in command mode
              ;bcf     PORTB,LCD_RW    ;Enable writing mode to the LCD
              clrf    PORTB           ;equivalent

              ; Send the reset sequence to the LCD
	      ; The Sequence recommended in the Ampire Datasheet sometimes
	      ; leaves the display crashed. This sequence is taken from
	      ; http://embeddedtutorial.com/2010/01/interfacing-lcd-with-8051/
	      ; and seems to be reliable.

	      ; First, some jiggery-pokery to set it to 8-bit mode,
	      ; whether it was in 8-bit mode, 4-bit mode or 4-bit mode
	      ; with one half of a command already sent.

              movlw   00000011B
              movwf   PORTA
              EN_STROBE

              movlw   50
              call    msDelay

              EN_STROBE

              movlw   10
              call    msDelay

              EN_STROBE

              movlw   1
              call    msDelay

              ;Now it is guaranteed to be in 8-bit mode

              ;Configure the data bus to 4 bits
              movlw   00000010B
              movwf   PORTA
              EN_STROBE

              movlw   50
              call    msDelay

              ;Set 2 display lines and 5x8 font
              movlw   0x28
              call    LcdSendCommand

              ;Display OFF, Curson OFF, Blink OFF
              movlw   0x0B
              call    LcdSendCommand

              ;Display ON, Curson OFF, Blink OFF
              movlw   0x0C
              call    LcdSendCommand

              ;Entry mode set, increment, no shift
              movlw   0x06
              call    LcdSendCommand

              ; Clear display
              movlw   0x01
              call    LcdSendCommand

              ; LcdSendCommand has cleared PORTB for us.

              return


;**********************************************************************
; Send a command to LCD
;**********************************************************************

LcdSendCommand
	      ; Our fake command 0 does a complete initialization of the LCD
	      addlw   0
              btfsc   STATUS,Z
	      goto    reinitialize ; tail call

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
; Read the RAM address pointer and busy flag into W
;**********************************************************************

LcdReadAddress
              bcf    PORTB,LCD_RS       ; RS = 0
              bsf    PORTB,LCD_RW       ; R/W = 1

              ; Make our four data lines inputs
              select_tris_bank
              movlw  11111111B
              movwf  TRISA
              select_port_bank

              ; raise the strobe so that the LCD's data lines become outputs
              bsf    PORTB,LCD_E
              swapf  PORTA,w            ; Fetch the top four bits
              andlw  11110000B
              movwf  tmpLcdRegister
              bcf    PORTB,LCD_E        ; lower the strobe

              bsf    PORTB,LCD_E
              movf   PORTA,w            ; Fetch the bottom four bits
              andlw  00001111B
              iorwf  tmpLcdRegister,f

              clrf   PORTB              ; lower the strobe and reset the
                                        ; bits scrambled by b[cs]f PORTB

              select_tris_bank
              movlw  11110000B          ; Set our data lines back to outputs
              movwf  TRISA
              select_port_bank

              movf   tmpLcdRegister,w   ; put the return value in W

              return


;**********************************************************************
;Subroutines End here
;**********************************************************************

              END
