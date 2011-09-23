# Makefile to control assembly and programming of Mizar32 PIC16F84 chip on
# LCD module

ALL=Display_module.hex

# By default, just create all the targets lister in $(ALL)
all: $(ALL)

# Add out funny suffixes to make's list of filename suffixes
.SUFFIXES: .asm .hex

# Generic rule to convert an .asm into a .hex
.asm.hex:
	gpasm $<

# Program the hex file to the board
program: $(ALL)
	picprog --device pic16f84a --burn --input $(ALL) --rdtsc

# Rule to remove automatically-created files
clean:
	rm -r *.hex *.cod *.lst
