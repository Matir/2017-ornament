DEVICE     = attiny2313a
CLOCK      = 4000000
PROGRAMMER = um232h
PORT	     = /dev/ttyUSB0
BAUD       = 19200
FILENAME   = main
CC         = avr-gcc
CFLAGS     = -Wall -Wno-main -O2 -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -g
SIMAVR		 = /home/david/tmp/simavr/simavr/run_avr
# Fusebits for avrdude
# 4MHz, no CKDIV8, no CKOUT
FUSEBITS	 = -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

OBJS = main.o

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex ${@:.hex=.elf} $@

main.elf: ${OBJS}
	${CC} ${CFLAGS} $< -o $@
	avr-size --mcu=${DEVICE} -C $@

%.o: %.c
	${CC} ${CFLAGS} -c $<

simavr: main.elf
	${SIMAVR} -f ${CLOCK} -m ${DEVICE} main.elf

clean:
	rm -f *.o *.hex *.elf
