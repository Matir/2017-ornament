DEVICE     = attiny2313a
CLOCK      = 4000000
PROGRAMMER = um232h
PORT	     = /dev/ttyUSB0
BAUD       = 19200
FILENAME   = main
CC         = avr-gcc
CFLAGS     = -Wall -Wno-main -O2 -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -g
SIMAVR		 = /home/david/tmp/simavr/simavr/run_avr
AVRDUDE_DEVICE = t2313
AVRDUDE_PROG = c232hm
# Fusebits for avrdude
# 4MHz, no CKDIV8, no CKOUT
FUSEBITS	 = -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

OBJS = main.o gamma.o

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex ${@:.hex=.elf} $@

main.elf: ${OBJS}
	${CC} ${CFLAGS} $^ -o $@
	avr-size --mcu=${DEVICE} -C $@

%.o: %.c
	${CC} ${CFLAGS} -c $<

gamma.c: gentable.py
	python gentable.py 128 $@

simavr: main.elf
	${SIMAVR} -f ${CLOCK} -m ${DEVICE} main.elf

.PHONY: avrdude

avrdude: main.hex
	avrdude -c ${AVRDUDE_PROG} -p ${AVRDUDE_DEVICE} -U flash:w:$< ${FUSEBITS}

clean:
	rm -f *.o *.hex *.elf
