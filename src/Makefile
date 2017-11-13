DEVICE     = attiny2313
CLOCK      = 1000000
PROGRAMMER = um232h
PORT	     = /dev/ttyUSB0
BAUD       = 19200
FILENAME   = main
CC         = avr-gcc
CFLAGS     = -Wall -Wno-main -O2 -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -g
SIMAVR		 = /home/david/tmp/simavr/simavr/run_avr

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
