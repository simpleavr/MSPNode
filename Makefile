
# makefile for building ti launchpad projects on cygwin based mspgcc system
# also works under linux if path point to right mspgcc installation + use mspdebug
#

TARGET=rfnode

GCCBIN=/cygdrive/c/mspgcc4-20110312/bin
CC=$(GCCBIN)/msp430-g++
SIZE=$(GCCBIN)/msp430-size
STRIP=$(GCCBIN)/msp430-strip
OBJCP=$(GCCBIN)/msp430-objcopy
PROG=/cygdrive/C/Users/chrisc/MSP430Flasher_1.1.2/MSP430Flasher.exe

MMCU=msp430x2012
DEVICE=MSP430G2231

#MMCU=msp430x2131
#DEVICE=MSP430G2452

CFLAGS=-Os -Wall -g -mmcu=$(MMCU) -ffunction-sections -fdata-sections -fno-inline-small-functions
#CFLAGS=-mmcu=msp430x2012 -Os -std=gnu99 -W -Wall -pedantic -Wstrict-prototypes -Wundef -funsigned-char -funsigned-bitfields -ffunction-sections -fpack-struct -fshort-enums -ffreestanding --combine -fno-inline-small-functions -fno-split-wide-types -fno-tree-scev-cprop -Wl,--relax,--gc-sections

LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--relax
LDFLAGS += -Wl,--gc-sections

OBJS=main.o

all: $(TARGET).elf

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET).elf $(OBJS) $(LDFLAGS)
	$(STRIP) $(TARGET).elf
	$(SIZE) --format=sysv $(TARGET).elf

flash: $(TARGET).elf
	$(OBJCP) -O ihex $(TARGET).elf $(TARGET).hex
	$(PROG) -n $(DEVICE) -w "$(TARGET).hex" -v -g -z [VCC]

#   for linux, use mspdebug
#	mspdebug rf2500 "prog $(TARGET).elf"

%.o: %.cpp rfm.h uart.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm -rf $(TARGET).elf $(OBJS) $(TARGET).map
