CC=g++

# pass BUILD_FOR as string to $(CC)
DEFINES= -D$(IF) -D$(PLATFORM)
INCLUDES= .
CFLAGS= -I$(INCLUDES) $(DEFINES) -Wall
LIBS=
DEPS= hcl.h hcl_gpio.h sensor_epsonCommon.h main_helper.h
DEPS_UART= hcl_uart.h sensor_epsonUart.h
DEPS_SPI= hcl_spi.h sensor_epsonSpi.h

OBJ= main_helper.o sensor_epsonCommon.o

# defaults Interface to UART
IF ?= UART

# defaults to NONE
PLATFORM ?= NONE

####### Adding IF Specific Files
ifeq ($(IF), UART)
	OBJ+= sensor_epsonUart.o hcl_uart.o
	DEPS+= $(DEPS_UART)
else ifeq ($(IF), SPI)
	OBJ+= sensor_epsonSpi.o hcl_spi_rpi.o
	DEPS+= $(DEPS_SPI)
	PLATFORM= RPI
endif

####### Adding PLATFORM Specific Files
ifeq ($(PLATFORM), NONE)
	OBJ+= hcl_linux.o hcl_gpio.o
else ifeq ($(PLATFORM), RPI)
	OBJ+= hcl_rpi.o hcl_gpio_rpi.o
	LIBS+= -lwiringPi -lpthread -lcrypt -lrt
endif

all: screen csvlogger regdump

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

screen: $(OBJ) main_screen.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

csvlogger: $(OBJ) main_csvlogger.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

regdump: $(OBJ) main_regdump.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean all tar help

clean:
	rm -f $(OBJ) *~ core *~ *.o
	rm -f csvlogger screen regdump

tar:
	tar cvzf archive.tar.gz *.c *.h README.md Makefile

help:
	@echo "supported make commands are:"
	@echo "\tmake clean"
	@echo "\tmake <targets>\n"
	@echo "valid <targets> are: all csvlogger screen or regdump\n"
	@echo "valid <interfaces, IF> are:"
	@echo "\tUART SPI"
	@echo "valid <platforms, PLATFORM> are:"
	@echo "\tNONE RPI"
	@echo "example:\n\tmake csvlogger (defaults PLATFORM=NONE, IF=UART)"
	@echo "\tmake screen PLATFORM=RPI (defaults to IF=UART)"
	@echo "\tmake regdump IF=SPI PLATFORM=RPI"
	@echo "\tmake all (defaults to PLATFORM=NONE, IF=UART, targets=csvlogger, screen, regdump)"
