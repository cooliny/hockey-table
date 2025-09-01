CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
LD=arm-none-eabi-ld
CCFLAGS=-mcpu=cortex-m0 -mthumb -g 
PORTN=/dev/$(shell ls /dev | grep "cu.usbserial")

# Search for the path of libraries.
LIBPATH1=/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/lib/gcc/arm-none-eabi/14.2.1/thumb/v6-m/nofp
LIBPATH2=/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/arm-none-eabi/lib/thumb/v6-m/nofp
LIBSPEC=-L"$(LIBPATH1)" -L"$(LIBPATH2)"

OBJS=main.o util.o tim1637.o ws2812b.o dfplayer.o startup.o serial.o newlib_stubs.o

all: main.hex flash

main.hex: $(OBJS)
	$(LD) $(OBJS) $(LIBSPEC) -Os -u _printf_float -nostdlib -lnosys -lgcc -T LDscripts/stm32l051xx.ld --cref -Map main.map -o main.elf
	arm-none-eabi-objcopy -O ihex main.elf main.hex
	@echo Success!

main.o: main.c util.h tim1637.h ws2812b.h dfplayer.h
	$(CC) -c $(CCFLAGS) main.c -o main.o

util.o: util.c 
	$(CC) -c $(CCFLAGS) util.c -o util.o

tim1637.o: tim1637.c 
	$(CC) -c $(CCFLAGS) tim1637.c -o tim1637.o

ws2812b.o: ws2812b.c 
	$(CC) -c $(CCFLAGS) ws2812b.c -o ws2812b.o

dfplayer.o: dfplayer.c 
	$(CC) -c $(CCFLAGS) dfplayer.c -o dfplayer.o

startup.o: Source/startup.c
	$(CC) -c $(CCFLAGS) -DUSE_USART1 Source/startup.c -o startup.o

serial.o: Source/serial.c
	$(CC) -c $(CCFLAGS) Source/serial.c -o serial.o
	
newlib_stubs.o: Source/newlib_stubs.c
	$(CC) -c $(CCFLAGS) Source/newlib_stubs.c -o newlib_stubs.o

clean: 
	rm -f $(OBJS) main.elf main.hex main.map

flash: main.hex
	../stm32flash/stm32flash -w main.hex -v -g 0x0 $(PORTN)


