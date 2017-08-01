# nucleo-sdcard

This project is designed as an example of a STM32CubeMX-generated system with FatFs middleware controlling
an SPI-connected MMC/SD memory card.

The project was initially created in CubeMX, and then code written by ChaN was ported to the CubeMX HAL.

The driver files `Src/user_diskio_spi.c` and `Inc/user_diskio_spi.h` should be usable in any CubeMX project (although the SPI handle `hspi1` may need to be changed depending on your requirements).

The CubeMX project file is `nucleo-sdcard.ioc` in the root directory. You can load this with CubeMX and change any of the settings and regenerate without affecting any of the code in this code base.

### Usage instructions

#### Programmer

This guide assumes Ubuntu linux.

Firstly, you will need to install texane st-link drivers from https://github.com/texane/stlink

The guide for doing this is https://github.com/texane/stlink/blob/master/doc/compiling.md

Once you have this installed and set up correctly you should be able to run 
`>st-info --chipid` and get a suitable response back (e.g. 0x0446)

#### Development board

I used the stm32nucleo-f303re development board for this project, but any suitable board should do. 

Connect the SPI lines SCK/MISO/MOSI up to the SD card, and an additional GPIO pin (I used PinB6) as your CS line (Can't use the hardware-controlled CS due to reasons, but you can use the pin it otherwise would be on).

If this isn't SPI1 on your board you'll need to change the two files mentioned previously.

Then, connect a UART TX from the board to a UART RX on your computer. I used UART1.

#### MMC/SD memory card

Make sure your memory card is formatted to some flavour of FAT (I used FAT32) and is empty except for a single text file "test.txt". I put "hello world" inside my file.

#### Download and Run

Firstly, you'll want to set up your UART RX. I use `minicom` in a separate terminal to my st-link. You should set it up with baud rate 38400 and hardware flow control off.

Then, you need to download the program to your micro.

I modified the default makefile from CubeMX to include a `make download` command.

It should say something like this:
```
make download
arm-none-eabi-gcc ........[etc]
arm-none-eabi-size build/nucleo-sdcard.elf
   text	   data	    bss	    dec	    hex	filename
  20448	    136	   1992	  22576	   5830	build/nucleo-sdcard.elf
arm-none-eabi-objcopy -O ihex build/nucleo-sdcard.elf build/nucleo-sdcard.hex
st-flash --format ihex write build/nucleo-sdcard.hex
st-flash 1.3.1-22-gdc8eb3e
2017-08-01T15:56:44 INFO src/common.c: Loading device parameters....
2017-08-01T15:56:44 INFO src/common.c: Device connected is: F303 high density device, id 0x10036446
2017-08-01T15:56:44 INFO src/common.c: SRAM size: 0x10000 bytes (64 KiB), Flash: 0x80000 bytes (512 KiB) in pages of 2048 bytes
2017-08-01T15:56:44 INFO src/common.c: Attempting to write 20596 (0x5074) bytes to stm32 address: 134217728 (0x8000000)
Flash page at addr: 0x08005000 erased
2017-08-01T15:56:44 INFO src/common.c: Finished erasing 11 pages of 2048 (0x800) bytes
2017-08-01T15:56:44 INFO src/common.c: Starting Flash write for VL/F0/F3/F1_XL core id
2017-08-01T15:56:44 INFO src/flash_loader.c: Successfully loaded flash loader in sram
 11/11 pages written
2017-08-01T15:56:45 INFO src/common.c: Starting verification of write complete
2017-08-01T15:56:45 INFO src/common.c: Flash written and verified! jolly good!
```

In your minicom, something along the lines of the following should now appear:

```
(Powering up)                                                                   
Mary had a little lamb --                                                       
I ate it with mint sauce.                                                       
                                                                                
SD card stats:                                                                  
   7707648 KiB total drive space.                                               
   7707456 KiB available.                                                       
I was able to open 'test.txt' for reading!                                      
Read string from 'test.txt' contents: hello world                               
I was able to open 'write.txt' for writing                                      
Wrote 19 bytes to 'write.txt'!      
```

The nice thing with this project is that everything is compatible with CubeMX templates, so you can change your CubeMX settings and regenerate the code without breaking anything.

### License

This repository has a lot of code under a number of different licenses. All C/H files should have appropriate licenses in their headers.

For the most part, though, this operates as a TL;DR:

```
Portions copyright (c) 2015, 2017 STMicroelectronics International N.V., all rights reserved.
Portions copyright (C) 2014, ChaN, all rights reserved.
Portions copyright (C) 2017, kiwih, all rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted, provided that the following conditions are met:

 1. Redistribution of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of STMicroelectronics nor the names of other 
    contributors to this software may be used to endorse or promote products 
    derived from this software without specific written permission.
 4. This software, including modifications and/or derivative works of this 
    software, must execute solely and exclusively on microcontroller or
    microprocessor devices manufactured by or for STMicroelectronics.
 5. Redistribution and use of this software other than as permitted under 
    this license is void and will automatically terminate your rights under 
    this license. 

 THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
 AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
 SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
