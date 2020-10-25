@REM
@REM   This file is part of IcedCoffeeOS
@REM   (https://github.com/rromanotero/IcedCoffeeOS).
@REM
@REM   Copyright (c) 2020 Rafael Roman Otero.
@REM
@REM   This program is free software: you can redistribute it and/or modify
@REM   it under the terms of the GNU General Public License as published by
@REM   the Free Software Foundation, either version 3 of the License, or
@REM   (at your option) any later version.
@REM
@REM   This program is distributed in the hope that it will be useful,
@REM   but WITHOUT ANY WARRANTY; without even the implied warranty of
@REM   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@REM   GNU General Public License for more details.
@REM
@REM   You should have received a copy of the GNU General Public License
@REM   along with this program.  If not, see <https://www.gnu.org/licenses/>.
@REM

@Passed by caller
set GCC_BIN_PATH=%1

@REM COMPILER COMMAND LINE
@echo off
set "cpuflags= -O3 -march=armv8-a+simd -mtune=cortex-a53 -mstrict-align -fno-tree-loop-vectorize -fno-tree-slp-vectorize"
set "asmflags=-nostdlib -nostartfiles -ffreestanding -fno-asynchronous-unwind-tables -fomit-frame-pointer -Wa,-a>output/list.txt"
set "linkerflags=-Wl,-gc-sections -Wl,--build-id=none -Wl,-Bdynamic -Wl,-Map,output/kernel.map"
set "outflags=-o output/kernel8.elf"
set "libflags=-lc -lm -lgcc"
@echo on
%GCC_BIN_PATH%\aarch64-none-elf-gcc.exe ^
    %cpuflags% %asmflags% %linkerflags% -Wl,-T,linker.ld ^
    -I ./src ^
    -I ./src/drivers ^
    -I ./src/drivers/delays ^
    -I ./src/drivers/fb ^
    -I ./src/drivers/gpio ^
    -I ./src/drivers/mbox ^
    -I ./src/drivers/sd ^
    -I ./src/drivers/uart ^
    -I ./build_temp/common/fonts ^
    -I ./src/hal ^
    -I ./src/loader ^
    -I ./src/syscalls ^
    -I ./build_temp/common/kprintf ^
    -I ./src/smartstart ^
    src/main.c ^
    src/system.c ^
    src/smartstart/SmartStart64.S ^
    src/smartstart/rpi-SmartStart.c ^
    src/drivers/delays/delays.c ^
    src/drivers/fb/fb.c ^
    src/drivers/mbox/mbox.c ^
    src/drivers/sd/sd.c ^
    src/drivers/uart/uart.c ^
    build_temp/common/fonts/fonts.c ^
    src/hal/hal_cpu.c ^
    src/hal/hal_cpu.S ^
    src/hal/hal_io.c ^
    src/hal/hal_storage.c ^
    src/hal/hal_video.c ^
    src/hal/hal_timer.c ^
    src/loader/loader.c ^
    src/syscalls/syscalls.c ^
    build_temp/common/kprintf/kprintf.c ^
    build_temp/common/kprintf/kprintf_hex_dump.c ^
    %outflags% %libflags%
@echo off
if %errorlevel% EQU 1 (goto build_fail)

exit /b 0

:build_fail
exit /b 1
