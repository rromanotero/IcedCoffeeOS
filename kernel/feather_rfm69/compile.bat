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
set "cpuflags=-D__SAMD21G18A__ -mthumb -mcpu=cortex-m0plus -Os -ffreestanding"
set "asmflags=-Wa,-a>output/list.txt"
set "linkerflags=-Wl,-gc-sections -Wl,--build-id=none -Wl,-Bdynamic -Wl,-Map,output/kernel.map"
set "outflags=-o output/kernel.elf"
set "libflags=-lc -lm -lgcc"
@echo on
%GCC_BIN_PATH%\arm-none-eabi-gcc ^
    %cpuflags% %asmflags% %linkerflags% -Wl,-T,linker.ld ^
    -I ./src/atmel_pack_samd21_dfp/arm/CMSIS/5.4.0/CMSIS/Core/Include ^
    -I ./src/atmel_pack_samd21_dfp/arm/CMSIS/5.4.0/CMSIS/CoreA/Include ^
    -I ./src/atmel_pack_samd21_dfp/atmel/1.3.395/samd21a/include ^
    -I ./src/atmel_pack_samd21_dfp/atmel/1.3.395/samd21a/include/component ^
    -I ./src/atmel_pack_samd21_dfp/atmel/1.3.395/samd21a/include/instance ^
    -I ./src/atmel_pack_samd21_dfp/atmel/1.3.395/samd21a/include/pio ^
    -I ./src/atmel_pack_samd21_dfp/atmel/1.3.395/samd21a/include_mcc ^
    -I ./src/atmel_pack_samd21_dfp/atmel/1.3.395/samd21a/include_mcc/component ^
    -I ./src/atmel_pack_samd21_dfp/atmel/1.3.395/samd21a/include_mcc/pio ^
    -I ./src ^
    -I ./src/Config/ ^
    -I ./src/Device_Startup/ ^
    -I ./src/documentation/ ^
    -I ./src/examples/ ^
    -I ./src/hal/ ^
    -I ./src/hal/documentation/ ^
    -I ./src/hal/include/ ^
    -I ./src/hal/src/ ^
    -I ./src/hal/utils/ ^
    -I ./src/hal/utils/include/ ^
    -I ./src/hal/utils/src/ ^
    -I ./src/hpl/ ^
    -I ./src/hpl/adc/ ^
    -I ./src/hpl/core/ ^
    -I ./src/hpl/dmac/ ^
    -I ./src/hpl/eic/ ^
    -I ./src/hpl/gclk/ ^
    -I ./src/hpl/pm/ ^
    -I ./src/hpl/port/ ^
    -I ./src/hpl/rtc/ ^
    -I ./src/hpl/sercom/ ^
    -I ./src/hpl/sysctrl/ ^
    -I ./src/hpl/systick/ ^
    -I ./src/hpl/tc/ ^
    -I ./src/hri/ ^
    -I ./src/smart_plug/ ^
    -I ./src/smart_plug/config/ ^
    -I ./src/smart_plug/documents/ ^
    -I ./src/smart_plug/smartconfig/ ^
    -I ./src/stdio_redirect/ ^
    -I ./src/stdio_redirect/gcc/ ^
    ./src/atmel_start.c ^
    ./src/Device_Startup/startup_samd21.c ^
    ./src/Device_Startup/system_samd21.c ^
    ./src/driver_init.c ^
    ./src/examples/driver_examples.c ^
    ./src/hal/src/hal_adc_sync.c ^
    ./src/hal/src/hal_atomic.c ^
    ./src/hal/src/hal_calendar.c ^
    ./src/hal/src/hal_delay.c ^
    ./src/hal/src/hal_ext_irq.c ^
    ./src/hal/src/hal_gpio.c ^
    ./src/hal/src/hal_i2c_m_sync.c ^
    ./src/hal/src/hal_init.c ^
    ./src/hal/src/hal_io.c ^
    ./src/hal/src/hal_sleep.c ^
    ./src/hal/src/hal_spi_m_sync.c ^
    ./src/hal/src/hal_timer.c ^
    ./src/hal/src/hal_usart_async.c ^
    ./src/hal/src/hal_usart_sync.c ^
    ./src/hal/utils/src/utils_assert.c ^
    ./src/hal/utils/src/utils_event.c ^
    ./src/hal/utils/src/utils_list.c ^
    ./src/hal/utils/src/utils_ringbuffer.c ^
    ./src/hal/utils/src/utils_syscalls.c ^
    ./src/hpl/adc/hpl_adc.c ^
    ./src/hpl/core/hpl_core_m0plus_base.c ^
    ./src/hpl/core/hpl_init.c ^
    ./src/hpl/dmac/hpl_dmac.c ^
    ./src/hpl/eic/hpl_eic.c ^
    ./src/hpl/gclk/hpl_gclk.c ^
    ./src/hpl/pm/hpl_pm.c ^
    ./src/hpl/rtc/hpl_rtc.c ^
    ./src/hpl/sercom/hpl_sercom.c ^
    ./src/hpl/sysctrl/hpl_sysctrl.c ^
    ./src/hpl/systick/hpl_systick.c ^
    ./src/hpl/tc/hpl_tc.c ^
    ./src/smart_plug/adc_sensor.c ^
    ./src/smart_plug/led_jd.c ^
    ./src/smart_plug/main_jd.c ^
    ./src/smart_plug/sw_timer.c ^
    ./src/stdio_redirect/gcc/read.c ^
    ./src/stdio_redirect/gcc/write.c ^
    ./src/stdio_redirect/stdio_io.c ^
    ./src/stdio_start.c ^
    %outflags% %libflags%
@echo off
if %errorlevel% EQU 1 (goto build_fail)

exit /b 0

:build_fail
exit /b 1
