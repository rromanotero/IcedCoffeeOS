<#
#   This file is part of IcedCoffeeOS
#   (https://github.com/rromanotero/IcedCoffeeOS).
#
#   Copyright (c) 2020 Rafael Roman Otero.
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#>

<#
#	 Params
#	 (edit path to ATPROGRAM here)
#>

$ATPROGRAM_BIN_PATH = "C:\Program Files (x86)\Atmel\Studio\7.0\atbackend"
$GCC_BIN_PATH = "C:\gcc-arm-none-eabi-9-2020-q2-update-win32\bin"

Clear-Host;

$ErrorActionPreference = "Stop"

<#
#	 Copy all common modules to build temp folder
#>
if (!(Test-Path ".\build_temp" -PathType Container)) {
    New-Item -ItemType Directory -Force -Path .\build_temp
}
Copy-item -Force -Recurse -Verbose ..\common -Destination .\build_temp


<#
#	 Build process
#>
Write-Host "  C L E A N I N G "
Write-Host "=================="

rm ./output/*

Write-Host "SUCCESS"

Write-Host "  C O M P I L I N G  "
Write-Host "==================="

.\compile.bat $GCC_BIN_PATH

if ($LASTEXITCODE -ne 0)
{
	Write-Host " ***** COMPILING FAILED *****"
	exit 1
}
Write-Host "SUCCESS"

Write-Host "  C R E A T I N G    H E X,  L S S,  etc   "
Write-Host "============================================"

& "$GCC_BIN_PATH\arm-none-eabi-objcopy.exe" .\output\kernel.elf -O binary .\output\kernel.bin
& "$GCC_BIN_PATH\arm-none-eabi-objcopy.exe" .\output\kernel.elf -O ihex -R .eeprom -R .fuse -R .lock -R .signature .\output\kernel.hex
& "$GCC_BIN_PATH\arm-none-eabi-objdump.exe" -D .\output\kernel.elf | Out-File -filepath output/kernel.lss -Encoding ASCII
& "$GCC_BIN_PATH\arm-none-eabi-objdump.exe" -s .\output\kernel.elf | Out-File -filepath output/kernel.dump -Encoding ASCII
& "$GCC_BIN_PATH\arm-none-eabi-size.exe" .\output\kernel.elf | Out-File -filepath output/kernel.size -Encoding ASCII

if ($LASTEXITCODE -ne 0)
{
	Write-Host "CREATING IMG et al FAILED"
	exit 1
}
Write-Host "SUCCESS"

Write-Host "  SIZE  "
Write-Host "========"
& "$GCC_BIN_PATH\arm-none-eabi-size.exe" .\output\kernel.elf

Write-Host "  FLASHING HEX  "
Write-Host "================"

& "$ATPROGRAM_BIN_PATH\atprogram.exe" -t samice -i swd -d atsamd21g18a -l output/output.log -cl 4Mhz program -c -f ".\output\kernel.hex" --verify
