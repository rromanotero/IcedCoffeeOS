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
$ATMEL_PACKS_SAMD21_PATH = "C:\Program Files (x86)\Atmel\Studio\7.0\packs\atmel\SAMD21_DFP\1.3.331"

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

.\compile.bat $GCC_BIN_PATH $ATMEL_PACKS_SAMD21_PATH 

if ($LASTEXITCODE -ne 0)
{
	Write-Host " ***** COMPILING FAILED *****"
	exit 1
}
Write-Host "SUCCESS"

Write-Host "  FLASHING HEX  "
Write-Host "================"

& "$ATPROGRAM_BIN_PATH\atprogram.exe" -t samice -i swd -s 28018294 -d atsamd21g18a -l output.log -cl 4Mhz program -c -f ".\output\feather_m0.hex" --verify
