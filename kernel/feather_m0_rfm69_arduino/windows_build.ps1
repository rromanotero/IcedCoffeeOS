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
#	 (edit paths here)
#>

$ARDUINO_CLI_PATH = "C:\Program Files\Arduino"
$ARDUINO_CLI_BUILD_PATH = "sketchbook/main/build/adafruit.samd.adafruit_feather_m0"
$GCC_PATH = "C:\gcc-arm-none-eabi-9-2020-q2-update-win32\bin"


Clear-Host;

$ErrorActionPreference = "Stop"

Write-Host ""
Write-Host "  P U T T I N G   T O G E T H E R  "
Write-Host "==================================="
python put_src_code_together.py


Write-Host ""
Write-Host "  C O M P I L I N G  "
Write-Host "==================="
& "$ARDUINO_CLI_PATH\arduino-cli.exe" compile --fqbn adafruit:samd:adafruit_feather_m0 sketchbook/main

###& "$GCC_PATH\arm-none-eabi-objdump.exe" -D $ARDUINO_CLI_BUILD_PATH\main.ino.elf > $ARDUINO_CLI_BUILD_PATH\main.lss
###& "$GCC_PATH\arm-none-eabi-objdump.exe" -s $ARDUINO_CLI_BUILD_PATH\main.ino.elf > $ARDUINO_CLI_BUILD_PATH\mains.dump

Write-Host ""
Write-Host "  FLASHING HEX  "
Write-Host "================"


$line = & "$ARDUINO_CLI_PATH\arduino-cli.exe" board list | Select-String Adafruit
$COM_PORT =  ($line -split " ")[0]

if ([string]::IsNullOrEmpty($COM_PORT))
{
    Write-Host "Failed to find a Feather. Is it plugged?"
    exit 1
}

Write-Host "Found Feather at PORT=$COM_PORT"
& "$ARDUINO_CLI_PATH\arduino-cli.exe" upload --port $COM_PORT --fqbn adafruit:samd:adafruit_feather_m0 sketchbook/main
