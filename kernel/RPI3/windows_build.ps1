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
#	 (edit path to GCC and QEMU here)
#>

$GCC_BIN_PATH = "C:\Program Files\gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf.tar\gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf\bin"
$QEMU_BIN_PATH = "C:\Program Files\qemu"

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

rm ./output/*.img
rm ./output/*.elf
rm ./output/*.lss
rm ./output/*.dump
rm ./output/*.map
rm ./output/*.txt

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

Write-Host "  C R E A T I N G    I M G,  L S S,  etc   "
Write-Host "============================================"

& "$GCC_BIN_PATH\aarch64-none-elf-objcopy.exe" .\output\kernel8.elf -O binary .\output\kernel8.img
& "$GCC_BIN_PATH\aarch64-none-elf-objdump.exe" -D .\output\kernel8.elf | Out-File -filepath output/kernel8.lss -Encoding ASCII
& "$GCC_BIN_PATH\aarch64-none-elf-objdump.exe" -s .\output\kernel8.elf | Out-File -filepath output/kernel8.dump -Encoding ASCII

if ($LASTEXITCODE -ne 0)
{
	Write-Host "CREATING IMG et al FAILED"
	exit 1
}
Write-Host "SUCCESS"


Write-Host "            R U N N I N G   "
Write-Host "====================================="
Write-Host "        PI'S  UART0          "
Write-Host "= = = = = = = = = = = = = = "
& "$QEMU_BIN_PATH\qemu-system-aarch64.exe" `
			-M raspi3 `
			-kernel output/kernel8.img  `
			-drive file=.\sd_card\sd_card_many_files.img,if=sd,format=raw `
			-serial stdio -serial null
