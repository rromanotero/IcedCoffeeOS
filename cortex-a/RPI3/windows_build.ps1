
<#
#	 Params
#	 (edit path to GCC and QEMU here)
#>

$GCC_BIN_PATH = "C:\Program Files\gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf.tar\gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf\bin"
$QEMU_BIN_PATH = "C:\Program Files\qemu"



<#
#	 Build process
#>

Clear-Host;

$ErrorActionPreference = "Stop"

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

set "cpuflags=-Wall -O3 -march=armv8-a+simd -mtune=cortex-a53 -mstrict-align -fno-tree-loop-vectorize -fno-tree-slp-vectorize"
set "asmflags=-nostdlib -nostartfiles -ffreestanding -fno-asynchronous-unwind-tables -fomit-frame-pointer -Wa,-a>output/list.txt"
set "linkerflags=-Wl,-gc-sections -Wl,--build-id=none -Wl,-Bdynamic -Wl,-Map,output/kernel.map"
set "outflags=-o output/kernel8.elf"
set "libflags=-lc -lm -lgcc"


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
