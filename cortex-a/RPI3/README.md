
NOTE:
**Currently the multicore example won't work on the physical PI when compiled with the toolchain on Docker (as explained below). I've added a windows_build.ps1 that can be used instead to both builds and run:** :

```
./windows_build.ps1
```
For some reason building it in this way makes it work. **You'll need to get the aarch64-none-elf toolchain from** [GNU-A Downloads](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-a/downloads), the exact version is in `windows_build.ps1`(look for GCC_BIN_PATH). Once downloaded make sure to place it somewhere it won't get deleted and edit GCC_BIN_PATH accordingly.


## Compiling on Windows
##### ( Docker Windows requires the FULL PATH TO THE RPI3 FOLDER to bind mount it)
```bash
docker run `
       -v C:\...\RPI3:/src `
       rromanotero/aarch64 `
       bash -c "cd src && make"
```   

### Compiling on Linux/Mac
```bash
docker run \
       -v ./RPI3:/src \
       rromanotero/aarch64 \
       bash -c "cd src && make"
```

## Running in QEMU

```
qemu-system-aarch64 -M raspi3 -kernel .\output\kernel8.img -drive file=.\sd_card\sd_card.img,if=sd,format=raw -serial stdio -serial null
```
## Running in a PI 3

You'll need a Raspberry PI 3 MOdel A+, a [USB to UART converter](https://www.adafruit.com/product/954), [PuTTY](https://www.putty.org/), and the boot files from [Raspbian Buster Lite image](https://www.raspberrypi.org/downloads/raspbian/).

1. Get an SDCard with [Raspbian Buster Lite](https://www.raspberrypi.org/downloads/raspbian/) installed on it (see [installation instructions](https://www.raspberrypi.org/documentation/installation/installing-images/README.md))
2. replace **kernel8.img in the boot partition of the SDCard** (you'll see it when reading the PI's SDCard from a laptop) with ./RPI3/output/kernel8.img
3. Insert back the SD Card onto the PI
4. Depending on the USB to UART converter, drivers may need to be installed (if you're using the one from Adafruit, they also have a tutorial)
5. Plug the PI's UART to yout laptop (via the converter) as per the [PI 3's pinout](https://pi4j.com/1.1/pins/model-a-plus.html), and access the PI from PuTTY:

  <img src="https://github.com/rromanotero/IcedCoffeeOS/blob/master/images/lab_setup_a.jpg" width="240"/>
  <img src="https://github.com/rromanotero/IcedCoffeeOS/blob/master/images/lab_setup_b.png" width="480"/>
