

## Compiling on Windows
##### ( Docker Windows requires the FULL PATH IcedCoffeeOS to bind mount it)
```bash
docker run `
       -v  C:\Users\...\IcedCoffeeOS:/src `
       rromanotero/arm-none-eabi `
       bash -c "cd src/kernel/feather_rfm69 && make"
```   

### Compiling on Linux/Mac
```bash
docker run \
       -v ./IcedCoffeeOS:/src \
       rromanotero/arm-none-eabi \
       bash -c "cd src/kernel/feather_rfm69 && make""
```

## Flashing the Feather M0 board (atprogram + Atmel SAM ICE)

```bash
& "PATH_TO_AT_PROGRAM\atprogram.exe" -t samice -i swd -d atsamd21g18a -l output/samice_output.log -cl 4Mhz program -c -f ".\output\feather_m0.hex" --verify
```

First install Atmel Studio and find `atprogram.exe`. For example, in my cmputer this is at `C:\Program Files (x86)\Atmel\Studio\7.0\atbackend`

## Flashing the Feather M0 board (Arduino bootloader)


Feather M0 is shipped with an Arduino bootloader, and it can be used to flash a binary:
[Link here](https://reprapdad.wordpress.com/2016/08/19/adafruit-feather-m0-with-atmel-studio/)
