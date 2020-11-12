The Feather M0 is powered by the ATSAMD21G18A. In this port IcedCoffee is built on Arduino libraries.

## Building and flashing with Arduino IDE
For setting up Arduino please follow the instructions at [Adafruit Feather M0 Radio with RFM69 Packet Radio](https://learn.adafruit.com/adafruit-feather-m0-radio-with-rfm69-packet-radio/setup). The only catch is that some later versions of Arduino won't let you install Adafruit's SAMD Boards. FWIW I installed Arduino 1.8.1 (as a ZIP), and set up Adafruit SAMD Boards version 1.6.3.


## Building and flashing with Arduino CLI (Windows)

### Initial set up

First Get [Arduino CLI](https://arduino.github.io/arduino-cli/latest/installation/). Then install Adafruit's libraries (the one(s) in `./arduino-yml.cli`):

```ps1
cd feather_m0_rfm69_arduino
& '$ARDUINO_CLI_PATH\arduino-cli.exe' core update-index
Updating index: package_index.json downloaded
Updating index: package_index.json.sig downloaded
```

This allows you to search for boards (here you should see Arduino SAMD board):

```
& '$ARDUINO_CLI_PATH\arduino-cli.exe' core search adafruit
ID                Version Name
arduino:avr  1.8.3   Arduino AVR Boards
arduino:samd 1.8.9   Arduino SAMD Boards (32-bits ARM Cortex-M0+) <<--- WE NEED THIS ONE
& '$ARDUINO_CLI_PATH\arduino-cli.exe' core search samd
ID                Version Name
Arrow:samd        2.1.0   Arrow Boards
arduino:samd      1.8.9   Arduino SAMD Boards (32-bits ARM Cortex-M0+)
arduino:samd_beta 1.6.25  Arduino SAMD Beta Boards (32-bits ARM Cortex-M0+) <<--- WE NEED THIS ONE
```

Now install:
 ```
& '$ARDUINO_CLI_PATH\arduino-cli.exe' core install adafruit:samd
Platform adafruit:samd@1.6.3 already installed
& '$ARDUINO_CLI_PATH\arduino-cli.exe' core install arduino:samd
Platform arduino:samd@1.8.9 already installed
 ```
The Feather M0 board should now appear listed:
```
& '$ARDUINO_CLI_PATH\arduino-cli.exe' board listall
Board Name                              FQBN
...
Adafruit Feather M0                     adafruit:samd:adafruit_feather_m0
...
```

Now if you connect the Feather M0 to your computer's USB you should be able to list it:
```
& '$ARDUINO_CLI_PATH\arduino-cli.exe' board list
Port Type              Board Name          FQBN                              Core
COM8 Serial Port (USB) Adafruit Feather M0 adafruit:samd:adafruit_feather_m0 adafruit:samd
```

There's some libraries that need manual installations. Copy the contents of `./sketchbook/libraries` (in this repo) to your local `sketchbook/libraries` directory (i.e. where the Arduino CLI will look for libraries when compiling).

Lastly, look for files in `./sketchbook/hardware/samd` (in this repo), and replace those with same name in your local hardware directory. In my case this was `C:\Users\USERNAME\AppData\Local\Arduino15\packages\adafruit\hardware`. Please note this folder won't exist until after installing Adafruit boards. So for example, if you look at `./sketchbook/hardware/samd`, you'll find the file `./sketchbook/hardware/samd/1.6.3/cores/arduino/WInterrupts.c` . Using this file replace  `WInterrupts.c` in your local installation. 

### Building and Flashing

Edit the path `ARDUINO_CLI_PATH` to reflect where you have installed Arduino CLI. Then:

```
.\windows_build.ps1
```

**Note that the first time it's run the Arduino CLI's upload command will complain with this**:

```
Error during Upload: uploading error: cannot execute upload tool: exec: "{runtime.tools.bossac-1.7.0-arduino3.path}/bossac.exe": file does not exist
```

In my case I solved it by pointing the var `tools.bossac.path` in `platform.txt` (C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\adafruit\hardware\samd\1.6.3) to the my bossac installation (in my case that was C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\arduino\tools\bossac\1.7.0-arduino3).

Arduino SAMD core code is at
```
C:\Users\rafael.romanotero\AppData\Local\Arduino15\packages\adafruit\hardware\samd\1.6.3
```
