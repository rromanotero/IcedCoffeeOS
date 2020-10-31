========================
SMART PLUG with JD Cloud
========================

Atmel smart plug is a highly integrated IoT solution featuring MCU, Wi-Fi
Connectivity, Security, and Sensing technology from Atmel.

The JD version is working with JD smart cloud and its Weilian APP, which is 
provided by JD, the Chinese retail Jiant.
JD smart cloud is a cloud platform for smart hardware products, providing 
IoT networking, big data analysis, mobile App etc. With JD smart cloud, smart
plug is able to be controlled locally or remotely.

Please look through the following application notes to learn about Smart Plug
reference design.

* [Atmel AT16225: Atmel Smart Plug Hardware User Guide](http://www.atmel.com/images/atmel-42689-atmel-smart-plug-hardware-user-guide_at16225_applicationnote.pdf)

* [Atmel AT16268: JD Based Smart Plug Getting Started Guide](http://www.atmel.com/images/atmel-42707-jd-smart-cloud-based-smart-plug-getting-started-guide_applicationnote_at16268.pdf)

* [Atmel AT16268: JD Based Smart Plug Getting Started Guide](http://acmsw1prd.atmel.com/zh/cn/Images/Atmel-42707-JD-Smart-Cloud-Based-Smart-Plug-Getting-Started-Guide_ApplicationNote_AT16268_Chinese.pdf)

* [Atmel AT16267: Firmware User Guide on JD Cloud Service Integration with Smart Plug](http://www.atmel.com/images/atmel-42720-firmware-user-guide-on-jd-smart-cloud-service-integration-with-smart-plug_at16267_applicationnote.pdf)

This example firmware is based on ASF V4, and user can generate the code
project through Atmel START. It uses Atmel Smart Plug kit and demostrate all
functions included in the reference design.

Prerequisites
-------------

* Atmel Smart Plug Kit
* Atmel ICE
* Wi-Fi AP 
* Smart phone (Android or iOS)
* [JD Weilian APP](http://devsmart.jd.com/app)

Hardware setup
--------------

Open the case and connect Atmel ICE onto its SWD connector.
Please refer to Chap. 6 of Application note AT16225. 


Drivers
-------
* USART_Async
* I2C_Master_Sync
* SPI_Master_Sync
* Calendar
* Timer
* GPIO
* ADC_Sync
* Ext_IRQ
* Delay

Procedure
---------

1. build this code project to get binary image (.elf)
2. program the .elf image to smart plug kit
3. Open the mobile App - JD Weilian
4. Add new device
5. control the smart plug and check its status by App, please refer to 
   the App note AT16268

Bootloader and OTAU
-------------------
By default, this example project does not support bootloader and OTAU

To support OTAU, 

* Step1: program bootloader.hex image into MCU bootloader.hex can be
  downloaded through the through following address and found in the "image"
  subfolder. (http://www.atmel.com/images/Atmel-42720-Firmware-User-Guide-on-JD-Smart-Cloud-Service-Integration-with-Smart-Plug_AT16267_ApplicationNote.zip)

* Step2: do following modification and build the 1st image APP1
  (.elf,.hex,.bin etc) APP1 uses first section of Flash
  (APP1_START_ADDRESS: 0x00004000)

  * Modify Device_Startup/samd21g18a_flash.ld
    rom      (rx)  : ORIGIN = 0x00004000, LENGTH = 0x00040000

* Step3: do following modification and build the 2nd image APP2
  (.elf,.hex,.bin etc) APP2 uses first section of Flash
  (APP2_START_ADDRESS: 0x00020000)

  * Modify Device_Startup/samd21g18a_flash.ld:
    rom      (rx)  : ORIGIN = 0x00020000, LENGTH = 0x00040000

* Step4: program APP2.elf into MCU Do not erase the bootloader in step1

* Step5: combine APP1.bin and APP2.bin into one bin file APP_FULL.bin

* Step6: Upload the combined binary file to JD cloud developer center
  (This can only be done if user created his own JD product)
