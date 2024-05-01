# Dettlaff

Dettlaff is a modular controller system for brushless flywheel Nerf blasters

<img src="https://user-images.githubusercontent.com/7078138/204194810-8d1cde03-946f-4490-8cfc-5caf0bca2ed5.jpg" width="500">

## Flashing Firmware over USB:
1. Install [VSCode]([https://platformio.org/install/ide?install=vscode](https://code.visualstudio.com/)) and [git](https://git-scm.com/downloads). On Windows you have to install [drivers for the CH340 USB adapter](https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all#drivers-if-you-need-them)
2. Download this repository, under the green Code button on this page, then in VSCode go to File > Open Folder and select the Dettlaff folder. If you downloaded the zip file then you need to select the inner Dettlaff folder, not the outer one. It will ask you to install the PlatformIO extension if you don't already have it
3. Power Dettlaff from a battery, and plug the USB adapter into your computer. There are 6 pins on the programmer that connect to 6 holes on the edge of Dettlaff, line up the pin with the square pad on the programmer with the hole with the square pad on Dettlaff, and insert the pins into the holes. Use the weight of the USB cable to apply a little torque to the adapter so that the pins make good contact, or solder a female header connector into the Dettlaff holes.
4. Update src/CONFIGURATION.h with your wifi name and password, and version of the board that you have, and any other settings for your blaster and then press the Upload button (arrow on the bottom toolbar)

USB Troubleshooting:

Dettlaff should enter upload mode automatically, but if it doesn't, the button on board can do it manually
* Try holding the button on the board as you hit upload, and keep holding until it says Uploading
* Try holding the button as you plug in the battery to Dettlaff, then letting go

## Flashing Firmware over WiFi:

1. First, flash a firmware over USB with your wifi SSID and password
2. In VSCode, change the Project Environment (button that says "Default" on the bottom toolbar) to "env:esp32-wifi"
3. Turn on or power cycle (unplug, wait a few seconds, and plug back in) your Dettlaff
4. Hit Upload within a few minutes of power cycling

## Wiring
* Look up in boards_config.h which pins to use for your version of Dettlaff
* Connect each switch between the numbered pin (yellow wire) and the - pin (black wire) on each connector
* Dettlaff is designed for the ESCs that we sell, but ESC connectors are not standardized. If you are using a different ESC, double check that all the pins on the ESC output match the pins on the ESC before connecting them! **This can fry your Dettlaff!**
* If you have a power switch, solder it between the - pin and the SHDN (shutdown) pin
* Solder your Dettlaff to the ESC and LiPo battery like below. The ground wire should go from the battery to the ESC, then from the ESC to the Dettlaff
![imgonline-com-ua-twotoone-U4kocRnWdYJE](https://github.com/ahalekelly/Dettlaff/assets/7078138/e3eaaa93-7a96-4e7d-b4a4-82774174daa4)

* Use the capacitors that came with the Dettlaff and the ESC. Use of capacitors with lower capacity or higher ESR could result in damage and will void your warranty
