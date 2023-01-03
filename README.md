# Dettlaff

Dettlaff is a modular controller system for brushless flywheel Nerf blasters

<img src="https://user-images.githubusercontent.com/7078138/204194810-8d1cde03-946f-4490-8cfc-5caf0bca2ed5.jpg" width="500">

## Flashing Firmware over USB:
1. Install [VSCode and the PlatformIO Extension](https://platformio.org/install/ide?install=vscode), and a git client, for which I use [Github Desktop](https://desktop.github.com/). On Windows you will may have to install [drivers for the CH340 USB adapter](https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all#drivers-if-you-need-them)
2. Use git to download this repository, then open the folder in VSCode
3. Power Dettlaff from a battery, and plug the USB adapter into your computer. There are 6 pins on the programmer that connect to 6 holes on the edge of Dettlaff, line up the pin with the square pad on the programmer with the hole with the square pad on Dettlaff, and insert the pins into the holes. Use the weight of the USB cable to apply a little torque to the adapter so that the pins make good contact, or solder a female header connector into the Dettlaff holes.
4. Update src/main.cpp with your wifi name and password, and the config settings for your blaster and then press the Upload button (arrow on the bottom toolbar)

USB Troubleshooting:

Dettlaff should enter upload mode automatically, but if it doesn't, the button on board can do it manually
* Try holding the button as you hit upload, and keep holding until it says Uploading
* Try holding the button on the board as you plug in the battery to Dettlaff, then letting go

## Flashing Firmware over WiFi:

1. First, flash a firmware over USB with your wifi SSID and password
2. Change the Project Environment (button that says "Default" on the bottom) to "env:esp32-wifi"
3. Power cycle your Dettlaff
4. Hit Upload within a few minutes of power cycling
