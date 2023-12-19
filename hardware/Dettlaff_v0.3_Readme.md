## Flashing Instructions:
1. [Install VSCode and the PlatformIO Extension](https://platformio.org/install/ide?install=vscode)
2. Linux and Mac come with the CH340G serial adapter drivers, on Windows you can install them from [here](https://sparks.gogo.co.nz/assets/_site_/downloads/CH34x_Install_Windows_v3_4.zip)
3. Use `git clone`, Github Desktop, or VSCode to download this repository. **Github's Download Zip button does not include the required libraries!**
4. Open the Dettlaff directory in VSCode, set your WiFi SSID and password in the source code so that you can program over wifi later
5. On the bottom bar, click `Default (Dettlaff)` and select `env:esp32-usb` for USB programming
6. Power your Dettlaff from a battery, connect the USB-C adapter to your computer, insert the adapter pins into the holes and let the adapter rest at an angle (no need to solder), and hit the PlatformIO: Upload button on the bottom bar (the arrow)
