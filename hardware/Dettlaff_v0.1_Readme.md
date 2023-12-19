# Core v0.1
## Pinout
![image](https://user-images.githubusercontent.com/7078138/157037702-bef4d977-e781-459f-ae9c-57d3b826614e.png)
## Flashing
Flashing with the FTDI is only necessary on the first flash or if the firmware crashes somehow. If the Dettlaff is running properly you can flash it over wifi. Put your wifi details into `arduino_secrets.h` before you flash, and once it flashes it should show up in the ports menu in Arduino.

There is no serial debugging over wifi, for serial debugging you can plug the FTDI adapter straight in to the Dettlaff Core
### Auto flashing issues:
* ESC2 needs to be bridged to ground to flash. Because of this, using ESC2 is not recomended.
* RTS was mistakenly labeled as CTS
* Auto reboot works, but for me (FT232 on Windows) it won't auto boot into flash mode, reasons unknown, probably a DTR/RTS timing issue
### Manual flashing instructions
Because USB auto flash is broken, this is necessary on first flash or if wifi flashing is broken
#### FTDI <> Dettlaff
* GND to GND
* TX to RX
* RX to TX
#### On Dettlaff:
* DTR to GND
* CTS to 3.3V
* ESC2 signal to GND

Make all these connections, then power up Dettlaff Core from lipo, then hit upload in Arduino

DTR, CTS, and ESC2 need to be connected at power up to put it into flashing mode but they don't need to stay connected for the entire flashing process

![IMG_20220304_162225](https://user-images.githubusercontent.com/7078138/157052359-af3dc516-6f1a-4b88-945f-a51d4a7ce641.jpg)

