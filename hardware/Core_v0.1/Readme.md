# Core v0.1
5 boards, designed 2022-02-22
## Pinout
![image](https://user-images.githubusercontent.com/7078138/157037702-bef4d977-e781-459f-ae9c-57d3b826614e.png)
## Auto flashing errata:
* ESC2 needs to be bridged to ground
* RTS was mistakenly labeled as CTS
* Auto reboot works, but for me (FT232 on Windows) it won't auto boot into flash mode, reasons unknown, probably a DTR/RTS timing issue
## Manual flashing instructions
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
