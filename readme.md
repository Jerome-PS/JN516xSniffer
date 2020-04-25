# Transform a Xiaomi Zigbee gadget into a Zigbee sniffer.

This has been tested on the Xiaomi Smart Button.

## Quickstart on Linux
Start sniffing session from terminal:
```
./Sniff.py /dev/ttyUSB0
```
Running `./Sniff.py -h` will give you a list of detected serial ports.

## Quickstart on macOS
Start sniffing session from terminal:
```
./Sniff.py /dev/cu.usbserial
```
Running `./Sniff.py -h` will give you a list of detected serial ports.

## Quickstart on Windows
Open command line, cd to the folder containing the scripts and launch:
```
python Sniff.py COM3
```
Running `python Sniff.py -h` will give you a list of detected serial ports.

# Table Of Content
- [Compiling the sniffer](#Compiling-the-sniffer)
	- [Precompiled binary](#Precompiled-binary)
	- [Compiling on macOS or Linux](#Compiling-on-macOS-or-Linux)
	- [Compiling on Windows](#Compiling-on-Windows)
- [Interfacing to Wireshark](#Interfacing-to-Wireshark)
- [Hardware](#Hardware)
	- [Xiaomi Door sensor](#Xiaomi-Door-sensor)
	- [Xiaomi smart button](#Xiaomi-smart-button)
	- [Xiaomi smart button UART1](#Xiaomi-smart-button-UART1)
- [Using the GUI to set the preferences](#Using-the-GUI-to-set-the-preferences)
- [Troubleshooting](#Troubleshooting)
- [To do list](#To-do-list)
- [Gory details](# Gory-details)

# Compiling the sniffer
You will need the NXP JN-SW-4163 SDK in order to compile the source code.

## Precompiled binary
There is a precompiled binary in the [bin folder](https://github.com/Jerome-PS/JN516xSniffer/tree/master/bin). **TODO: Check that it is up to date...**

## Compiling on macOS or Linux
Follow the instructions found here: https://github.com/alephsecurity/BA2-toolchain

You can use the latest version of [JennicModuleProgrammer](https://github.com/Jerome-PS/JennicModuleProgrammer) in order to flash you JN5169 device.

## Compiling on Windows
You can download NXP's Beyond Studio in order to compile the source code. In eclipse, use the import from C/C++ Makefile menu.
You can then use the programmer integrated in the IDE. Do not use any other programmer provided by NXP, because they do not support JN5169 (at least none that I could try out).

# Using the sniffer
## Using on macOS or Linux
On Linux, you will need to be in the dialout group, in order to have enough access rights to access the serial port, as well as in the wireshark group for executing pcap.
The script will prompt you to do it if it detects the issue.
```
sudo usermod -a -G dialout $USER
sudo usermod -a -G wireshark $USER
```

## Using on Windows
If you are using Windows, you will have to run the Sniffer.py script (you'll need the win32api and PySerial modules). You must pass the serial port name as the first parameter and optionally the wireshark.exe path as the second argument.
Sadly, lua script parameter forwarding does not seem to work. So you might have to use the GUI in order to set your preferences.
You can use an installed version of Wireshark or WiresharkPortable.
You will need to run the python script as follows (from the directory where the .py and .lua scripts are and change the **COM** port and **path** as needed):
```
python Sniff.py COM3 C:\Users\snif\Downloads\WiresharkPortable\WiresharkPortable.exe
```

## General usage
You must send a start command in order to initialize Wireshark and the sniffer device and subsequently get packets. Use the ZB menu. **You will not see anything in Wireshark until you send the start command using the Tools/ZB/ZB Start menu command.**
![ZB menu](https://github.com/Jerome-PS/JN516xSniffer/blob/master/doc/WS_menu_ZB.png)
Please make sure to select the correct channel, or you won't capture any frames. The Sniffer will send you a dummy frame to indicate the current channel every time you change it.
![ZB dialog](https://github.com/Jerome-PS/JN516xSniffer/blob/master/doc/WS_dialog_Options.png)

# Using the GUI to set the preferences
![Preferences menu](https://github.com/Jerome-PS/JN516xSniffer/blob/master/doc/WS_menu.png)
![Preferences dialog](https://github.com/Jerome-PS/JN516xSniffer/blob/master/doc/WS_dialog.png) 
Note that changing the parameters in this dialog will try and send an update to the MCU.

# Hardware
## Xiaomi Door sensor
Wiring colors are :
1. Purple: Tx (OUT from MCU)
2. Green : Rx (IN  to   MCU)
3. Bleue : nBootloader
4. Red   : 3V3
5. White : nReset
6. Black : GND
You can optionally connect the nReset signal to your serial port nRTS signal and your nBootloader to the nDTR output.
![Xiaomi_smart_button](https://github.com/Jerome-PS/JN516xSniffer/blob/master/doc/Xiaomi_Door_sensor.JPG)

## Xiaomi smart button
1. Purple: Tx (OUT from MCU)
2. Green : Rx (IN  to   MCU)
3. Bleue : nBootloader
4. Red   : 3V3
5. White : nReset
6. Black : GND
You can optionally connect the nReset signal to your serial port nRTS signal and your nBootloader to the nDTR output.
![Xiaomi_smart_button](https://github.com/Jerome-PS/JN516xSniffer/blob/master/doc/Xiaomi_smart_button.JPG)

## Xiaomi smart button UART1
1. For UART1 Tx, on this side of the resistor, we have the CPU pin directly.
2. You can find some GND on this tantalum capacitor.
![Xiaomi_smart_button_UART1](https://github.com/Jerome-PS/JN516xSniffer/blob/master/doc/Xiaomi_smart_button_UART1.JPG)


# Acknowledgements
This project is based on [work](https://github.com/KiwiHC16/ZigBeeSniffer) from @KiwiHC16 . He also helped me debug this documentation and a few version incompatibilities.

# Troubleshooting
Wireshark might get confused if your MCU sends data before Wireshark has been initialised properly. To avoid this, please reset your MCU and ask it to send data with the ZB/start menu.

You can debug the communication with the following command:
```
stty -f /dev/cu.usbserial 38400 raw & cat /dev/cu.usbserial | tee /tmp/sf.bin > /tmp/sharkfifo
```
You can view the file content like this:
```
hexdump -C /tmp/sf.bin
```

The lua script creates a file named lua.log in the folder wireshark is started.

You can also pass the dissector parameters through environment varaibles:
```
env ZBL_CHANNEL=12 ZBL_COMPORT=/dev/ttyUSB1 wireshark -X lua_script:zb.lua -k -i /tmp/sharkfifo &
```

## FIFO troubles (obsolete)

If you get stuck with remaining data in the FIFO that repeatedly crashes wireshark, you can destroy the pipe and re-create it:
```
rm -f /tmp/sharkfifo && mkfifo /tmp/sharkfifo
```
This should not be necessary any more, as the FIFOs are destroyed when leaving the Sniff.py script.

## Python version
The Sniff.py script has been developped using Python3.6, so this, or a more recent, version of Python3 is recommended. It has not been extensively tested using Python2.

# To do list
- Add LQI information to the packet
- Add frame duration computation as well as inter frame gap (wireshark)

# Gory details
Original serial queuing functions took 43125us for 26 bytes (166us/byte) and 4923 us for 31 bytes (159us/byte) with a 115200 baud/s UART
and 10272us for 26 bytes (395us/byte) and 12860 for 31 bytes (415us/byte) with a 1MBaud/s UART!!! What the hell?
memcpy algo takes 636 us for 26 bytes (24us/byte) and 848 for 43 bytes (20us/byte).
The longer runtime/byte might come from the fact that the likeliness of having time stolen by an ISR is higher if you take more time. I might do some additional tests with disabled IT to try and get more consistent results...

## Packets timestamps
At 2.4GHz, Zigbee uses a 62500 Hz symbol clock. These symbols encode 4 bits each, providing an on the air bitrate of 250Kbits/s.

Packets are timed with this clock, and have therefore a 16µs resolution.

## Timings
Send ACK between 
macSIFSPeriod = 12 symbols >> 192µs
and 
macAckWaitDuration = macSIFSPeriod + phySHRDuration + ceiling(7 × phySymbolsPerOctet) = 12 symbols + 

bits are 250kHz (4µs), symbols are 62.5kHz (16-ary, so 4 bits/symbol)
192µs < Tack < 512µs

Inter Frame Spacing
If lengthMPDU ≤ aMaxSIFSFrameSize (18 octets)
then,
symbolsIFS ≥ aMinSIFSPeriod = 12 symbols (192µs)
else,
symbolsIFS ≥ aMinLIFSPeriod = 40 symbols (640µs)

![MPDU](https://github.com/Jerome-PS/JN516xSniffer/blob/master/doc/MPDU.png)

# TODO
- Check IT priority, so UART does not prevent packet management at the radio level.
- Fill Tx FIFO before starting Tx ISR to limit the number of ISR.

