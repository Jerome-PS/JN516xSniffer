# Transform a Xiaomi Zigbee gadget into a Zigbee sniffer.

This has been tested on the Xiaomi Smart Button.

## Quickstart on Linux
Once per session (until you reboot the computer for example), create the pipe and start wireshark:
```
mkfifo /tmp/sharkfifo
wireshark -k -i /tmp/sharkfifo -X lua_script:zb.lua -X lua_script1:comport=/dev/ttyUSB0 -X lua_script1:channel=25 &
```
When capture stopped and the pipe broke, restart the capture in wireshark and rebuild the pipe:
```
stty -F /dev/ttyUSB0 38400 raw ignbrk -onlcr -iexten -echo -echoe -echok -echoctl -echoke & cat /dev/ttyUSB0 | tee /tmp/sf.bin > /tmp/sharkfifo
```

## Quickstart on macOS
Once per session (until you reboot the computer for example), create the pipe and start wireshark:
```
mkfifo /tmp/sharkfifo
wireshark -k -i /tmp/sharkfifo -X lua_script:zb.lua -X lua_script1:comport=/dev/cu.usbserial -X lua_script1:channel=25 &
```
When capture stopped and the pipe broke, restart the capture in wireshark and rebuild the pipe:
```
stty -F /dev/cu.usbserial 38400 raw & cat /dev/cu.usbserial | tee /tmp/sf.bin > /tmp/sharkfifo
```

## Quickstart on Windows
Open command line, cd to the folder containing the scripts and launch:
```
python Sniff.py COM3 C:\Users\snif\Downloads\WiresharkPortable\WiresharkPortable.exe
```

# Compiling the sniffer
You will need the NXP JN-SW-4163 SDK in order to compile the source code.

## Precompiled binary
There is a precompiled binary in the [bin folder](https://github.com/Jerome-PS/JN516xSniffer/tree/master/bin). **TODO: Check that it is up to date...**

## Compiling on macOS or Linux
Compilation using ba-elf-gcc 4.7.4 is a PITA, as some paths are not recognised, even when recompiling the compiler with the correct prefix.
Particularly, in macOS, if you install the compiler suite in /usr/local/ you have to create a link to as in /usr/local/lib/gcc/ba-elf/:
```
	$ cd /usr/local/lib/gcc/ba-elf/
	$ ln -s /usr/local/ba-elf/bin/as
```
Also include dirs do not seem to be properly managed, as the compiler won't find <stdint.h>, I hardcoded its location in the Makefile (if someone could improve on this it would be perfect!)
Also LinkTimeOptimization doesn't work on macOS, so it is forcefully disabled in the Makefile.

You can use the latest version of [JennicModuleProgrammer](https://github.com/Jerome-PS/JennicModuleProgrammer) in order to flash you JN5169 device.

## Compiling on Windows
You can download NXP's Beyond Studio in order to compile the source code. In eclipse, use the import from C/C++ Makefile menu.
You can then use the programmer integrated in the IDE. Do not use any other programmer provided by NXP, because they do not support JN5169 (at least none that I could try out).

# Interfacing to Wireshark
~~Put the zb.lua Wireshark plugin in your $(HOME)/.wireshark/plugins folder.
If you use WiresharkPortable, copy the script in WiresharkPortable\App\Wireshark\plugins.~~

## Using on macOS or Linux
On Linux, you will need to be in the dialout group, in order to have enough access rights to access the serial port, as well as in the wireshark group for executing pcap:
```
sudo usermod -a -G dialout $USER
sudo usermod -a -G wireshark $USER
```
If you are running Linux or macOS, you will have to create a pipe:
```
mkfifo /tmp/sharkfifo
```

And then route data from the serial port, passing the serial port for commands as a parameter and optionally the channel number (change **comport** and **channel** as needed):
```
wireshark -k -i /tmp/sharkfifo -X lua_script:zb.lua -X lua_script1:comport=/dev/cu.usbserial -X lua_script1:channel=25 &
```
Sadly here we have a small difference between macOS and Linux (notice capital letter F or small print f)!
This is the Linux version (raw is actually now that raw, so a lot of things have to be switched off manually):
```
stty -F /dev/ttyUSB0 38400 raw ignbrk -onlcr -iexten -echo -echoe -echok -echoctl -echoke & cat /dev/ttyUSB0 > /tmp/sharkfifo
```
This is the macOS version:
```
stty -f /dev/cu.usbserial 38400 raw & cat /dev/cu.usbserial > /tmp/sharkfifo
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
If you get stuck with remaining data in the FIFO that repeatedly crashes wireshark, you can destroy the pipe and re-create it:
rm -f /tmp/sharkfifo && mkfifo /tmp/sharkfifo

