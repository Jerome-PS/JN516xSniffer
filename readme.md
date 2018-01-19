Transform a Xiaomi Zigbee gadget into a Zigbee sniffer.

This has been tested on the Smart Button.

Compilation using ba-elf-gcc 4.7.4 is a PITA, as some paths are not recognised, even when recompiling the compiler with the correct prefix.
Particularly, in macOS, if you install the compiler suite in /usr/local/ you have to create a link to as in /usr/local/lib/gcc/ba-elf/:
```
	$ cd /usr/local/lib/gcc/ba-elf/
	$ ln -s /usr/local/ba-elf/bin/as
```
Also include dirs do not seem to be properly managed, as the compiler won't find <stdint.h>, so I hardcoded its location in the Makefile (if someone could improve on this it would be perfect!)
Also LinkTimeOptimization doesn't worl on macOS, so it is forcefully disabled in the Makefile.

You can use the latest version of [JennicModuleProgrammer](https://github.com/Jerome-PS/JennicModuleProgrammer) in order to flash you JN5169 device.

~~Put the zb.lua Wireshark plugin in your ~/.wireshark/plugins folder.~~
~~If you use WiresharkPortable, copy the script in WiresharkPortable\App\Wireshark\plugins.~~

If you are using Windows, you will have to run the Sniffer.py script (you'll need the win32api and PySerial modules). You must pass the serial port name as the first parameter and optionally the wireshark.exe path as the second argument.
Sadly, lua script parameter forwarding does not seem to work. So you might have to use the GUI in order to set your preferences.

If you are running Linux or macOS, you will have to create a pipe:
```
mkfifo /tmp/sharkfifo
```
And then route data from the serial port, passing the serial port for commands as a parameter and optionally the channel number:
```
wireshark -k -i /tmp/sharkfifo -X lua_script:zb.lua -X lua_script1:comport=/dev/cu.usbserial -X lua_script1:channel=25 &
stty -f /dev/cu.usbserial 38400 raw & cat /dev/cu.usbserial > /tmp/sharkfifo
```

You must send a start command in order to initialize Wireshark and the sniffer device. Use the ZB menu.
![ZB menu]()

# Using the GUI to set the preferences
![Preferences menu]()
![Preferences dialog]() 

This project is based on [work](https://github.com/KiwiHC16/ZigBeeSniffer) from @kiwiHC16.

