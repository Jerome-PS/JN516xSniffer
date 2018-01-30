# Win32 Wireshark named pipes example
# Requires Python for Windows, the Python for Windows Extensions and PySerial:
# http://www.python.org
############ PySerial
# http://sourceforge.net/projects/pywin32/

import win32pipe, win32api, win32file
import time
import subprocess
import serial
import sys
import time
import threading
import pywintypes

# command line example: python Sniff.py COM3 C:\Users\snif\Downloads\WiresharkPortable\WiresharkPortable.exe

if(len(sys.argv)<2):
	print("This script needs at least one parameter:")
	print("\tpython Sniff.py COMx")
	print("or\tpython Sniff.py COMx path\\to\\wireshark")
	exit()

comport = sys.argv[1]

print('Use \\.\pipe\wiresharkTx to send commands from wireshark')

#ser = serial.Serial(comport, 38400, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=0, rtscts=0)
#print(ser.name)         # check which port was really used

#TODO: check for wireshark in the registry
#open Wireshark, configure pipe interface and start capture (not mandatory, you can also do this manually)
wireshark_cmd=['C:\Program Files\Wireshark\Wireshark.exe', r'-i\\.\pipe\wireshark','-k', '-X', 'lua_script:zb.lua', '-X', 'lua_script1:comport=' + r'\\.\pipe\wiresharkTx', '-X', 'lua_script1:channel=25']
if(len(sys.argv)>2):
	wireshark_cmd=[sys.argv[2], r'-i\\.\pipe\wireshark','-k', '-X', 'lua_script:zb.lua', '-X', 'lua_script1:comport=' + r'\\.\pipe\wiresharkTx', '-X', 'lua_script1:channel=25']
#print(' '.join(wireshark_cmd))
proc=subprocess.Popen(wireshark_cmd)

#create the named pipe \\.\pipe\wireshark
pipe = win32pipe.CreateNamedPipe(
    r'\\.\pipe\wireshark',
    win32pipe.PIPE_ACCESS_OUTBOUND,
#    win32pipe.PIPE_ACCESS_DUPLEX,
    win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_WAIT,
    1, 65536, 65536,
    300,
    None)

pipeTx = win32pipe.CreateNamedPipe(
    r'\\.\pipe\wiresharkTx',
    win32pipe.PIPE_ACCESS_INBOUND,
#    win32pipe.PIPE_ACCESS_DUPLEX,
    win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_WAIT,
    1, 65536, 65536,
    300,
    None)

#connect to pipes
win32pipe.ConnectNamedPipe(pipe, None)
win32pipe.ConnectNamedPipe(pipeTx, None)

# ser.write(b'hello')     # write a string
#ser.write('a')
#ser.write('a')
#ser.write('h')

# win32file.WriteFile(pipe, 0xd4); win32file.WriteFile(pipe, 0xc3); win32file.WriteFile(pipe, 0xb2); win32file.WriteFile(pipe, 0xa1);
#win32file.WriteFile(pipe, bytes([0xd4, 0xc3, 0xb2, 0xa1]) );

#win32file.WriteFile(pipe, 0x02); win32file.WriteFile(pipe, 0x00);
#win32file.WriteFile(pipe, bytes([0x02, 0x00]) );

#win32file.WriteFile(pipe, 0x04); win32file.WriteFile(pipe, 0x00);
#win32file.WriteFile(pipe, bytes([0x04, 0x00]) );

#win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00);
#win32file.WriteFile(pipe, bytes([0x00, 0x00, 0x00, 0x00]) );

#win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00);
#win32file.WriteFile(pipe, bytes([0x00, 0x00, 0x00, 0x00]) );

#win32file.WriteFile(pipe, 0xff); win32file.WriteFile(pipe, 0xff); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00);
#win32file.WriteFile(pipe, bytes([0xff, 0xff, 0x00, 0x00]) );

# C3 : LINKTYPE_IEEE802_15_4	    195	DLT_IEEE802_15_4	    IEEE 802.15.4 wireless Personal Area Network, with each packet having the FCS at the end of the frame.
# E6 : LINKTYPE_IEEE802_15_4_NOFCS	230	DLT_IEEE802_15_4_NOFCS	IEEE 802.15.4 wireless Personal Area Network, without the FCS at the end of the frame.
#win32file.WriteFile(pipe, 0xc3); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00); win32file.WriteFile(pipe, 0x00);
# win32file.WriteFile(pipe, bytes([0xc3, 0x00, 0x00, 0x00]) );
#win32file.WriteFile(pipe, bytes([0xe6, 0x00, 0x00, 0x00]) );

ser.flushInput()
ser.flushOutput()
time.sleep(.5)

print("Start loop")

def rxThread():
	while 1:
		#data = cf.read()
		data = ser.read() # read one byte
#		print("{0:x}".format(ord(data))),
		#if data == 136:
		#  print("\n")
		#sys.stdout.write( "{0:x}".format(ord(data)) )
		#sys.stdout.write( " " )
		# data = ser.read(10)        # read up to ten bytes (timeout)

		#wait 2 second (not mandatory, but this let watching data coming trough the pipe)
		#time.sleep(2)

		#send pcap data trough the pipe
		#then pcap data appears into wireshark
		win32file.WriteFile(pipe, data) 
	

threading.Thread(target=rxThread).start()

while(1):
	try:
		(hr, data) = win32file.ReadFile(pipeTx,1) 
#		print("<0:x>".format(ord(data))),
		ser.write(data) # read one byte
	except pywintypes.error as e:
		if(e[0]==109):
			win32pipe.DisconnectNamedPipe(pipeTx)
			win32api.CloseHandle(pipeTx)
			pipeTx = win32pipe.CreateNamedPipe(r'\\.\pipe\wiresharkTx', win32pipe.PIPE_ACCESS_INBOUND, win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_WAIT, 1, 65536, 65536, 300, None)
			win32pipe.ConnectNamedPipe(pipeTx, None)
		else:
			raise
	except:
		raise

ser.close()             # close port
