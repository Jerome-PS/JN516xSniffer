# Win32 Wireshark named pipes example
# Requires Python for Windows, the Python for Windows Extensions and PySerial:
# http://www.python.org
############ PySerial
# http://sourceforge.net/projects/pywin32/

# command line example: python Sniff.py COM3 C:\Users\snif\Downloads\WiresharkPortable\WiresharkPortable.exe

bIsWindows = False
bIsLinux   = False
bIsmacOS   = False
bIsPosix   = False

def main(argv):
	import argparse
	parser = argparse.ArgumentParser(description="interface between wireshark and a JN516x chip used as a ZigBee traffic sniffer")
	parser.add_argument("comport", help="serial interface (COMx or /dev/ttyx) to the sniffer")
	group = parser.add_mutually_exclusive_group()
	group.add_argument("-ws", "--wireshark", nargs='?', const='#@search', default='#@search', help="launch wireshark at given path")
	group.add_argument("-nws", "--no-wireshark", help="do not launch wireshark", action="store_true")
	parser.add_argument("-c", '--channel', type=int, choices=range(11,26+1), help="set ZigBee radio channel")
	parser.add_argument("-V", '--version', action='version', version='%(prog)s 1.0')
	args = parser.parse_args()

	global bIsWindows, bIsLinux, bIsmacOS, bIsPosix
	import platform
	if(int(platform.python_version_tuple()[0])!=3):
		print("WARNING this program has only been tested with Python 3!")
	if(platform.system()=="Windows"):
		bIsWindows = True
		print("Detected platform is Windows")
	elif(platform.system()=="Linux"):
		bIsLinux = True
		bIsPosix = True
		print("Detected platform is Linux")
	elif(platform.system()=="Darwin"):
		bIsmacOS = True
		bIsPosix = True
		print("Detected platform is macOS")
	else:
		print("Warning: unknown platform '%s', assuming Linux" % platform.system())


	try:
		import serial
	except ImportError as e:
#	except ModuleNotFoundError as e:
		print("~~~~~~~~~~~~~~~~~~")
		print("Error, cannot load python serial module")
		print('\tTry: "%s" -m pip install pyserial' % platform.sys.executable)
		if(bIsLinux):
			print("\tOn Debian/Ubuntu, you might need to execute this first: sudo apt install python3-pip")
			print("\tOn Debian/Ubuntu, if the above does not work, try: sudo apt-get install python3-serial")
		print("##################")
		print("# Cannot proceed #")
		print("##################")
		raise e
		exit(-1)

	if(bIsWindows):
		try:
			import win32pipe
		except ImportError as e:
#		except ModuleNotFoundError as e:
			print("~~~~~~~~~~~~~~~~~~")
			print("Error, cannot load python win32 module")
			print('\tTry: "%s" -m pip install pywin32' % platform.sys.executable)
			print("~~~~~~~~~~~~~~~~~~")
			print("If you get a DLL load exception try running as an administrator")
			print('\t"%s" scripts\pywin32_postinstall.py -install from a dir like C:\Program Files\Python36' % platform.sys.executable)
			print("~~~~~~~~~~~~~~~~~~")
			print("##################")
			print("# Cannot proceed #")
			print("##################")
			raise e
		
	wiresharkPath = None
	if(args.wireshark):
		if(args.wireshark != '#@search'):
			wiresharkPath = args.wireshark
		else:
			if(bIsPosix):
				wiresharkPath = 'wireshark'
			elif(bIsWindows):
				import winreg
#				regkey = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, "SOFTWARE\Microsoft\Windows\CurrentVersion\App Paths\Wireshark.exe")
				wiresharkPath = winreg.QueryValue(winreg.HKEY_LOCAL_MACHINE, "SOFTWARE\Microsoft\Windows\CurrentVersion\App Paths\Wireshark.exe")

	if(bIsWindows):
		pipeRxName = r'\\.\pipe\wiresharkRx'
		pipeTxName = r'\\.\pipe\wiresharkTx'
	else:
		pipeRxName = '/tmp/wiresharkRx'
		pipeTxName = '/tmp/wiresharkTx'
	pipeRx = PyFIFO(pipeRxName, "w")
	pipeTx = PyFIFO(pipeTxName, "r")

	#import win32pipe, win32api, win32file, winerror, pywintypes
	import subprocess
	import time
	import threading

	try:
		ser = serial.Serial(args.comport, 38400, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=0, rtscts=0)
#	except PermissionError:
	except Exception as e:
		print("~~~~~~~~~~~~~~~~~~")
		print("Error, cannot open serial port '%s'", args.comport)
		print('\tTry: sudo usermod -a -G dialout $USER')
		print("~~~~~~~~~~~~~~~~~~")
		raise e
		
	ser.rts = 0	# nRESET when wired
	ser.dtr = 0	# nPROG  when wired
	print("Using serial port '%s' at %d Baud" % (ser.name, ser.baudrate))         # check which port was really used

	#TODO: check for wireshark in the registry
	#open Wireshark, configure pipe interface and start capture (not mandatory, you can also do this manually)
	wireshark_cmd=[wiresharkPath, '-i'+pipeRxName, '-k', '-X', 'lua_script:zb.lua', '-X', 'lua_script1:comport=' + pipeTxName]
	if(args.channel):
		wireshark_cmd+=['-X', 'lua_script1:channel=%d' % args.channel]
	if(not args.no_wireshark):
		print("launching " + ' '.join(wireshark_cmd))
		proc=subprocess.Popen(wireshark_cmd)

	# C3 : LINKTYPE_IEEE802_15_4	    195	DLT_IEEE802_15_4	    IEEE 802.15.4 wireless Personal Area Network, with each packet having the FCS at the end of the frame.
	# E6 : LINKTYPE_IEEE802_15_4_NOFCS	230	DLT_IEEE802_15_4_NOFCS	IEEE 802.15.4 wireless Personal Area Network, without the FCS at the end of the frame.

	ser.flushInput()
	ser.flushOutput()
	time.sleep(.5)

	print("Start loop")

	def rxThread():
		#connect to pipes
#		print("Connecting to pipeRx")
#		pipeRx.open()
#		initseq = bytearray([0xD4, 0xC3, 0xB2, 0xA1 , 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xE6, 0x00, 0x00, 0x00])
#		print("writing data", initseq, pipeRx.bIsOpen)
#		pipeRx.write(initseq)
#		pipeRx.write(bytearray([0x41]))
#		print("Written")
		while 1:
			data = ser.read() # read one byte
			print("rx", data)
			try:
				if(not pipeRx.bIsOpen):
					print("Trying to open Rx pipe")
					pipeRx.tryOpen()
					if(pipeRx.bIsOpen):
						print("Sending wireshark init sequence")
# E6 : LINKTYPE_IEEE802_15_4_NOFCS   230	DLT_IEEE802_15_4_NOFCS   IEEE 802.15.4 wireless Personal Area Network, without the FCS at the end of the frame.
#						pipeRx.write(bytearray([0xD4, 0xC3, 0xB2, 0xA1 , 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xE6, 0x00, 0x00, 0x00]))
# C3 : LINKTYPE_IEEE802_15_4         195	DLT_IEEE802_15_4         IEEE 802.15.4 wireless Personal Area Network, with each packet having the FCS at the end of the frame.
						pipeRx.write(bytearray([0xD4, 0xC3, 0xB2, 0xA1 , 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xC3, 0x00, 0x00, 0x00]))
				if(pipeRx.bIsOpen):
					pipeRx.write(data)
			except FIFOClosedException as e:
				print("Clearing pipe")
				pipeRx.clear()

	threading.Thread(target=rxThread).start()

	print("Connecting to pipeTx")
	pipeTx.open()
	cmdbuf = bytearray()
	while(1):
		try:
			data = pipeTx.read()
			print("tx", data)
			cmdbuf += data
			ser.write(data) # read one byte
			ser.flush()
			if(data==b'\n'):
#				print("detected CR", cmdbuf)
				if(cmdbuf==b"BRD:1000000\n"):
					print("Switching to 1Mbaud")
					ser.baudrate = 1000000
				cmdbuf = bytearray()
		except FIFOClosedException as e:
			pipeTx.open()

	ser.close()             # close port

class FIFOClosedException(Exception):
	def __init__(self, *args, **kwds):
		Exception.__init__(self, *args, **kwds)
	

class PyFIFO():
	def __init__(self, path, mode, *args, **kwds):
		self.path = path
		self.mode = mode
		self.fileobject = None
		self.bIsWindows = False
		self.bIsLinux   = False
		self.bIsmacOS   = False
		self.bIsPosix   = False
		self.bIsOpen    = False
		import platform
		if(platform.system()=="Windows"):
			self.bIsWindows = True
		elif(platform.system()=="Linux"):
			self.bIsLinux = True
			self.bIsPosix = True
		elif(platform.system()=="Darwin"):
			self.bIsmacOS = True
			self.bIsPosix = True
		else:
			print("Warning: unknown platform '%s', assuming Linux" % platform.system())

		if(self.bIsWindows):
			import win32pipe
			if(mode=="r"):direction=win32pipe.PIPE_ACCESS_OUTBOUND
			elif(mode=="w"):direction=win32pipe.PIPE_ACCESS_INBOUND
			self.fileobject = win32pipe.CreateNamedPipe(
				self.path,
				direction,
				#    win32pipe.PIPE_ACCESS_DUPLEX,
				win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_WAIT,
				1, 65536, 65536,
				300,
				None)
		elif(bIsPosix):
			import os
			self.delfifo()
			os.mkfifo(self.path)
			import atexit
			atexit.register(self.delfifo)


	def delfifo(self):
		try:
			import os
			os.remove(self.path)
			print("delete pipe : " + self.path)
		except FileNotFoundError as e:
			print("not found")
			pass

	def open(self):
		if(self.bIsWindows):
			import win32pipe
			win32pipe.ConnectNamedPipe(self.fileobject, None)
		elif(self.bIsPosix):
			import os
			if(self.mode=="r"):	mode = os.O_RDONLY
			elif(self.mode=="w"):	mode = os.O_WRONLY | os.O_NONBLOCK
			try:
				self.fileobject = os.open(self.path, mode)
			except OSError as e:
				if(e.errno!=6):
					raise e
				else:
					self.bIsOpen    = False
					return False
			else:
				self.bIsOpen    = True
				return True

	def tryOpen(self):
		if(self.bIsWindows):
			pass
		elif(self.bIsPosix):
			import os
			if(self.mode=="r"):	mode = os.O_RDONLY
			elif(self.mode=="w"):	mode = os.O_WRONLY | os.O_NONBLOCK
			try:
				self.fileobject = os.open(self.path, mode)
			except OSError as e:
				if(e.errno!=6):
					raise e
				else:
					self.bIsOpen    = False
					return False
			else:
				self.bIsOpen    = True
				return True

	def clear(self):
		import os
		self.delfifo()
		os.mkfifo(self.path)

#	def reopen(self):
#		if(self.bIsWindows):
#			import win32pipe
#			win32pipe.ConnectNamedPipe(self.fileobject, None)
#		elif(self.bIsPosix):
#			import os
#			self.delfifo()
#			os.mkfifo(self.path)
#			if(self.mode=="r"):	mode = os.O_RDONLY
#			elif(self.mode=="w"):	mode = os.O_WRONLY | os.O_NONBLOCK
#			try:
#				self.fileobject = os.open(self.path, mode)
#			except OSError as e:
#				if(e.errno!=6):
#					raise e
#				else:
#					self.bIsOpen    = False
#					return False
#			else:
#				self.bIsOpen    = True
#				return True

	def read(self):
		if(self.bIsWindows):
			try:
				(hr, data) = win32file.ReadFile(self.fileobject,1) 
			except pywintypes.error as e:
				if(e.winerror==winerror.ERROR_BROKEN_PIPE):
					win32pipe.DisconnectNamedPipe(pipeTx)
					win32api.CloseHandle(pipeTx)
					pipeTx = win32pipe.CreateNamedPipe(r'\\.\pipe\wiresharkTx', win32pipe.PIPE_ACCESS_INBOUND, win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_WAIT, 1, 65536, 65536, 300, None)
					win32pipe.ConnectNamedPipe(pipeTx, None)
				else:
					raise
			return data
		elif(self.bIsPosix):
			import os
#			print("Reading data")
			data = os.read(self.fileobject, 1)
			if(len(data)==0): raise FIFOClosedException()
#			print("Read",data)
			return data


	def write(self, data):
		if(self.bIsWindows):
			win32file.WriteFile(self.fileobject, data) 
		elif(self.bIsPosix):
			if(not self.bIsOpen):
				self.open()
			if(self.bIsOpen):
				import os
				try:
					os.write(self.fileobject, data)
#					os.fsync(self.fileobject)
				except BrokenPipeError as e:
					self.deletepipe()
					raise FIFOClosedException()
	
	def deletepipe(self):
		self.bIsOpen    = False
		self.delfifo()
		self.fileobject = None

# Using the main trick
import sys
main(sys.argv)

