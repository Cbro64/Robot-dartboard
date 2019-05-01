import serial
import time

ser = serial.Serial('/dev/ttyS0',57600)
print('started')
while True:
	#x = int(input(">> "))
	#ser.write(x.to_bytes(1,'little'))
	ser.flush()
	#print('sent')
	#time.sleep(1)
	#print("waiting")
	#ser.read(3)
	message = ser.read(1)
	print(list(message))