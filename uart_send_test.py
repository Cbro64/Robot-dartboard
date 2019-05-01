import serial
import time

ser = serial.Serial('/dev/ttyS0',57600)
print('started')
while True:
	x = int(input(">> "))
	ser.write(x.to_bytes(1,'little'))
	ser.flush()
	#print('sent')
	#print("waiting")
	#time.sleep(1)
	#message = ser.read()
	#print(list(message))