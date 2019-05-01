import serial
import time

ser = serial.Serial('/dev/ttyS0',57600)
print('started')
while True:
	#x = int(input(">> "))
	#ser.write(x.to_bytes(1,'little'))
	ser.flush()
	#print('sent')
<<<<<<< HEAD
	#print("waiting")
	#time.sleep(1)
	#message = ser.read()
	#print(list(message))
=======
	#time.sleep(1)
	#print("waiting")
	#ser.read(3)
	message = ser.read(1)
	print(list(message))
>>>>>>> a6ddd2b44dc45c2827bfa763b4c4e2a6748a0567
