import serial
import time
import struct


ser = serial.Serial(
    port='COM3',
    baudrate=115200,
    timeout=1
)

print(ser.name)


if ser.isOpen():
    print('Port was open. Closing...')
    ser.close()
ser.open()


currenttime = str(int(time.time()))    #Get current UNIX UTC time in seconds.
print('Current time:'+currenttime)

# values=[]
# for i in currenttime:
#     print(i)
#     values.append(i)
    
# string = b''
# input()

# for i in values:
#     string += struct.pack('!B',i)
# input()

# ser.write(bytes('123456'))

data = (1,2,3)      #Array of ints.

print(string)
print(bytes( (1,2,3) ))


# string+='h'
# string+='o'
# string+='l'
# string+='a'
# print(string)

input()
ser.write(bytes(data))

print('read data: '+str(ser.readline().decode('ascii') ))
ser.close()

input()


