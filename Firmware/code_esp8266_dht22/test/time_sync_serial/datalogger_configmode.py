import serial
import time

MSG_SYNC_TIME =             "Time updated."
COMMAND_FORCE_MEASUREMENT = "FORCE_MEASUREMENT"
COMMAND_CONFIG_MODE =       "CONFIG_MODE"

ser = serial.Serial(
    port='COM5',
    baudrate=115200,
    timeout=1
)

print(ser.name)


if ser.isOpen():        
    print('Port was open. Closing...')
    ser.close()

for i in range(0,2):
    print("Resetting...")
    ser.open()
    print("dtr: "+str(ser.dtr))
    print("rts: "+str(ser.rts))
    ser.rts=1
    ser.dtr=0
    time.sleep(0.5)
    ser.close()
    time.sleep(0.1)

ser.rts=1
ser.dtr=1
ser.open()
print("Serial opened.")


currenttime = str(int(time.time()))    #Get current UNIX UTC time in seconds.
command = "CONFIG_MODE"

data = [ord(c) for c in command]
data = tuple(data)

time.sleep(.5)
# print('data:' + str(data))
ser.write(bytes(data))      #Sends current timestamp.

time.sleep(2)

read_data = str(ser.read_all())
# read_data.rfind()


print('read data: '+read_data)
# if(read_data == currenttime):
#     print('Time updated correctly.')
# else:
#     print('ERROR. Time not updated.')




ser.close()

input()


