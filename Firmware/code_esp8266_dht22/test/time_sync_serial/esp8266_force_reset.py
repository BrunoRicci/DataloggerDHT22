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