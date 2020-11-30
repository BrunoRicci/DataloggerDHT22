import sys

print('Test script')

 
# Print total number of arguments
print ('Total number of arguments:', format(len(sys.argv)))
 
# Print all arguments
print ('Argument List:', str(sys.argv))

device_id='6'   #FORCE DEVICE ID
flag_reset_sent_pointer = False


if(len(sys.argv) > 1):  # if device id specified...
    device_id = int(sys.argv[1])

    if(len(sys.argv)>2):    #If 2nd parameter specified...
        if(sys.argv[2] == 'r_asp'):  #If "asp" argument received...
            flag_reset_sent_pointer = True
            print('Reset archive_sent_pointer.')

print('Device ID: '+str(device_id))
# print(sys.argv)