cd C:\Users\BRUNO - PC\AppData\Local\Programs\Python\Python37\Scripts

rem esptool.py erase_flash
esptool.py -b1000000 write_flash 0 "C:\Users\BRUNO - PC\REPOS_GIT\DataloggerDHT22\Firmware\code_esp8266_dht22\test\firmware_upload\firmware.bin"

pause