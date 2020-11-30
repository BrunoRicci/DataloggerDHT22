@echo off
cd "C:\Users\BRUNO - PC\AppData\Local\Programs\Python\Python37\Scripts"

set /P INPUT=Enter device transceiver_id: %=%
echo device_id=%INPUT%

esptool.py erase_flash
echo ----- FLASH ERASED -----
esptool.py -b1000000 write_flash 0 "C:\Users\BRUNO - PC\REPOS_GIT\DataloggerDHT22\Firmware\code_esp8266_dht22\test\devices_flash_memories\flash_d%INPUT%_27-11-2020.bin"
echo ----- FLASH WRITTEN-----
esptool.py -b1000000 write_flash 0 "C:\Users\BRUNO - PC\REPOS_GIT\DataloggerDHT22\Firmware\code_esp8266_dht22\.pio\build\nodemcuv2\firmware.bin"
echo ----- FIRMWARE UPDATED -----

cd "C:\Users\BRUNO - PC\REPOS_GIT\DataloggerDHT22\Firmware\code_esp8266_dht22\test\time_sync_serial"
datalogger_configmode.py

PAUSE Connect to "Datalogger" AP and press ENTER...


cd "C:\Users\BRUNO - PC\REPOS_GIT\DataloggerDHT22\Firmware\code_esp8266_dht22\test\HTTP_config"
python device_config.py %INPUT% r_asp

echo ----- DEVICE %INPUT% PARAMETERS SET -----
PAUSE Press ENTER to send the data to the server.

cd "C:\Users\BRUNO - PC\REPOS_GIT\DataloggerDHT22\Firmware\code_esp8266_dht22\test\time_sync_serial"
datalogger_timesync.py