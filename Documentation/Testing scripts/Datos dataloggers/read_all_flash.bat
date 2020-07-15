cd C:\Users\BRUNO - PC\AppData\Local\Programs\Python\Python37\Scripts
esptool.py -b1000000 read_flash 0 0x400000 flash_d1_08-07-2020.bin

@echo off
echo  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
echo  *
echo  *
echo  *
echo                PUT DEVICE 2
pause 

cd C:\Users\BRUNO - PC\AppData\Local\Programs\Python\Python37\Scripts
esptool.py -b1000000 read_flash 0 0x400000 flash_d2_08-07-2020.bin

@echo off
echo  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
echo  *
echo  *
echo  *
echo                PUT DEVICE 3
pause 

cd C:\Users\BRUNO - PC\AppData\Local\Programs\Python\Python37\Scripts
esptool.py -b1000000 read_flash 0 0x400000 flash_d3_08-07-2020.bin

@echo off
echo  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
echo  *
echo  *
echo  *
echo                PUT DEVICE 4
pause 

cd C:\Users\BRUNO - PC\AppData\Local\Programs\Python\Python37\Scripts
esptool.py -b1000000 read_flash 0 0x400000 flash_d4_08-07-2020.bin

@echo off
echo  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
echo  *
echo  *
echo  *
echo                PUT DEVICE 5
pause 

cd C:\Users\BRUNO - PC\AppData\Local\Programs\Python\Python37\Scripts
esptool.py -b1000000 read_flash 0 0x400000 flash_d5_08-07-2020.bin

@echo off
echo  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
echo  *
echo  *
echo  *
echo               FINISHED!!
pause 
