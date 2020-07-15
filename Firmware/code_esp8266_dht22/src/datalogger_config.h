#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#ifndef __DATALOGGER_CONFIG_HPP_INCLUDED__
#define __DATALOGGER_CONFIG_HPP_INCLUDED__

#define DATALOGGER_SENSOR_COUNT         4    //Number of sensors per logger.
#define DATALOGGER_SENSE_TIME           3600       //Time between measurements, in seconds.
#define MEASUREMENTS_DAILY_QUANTITY     (86400/DATALOGGER_SENSE_TIME)

#define DATALOGGER_CONNECTION_TIME      86400           //Time between server connection in seconds
#define MEASUREMENTS_DAILY_ARCHIVE_SIZE (DATALOGGER_CONNECTION_TIME / DATALOGGER_SENSE_TIME)   
#define DAILY_ARCHIVE_SAVE_TIME         DATALOGGER_CONNECTION_TIME           //Time for archive to be saved in the flash memory.
#define SENSOR_ON_TIME                  1000

#define TEMPERATURE     1
#define HUMIDITY        2

#define SWITCH_HOLD_TIME_CONFIG         10000            //Time to hold the switch until config mode is activated (in milliseconds)
#define CONFIG_MODE_IDLE_TIMEOUT        40000           //Time to automatically exit config mode if no device connected.      

//Sensor model (DHT sensor library configuration).
#define DHTTYPE DHT22     

/*  GPIOs:
        NAME                    GPIO number
---------------------------------------------------------*/
#define DHT_SENSOR_1_PIN        5
#define DHT_SENSOR_2_PIN        4
#define DHT_SENSOR_3_PIN        0
#define DHT_SENSOR_4_PIN        2       

#define CHARGER_DETECT_PIN      3       //GPIO3 to sense charger.   
#define PWR_CONTROL_PIN         12      //GPIO12 to connect or disconnect battery.
#define PWR_SENSORS_PIN_P       13      //GPIO13 to control sensors power supply (high side).
#define PWR_SENSORS_PIN_N       15      //GPIO15 to control sensors power supply (low side).
#define REED_SWITCH_PIN         14      //GPIO14 to sense switch state.
#define BATTERY_SENSE_PIN       A0      //A0 oin as analog

#define LED_R_PIN       4
#define LED_G_PIN       0
#define LED_B_PIN       5
#define COLOR_RED       (0xFF0000)
#define COLOR_GREEN     (0x00FF00)
#define COLOR_BLUE      (0x0000FF)
#define COLOR_YELLOW    (0xFFB400)
#define COLOR_VIOLET    (0xFF00FF)
#define COLOR_CYAN      (0x00FF00)
#define COLOR_WHITE     (0xFFFFFF)
#define COLOR_BLACK     (0x0)

      
//Power managements parameters.
#define ADC_VOLTAGE_MV          1000
#define VBAT_VADC_RATIO         (4.2)  //4.2V/1v
#define Q_VCE_COMPENSATION      0      //Voltage to add to the result, lost in transistor VCE.
#define BATTERY_MAX_VOLTAGE     4200
#define BATTERY_MIN_VOLTAGE     3300
#define BATTERY_MIN_PERCENTAGE  5

#define ON  1
#define OFF 0

/*  - - - - - - - - - - - - - - - RTC memory: - - - - - - - - - - - - - - - -
.      START                                     END
.        | reserved            available          |
.        |----------|-----------------------------|
.        | - - - - - -  MEMORY_SIZE - - - - - - - |                       
.
.       PARAMETER                       VALUE
-----------------------------------------------------------*/
#define RTC_MEMORY_SIZE                 768
#define RTC_MEMORY_START_ADDRESS        256             
#define RTC_MEMORY_BLOCK_SIZE           4       //Block size in bytes.   

#define RTC_MEMORY_RESERVED_BLOCKS      10      //Blocks not used for measurements.
#define RTC_MEMORY_END_ADDRESS          (RTC_MEMORY_SIZE)
#define RTC_MEMORY_START_BLOCK          ((RTC_MEMORY_START_ADDRESS/RTC_MEMORY_BLOCK_SIZE))                    
#define RTC_MEMORY_END_BLOCK            (RTC_MEMORY_END_ADDRESS/RTC_MEMORY_BLOCK_SIZE)

#define RTC_MEMORY_MEASUREMENTS_START_BLOCK     (RTC_MEMORY_START_BLOCK + RTC_MEMORY_RESERVED_BLOCKS)
#define RTC_MEMORY_MEASUREMENT_BLOCK_SIZE       (sizeof(Measurement)/RTC_MEMORY_BLOCK_SIZE)
#define RTC_MEMORY_MEASUREMENTS_COUNT           12      //12 measurements maximum.
#define RTC_MEMORY_MEASUREMENTS_END_BLOCK       (RTC_MEMORY_MEASUREMENTS_START_BLOCK + (RTC_MEMORY_MEASUREMENTS_COUNT * RTC_MEMORY_MEASUREMENT_BLOCK_SIZE))

#define RTC_MEMORY_MEASUREMENTS_POINTER_BLOCK   RTC_MEMORY_START_BLOCK  //For compatibility...
#define RTC_MEMORY_VARIABLES_START_BLOCK        (RTC_MEMORY_START_BLOCK + 1)
#define RTC_MEMORY_VARIABLES_BLOCK_SIZE         7       //7 blocks for all the variables -> may vary.
#define RTC_MEMORY_POWER_CHECK_VARIABLE         0xB1            

typedef struct{
        uint32_t timestamp;     //seconds from 1970 at UTC
        uint16_t id_sensor[DATALOGGER_SENSOR_COUNT];     //sensor id
        int16_t  temperature[DATALOGGER_SENSOR_COUNT];   //temperature
        uint8_t  humidity[DATALOGGER_SENSOR_COUNT];      //humidity
}Measurement;      //structure alias.


/*---------------------------------------------------------*/

#define MEASUREMENTS_FILE_NAME          "measurements.txt"
#define CONFIG_FILE_NAME                "config.txt"
#define NVM_POINTERS_FILE_NAME          "variables.txt"
#define LOGS_FILE_NAME                  "logs.txt"

//todo: MAKE DEFINE FOR MAXIMUM NUMBER OF PACKETS PER ARCHIVE READ AND WRITE OPERATION.
/*---------------------------------------------------------*/

#define STATE_SEALED            0         //State actually doesn't exist as it operates when device is not powered.
#define STATE_CONFIGURATION     1
#define STATE_WAKE              2
#define STATE_GET_MEASUREMENTS  3
#define STATE_SAVE_MEASUREMENTS 4
#define STATE_TRANSMISSION      5
#define STATE_DEEP_SLEEP        6       //State to go right before deepSleep.
#define STATE_FORCE_MEASUREMENT 7       //State to take measurement, force save to flash, and transmit to server.

/*---------------------------------------------------------*/
#define SEND_MEASUREMENTS_URL   "/sendmeasurements"
#define GET_TIME_URL            "/gettime"
#define MAX_PACKET_PER_REQUEST       12      //Maximum number of packets to send per request.



#endif  //__DATALOGGER_CONFIG_HPP_INCLUDED__