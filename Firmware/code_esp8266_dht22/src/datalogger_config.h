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

#define TEMPERATURE     1
#define HUMIDITY        2


/*  GPIOs:
        NAME                    GPIO number
---------------------------------------------------------*/
#define DHT_SENSOR_1_PIN        2
#define DHT_SENSOR_2_PIN        0
#define DHT_SENSOR_3_PIN        4
#define DHT_SENSOR_4_PIN        5

#define PWR_SENSORS_PIN         13
#define BATTERY_SENSE_PIN       A0      //A0 oin as analog


//Sensor model (DHT sensor library configuration).
#define DHTTYPE DHT22           

//Power managements parameters.
#define ADC_VOLTAGE_MV          3300
#define VBAT_VADC_RATIO         (1.27)
#define BATTERY_MAX_VOLTAGE     4200
#define BATTERY_MIN_VOLTAGE     3300

#define ON  1
#define OFF 0

/* RTC RAM memory:
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
#define RTC_MEMORY_MEASUREMENT_BLOCK_SIZE       (sizeof(measurement)/RTC_MEMORY_BLOCK_SIZE)
#define RTC_MEMORY_MEASUREMENTS_END_BLOCK       (RTC_MEMORY_MEASUREMENTS_START_BLOCK + (12 * RTC_MEMORY_MEASUREMENT_BLOCK_SIZE))

#define RTC_MEMORY_MEASUREMENTS_POINTER_BLOCK   RTC_MEMORY_START_BLOCK
#define RTC_MEMORY_F_LAST_SAVED_POINTER_BLOCK   (RTC_MEMORY_START_BLOCK + 1)
#define RTC_MEMORY_F_LAST_SENT_POINTER_BLOCK    (RTC_MEMORY_START_BLOCK + 2)


typedef struct{
        uint32_t timestamp;     //seconds from 1970 at UTC
        uint16_t id_sensor[DATALOGGER_SENSOR_COUNT];     //sensor id
        int16_t  temperature[DATALOGGER_SENSOR_COUNT];   //temperature
        uint8_t  humidity[DATALOGGER_SENSOR_COUNT];      //humidity
}measurement;      //structure alias.


/*---------------------------------------------------------*/

#define MEASUREMENTS_FILE_NAME  "measurements.txt"
#define CONFIG_FILE_NAME        "config.txt"
//todo: MAKE DEFINE FOR MAXIMUM NUMBER OF PACKETS PER ARCHIVE READ AND WRITE OPERATION.



#endif  //__DATALOGGER_CONFIG_HPP_INCLUDED__