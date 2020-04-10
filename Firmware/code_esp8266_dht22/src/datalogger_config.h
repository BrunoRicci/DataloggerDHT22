#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

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

#define PWR_SENSORS_PIN  13

//Sensor model (DHT sensor library configuration).
#define DHTTYPE DHT22           

//Power managements parameters.
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

#define RTC_MEMORY_MEASUREMENTS_POINTER_BLOCK   RTC_MEMORY_START_BLOCK
#define RTC_MEMORY_MEASUREMENTS_START_BLOCK     (RTC_MEMORY_START_BLOCK + RTC_MEMORY_RESERVED_BLOCKS)
#define RTC_MEMORY_MEASUREMENTS_END_BLOCK       (RTC_MEMORY_MEASUREMENTS_START_BLOCK + (MEASUREMENTS_DAILY_ARCHIVE_SIZE * 3))
// #define RTCMEMORYLEN 127
// #define RTCMEMORYSTART RTC_MEMORY_START_BLOCK

typedef struct{
        uint32_t timestamp;     //seconds from 1970
        uint16_t id_sensor;     //sensor id
        int16_t  temperature;   //temperature
        uint8_t  humidity;      //humidity
}measurement;      //structure alias.

typedef struct{
        uint16_t pointer_last_sent_measurement;
        uint16_t pointer_obtained_measurement;        //Block with last saved daily data.
}rtcMemory;      //structure alias.

/*---------------------------------------------------------*/
