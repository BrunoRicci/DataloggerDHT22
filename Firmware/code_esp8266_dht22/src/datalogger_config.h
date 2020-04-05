#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define DATALOGGER_SENSOR_COUNT         4    //Number of sensors per logger.
#define DATALOGGER_SENSE_TIME           3600       //Time between measurements, in seconds.

#define DATALOGGER_CONNECTION_TIME      86400           //Time between server connection in seconds
#define MEASUREMENTS_DAILY_ARCHIVE_SIZE (DATALOGGER_CONNECTION_TIME / DATALOGGER_SENSE_TIME)   
#define DAILY_ARCHIVE_SAVE_TIME         DATALOGGER_CONNECTION_TIME           //Time for archive to be saved in the flash memory.

#define TEMPERATURE     1
#define HUMIDITY        2


/*  GPIOs:
        NAME                    GPIO number
-----------------------------------------------------------*/
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



