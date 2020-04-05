#include <Arduino.h>
#include <string.h>
#include <time.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>

#include <datalogger_config.h>

#include <FS.h>

DHT dht22_sensor_1(DHT_SENSOR_1_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_2(DHT_SENSOR_2_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_3(DHT_SENSOR_3_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_4(DHT_SENSOR_4_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object

//---------------------------------------------------------------------------------------------------------------------
//Initialization functions.
void portInit(void);
void SetSensorPower(unsigned char state);
void goDeepSleep(uint64_t time);

void* stringToArray(std::string origin_string);
String generateMeasurementValue(unsigned char type, float value);  //Converts measurements into valid format
//---------------------------------------------------------------------------------------------------------------------
// Last temperature & humidity, updated in loop()
//  [0] --> Sensor 1
//  [1] --> Sensor 2
signed short int temperature[DATALOGGER_SENSOR_COUNT];
unsigned char humidity[DATALOGGER_SENSOR_COUNT];

float temperature_float [DATALOGGER_SENSOR_COUNT];
float humidity_float [DATALOGGER_SENSOR_COUNT];


ESP8266WiFiMulti WiFiMulti;

void setup() {
  portInit();

  dht22_sensor_1.begin();           //Initializes objects.
  dht22_sensor_2.begin();
  dht22_sensor_3.begin();
  dht22_sensor_4.begin();
  


  Serial.begin(115200);

  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay(1);

  WiFi.forceSleepWake();  //Forces 
  delay(1);

  Serial.printf("Turning on sensor...");
  SetSensorPower(ON);
  delay(1000);

  temperature_float[0] = dht22_sensor_1.readTemperature();
  temperature_float[1] = dht22_sensor_2.readTemperature();
  temperature_float[2] = dht22_sensor_3.readTemperature();
  temperature_float[3] = dht22_sensor_4.readTemperature();
  humidity_float[0] = dht22_sensor_1.readHumidity();
  humidity_float[1] = dht22_sensor_2.readHumidity();
  humidity_float[2] = dht22_sensor_3.readHumidity();
  humidity_float[3] = dht22_sensor_4.readHumidity();

  generateMeasurementValue(HUMIDITY, humidity_float[0]);

  for (unsigned char i = 0; i<4 ; i++){
    Serial.printf("\nTemperature_%d: %f\n", i+1, temperature_float[i] );
    Serial.printf("Humidity_%d: %f\n", i+1,humidity_float[i] );
  }

  SetSensorPower(OFF);

////////////////////////////////
  WiFi.mode(WIFI_STA);  // Bring up the WiFi connection.
  WiFi.persistent(false);
  WiFiMulti.addAP("Fibertel WiFi866 2.4GHz", "01416592736");

}

void loop() {

  //Waits for WiFi connection
  if ((WiFiMulti.run() == WL_CONNECTED)) {
    Serial.printf("WiFi Connected!");
    
    WiFiClient client;
    HTTPClient http;
  
    Serial.print("[HTTP] begin...\n");
    if (http.begin(client, "http://192.168.0.172:8080/post")) {  // HTTP
    //                              test?value=12345
      String request;

      //request= "t1=1&t2=2&t3=3&t4=4&h1=1&h2=2&h3=3&h4=4";
      //generateMeasurementValue(TEMPERATURE, temperature[0]);
      request= "t1=";
      request+= generateMeasurementValue(TEMPERATURE, temperature_float[0]);
      request+= "&t2=";
      request+= generateMeasurementValue(TEMPERATURE, temperature_float[1]);
      request+= "&t3=";
      request+= generateMeasurementValue(TEMPERATURE, temperature_float[2]);
      request+= "&t4=";
      request+= generateMeasurementValue(TEMPERATURE, temperature_float[3]);
      request+= "&h1=";
      request+= generateMeasurementValue(HUMIDITY, humidity_float[0]);
      request+= "&h2=";
      request+= generateMeasurementValue(HUMIDITY, humidity_float[1]);
      request+= "&h3=";
      request+= generateMeasurementValue(HUMIDITY, humidity_float[2]);
      request+= "&h4=";
      request+= generateMeasurementValue(HUMIDITY, humidity_float[3]);

      //Send POST request
      Serial.print("[HTTP] POST...:\n");
      Serial.print(request);
      int httpCode = http.POST( request );
      //int httpCode = http.GET();



      // httpCode will be negative on error
      if (httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        Serial.printf("[HTTP] POST... code: %d\n", httpCode);
        
        // file found at server
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload = http.getString();
          Serial.println(payload);
        }

        goDeepSleep(60e6);  //Deep sleep for low power consumption.

      } else {
        Serial.printf("[HTTP] GET failed, error: %s\n", http.errorToString(httpCode).c_str());
        goDeepSleep(60e6);
      }
      http.end();
    } 
    else {
      Serial.printf("[HTTP] Unable to connect\n");
      goDeepSleep(60e6);  //Deep sleep for low power consumption.
    }
  }




  
}



String generateMeasurementValue(unsigned char type, float value){  //Converts measurements into valid format
  char measurement [10];  //4B string array for the value.
  measurement[(sizeof(measurement)-1)] = 0; //Put NULL character at the end of the string.

  if (type == TEMPERATURE){
     // -200 ~ 800 -> -20ºC to 80ºC
     itoa((signed short int)(value * 10), measurement, 10);   //Takes the first digit after comma.
  }
  else if (type == HUMIDITY)
  { // 0 ~ 100 ->  0% to 100%
    itoa((unsigned char)(value), measurement, 10);    //Takes only integer part.
  }

  measurement[(sizeof(measurement)-1)] = 0; //Put NULL character at the end of the string
  Serial.printf("\n\nMeasurement:");
  Serial.printf(measurement);
  return( String(measurement) );
}

void* stringToArray(std::string origin_string){
    //This function should return -1 if the size of "a" (destination array) is smaller than "s"
    //to avoid buffer overflow and possible fragmentation faults...
    int string_length = (origin_string.length()) + 1 ;

    char* s = (char*) malloc(string_length);
    strcpy(s, origin_string.c_str());

    return (s);
}

void goDeepSleep(uint64_t time){
  WiFi.disconnect(true);  //Turns off wifi module, so in the wake up it won't turn on automatically until required.
  delay(1);
  Serial.printf("Going deep sleep...  rtc_time=%d", system_get_rtc_time());
  ESP.deepSleep(time, WAKE_RF_DISABLED);  //deep sleeps for 5 seconds...
}

void SetSensorPower(unsigned char state){
  if (state == ON)
  {
    digitalWrite(PWR_SENSORS_PIN, LOW);  //Turn sensors supply on.
  }
  if (state == OFF)
  {
    digitalWrite(PWR_SENSORS_PIN, HIGH);  //Turn sensors supply off.
  }
}

void portInit(void){
  pinMode(PWR_SENSORS_PIN, OUTPUT);   //GPIO12 as output
  SetSensorPower(OFF);          //Sensors start turned off.
}