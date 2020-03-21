#include <Arduino.h>
#include <time.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#include <Adafruit_Sensor.h>

#include <DHT.h>
#define DHT_SENSOR_1_PIN 2
#define DHT_SENSOR_2_PIN 0
#define DHT_SENSOR_3_PIN 4
#define DHT_SENSOR_4_PIN 5

#define DHTTYPE DHT22
DHT dht22_sensor_1(DHT_SENSOR_1_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_2(DHT_SENSOR_2_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_3(DHT_SENSOR_3_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_4(DHT_SENSOR_4_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object


#define PWR_SENSORS_PIN  13
#define ON  1
#define OFF 0

//---------------------------------------------------------------------------------------------------------------------
//Initialization functions.
void portInit(void);
void SetSensorPower(unsigned char state);

//---------------------------------------------------------------------------------------------------------------------
// Last temperature & humidity, updated in loop()
float temperature = 0.0;
float humidity = 0.0;

ESP8266WiFiMulti WiFiMulti;

void setup() {
  dht22_sensor_1.begin();           //Initializes objects.
  dht22_sensor_2.begin();
  dht22_sensor_3.begin();
  dht22_sensor_4.begin();
  
  portInit();

  Serial.begin(115200);
  // Serial.setDebugOutput(true);

  // WiFi.mode(WIFI_STA);
  // WiFiMulti.addAP("Fibertel WiFi866 2.4GHz", "01416592736");

}

void loop() {

  //waits for WiFi connection
  /* if ((WiFiMulti.run() == WL_CONNECTED)) {
    Serial.printf("WiFi Connected!");
    
    WiFiClient client;
    HTTPClient http;

    Serial.print("[HTTP] begin...\n");
    if (http.begin(client, "http://192.168.0.172:8080/test?value=12345")) {  // HTTP
      
      //http.POST.sendRequest;
      //http.GET.sendRequest;

      Serial.print("[HTTP] GET...\n");
      // start connection and send HTTP header
      int httpCode = http.GET();

      // httpCode will be negative on error
      if (httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);

        // file found at server
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload = http.getString();
          Serial.println(payload);
        }
      } else {
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }

      http.end();
    } 
    else {
      Serial.printf("[HTTP] Unable to connect\n");
    }
  } */

  Serial.printf("Turning on sensor...");
  SetSensorPower(ON);
  delay(800);

  Serial.printf("\nTemperature_1: %f\n", dht22_sensor_1.readTemperature() );
  Serial.printf("Humidity_1: %f\n", dht22_sensor_1.readHumidity() );

  Serial.printf("\nTemperature_2: %f\n", dht22_sensor_2.readTemperature() );
  Serial.printf("Humidity_2: %f\n", dht22_sensor_2.readHumidity() );

  Serial.printf("\nTemperature_3: %f\n", dht22_sensor_3.readTemperature() );
  Serial.printf("Humidity_3: %f\n", dht22_sensor_3.readHumidity() );

  Serial.printf("\nTemperature_4: %f\n", dht22_sensor_4.readTemperature() );
  Serial.printf("Humidity_4: %f\n", dht22_sensor_4.readHumidity() );
  SetSensorPower(OFF);

  Serial.printf("Going deep sleep...  rtc_time=%d", system_get_rtc_time() );
  ESP.deepSleep(5e6);  //deep sleeps for 5 seconds...
  Serial.printf("Woke up.  rtc_time=%d", system_get_rtc_time() );
  
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