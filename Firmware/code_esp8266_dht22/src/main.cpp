#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>

#define DHTPIN_1 5
#define DHTPIN_2 4
#define DHTTYPE DHT22
DHT dht22_sensor_1(DHTPIN_1, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_2(DHTPIN_2, DHTTYPE);    //Creates DHT22 sensor "sensor" object


// Last temperature & humidity, updated in loop()
float temperature = 0.0;
float humidity = 0.0;

ESP8266WiFiMulti WiFiMulti;

void setup() {
  dht22_sensor_1.begin();           //Initializes object.
  dht22_sensor_2.begin();           //Initializes object.


  Serial.begin(115200);
  // Serial.setDebugOutput(true);

  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("Fibertel WiFi866 2.4GHz", "01416592736");

}

void loop() {

  //waits for WiFi connection
  if ((WiFiMulti.run() == WL_CONNECTED)) {
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
  }

  
  temperature = dht22_sensor_1.readTemperature();
  humidity = dht22_sensor_1.readHumidity();
  Serial.printf("\nTemperature_1: %f\n", temperature );
  Serial.printf("Humidity_1: %f\n", humidity );

  temperature = dht22_sensor_2.readTemperature();
  humidity = dht22_sensor_2.readHumidity();
  Serial.printf("\nTemperature_2: %f \n", temperature );
  Serial.printf("Humidity_2: %f\n", humidity );

  Serial.printf("Going deep sleep...  rtc_time=%d", system_get_rtc_time() );
  ESP.deepSleep(5e6);  //deep sleeps for 5 seconds...
  Serial.printf("Woke up.  rtc_time=%d", system_get_rtc_time() );


  //delay(10000);
}