#include <Arduino.h>
//#include <string.h>
#include <time.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#include <Hash.h>
//#include <ESPAsyncWebServer.h>
#include <ESP8266WebServer.h>
//#include <ESPAsyncTCP.h>
#include <WebServerFiles.cpp>

#include <FS.h>           //SPIFFS libraries.
#include <user_interface.h>   //Functions to handle RTC memory.

#include <datalogger_config.h>  //Configuration parameters.

//----------------------------------------------------------------------------------------
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
unsigned char saveDataRTC(int address, void *data, unsigned short int bytes); //Saves data to RTC memory.
void readDataRTC(int address, void *data, unsigned short int bytes);
void rtcMemoryInit(void);

void wifiTurnOn(void);
void wifiTurnOff(void);
unsigned char runWebServer(void);
//---------------------------------------------------------------------------------------------------------------------
// Last temperature & humidity, updated in loop()
//  [0] --> Sensor 1
//  [1] --> Sensor 2

float temperature_float [DATALOGGER_SENSOR_COUNT];
float humidity_float [DATALOGGER_SENSOR_COUNT];

ESP8266WiFiMulti WiFiMulti;

typedef struct {
  int battery;
  int other;
} rtcStore;

rtcStore rtcMem;
int i;
int buckets;
bool toggleFlag;

ESP8266WebServer server(8080);



void setup() {
  portInit();

  wifiTurnOff();

  Serial.begin(115200);

  dht22_sensor_1.begin();           //Initializes objects.
  dht22_sensor_2.begin();
  dht22_sensor_3.begin();
  dht22_sensor_4.begin();

  runWebServer();

//////////////////////////////////////////////////////////////
/*   Serial.println();
  Serial.println("Start");
  buckets = (sizeof(rtcMem) / 4);   //Number of buckets needed to store X bytes in the memory.
  if (buckets == 0) buckets = 1;    //Always greater than 1
  Serial.printf("Buckets %d\n",buckets);
  system_rtc_mem_read(64, &toggleFlag, 4);  //Reads first block (bucket) which contains a flag
  Serial.printf("toggle Flag: %d\n",toggleFlag);
  if (toggleFlag) {
    Serial.println("Start Writing");
    for (i = 0; i < RTCMEMORYLEN / buckets; i++) {
      rtcMem.battery = i;
      rtcMem.other = i * 11;
      int rtcPos = RTCMEMORYSTART + RTC_MEMORY_RESERVED_BLOCKS + i * buckets;
      //                  address    data   quantity of bytes
      system_rtc_mem_write(rtcPos, &rtcMem, buckets * 4);  //Writes data
      toggleFlag = false;   
      system_rtc_mem_write(64, &toggleFlag, 4); //Modifies the flag.

      Serial.printf("i: %d\n", i);
      Serial.printf(" Position: %d\n", rtcPos);
      Serial.printf(", battery: %d\n", rtcMem.battery);
      Serial.printf(", other: %d\n", rtcMem.other);
      yield();
    }
    Serial.println("Writing done");
  }
  else {
    Serial.println("Start reading");
    for (i = 0; i < RTCMEMORYLEN / buckets; i++) {
      int rtcPos = RTCMEMORYSTART+ RTC_MEMORY_RESERVED_BLOCKS + i * buckets;
      system_rtc_mem_read(rtcPos, &rtcMem, sizeof(rtcMem));
      toggleFlag = true;
      system_rtc_mem_write(64, &toggleFlag, 4);

      Serial.printf("i: %d\n", i);
      Serial.printf(" Position: %d\n", rtcPos);
      Serial.printf(", battery: %d\n", rtcMem.battery);
      Serial.printf(", other: %d\n", rtcMem.other);
      yield();
    }
    Serial.println("reading done");
    for (i = 0; i < RTCMEMORYLEN / buckets; i++) {
      rtcMem.battery = 0;
      rtcMem.other = 0;
      int rtcPos = RTCMEMORYSTART + i * buckets;
      system_rtc_mem_write(rtcPos, &rtcMem, buckets * 4);
    }
  }
  Serial.println("before sleep");
  goDeepSleep(60e6);


 */
//////////////////////////////////////////////////////////////



/* 
  SetSensorPower(ON);
  delay(1000);
  //GET MEASUREMENTS
  for (unsigned char i = 0; i<4 ; i++){
    Serial.printf("\nTemperature_%d: %f\n", i+1, temperature_float[i] );
    Serial.printf("Humidity_%d: %f\n", i+1,humidity_float[i] );
  } 
  SetSensorPower(OFF);
*/
  

}

void loop() {

  server.handleClient(); //Handling of incoming requests
  //Waits for WiFi connection
/*   if ((WiFiMulti.run() == WL_CONNECTED)) {
    Serial.printf("WiFi Connected!");
    
    WiFiClient client;
    HTTPClient http;
  
    Serial.print("[HTTP] begin...\n");
    if (http.begin(client, "http://192.168.0.172:8080/sendmeasurements")) {  // HTTP
      String request;

      
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
*/

  
  //rtcMemoryInit();


  // uint8 buf[64];
  // readDataRTC(RTC_MEMORY_START_BLOCK, &buf, 4);

  // measurement m;
  // m.id_sensor= 1;
  // m.humidity=75;
  // m.temperature=240;
  // m.timestamp=1234567890;

  // saveDataRTC(0, &m, sizeof(m));

  //goDeepSleep(60e6);

  
}

void rtcMemoryInit(void){
  //Initializes 
  uint8_t initpos = RTC_MEMORY_POINTER_ADDRESS;
  int mem_pointer = RTC_MEMORY_POINTER_ADDRESS;
  system_rtc_mem_write(mem_pointer, &initpos, sizeof(initpos)); 
}

unsigned char saveDataRTC(int address, void *data, unsigned short int bytes){
  for (unsigned short i = RTC_MEMORY_START_BLOCK+1; i < RTC_MEMORY_END_BLOCK; i++)
  { 

    //uint8_t pointer_last_saved_measurement;
    //system_rtc_mem_read(RTC_MEMORY_START_BLOCK, pointer_last_saved_measurement, sizeof(uint8_t));
    system_rtc_mem_write(i,data,bytes);
    yield();

    uint8 d = ((uint8*)(data))[0];
    Serial.printf("\nData written to RTC memory:  ");
    Serial.printf("address: %d / ", i);
    Serial.printf("bytes: %d \n", bytes);
    Serial.printf("data:");

    for (unsigned short x=0; x < bytes; x++){
    Serial.printf("%x",  d);
    }

  }
  return 1;
}

void readDataRTC(int address, void *data, unsigned short int bytes) {
  system_rtc_mem_read(address, data, bytes);
  yield();

  Serial.printf("\n\n     Read data address %d:\n",address);
  for(unsigned short int i=0; i < bytes; i++){
      Serial.printf("%d",((uint8*)(data))[i]);
  }

  
}

unsigned char saveMeasurementsToRAM(void){
  
  return(1);

}

String generatePOSTRequest( uint16_t id_transceiver, uint8_t battery_level,   //Header elements
                            uint32_t *timestamp,  //Array elements
                            uint16_t *id_sensor,
                            int16_t *temperature,
                            uint32_t *humidity,
                            uint8_t length        //Number of elements to be sent.
                            ){
  /*This function receives data saved to flash (array) and generates the POST request to send
    it to the server, which is returned.
    Necessary data:
      -id_transceiver
      -battery_level
        -timestamp[]
        -id_sensor[]
        -temperature[]
        -humidity[]
  */


  String request = "";  //Initialize empty string.

  String aux_string;


  //request= "t1=1&t2=2&t3=3&t4=4&h1=1&h2=2&h3=3&h4=4";
      //generateMeasurementValue(TEMPERATURE, temperature[0]);

       
      request += "id_transceiver=1";
      request += "battery_level=100";
      request += "timestamp=[,,,,,,,]";
      request += "id_sensor=[1,2,3,4,1,2,3,4]";
      request += "temperature=[19.7,19.5,20.2,19.5,20.2,21.0,20.5,20.7]";
      request += "humidity=[35,39,35,40,43,46,48,45]"; 
      

     /*  request+= "t1=";
      request+= generateMeasurementValue(TEMPERATURE, dht22_sensor_1.readTemperature());
      request+= "&t2=";
      request+= generateMeasurementValue(TEMPERATURE, dht22_sensor_2.readTemperature());
      request+= "&t3=";
      request+= generateMeasurementValue(TEMPERATURE, dht22_sensor_3.readTemperature());
      request+= "&t4=";
      request+= generateMeasurementValue(TEMPERATURE, dht22_sensor_4.readTemperature());
      request+= "&h1=";
      request+= generateMeasurementValue(HUMIDITY, dht22_sensor_1.readHumidity());
      request+= "&h2=";
      request+= generateMeasurementValue(HUMIDITY, dht22_sensor_2.readHumidity());
      request+= "&h3=";
      request+= generateMeasurementValue(HUMIDITY, dht22_sensor_3.readHumidity());
      request+= "&h4=";
      request+= generateMeasurementValue(HUMIDITY, dht22_sensor_4.readHumidity());
    */
  // temperature_float[0] = dht22_sensor_1.readTemperature();
  // temperature_float[1] = dht22_sensor_2.readTemperature();
  // temperature_float[2] = dht22_sensor_3.readTemperature();
  // temperature_float[3] = dht22_sensor_4.readTemperature();
  // humidity_float[0] = dht22_sensor_1.readHumidity();
  // humidity_float[1] = dht22_sensor_2.readHumidity();
  // humidity_float[2] = dht22_sensor_3.readHumidity();
  // humidity_float[3] = dht22_sensor_4.readHumidity();
  return request;
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
  Serial.printf("\nGoing deep sleep...  rtc_time=%d", system_get_rtc_time());
  ESP.deepSleep(time, WAKE_RF_DISABLED);  //deep sleeps for 5 seconds...
}

void SetSensorPower(unsigned char state){
  if (state == ON)
  {
    digitalWrite(PWR_SENSORS_PIN, LOW);  //Turn sensors supply on.
    Serial.printf("Sensors turned on.");
  }
  if (state == OFF)
  {
    digitalWrite(PWR_SENSORS_PIN, HIGH);  //Turn sensors supply off.
    Serial.printf("Sensors turned off.");
  }
}

void portInit(void){
  pinMode(PWR_SENSORS_PIN, OUTPUT);   //GPIO12 as output
  SetSensorPower(OFF);          //Sensors start turned off.
}

/*
    Make funcions to:
        Get last measurements and save them to RTC RAM (non-volatile on boot).
        Get gathered data packets (temperature and humidity) from RTC RAM and save them to FLASH (SPIFFS).
        Update/increment measurements counter in RTC RAM (not to count each measurement taken from the file).
    -----------------------------------------
        Initialize RTC memory and FLASH memory.
        Program must boot and call this function. If device 



    -----------------------------------------
        Get pending data packets from FLASH memory. Pending packets are determined by the difference between 
      the measurements_counter and the last_sent_measurement index.
        Transform them into human-legible format (i.e. -135 -> -
      º  13,5ºC  or 45 -> 45% RH and so).
        Generate POST request with every value to be sent:
          id_transceiver, battery_level, id_sensor[], timestamp[], temperature[], humidity[].
          In case the quantity of packets to be transmitted would exceed the maximum supported (see 
          datalogger_config.h), only a bunch of them is loaded, and the following will be sent in a separate 
          request following this one.
          After each packet is correctly sent, the last_sent_measurement index is updated so in the next
          request the device will send only the not sent ones.
    -----------------------------------------  
      Get date and time from server. When connected to it, once packets are sent, a request for 
        date and time is generated
    -----------------------------------------
        Manage times: 
          From the time the device goes deep-sleep mode (sent to the funcion goDeepSleep) to the time it actually
          boots and takes measurements, connect and send the packets, there is a lapse which must be tracked or 
          taken into account. Function millis() can be used to that.
*/

void wifiTurnOff(void){
  WiFi.mode( WIFI_OFF );  //Wifi starts turned off after wake up, in order to save energy.
  WiFi.forceSleepBegin(); 
  delay(1);
}

void wifiTurnOn(void){
  WiFi.forceSleepWake();    //Turns on radio.
  delay(1);   //Necessary delay to settle.
  WiFi.mode(WIFI_STA);  // Bring up the WiFi connection.
  WiFi.persistent(false); //Wifi credentials loaded directly from flash (as specified below) and not overwritten to it.
  WiFiMulti.addAP("Fibertel WiFi866 2.4GHz", "01416592736");
}


void handleHome(void){
  //Returns main page.
  server.send(200, "text/html", index_html);
  Serial.println("Homepage sent.");
}

unsigned char runWebServer(void){

  String ap_ssid="Datalogger";
  String ap_pssw="123456789!";
  
  //IPAddress local_IP(192,168,2,2);  //192.168.2.2
  //IPAddress gateway(192,168,2,1);   //192.168.2.1
  //IPAddress subnet(255,255,255,0);  //255.255.255.255
  //WiFi.softAPConfig(local_IP, gateway, subnet);

  WiFi.mode(WIFI_AP);
  while(!WiFi.softAP(ap_ssid, ap_pssw))
  {
    Serial.println(".");
    delay(100);
  }
  
  // Iniciar servidor
  server.begin();
  Serial.println("HTTP server started");

  Serial.println("");
  Serial.print("Initialized AP:\t");
  Serial.println(ap_ssid);
  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());
  Serial.print("MAC address:\t");
  Serial.println(WiFi.macAddress());
 
  
  server.on("/",handleHome);
  server.on("/box_network_config",);
  // server.on("",);
  // server.on("",);
  // server.on("",);
  // server.on("",);


  // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  //   Serial.print("\n HTTP / :");
  //   request->send_P(200, "text/html", index_html, processor);
  // });
  // server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request){
  //   Serial.print("\n HTTP /test :");
  //   request->send_P(200, "text/plain", "\n\n    TEST OK!!");
  // });
  // server.on("/change_network_config", HTTP_POST, [](AsyncWebServerRequest *request){
  //   Serial.print("\n HTTP /change_network_config :");
  //   request->send_P(200, "text/plain", "\n\nParámetros modificados correctamente.");
  // });
  //  server.on("/change_server_config", HTTP_POST, [](AsyncWebServerRequest *request){
  //   Serial.print("\n HTTP /change_server_config :");
  //   request->send_P(200, "text/plain", "\n\nParámetros modificados correctamente.");
  // });

  
  return(1);

}