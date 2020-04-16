#include <Arduino.h>
//#include <string.h>
#include <time.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#include <Hash.h>
//#include <ESPAsyncWebServer.h>
//#include <ESPAsyncTCP.h>
#include <WebServerFiles.cpp>

#include <FS.h>           //SPIFFS libraries.
#include <user_interface.h>   //Functions to handle RTC memory.
#include <rtc_memory.hpp>

#include <datalogger_config.h>  //Configuration parameters.

//----------------------------------------------------------------------------------------
DHT dht22_sensor_1(DHT_SENSOR_1_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_2(DHT_SENSOR_2_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_3(DHT_SENSOR_3_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object
DHT dht22_sensor_4(DHT_SENSOR_4_PIN, DHTTYPE);    //Creates DHT22 sensor "sensor" object

//---------------------------------------------------------------------------------------------------------------------
//Initialization functions.
void portInit(void);
uint8_t getBatteryLevel(void);           //Get battery level percentage (0 to 100%).
void setBatteryState(uint8 state=ON);     //Connects or disconnects battery.
void SetSensorPower(unsigned char state);
void wifiTurnOn(void);
void wifiTurnOff(void);
void goDeepSleep(uint64_t time);

void* stringToArray(std::string origin_string);
int32_t generateMeasurementValue(unsigned char type, float value);
String formatMeasurementValue(unsigned char type, float value);  //Converts measurements into valid format

bool writeDataToFlash(String path, void* data, unsigned int bytes);
bool readDataFromFlash(String path, uint32_t index, void* data, unsigned int bytes);
// void clearMeasurements(void);
// unsigned char saveMeasurements(void *data, unsigned short int bytes);
// void readData(int address, void *data, unsigned short int bytes);
// void init(void);

//Functions to handle 
void handleHome(void);
void handleChangeNetworkConfig(void);
void handleChangeServerConfig(void);
unsigned char runWebServer(void);
unsigned char handleFormatRam(void);
unsigned char handleFormatFlash(void);
//---------------------------------------------------------------------------------------------------------------------

ESP8266WiFiMulti WiFiMulti;
ESP8266WebServer server(8080);

RtcMemory rtcmem;


void setup() {
  portInit();
  wifiTurnOff();
  Serial.begin(115200);


  delay(200);
  
  if (digitalRead(14))    //
  {
    digitalWrite(15,HIGH);
    getBatteryLevel();
   
  }
  else
  {
    digitalWrite(15,LOW);
  }

  SPIFFS.begin();
  
  runWebServer();

  handleFormatFlash();  //FOR TESTING ONLY!
  handleFormatRam();


//////////////////////////////////////////////////////////////

/* 
  dht22_sensor_1.begin();           //Initializes objects.
  dht22_sensor_2.begin();
  dht22_sensor_3.begin();
  dht22_sensor_4.begin();

  SetSensorPower(ON);
  delay(1000);
  //GET MEASUREMENTS
  for (unsigned char i = 0; i<4 ; i++){
    Serial.printf("\nTemperature_%d: %f\n", i+1, temperature_float[i] );
    Serial.printf("Humidity_%d: %f\n", i+1,humidity_float[i] );
  } 
  SetSensorPower(OFF);
*/

  // DEBUG: Write arbitrary data to  RTC memory.
      //Get measurements...
    measurement m;    
    m.timestamp=1234567890+millis(); //Put current timestamp
    m.id_sensor[0]=1;   
    m.id_sensor[1]=2; 
    m.id_sensor[2]=3; 
    m.id_sensor[3]=4;
    m.temperature[0]=190; 
    m.temperature[1]=195;
    m.temperature[2]=195;
    m.temperature[3]=192;
    m.humidity[0]=56; 
    m.humidity[1]=54; 
    m.humidity[2]=54; 
    m.humidity[3]=52; 
    
    //Save measurements into temporary memory.
    if ( ! rtcmem.saveMeasurements(&m, sizeof(m))) //If data is not saved correctly in RAM...
    {
      Serial.printf("RTC memory full. Saving data to flash and clearing memory...");
      /* save to flash, clear RAM and then rewrite to it */
      
      //////////////////////////////////// Read data from RTC memory and stores in Flash ////////////////
          // uint8 buf[288];   //Temporary buffer to read measurements
          // for (uint8 i = RTC_MEMORY_MEASUREMENTS_START_BLOCK; 
          //     i <= RTC_MEMORY_MEASUREMENTS_END_BLOCK-RTC_MEMORY_MEASUREMENT_BLOCK_SIZE;
          //     i+= RTC_MEMORY_MEASUREMENT_BLOCK_SIZE){
          //       rtcmem.readData(i, &buf, RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*4);
          //     }
          
          uint8 buf[288];   //Temporary buffer to read measurements
          rtcmem.readMeasurements(buf, 12);
          // rtcmem.readData(RTC_MEMORY_MEASUREMENTS_START_BLOCK, &buf, (RTC_MEMORY_MEASUREMENTS_END_BLOCK - RTC_MEMORY_MEASUREMENTS_START_BLOCK)*4);

         
          
      

      //THE PROEBLEM IS HERE:
      /*
          the "buf" buffer is sent repeatedly to the method readData, which will return "RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*4"
          bytes of data (currently 24B). So it is overwritting the same first 24B every time, not concatenating
          the data. What should be done is reading the whole rtcmemory measurement data and copying it to the buffer.
          Then, send the buffer to save.

          Also, data lenght is not consistant in 24B because some variables are not "ocupying" the full size.
          See what's up with this. 
          Data length stored in flash must remain the same as RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*4 is (24B) so
          when read, measurements are separated orderly.

      */

          writeDataToFlash(MEASUREMENTS_FILE_NAME, buf, sizeof(buf));
        ///////////////////////////////////////////////////////////////////////////////////////////////////

      rtcmem.clearMeasurements();   //Clears rtc memory

      m.timestamp=millis();   //Put actual time for the moment the pending to store measurement is stored.
      rtcmem.saveMeasurements(&m, sizeof(m));   //Saves current measurement.
    }

  
  
  
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

/* 
  measurement m;
  m.timestamp=1234567890; //Put current timestamp
  m.id_sensor[0]=1;   
  m.id_sensor[1]=2; 
  m.id_sensor[2]=3; 
  m.id_sensor[3]=4;
  m.temperature[0]=generateMeasurementValue(TEMPERATURE, dht22_sensor_1.readTemperature()); 
  m.temperature[1]=generateMeasurementValue(TEMPERATURE, dht22_sensor_2.readTemperature());
  m.temperature[2]=generateMeasurementValue(TEMPERATURE, dht22_sensor_3.readTemperature());
  m.temperature[3]=generateMeasurementValue(TEMPERATURE, dht22_sensor_4.readTemperature());
  m.humidity[0]=generateMeasurementValue(HUMIDITY, dht22_sensor_1.readHumidity()); 
  m.humidity[1]=generateMeasurementValue(HUMIDITY, dht22_sensor_2.readHumidity()); 
  m.humidity[2]=generateMeasurementValue(HUMIDITY, dht22_sensor_3.readHumidity()); 
  m.humidity[3]=generateMeasurementValue(HUMIDITY, dht22_sensor_4.readHumidity());  
  saveMeasurements(&m, sizeof(m)); 
*/
  

 
  



  
}

void portInit(void){
  pinMode(PWR_SENSORS_PIN, OUTPUT);   //GPIO12 as output
  pinMode(14, INPUT);
  pinMode(15, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(15,LOW);
  digitalWrite(13,HIGH);
  digitalWrite(12,HIGH);
  SetSensorPower(OFF);          //Sensors start turned off.


}

uint8_t getBatteryLevel(void){
  uint16_t voltage = (uint16_t)((analogRead(BATTERY_SENSE_PIN))*(ADC_VOLTAGE_MV/1023));
  uint16_t real_voltage = (uint16_t)((float)(voltage*VBAT_VADC_RATIO));   //Actual voltage before divider.
  // Serial.printf("\nReal voltage = %dmV" ,real_voltage);
  uint8_t percentage = ( ((real_voltage - BATTERY_MIN_VOLTAGE)*100) / (BATTERY_MAX_VOLTAGE-BATTERY_MIN_VOLTAGE) );
  // Serial.printf("\nPercentage: %d" ,percentage);

  return percentage;  //Returns battery percentage.
}


bool readDataFromFlash(String path, uint32_t index, void* data, unsigned int bytes){

  File file = SPIFFS.open(path, "r");
 
  if (!file) {
    Serial.println("Failed to open file for reading");
    return false;
  }
  else{
    Serial.printf("File Content: from position %d \n", file.position());
  
    while (file.available()) {
      Serial.printf("%X",file.read()); //Prints file content.
    }
    file.close();    //Closes file
    return true;
  }
}

bool writeDataToFlash(String path, void* data, unsigned int bytes) { // send the right file to the client (if it exists)
  //todo: validate data and bytes parameters. ALso chech if path exists to avoid creating multiple files unnecessarily.
  
  Serial.printf("\n\n Data to write to flash:\n  buf=");
  for (uint16 i = 0; i < bytes; i++)
  {
    Serial.printf("%X", ((const char*)data)[i]);
  }


  File file = SPIFFS.open(path, "a"); //Opens file specified in path parameter to write.
  if (!file) {    //If file is unable to open...
    Serial.println("Error opening file for writing");
    return false;
  }
  else{     //If file opened correctly...
    
    // Serial.printf("\nCursor position at %d",file.seek(0,fs::SeekEnd));     //Put cursor to the end of the file.
    unsigned int pos = 0;//file.position();
    int bytesWritten = file.write((const char*)data,bytes);
    // int bytesWritten = file.printf("measurement: %d\n",millis());
    if (bytesWritten > 0) {
        Serial.printf("\nFile was written in position %d, %d bytes", pos, bytesWritten);
    } else {
      Serial.println("\nFile write failed");
    }
    file.close();
    return true;
  }
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
      //formatMeasurementValue(TEMPERATURE, temperature[0]);

       
      request += "id_transceiver=1";
      request += "battery_level=100";
      request += "timestamp=[,,,,,,,]";
      request += "id_sensor=[1,2,3,4,1,2,3,4]";
      request += "temperature=[19.7,19.5,20.2,19.5,20.2,21.0,20.5,20.7]";
      request += "humidity=[35,39,35,40,43,46,48,45]"; 
      

     /*  request+= "t1=";
      request+= formatMeasurementValue(TEMPERATURE, dht22_sensor_1.readTemperature());
      request+= "&t2=";
      request+= formatMeasurementValue(TEMPERATURE, dht22_sensor_2.readTemperature());
      request+= "&t3=";
      request+= formatMeasurementValue(TEMPERATURE, dht22_sensor_3.readTemperature());
      request+= "&t4=";
      request+= formatMeasurementValue(TEMPERATURE, dht22_sensor_4.readTemperature());
      request+= "&h1=";
      request+= formatMeasurementValue(HUMIDITY, dht22_sensor_1.readHumidity());
      request+= "&h2=";
      request+= formatMeasurementValue(HUMIDITY, dht22_sensor_2.readHumidity());
      request+= "&h3=";
      request+= formatMeasurementValue(HUMIDITY, dht22_sensor_3.readHumidity());
      request+= "&h4=";
      request+= formatMeasurementValue(HUMIDITY, dht22_sensor_4.readHumidity());
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

int32_t generateMeasurementValue(unsigned char type, float value){
  int32_t measurement=0; 

  if (type == TEMPERATURE){
     // -200 ~ 800 -> -20ºC to 80ºC
     measurement = (int16_t)(value*10);
  }
  else if (type == HUMIDITY)
  { // 0 ~ 100 ->  0% to 100%
    measurement = (uint8_t)(value);
  }

  Serial.printf("\n\nMeasurement: %d \n", measurement);

  return(measurement);
}

String formatMeasurementValue(unsigned char type, float value){  //Converts measurements into valid format
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
  server.on("/change_network_config", handleChangeNetworkConfig);
  server.on("/change_server_config", handleChangeServerConfig);
  server.on("/format_ram",handleFormatRam);
  server.on("/format_flash",handleFormatFlash);

  return(1);
}

void handleHome(void){
  Serial.print("/");
  //Returns main page.
  server.send(200, "text/html", index_html);
}

void handleChangeNetworkConfig(void){
    Serial.print("/change_network_config");
    String new_ssid, new_password;
    
    if (server.hasArg("new_ssid") && server.hasArg("new_password"))
    {
        new_ssid = server.arg("new_ssid");
        new_password = server.arg("new_password");
        //Change parameters in Flash config file.

        server.send(200, "text/plain", "Network credentials modified correctly.");
        // Serial.printf("\nNetwork credentials modified:\nSSID:%s\nPass:%s",new_ssid,new_password);
    }
    else
    {
        server.send(200, "text/html", "ERROR: Wrong parameters.");
    }

   
  readDataFromFlash(MEASUREMENTS_FILE_NAME, 0, 0, 0);
}

void handleChangeServerConfig(void){
  Serial.print("/change_server_config");
  String new_ip, new_port;
  
  if (server.hasArg("ip") && server.hasArg("port"))
  {
      new_ip = server.arg("new_ip");
      new_port = server.arg("new_port");
      /* Validate ip and port number */

      //Change parameters in Flash config file.
      server.send(200, "text/plain", "Server parameters modified correctly.");
      // Serial.printf("\nServer parameters modified:\nIP:%s\nPort:%s",new_ip,new_port);
  }
  else
  {
      server.send(200, "text/html", "ERROR: Wrong parameters.");
  } 
}

unsigned char handleFormatRam(void){
  if (server.hasArg("command"))
  {
    if(server.arg("command") == "format_confirm"){
      /*Call to format RAM*/
      rtcmem.clearMeasurements();
      Serial.printf("RAM Formatted.");
      server.send(200, "text/plain", "Server parameters modified correctly.");

      return(1);  //Returns 1 to inform that RAM has been erased correctly. This should drive to a reboot.
    }
  }
  server.send(200, "text/html", "ERROR: Wrong parameters.");
  return(0);  //Returns 1 to inform that RAM has NOT been erased correctly.
}

unsigned char handleFormatFlash(void){
  
  if(SPIFFS.remove(MEASUREMENTS_FILE_NAME)){    //Delete file.
    //SPIFFS.format();
    Serial.printf("\nFLASH Formatted.\n");
    return(1);  //Returns 1 to inform that FLASH has been erased correctly. This should drive to a reboot.
  }
  else return 0;

}


