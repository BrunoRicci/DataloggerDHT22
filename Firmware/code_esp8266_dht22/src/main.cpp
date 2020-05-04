#include <Arduino.h>
#include <time.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <TimeLib.h>

#include <datalogger_config.h>  //Configuration parameters.

#include <Hash.h>
#include <WebServerFiles.cpp>

#include <FS.h>           //SPIFFS libraries.
#include <rtc_memory.hpp>
#include <statemachine.hpp>


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
bool checkBattery(void);
void setSensorPower(unsigned char state);
void wifiTurnOn(void);
void wifiTurnOff(void);
void goDeepSleep(uint64_t time);
bool isCharging(void);
bool reedSwitchIsPressed(void);

void* stringToArray(std::string origin_string);
int32_t generateMeasurementValue(unsigned char type, float value);
String formatMeasurementValue(unsigned char type, float value);  //Converts measurements into valid format
String generatePOSTRequest(void* data, uint16_t packets);
uint16_t sendMeasurements(uint16_t start_packet, uint16_t packets=0);
Measurement getMeasurements(void);
uint32_t getServerTimeUnix(void);

bool writeDataToFlash(String path, void* data, unsigned int bytes);
bool readDataFromFlash(String path, uint32_t index, void* data, unsigned int bytes);
bool archiveWrite(void* data, uint16_t bytes);
uint32_t archiveRead(void* data, uint32_t first_packet, uint32_t packets);
uint32_t archiveGetPointer(void);
bool initglobals(void);

//Functions to handle web server
void handleHome(void);
void handleChangeNetworkConfig(void);
void handleChangeServerConfig(void);
unsigned char runWebServer(void);
unsigned char handleFormatRam(void);
unsigned char handleFormatFlash(void);
//---------------------------------------------------------------------------------------------------------------------

typedef struct{
  char server_ap_ssid[31];
  char server_ap_pass[31];   
  char server_ip[16];
  uint16_t server_port;
  char local_ip[16];      //192.168.255.255 ->15 + NULL
  char wifi_security_type[10];   //// enum wifi_security_type{WEP, WPA, WPA2, WPA2E};

  uint8_t   connection_retry;     //amount of retrials to connect in case of failure. (def:1).
  uint32_t  connection_timeout;   //milliseconds to try connecting  (def:5000).
  uint32_t  response_timeout;     //milliseconds to await response  (def:2000).
  uint32_t  server_connection_timeout;
  // todo: evaluate which WPA2-Enterprise parameters are needed.
  
  uint16_t id_sensor_1;     //Parameters to put into POST request.
  uint16_t id_sensor_2;
  uint16_t id_sensor_3;
  uint16_t id_sensor_4;
  uint16_t id_transceiver;    

  uint16_t  sample_time;    //lapse between measurements (in seconds)

} Config_globals;

ESP8266WiFiMulti WiFiMulti;             //Wifi client side handle.
ESP8266WebServer server(8080);          //Server for configuration.
StateMachine statem(STATE_WAKE);  //Create StateMachine class object. Starts at wake.
RtcMemory rtcmem;                       //Object to handle rtc memory.
Config_globals config_globals;


void setup() {
  portInit();
  wifiTurnOff();
  Serial.begin(115200);
  initglobals();

  SPIFFS.begin();

   //Checks if device has lost power supply.
  if(rtcmem.checkPowerdown()){
    setBatteryState(ON);  //Connect battery.
    if(rtcmem.initialize())  //Recovers variables
      {Serial.print("\nDevice powered off. RTC memory recovered correctly.");
    // digitalWrite(15,HIGH);      //DEBUG.
      }
    else  
    Serial.print("\n RTC memory recovery failed.");
  }
}


void loop() {

  //Make method to detect if power was disconnected (to automatically call method
  // recoverVariables() and initialize rtc memory properly.)
  
  if(statem.getState() == STATE_WAKE){
    
    if(statem.stateInit()){ //State initialization.
      Serial.print("\n --- State: STATE_WAKE ---");
      wifiTurnOff();
      digitalWrite(15,HIGH);      //DEBUG.
      if(checkBattery()){ //If enough battery remaining...
        /*RTC and external interrupt are not differenced once 
        the device goes into deep sleep. It will always detect 
        as "RTC interrupt".*/

        //Transmission should be controlled by time, not by rtc memory being filled... -> CHANGED!!
        setTime(rtcmem.rwVariables().current_time);
        Serial.print("\nTime now:");
        Serial.printf("\n   %2d:%2d:%2d", hour(), minute(), second());
        Serial.printf("\n    %2d/%2d/%4d", day(), month(), year());



        if(reedSwitchIsPressed()){  //if external interrupt (switch was pressed)...
          Serial.print("\n\nExternal interrupt.");
          
          /*
              To solve: When waking up from switch (not RTC interrupt), the time will unsincronize
              as the device will go deep sleep from another full hour after that.

              This affects only when the switch being pressed is not detected -> put capacitor to hold
              the signal the time needed to boot.

              A more effective way would be to get the actual value of RTC counter (in milliseconds) and compare
              if it has reach the "timer module" -> if it does, this means the wake up was made by the timer and
              not from the switch.
          */  
          if( ! isCharging()){   //If charger is not connected...
            statem.setState(STATE_FORCE_MEASUREMENT);    //Forces measurement.
          }
          else  //If charger is connected...
          {
            Serial.printf("\n awaiting hold for config mode...");
            delay(SWITCH_HOLD_TIME_CONFIG);  //Waits for a moment...
            if(reedSwitchIsPressed()){       //If switch is still pressed...
              statem.setState(STATE_CONFIGURATION); //Go to configuration mode.
            }
            else    //if not hold...
              statem.setState(STATE_FORCE_MEASUREMENT);  //Forces measurement.
          }
        }  
        else{   //If switch was not pressed (RTC interrupt.)
          Serial.print("\n\nRTC interrupt.");
          statem.setState(STATE_GET_MEASUREMENTS);
        }
      }
      else      //If out of battery.
      {
        statem.setState(STATE_SEALED);    //Turn off.
      }
    }  
  } 
  else if (statem.getState() == STATE_GET_MEASUREMENTS){
    
    if(statem.stateInit()){
      Serial.print("\n --- State: STATE_GET_MEASUREMENTS ---");

      Serial.print("\n Obtaining measurements... ");
      
      Measurement m = getMeasurements();
    
      //Save measurements into temporary memory.
      if ( ! rtcmem.saveMeasurements(&m, sizeof(m))) //If data is not saved correctly in RAM...
      {
        Serial.printf(" \n -------- RTC memory full. Saving data to flash and clearing memory... --------");
        /* save to flash, clear RAM and then rewrite to it */
        uint8 buf[288];   //Temporary buffer to read measurements.

        uint16_t packets = rtcmem.readMeasurements(buf);
        archiveWrite(buf, sizeof(Measurement)*packets);   //Write data into archive (flash memory).
    
        rtcmem.clearMeasurements();               //Clears rtc memory.
        rtcmem.saveMeasurements(&m, sizeof(m));   //Saves current measurement.

        // statem.setState(STATE_TRANSMISSION);   //Transmit pending data.
      }
  
      statem.setState(STATE_DEEP_SLEEP);    //Goes deep sleep...
    }
  }
  else if (statem.getState() == STATE_SAVE_MEASUREMENTS){
    if(statem.stateInit()){
      Serial.print("\n --- State: STATE_SAVE_MEASUREMENTS ---");

    }
    

  }
  else if (statem.getState() == STATE_TRANSMISSION){
    if(statem.stateInit()){
      Serial.print("\n --- State: STATE_TRANSMISSION ---");

      uint8_t buf[576];
      uint16_t pending_packets, sent_packets;
      pending_packets = rtcmem.rwVariables().archive_saved_pointer - rtcmem.rwVariables().archive_sent_pointer - 1;
      Serial.printf("\n\n archive_sent_pointer= %d", rtcmem.rwVariables().archive_sent_pointer);
      Serial.printf("\n archive_saved_pointer= %d", rtcmem.rwVariables().archive_saved_pointer);
      Serial.printf("\n packets pending: %d", pending_packets);
 
      
      //Send pending measurements and increments the pointer by the amount of sent packets.
      rtcmem.var.archive_saved_pointer = archiveGetPointer(); //Make sure that pointer is updated (if value lost due power failure).
      sent_packets = sendMeasurements(rtcmem.rwVariables().archive_sent_pointer, 0);
      rtcmem.var.archive_sent_pointer += sent_packets;
      rtcmem.rwVariables();

      Serial.printf("\n\n %d packets succesfully sent.", sent_packets);

      rtcmem.safeDisconnect();  //Save current values.
      statem.setState(STATE_DEEP_SLEEP);
    }
  }
  else if (statem.getState() == STATE_FORCE_MEASUREMENT){
    if(statem.stateInit()){
      Serial.print("\n --- State: FORCE STATE_FORCE_MEASUREMENT ---");
      Serial.print("\n Obtaining measurements... ");
      
      uint8 buf[RTC_MEMORY_MEASUREMENTS_COUNT*sizeof(Measurement)];   //Temporary buffer to read measurements.
      uint16_t packets;
      Measurement m = getMeasurements();
      
      packets = rtcmem.readMeasurements(buf); //Read all the measurements stored into RTC memory.
      
      archiveWrite(buf, sizeof(Measurement)*packets);   //Write data into archive (flash memory).
      archiveWrite(&m, sizeof(Measurement));    //Write last measurement
      rtcmem.clearMeasurements();   //Clears rtc memory.
      
      //Current measurement is not saved to RTC memory, as it is directly saved into archive
      //to be sent to the server.
      
      statem.setState(STATE_TRANSMISSION);   //Transmit pending data.
    }
  }
  else if (statem.getState() == STATE_CONFIGURATION){
   
    if(statem.stateInit()){
      Serial.print("\n --- State: STATE_CONFIGURATION ---");

      digitalWrite(15,HIGH);      //DEBUG.
      runWebServer();   //Run web server.
      // rtcmem.safeDisconnect();   //Better use this when entering configuration mode (save variables)
    }

    server.handleClient(); //Handling of incoming requests
  }
  else if (statem.getState() == STATE_SEALED){
    if(statem.stateInit()){
      Serial.print("\n --- State: STATE_SEALED ---");
      rtcmem.safeDisconnect();    //Saves variables...
      //Log power failure.
      delay(100);     //100ms delay.
      setBatteryState(OFF); //Disconnect battery, device will turn off.
    }

  }
  else if (statem.getState() == STATE_DEEP_SLEEP){
    if(statem.stateInit()){
      Serial.print("\n --- State: STATE_DEEP_SLEEP ---");

      Serial.printf("\nDevice's been running for %d ms.", millis());
      goDeepSleep(60e6-millis());
    }
    
  }


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

  
}

void portInit(void){
  pinMode(PWR_SENSORS_PIN, OUTPUT);   //GPIO12 as output
  pinMode(PWR_CONTROL_PIN, OUTPUT);
  pinMode(14, INPUT);
  pinMode(15, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(PWR_CONTROL_PIN, OUTPUT);
  digitalWrite(PWR_CONTROL_PIN,HIGH);
  digitalWrite(15,LOW);
  digitalWrite(13,HIGH);
  setSensorPower(OFF);          //Sensors start turned off.
}

void setBatteryState(uint8 state){
  if (state == ON)
  {
    digitalWrite(PWR_CONTROL_PIN, HIGH);
    pinMode(PWR_CONTROL_PIN, OUTPUT);
  }
  else if (state == OFF)
  {
    pinMode(PWR_CONTROL_PIN, OUTPUT);
    digitalWrite(PWR_CONTROL_PIN, LOW);
  }  
}

uint8_t getBatteryLevel(void){
  uint16_t voltage = (uint16_t)((analogRead(BATTERY_SENSE_PIN))*(ADC_VOLTAGE_MV/1023));
  uint16_t real_voltage = (uint16_t)((float)(voltage*VBAT_VADC_RATIO));   //Actual voltage before divider.
  // Serial.printf("\nReal voltage = %dmV" ,real_voltage);
  uint8_t percentage = ( ((real_voltage - BATTERY_MIN_VOLTAGE)*100) / (BATTERY_MAX_VOLTAGE-BATTERY_MIN_VOLTAGE) );
  // Serial.printf("\nPercentage: %d" ,percentage);

  return percentage;  //Returns battery percentage.
}

bool checkBattery(void){
  if (getBatteryLevel() >= BATTERY_MIN_PERCENTAGE) 
    return true;    
  else            //when out of battery
    return false;     
}

bool isCharging(void){
  //Returns true if the device is being charged.
  setSensorPower(OFF);    //Sensors must be powered off in order to detect the charger properly.
  if ( digitalRead(CHARGER_DETECT_PIN) )  //
   { Serial.print("\nCharging.");
     return true;}
  else
   { Serial.print("\nNot charging.");
   return false; } 
}

bool reedSwitchIsPressed(void){
  //Returns true if reed switch is pressed (when GPIO associated is in logic 1).
  if(digitalRead(REED_SWITCH_PIN))  return true;
  else  return false;
}

bool writeDataToFlash(String path, void* data, unsigned int bytes) { // send the right file to the client (if it exists)
  //todo: validate data and bytes parameters. ALso chech if path exists to avoid creating multiple files unnecessarily.
  
  Serial.printf("\n\n Data to write to flash:\n  buf=");
  for (uint16 i = 0; i < bytes; i++)
  {
    Serial.printf("%X ", ((const char*)data)[i]);
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

bool readDataFromFlash(String path, uint32_t index, void* data, unsigned int bytes){

  File file = SPIFFS.open(path, "r");   //Open file

  if (!file) {
    Serial.println("Failed to open file for reading");
    return false;
  }
  else{
    file.seek(index, fs::SeekSet);    //Move the cursor "index" bytes from the beginning.
    uint16 currpos = file.position();
    Serial.printf("\ncurrpos = %d\n", currpos);
    for (uint16 i = 0; i < bytes; i++)  //Read "bytes" number of bytes from the index position. 
    {
      ((uint8_t*)data)[i] = file.read(); 
      Serial.printf("%X ",((uint8_t*)data)[i]); //Prints file content.
    } 

    file.close();    //Close file.
    return true;
  }
}

bool archiveWrite(void* data, uint16_t bytes){
  //Write "bytes" number of packets into the archive (flash memory).
  if(bytes >= RTC_MEMORY_MEASUREMENT_BLOCK_SIZE){
    
    // Serial.printf("\n\n Data to write to archive:\n  buf=");
    // for (uint16 i = 0; i < bytes; i++)
    // {
    //   Serial.printf("%X ", ((const char*)data)[i]);
    // }

    File file = SPIFFS.open(MEASUREMENTS_FILE_NAME, "a"); //Opens archive file to append measurements.
      // file.size();
    if (file) {    //If file opened correctly...
      
      int bytesWritten = file.write((const char*)data,bytes);

      if (bytesWritten == bytes) {  //If every byte is written...
        rtcmem.var.archive_saved_pointer=archiveGetPointer();   //Move pointer.
        rtcmem.rwVariables();
        Serial.printf("\nFile was written, %d bytes",bytesWritten);
      }else {
        Serial.println("\nFile write failed");
        file.close();
        return false;
      }
      file.close();
      return true;
    }
    else{        //If file is unable to open...
      Serial.println("Error opening file for writing");
    }
  }
  return false;
}

uint32_t archiveRead(void* data, uint32_t first_packet, uint32_t packets){
  /*  This function gets "packets" number of packets from the archive, starting from the "first_packet".
      If packets value is illegal (null or greater than the maximum possible) then returns 0.
      If packets is valid, it will read the amount specified and load it to *data. Then returns the amount read.
  */

  //If packets to be read are null or exceed the maximum available,  return 0. 
  if(first_packet + packets < rtcmem.rwVariables().archive_saved_pointer && packets != 0){
    
    uint32_t total_bytes = packets*sizeof(Measurement);
    
    File file = SPIFFS.open(MEASUREMENTS_FILE_NAME, "r");   //Open file
    if (file){        //If file opens correctly...
      //If file end is not going to be reached...
      if( file.available() >= total_bytes )
      { 
          file.seek(first_packet*sizeof(Measurement),
                    fs::SeekSet);    //Move the cursor to the first_packet position.
          
          /////// DEBUG /////
          // Serial.printf("archiveRead() first_packet: %d, last_packet: %d,   seek value:%d", first_packet, last_packet, first_packet*sizeof(Measurement));
          // Serial.printf("\nData read from archive:   (currpos = %d)\n", file.position());   //For debugging...
          ///////////////////
          for (uint32 i = 0; i < total_bytes; i++)    //Writes data output buffer
          {
            ((uint8_t*)data)[i] = file.read(); 
            // Serial.printf("%X ",((uint8_t*)data)[i]); //Prints file content.                //For debugging...  
          }
          file.close();    //Close file.
          return packets;
      }
      else{
        file.close();    //Close file.
        return 0;
      }
    }
    else{         //If file fails to open...
      Serial.println("Failed to open file for reading");
      return 0;
    }
  }
  else{
    return 0;
  }
}

uint32_t archiveGetPointer(void){
  //Return position of pointer, where next packet will be written.
  uint32_t size = 0;
  File file = SPIFFS.open(MEASUREMENTS_FILE_NAME, "r");   //Open file
    if (file){        //If file opens correctly...
      size = file.size();
      file.close();    //Close file.
      // Serial.printf("\nFile size = %d)\n", size);   //For debugging...
    }
  return (1+size/(RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*RTC_MEMORY_BLOCK_SIZE));   
}

Measurement getMeasurements(void){
  //Turns sensors on, take measurements, format them and return.
  Measurement m;
  dht22_sensor_1.begin();           //Initializes objects.
  dht22_sensor_2.begin();
  dht22_sensor_3.begin();
  dht22_sensor_4.begin();

  setSensorPower(ON);
  delay(1000);  
  //GET MEASUREMENTS
  m.timestamp=rtcmem.getCurrentTime();  //Put current timestamp
  m.id_sensor[0] = 1;   //Read sensor number from configuration file.
  m.id_sensor[1] = 2; 
  m.id_sensor[2] = 3; 
  m.id_sensor[3] = 4;
  m.temperature[0] = generateMeasurementValue(TEMPERATURE, dht22_sensor_1.readTemperature());
  m.temperature[1] = generateMeasurementValue(TEMPERATURE, dht22_sensor_2.readTemperature());
  m.temperature[2] = generateMeasurementValue(TEMPERATURE, dht22_sensor_3.readTemperature());
  m.temperature[3] = generateMeasurementValue(TEMPERATURE, dht22_sensor_4.readTemperature());
  m.humidity[0] = generateMeasurementValue(HUMIDITY, dht22_sensor_1.readHumidity());; 
  m.humidity[1] = generateMeasurementValue(HUMIDITY, dht22_sensor_2.readHumidity());; 
  m.humidity[2] = generateMeasurementValue(HUMIDITY, dht22_sensor_3.readHumidity());; 
  m.humidity[3] = generateMeasurementValue(HUMIDITY, dht22_sensor_4.readHumidity());;  

  setSensorPower(OFF);

  return(m);
}

String generatePOSTRequest(void* data, uint16_t packets){
  /*This function receives data saved to flash (array) and generates the POST request to send
    it to the server, which is returned.
    Necessary data:
  */
  
  String request="";  //Initialize empty string.
  String values_timestamp=  "[";
  String values_id_sensor=  "[";
  String values_temperature="[";
  String values_humidity=   "[";

  Measurement m;  //Temporary variable.
  for (uint16_t p = 0; p < packets; p++){ //For each packet...
    memcpy(&m, data+(p * sizeof(m)), sizeof(m));    //Copies one packet into the temporary memory.

    for (uint8_t s = 0; s < DATALOGGER_SENSOR_COUNT; s++){ //For each sensor...
      values_timestamp+= (String)m.timestamp;
      values_id_sensor  +=(String)m.id_sensor[s];
      values_temperature+=(String)m.temperature[s];
      values_humidity   +=(String)m.humidity[s];

      if( p < packets-1){   //If it is not the last packet...
          values_timestamp+=  ",";          
          values_id_sensor+=  ",";
          values_temperature+=",";
          values_humidity+=   ",";
      }
      else{   //If it is the last element, closes the bracket.
        if(s < DATALOGGER_SENSOR_COUNT - 1){
          values_timestamp+=  ",";          
          values_id_sensor+=  ",";
          values_temperature+=",";
          values_humidity+=   ",";
        }  
        else{
          values_timestamp+=  "]";      
          values_id_sensor+=  "]";      
          values_temperature+="]";      
          values_humidity+=   "]"; 
        } 
      }
    }
  }
  
  request += "id_transceiver=1";      //? symbol deleted as it is not needed into POST method.
  request += "&battery_level="+(String)getBatteryLevel();
  request += "&timestamp="+   values_timestamp;
  request += "&id_sensor="+   values_id_sensor;
  request += "&temperature="+ values_temperature;
  request += "&humidity="+    values_humidity; 
  
  // Serial.print("\n Generated POST request: ");
  // Serial.println(request);  
  
  return request;
}

uint16_t sendMeasurements(uint16_t start_packet, uint16_t packets){
  /*
      This function takes data packets and sends them to the server.
      "packets" default value is 0, so if not specified in function call, it will send all the
      pending packets instead of the amount specified. 
  */
  HTTPClient http;
  uint16_t start_connection_time, start_request_time;
  uint8_t connection_retries=0, request_retries=0;
  bool connection_timeout=false, request_timeout=false;
  uint16_t sent_packets_amount = 0;

  wifiTurnOn(); //Turn on wifi.
  Serial.setDebugOutput(true);  //DEBUG
  WiFi.begin(config_globals.server_ap_ssid, config_globals.server_ap_pass);
  
  Serial.print("\nConnecting to network...");
  start_connection_time = millis();
  while(WiFi.status() != WL_CONNECTED && (millis() - start_connection_time < config_globals.connection_timeout)){ //Blocks until Wifi is connected. 
    yield(); 
  }

  if(WiFi.status() == WL_CONNECTED){
    
    uint8_t buf[MAX_PACKET_PER_REQUEST*sizeof(Measurement)]; //288 bytes.
    //Generate full URL.
    String url = "http://"+(String)config_globals.server_ip+":"+(String)config_globals.server_port;
    url+=SEND_MEASUREMENTS_URL;
    Serial.print("\n url: ");   Serial.println(url);

    Serial.printf("\n HTTP client started: %d",  http.begin(url));   //Specify request destination

    uint16_t end_packet, i, n, pending_packets;

    if(packets == 0){ //Initialization of start and final packets index.
      start_packet = rtcmem.rwVariables().archive_sent_pointer; //The last sent packet following one.
      end_packet = rtcmem.rwVariables().archive_saved_pointer-1;    //The last saved packet into archive (maximum).
    }
    else{ 
      end_packet = start_packet + packets;
      if(end_packet > rtcmem.rwVariables().archive_saved_pointer-1)
        end_packet = rtcmem.rwVariables().archive_saved_pointer-1;
    }
    i = start_packet;
    n = MAX_PACKET_PER_REQUEST;
    if(i+n > end_packet){    //Limit the amount of packets to read if there are less than the maximum.
      n = end_packet-i; //Equals to pending packets.
    }
    
    while(n > 0 && request_retries < config_globals.connection_retry){   //While there are pending packets...
      
      archiveRead(buf, i,n); //Read "n" packets starting from packet number "i"

      String request = generatePOSTRequest(buf, n); //Generate request with n elements.
      Serial.print("\n[HTTP] POST...:\n");  Serial.print(request);

      http.addHeader("Content-Type", "text/plain");  //Specify content-type header
      int httpCode = http.POST(request);   //Send the request
      if(httpCode > 0){
        String payload = http.getString();  
        Serial.print("\n Response:"); Serial.println(payload);  
        http.end(); //for test.

        sent_packets_amount += n;   //Increase sent packets counter.

        i += n;  //"i" is now the last read packet.
        n = MAX_PACKET_PER_REQUEST;
        if(i+n > end_packet)    //Limit the amount of packets to read if there are less than the maximum.
          n = end_packet-i; //equal n to que amount of pending packets.
        
        Serial.printf("\n      DATA SENT. Packets pending: %d", end_packet-i);
        Serial.printf("\n  Packets in next request: %d", n);
        Serial.printf("\n  i=%d  /  n=%d\n", i, n);
      }
      else{
        Serial.printf("\n Error. Response code:%d", httpCode);
        request_retries++;
        if(request_retries < config_globals.connection_retry);
          Serial.printf("\n HTTP client started: %d",  http.begin(url));   //Specify request destination
      }
      yield();
    }
    
    //Get current time.
    url = "http://"+(String)config_globals.server_ip+":"+(String)config_globals.server_port;
    url+=GET_TIME_URL;
    Serial.print("\n url: ");   Serial.println(url);
    http.begin(url);
    if(http.GET() > 0){   //If return code is valid
      String payload = http.getString();
      payload = payload.substring(payload.indexOf("\n")+1); //Get only the numeric value.
      rtcmem.var.current_time = payload.toInt();
      rtcmem.var.last_sync_time = rtcmem.var.current_time;
      rtcmem.rwVariables();
      
      Serial.printf("\n UTC time from server: %d\n", rtcmem.rwVariables().last_sync_time);
    }
    else
      Serial.print("\nUnable to obtain time.");
    
    http.end();
    wifiTurnOff();
    return sent_packets_amount;
  }
  else{
    Serial.print("\nConnection timeout.");
    wifiTurnOff();
    return 0;
    //Connection failed.
  }


  // Serial.printf("\n Time taken: %dms.", (int)(millis() - start_time)); */
}

uint32_t getServerTimeUnix(void){ //Obsolete!!
  //Cnnects to server, sends a request and returns unix time in UTC from server.
  return 0;
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
  if(time > 3600000000 ) time = 3600000000;  //Maximum 1h (3600 sec).
  
  uint32_t t = time/1000000;
 
  rtcmem.setElapsedTime(t+(millis()/1000));   //Wrong formula -> time is in us and millis() in ms...

  Serial.printf("\nGoing deep sleep for %d seconds... ", (t));

  ESP.deepSleep(time, WAKE_RF_DISABLED);  //deep sleeps for 5 seconds...
}

void setSensorPower(unsigned char state){
  if (state == ON)
  {
    digitalWrite(PWR_SENSORS_PIN, LOW);  //Turn sensors supply on.
    // Serial.printf("\nSensors turned on.");
  }
  if (state == OFF)
  {
    digitalWrite(PWR_SENSORS_PIN, HIGH);  //Turn sensors supply off.
    // Serial.printf("\nSensors turned off.");
  }
}

void wifiTurnOff(void){
  WiFi.disconnect(true);  //Turns off wifi module, so in the wake up it won't turn on automatically until required.
  WiFi.mode( WIFI_OFF );  //Wifi starts turned off after wake up, in order to save energy.
  delay(1);
  WiFi.forceSleepBegin(); 
  delay(1);
}

void wifiTurnOn(void){
  WiFi.forceSleepWake();    //Turns on radio.
  delay(1);   //Necessary delay to settle.
  WiFi.mode(WIFI_STA);  // Bring up the WiFi connection.
  WiFi.persistent(false); //Wifi credentials loaded directly from flash (as specified below) and not overwritten to it.
}

bool initglobals(void){
  /*  Read configuration file (JSON) and load data into Config_globals struct. 
  First load all the data from file into a string, and then parse it to a Json buffer,
  to then extract the values from each keyword.
  If file not found, load default values, defined in this function.
  */

  void* p = &config_globals;
  uint8_t buf[sizeof(Config_globals)]; //temporary buffer.

  File file = SPIFFS.open(CONFIG_FILE_NAME, "r");   //Open file
  if (file){        //If file opens correctly...
     
    Serial.print("\n  Config_globals: ");  
    for (uint16_t i = 0; i < sizeof(buf); i++)
    {
      buf[i] = file.read(); //Copies the read byte 
      Serial.printf("%X ",  buf[i]);
    } 
    memcpy(p, buf, sizeof(buf));  //Copies read data to the variable.
    // rwVariables();  //Stores read data to RTC memory.    ////////////////// REMOVED FOR TESTING ///////////
    return true;
  }
  else{     //Load default values.
    // strcpy(config_globals.server_ap_ssid,"DATALOGGER SERVER");
    // strcpy(config_globals.server_ap_pass,"!UBA12345!");
    // strcpy(config_globals.server_ip, "192.168.137.1");
    strcpy(config_globals.server_ap_ssid,"Fibertel WiFi866 2.4GHz");
    strcpy(config_globals.server_ap_pass,"01416592736");
    strcpy(config_globals.server_ip, "192.168.0.172");
    strcpy(config_globals.local_ip,"192.168.4.1");
    strcpy(config_globals.wifi_security_type,"WPA2");

    config_globals.server_port=8080;
    config_globals.connection_retry=2;
    config_globals.connection_timeout=20000;
    config_globals.server_connection_timeout=2000;
    config_globals.response_timeout=5000;
    config_globals.id_transceiver=1;
    config_globals.id_sensor_1=1;
    config_globals.id_sensor_2=2;
    config_globals.id_sensor_3=3;
    config_globals.id_sensor_4=4;
    config_globals.sample_time=3600;
  }
  return false;
}

//////////////////////////// WEB SERVER FUNCTIONS /////////////////////////////////
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

  // uint8_t buf[RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*4]; //24 Bytes
  // readDataFromFlash(MEASUREMENTS_FILE_NAME, 0, buf, sizeof(buf));

  Measurement m;  
  for (uint16 i = 0; i < 12; i++)
  {
    // readDataFromFlash(MEASUREMENTS_FILE_NAME, i*RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*4, &m, sizeof(buf));
    archiveRead(&m, i, i);
    Serial.printf("\nRead measurements from flash:   (index: %d)   data: \n",i*RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*4);
    Serial.printf("\ntimestamp: %d\n",m.timestamp);
    Serial.printf("id_sen sor: [%d,%d,%d,%d] \n",m.id_sensor[0],m.id_sensor[1],m.id_sensor[2],m.id_sensor[3]);
    Serial.printf("temperature: [%d,%d,%d,%d] \n",m.temperature[0],m.temperature[1],m.temperature[2],m.temperature[3]);
    Serial.printf("humidity: [%d,%d,%d,%d] \n",m.humidity[0],m.humidity[1],m.humidity[2],m.humidity[3]);
    Serial.print("----------------------------------\n\n");
  }
 
  Serial.printf("\nArchive pointer: %d", archiveGetPointer());

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
      rtcmem.clear();                   //Reset values to dafault.
      rtcmem.safeDisconnect();          //Saves current values in NVM.
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
    SPIFFS.remove(NVM_POINTERS_FILE_NAME);    //Delete variables backup.
    SPIFFS.remove(CONFIG_FILE_NAME);          //Delete 
    Serial.printf("\nFLASH Formatted.\n");

    rtcmem.clear();             //Write default values into rtc memory.
    rtcmem.safeDisconnect();    //Saves default values into NVM.

    return(1);  //Returns 1 to inform that FLASH has been erased correctly. This should drive to a reboot.
  }
  else return 0;

}

bool handleChangeConfig(void){
  /*
    Open configuration file (JSON) and overwrite the values received. 
    If any entry is invalid or blank/null, don't overwrite it.
    Also data should be validated from both browser (Javascript code) 
    and this function too, in order to avoid errors.
  */
 
  void* p = &config_globals;

  File file = SPIFFS.open(CONFIG_FILE_NAME, "w");
  if (file) {       //If file opened correctly...

      Serial.printf("\nglobals saved:");
      for (uint16 i = 0; i < sizeof(config_globals); i++)
      {
        Serial.printf("%X ", ((const char*)p)[i]);
      }

      file.write((const char*)(p), sizeof(Variables)); //Overwrites previous data.
      file.close();
      return true;
  }

  return false;
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
