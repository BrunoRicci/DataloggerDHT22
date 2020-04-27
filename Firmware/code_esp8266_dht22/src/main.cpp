#include <Arduino.h>
//#include <string.h>
#include <time.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

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
Measurement getMeasurements(void);
uint32_t getServerTimeUnix(void);

bool writeDataToFlash(String path, void* data, unsigned int bytes);
bool readDataFromFlash(String path, uint32_t index, void* data, unsigned int bytes);
bool archiveWrite(void* data, uint16_t bytes);
bool archiveRead(void* data, uint32_t first_packet, uint32_t last_packet);
uint32_t archiveGetPointer(void);

//Functions to handle web server
void handleHome(void);
void handleChangeNetworkConfig(void);
void handleChangeServerConfig(void);
unsigned char runWebServer(void);
unsigned char handleFormatRam(void);
unsigned char handleFormatFlash(void);
//---------------------------------------------------------------------------------------------------------------------

ESP8266WiFiMulti WiFiMulti;             //Wifi client side handle.
ESP8266WebServer server(8080);          //Server for configuration.
StateMachine statem(STATE_WAKE);  //Create StateMachine class object. Starts at wake.
RtcMemory rtcmem;                       //Object to handle rtc memory.


void setup() {
  portInit();
  wifiTurnOff();
  Serial.begin(115200);
  
  
  SPIFFS.begin();

  // handleFormatFlash();  //FOR TESTING ONLY!
  // handleFormatRam();

//////////////////////////////////////////////////////////////

 
  
/*   // DEBUG: Write arbitrary data to  RTC memory.
      //Get measurements...
    Measurement m;    
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
    m.humidity[3]=52;  */
    
  //   //Save measurements into temporary memory.
  //   if ( ! rtcmem.saveMeasurements(&m, sizeof(m))) //If data is not saved correctly in RAM...
  //   {
  //     Serial.printf(" \n -------- RTC memory full. Saving data to flash and clearing memory... --------");
  //     /* save to flash, clear RAM and then rewrite to it */
      
  //     uint8 buf[(RTC_MEMORY_MEASUREMENTS_END_BLOCK - RTC_MEMORY_MEASUREMENTS_START_BLOCK)*4];   //Temporary buffer to read measurements
  //     rtcmem.readData(RTC_MEMORY_MEASUREMENTS_START_BLOCK, &buf, (RTC_MEMORY_MEASUREMENTS_END_BLOCK - RTC_MEMORY_MEASUREMENTS_START_BLOCK)*4);

  //     // writeDataToFlash(MEASUREMENTS_FILE_NAME, buf, sizeof(buf)); //Save measurements to flash.
  //     archiveWrite(buf, sizeof(buf));
  //     rtcmem.clearMeasurements();   //Clears rtc memory.

  //     m.timestamp=millis();   //Put actual time for the moment the pending to store measurement is stored.
  //     rtcmem.saveMeasurements(&m, sizeof(m));   //Saves current measurement.
  
  //   }

  
  
  

}


void loop() {

  //Make method to detect if power was disconnected (to automatically call method
  // recoverVariables() and initialize rtc memory properly.)
  
  if(statem.getState() == STATE_WAKE){
    
    if(statem.stateInit()){ //State initialization.
       Serial.print("\n --- State: STATE_WAKE ---");
      wifiTurnOff();
      // if(checkBattery()){
      if(checkBattery()){ //If enough battery remaining...
         //Checks if device has lost power supply.
        if(rtcmem.checkPowerdown()){
          setBatteryState(ON);  //Connect battery.
          if(rtcmem.initialize())  //Recovers variables
            Serial.print("\nDevice powered off. RTC memory recovered correctly.");
          else  
          Serial.print("\n RTC memory recovery failed.");
        }
        
        /*RTC and external interrupt are not differenced once 
        the device goes into deep sleep. It will always detect 
        as "RTC interrupt".*/
        /*Use the previous detection mode (reedSwitchIsPressed())*/

       
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
        Serial.printf("\n    Measurements read from RTC / amount=%d ", packets);
        archiveWrite(buf, sizeof(Measurement)*packets);   //Write data into archive (flash memory).
    
        rtcmem.clearMeasurements();   //Clears rtc memory.
        rtcmem.saveMeasurements(&m, sizeof(m));   //Saves current measurement.
      }

      statem.setState(STATE_DEEP_SLEEP);
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

      wifiTurnOn();


    }
  }
  else if (statem.getState() == STATE_FORCE_MEASUREMENT){
    if(statem.stateInit()){
      Serial.print("\n --- State: FORCE STATE_FORCE_MEASUREMENT ---");
      Serial.print("\n Obtaining measurements... ");
      
      Measurement m = getMeasurements();
      //   //Save measurements into temporary memory.
      if ( ! rtcmem.saveMeasurements(&m, sizeof(m))) //If data is not saved correctly in RAM...
      {
        Serial.printf(" \n -------- RTC memory full. Saving data to flash and clearing memory... --------");
        /* save to flash, clear RAM and then rewrite to it */
        
        uint8 buf[288];   //Temporary buffer to read measurements.

        uint16_t packets = rtcmem.readMeasurements(buf);
        Serial.printf("\n    Measurements read from RTC / amount=%d ", packets);
        archiveWrite(buf, sizeof(Measurement)*packets);   //Write data into archive (flash memory).
      /* ////////////////// For testing //////////////////
        Measurement m;
        for (int16_t i = -13; i < -1; i++)
        {
          archiveRead(&m, rtcmem.rwVariables().archive_saved_pointer+i, rtcmem.rwVariables().archive_saved_pointer+i);
          Serial.printf("\nRead measurements from flash:   (packet: %d)   data: \n",rtcmem.rwVariables().archive_saved_pointer+i);
          Serial.printf("\ntimestamp: %d\n",m.timestamp);
          Serial.printf("id_sensor: [%d,%d,%d,%d] \n",m.id_sensor[0],m.id_sensor[1],m.id_sensor[2],m.id_sensor[3]);
          Serial.printf("temperature: [%d,%d,%d,%d] \n",m.temperature[0],m.temperature[1],m.temperature[2],m.temperature[3]);
          Serial.printf("humidity: [%d,%d,%d,%d] \n",m.humidity[0],m.humidity[1],m.humidity[2],m.humidity[3]);
          Serial.print("----------------------------------\n\n");
        }
        //////////////////////////////////////////////////////
      */
        rtcmem.clearMeasurements();   //Clears rtc memory.
        rtcmem.saveMeasurements(&m, sizeof(m));   //Saves current measurement.
      }
      // statem.setState(STATE_TRANSMISSION);   //Transmit pending data.

      
      statem.setState(STATE_DEEP_SLEEP);
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
      goDeepSleep(30e6-millis());
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
    
    Serial.printf("\n\n Data to write to archive:\n  buf=");
    for (uint16 i = 0; i < bytes; i++)
    {
      Serial.printf("%X ", ((const char*)data)[i]);
    }

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

bool archiveRead(void* data, uint32_t first_packet, uint32_t last_packet){

  //Read archive from one packet index to other. "start index" is the first reference and "end_index" the last one.
  //This function will copy the content from the first packet in the archive to the last one, including both
  //mentioned packets and each one between them, in the order they are saved into the archive.
  //If both indexes are equal, only one packet is read.
    
  //Validate that the index values are both multiple of RTC_MEMORY_MEASUREMENT_BLOCK_SIZE (24)!!
  
  uint32_t packet_amount = 1+(last_packet-first_packet);
  uint32_t total_bytes = packet_amount*RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*RTC_MEMORY_BLOCK_SIZE;
  if ( first_packet >= 0 && last_packet >= 0 && (packet_amount >= 0))  //If packets indexes are valid...
  {
    File file = SPIFFS.open(MEASUREMENTS_FILE_NAME, "r");   //Open file
    if (file){        //If file opens correctly...
      //If file end is not going to be reached...
      if( file.available() >= total_bytes )
      { 
          file.seek(first_packet*sizeof(Measurement),
                    fs::SeekSet);    //Move the cursor to the first_packet position.
          Serial.printf("archiveRead() first_packet: %d, last_packet: %d,   seek value:%d", first_packet, last_packet, first_packet*sizeof(Measurement));
          Serial.printf("\nData read from archive:   (currpos = %d)\n", file.position());   //For debugging...
          
          for (uint32 i = 0; i < total_bytes; i++)    //Writes data output buffer
          {
            ((uint8_t*)data)[i] = file.read(); 
            Serial.printf("%X ",((uint8_t*)data)[i]); //Prints file content.                //For debugging...  
          }
      }
      file.close();    //Close file.
      return true;
    }
    else{         //If file fails to open...
      Serial.println("Failed to open file for reading");
      return false;
    }
  }
  else
  {
    return false;
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

uint32_t getServerTimeUnix(void){
  //Cnnects to server, sends a request and returns unix time in UTC from server.
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
  WiFi.disconnect(true);  //Turns off wifi module, so in the wake up it won't turn on automatically until required.
  delay(1);
  rtcmem.setElapsedTime(t+(millis()/1000));

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
    Serial.printf("\nFLASH Formatted.\n");

    rtcmem.clear();             //Write default values into rtc memory.
    rtcmem.safeDisconnect();    //Saves default values into NVM.

    return(1);  //Returns 1 to inform that FLASH has been erased correctly. This should drive to a reboot.
  }
  else return 0;

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
