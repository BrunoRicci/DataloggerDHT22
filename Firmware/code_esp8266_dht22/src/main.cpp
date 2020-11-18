#include <Arduino.h>
#include <ArduinoOTA.h>
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



typedef struct{
  uint16_t id_transceiver;  //Placed first to avoid value corruption when struct is modified.

  char network_ap_ssid[31];
  char network_ap_pass[31];   
  char server_ip[64];
  uint16_t server_port;
  char local_ip[16];      //192.168.255.255 ->15 + NULL
  char send_measurements_path[64];
  char get_time_path[64];

  char wifi_security_type[10];   //// enum wifi_security_type{WEP, WPA, WPA2, WPA2E};
  
  char client_static_ip[4];       //255.255.255.255 
  char client_gateway_ip[4];
  char client_subnet_mask[4];
  char client_dns1_ip[4];
  char client_dns2_ip[4];

  uint8_t   server_connection_retry;     //amount of retrials to connect in case of failure. (def:1).
  uint32_t  network_connection_timeout;   //milliseconds to try connecting  (def:5000).
  uint32_t  response_timeout;     //milliseconds to await response  (def:2000).
  uint32_t  server_connection_timeout;
  // todo: evaluate which WPA2-Enterprise parameters are needed.
  
  uint16_t id_sensor_a;     //Parameters to put into POST request.
  uint16_t id_sensor_b;
  uint16_t id_sensor_c;
  uint16_t id_sensor_d;
      
  uint16_t  sample_time;    //lapse between measurements (in seconds)
  uint8_t   connection_time[2]; //[hours][minutes]

  uint8_t operation_mode;
  bool ischarging;
} Config_globals;

typedef struct{
  uint32_t  timestamp;
  uint8_t   battery_level;
  uint8_t   event_type;
  enum EventTypes {POWER_FAIL, FORCE_TURN_OFF, FORCE_MEASUREMENT, CONFIG_MODE} ;
}Logs;

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
void dischargeCapacitor(void);
void LED_Color(uint32 color);


void* stringToArray(std::string origin_string);
int32_t generateMeasurementValue(unsigned char type, float value);
String formatMeasurementValue(unsigned char type, float value);  //Converts measurements into valid format
String generatePOSTRequest(void* data, uint16_t packets);
uint16_t sendMeasurements(uint16_t start_packet, uint16_t packets=0);
Measurement getMeasurements(void);
uint32_t getServerTimeUnix(void);
void scheduleConnect(void);

bool writeDataToFlash(String path, void* data, unsigned int bytes);
bool readDataFromFlash(String path, uint32_t index, void* data, unsigned int bytes);
bool archiveWrite(void* data, uint16_t bytes);
uint32_t archiveRead(void* data, uint32_t first_packet, uint32_t packets);
uint32_t archiveGetPointer(void);
bool initglobals(void);
bool saveGlobals(void);
Config_globals readGlobals(void);
bool log_event(uint16_t event_type);
uint16_t read_log(void* data, uint16_t index=0, uint16_t amount=0);

//Functions to handle web server
void handleHome(void);
void handleChangeNetworkConfig(void);
void handleChangeServerConfig(void);
unsigned char runWebServer(void);
unsigned char handleFormatRam(void);
unsigned char handleFormatFlash(void);
unsigned char handleResetSentPointer(void);
unsigned char handleTurnOffDevice(void);
unsigned char handleGetParameters(void);
bool handleChangeConfig(void);

//---------------------------------------------------------------------------------------------------------------------

ESP8266WiFiMulti WiFiMulti;             //Wifi client side handle.
ESP8266WebServer server(8080);          //Server for configuration.
StateMachine statem(STATE_WAKE);  //Create StateMachine class object. Starts at wake.
RtcMemory rtcmem;                       //Object to handle rtc memory.
Config_globals config_globals;
uint32_t idle_time_start;



void setup() {
  wifiTurnOff();
  portInit();
  delay(50);      //Delay 50ms to avoid false "forced-turn on" after boot.
  SPIFFS.begin();
  initglobals();
  config_globals.ischarging = isCharging();

  Serial.begin(115200);
  
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
      if(checkBattery()){ //If enough battery remaining...
        /*RTC and external interrupt are not differenced once 
        the device goes into deep sleep. It will always detect 
        as "RTC interrupt".*/

        //Transmission should be controlled by time, not by rtc memory being filled... -> CHANGED!!
        setTime(rtcmem.rwVariables().current_time);
        Serial.print("\nTime now:");
        Serial.printf("\n   %d:%d:%d (UTC)", hour(), minute(), second());
        Serial.printf("\n    %2d/%2d/%4d", day(), month(), year());

        //Show reset reason:
        rst_info *resetinfo;
        resetinfo = ESP.getResetInfoPtr();
        Serial.printf("\n Reset casue: ");
        Serial.println(resetinfo->reason);
        ///////////////////////////////////
        
        if (resetinfo->reason == rst_reason::REASON_DEEP_SLEEP_AWAKE){  //If woke up from deep sleep
          if(reedSwitchIsPressed()){  //if external interrupt (switch was pressed)...
            Serial.print("\n\nExternal interrupt.");
            statem.setState(STATE_FORCE_MEASUREMENT);    //Forces measurement.
          }  
          else{             //If switch was not pressed (RTC interrupt.)
            Serial.print("\n\nRTC interrupt.");
            
              //GMT-3 time                            hours
            if(config_globals.connection_time[0] == hour()){  //FIXME: This only works with hourly wake-up, where this value won't repeat multiple times a day.
                Serial.printf("\nSchedule connection. Hours: %d (UTC)",hour());
                // scheduleConnect();
                statem.setState(STATE_FORCE_MEASUREMENT);
            }
            else{
              statem.setState(STATE_GET_MEASUREMENTS);    //Normal measurement 
            }
          }
        }
        else if (resetinfo->reason == rst_reason::REASON_EXT_SYS_RST){  // If reset while running...
          
          LED_Color(COLOR_VIOLET);
          Serial.flush();
          delay(500);
          String serialdata = Serial.readString();
          uint32_t t;

          if(! serialdata.isEmpty() ){  //If received any data...
            t = serialdata.toInt();
            if (t > 0 && t < 4294967296 ){ //Validate value -> if t=0, the value is not int.
              Serial.print(MSG_SYNC_TIME);
              Serial.printf("\n New time value: %d", t);
              // t -= config_globals.sample_time;  //Substract 1 hour for the hour that will be summed after this deep sleep.
              rtcmem.var.current_time=t;  //Saves updated time.
              rtcmem.rwVariables();
              statem.setState(STATE_FORCE_MEASUREMENT);  //Forces measurement.
            }
            else{ //If data was not integer (string)
              if (serialdata == COMMAND_FORCE_MEASUREMENT)
              {
                Serial.print("\n Force measurement command received.");
                statem.setState(STATE_FORCE_MEASUREMENT);  //Forces measurement.
              }
              else if (serialdata == COMMAND_CONFIG_MODE)
              {
                Serial.print("\n Config mode command received.");
                statem.setState(STATE_CONFIGURATION); //Go to configuration mode.
              }
            }       
          }
          else{
            uint16_t cfg_time_start = millis();
            if (getBatteryLevel() == 100){  //Protection for when it's not connected to USB charger.
              Serial.printf("\n awaiting hold for config mode...");
              while (reedSwitchIsPressed()){
                yield();
                if((millis() - cfg_time_start) >= SWITCH_HOLD_TIME_CONFIG){
                  statem.setState(STATE_CONFIGURATION); //Go to configuration mode.
                  break;
                }
              }
              if((millis() - cfg_time_start) < SWITCH_HOLD_TIME_CONFIG){
                statem.setState(STATE_FORCE_MEASUREMENT);  //Forces measurement.
              }
            }
            else{
              statem.setState(STATE_FORCE_MEASUREMENT);  //Forces measurement.
            }
          }
        }
        else{
          statem.setState(STATE_FORCE_MEASUREMENT);   //Forces measurement -> If reboots for another reason, 
                                                      //connects to server so low battery level can be checked remotely...
        }
      }
      else{ //If out of battery.
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
      // rtcmem.var.archive_sent_pointer += sent_packets;
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

      runWebServer();   //Run web server.      
      // rtcmem.safeDisconnect();   //Better use this when entering configuration mode (save variables)
    }

    server.handleClient(); //Handling of incoming requests
    
    if(wifi_softap_get_station_num() > 0){  //If there is any station connected to the local AP...
      idle_time_start = millis(); //Set new reference to current time.
      if (idle_time_start >= 1800000){    //If more than 30 min. elapsed...
        statem.setState(STATE_DEEP_SLEEP);  //Forces deep sleep.
      }
    }
    else{
      if((millis() - idle_time_start) >= CONFIG_MODE_IDLE_TIMEOUT){
        statem.setState(STATE_DEEP_SLEEP);
      }
    }

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

      pinMode(DHT_SENSOR_1_PIN, INPUT_PULLUP);
      pinMode(DHT_SENSOR_2_PIN, INPUT_PULLUP);
      pinMode(DHT_SENSOR_3_PIN, INPUT_PULLUP);
      pinMode(DHT_SENSOR_4_PIN, INPUT_PULLUP);

      wifiTurnOff();
      Serial.printf("\nDevice has been running for %d ms.", millis());
      goDeepSleep(config_globals.sample_time*1000 - millis());
    }
  }

}

void portInit(void){
  pinMode(PWR_SENSORS_PIN_P, OUTPUT);   //GPIO13 as output
  pinMode(PWR_SENSORS_PIN_N, OUTPUT);   //GPIO13 as output
  pinMode(PWR_CONTROL_PIN, OUTPUT);
  pinMode(REED_SWITCH_PIN, INPUT);
  pinMode(PWR_CONTROL_PIN, OUTPUT);

  dischargeCapacitor();

  pinMode(DHT_SENSOR_1_PIN, INPUT);
  pinMode(DHT_SENSOR_2_PIN, INPUT);
  pinMode(DHT_SENSOR_3_PIN, INPUT);
  pinMode(DHT_SENSOR_4_PIN, INPUT);

  digitalWrite(PWR_CONTROL_PIN,HIGH);

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
  //It is necessary to turn on the sensors in order to measure the battery level:
  // setSensorPower(ON);
  // delay(10);  //10ms delay...
  uint16 adcr = analogRead(BATTERY_SENSE_PIN);
  // setSensorPower(OFF);

  uint16_t voltage = (uint16_t)((adcr * ADC_VOLTAGE_MV)/(1023)); //Read value
  // Serial.printf("\nVoltage = %dmV" ,voltage);
  uint16_t real_voltage = (uint16_t)((float)(voltage*VBAT_VADC_RATIO) + Q_VCE_COMPENSATION);   //Actual voltage before divider.
  // Serial.printf("\nReal voltage = %dmV" ,real_voltage);
  uint8_t percentage = ( ((real_voltage - BATTERY_MIN_VOLTAGE)*100) / (BATTERY_MAX_VOLTAGE-BATTERY_MIN_VOLTAGE) );
  // Serial.printf("\nPercentage: %d" ,percentage);

  if(percentage > 100)  percentage=100; //Limit maximum value.

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
/*   setSensorPower(OFF);    //Sensors must be powered off in order to detect the charger properly.
  pinMode(PWR_SENSORS_PIN_P, INPUT);  //As the same pin is used to power sensors and sense USB connected, put it as input temporarily.
  if ( !digitalRead(CHARGER_DETECT_PIN) ){  
    Serial.print("\nCharging.");
    pinMode(PWR_SENSORS_PIN_P, OUTPUT);    //Returns to original state.
    return true;}
  else{ 
    Serial.print("\nNot charging.");
    pinMode(PWR_SENSORS_PIN_P, OUTPUT);    //Returns to original state.
    return false; 
  }  */

  pinMode(CHARGER_DETECT_PIN, INPUT); //GPIO3 (RX) pin put as input to sense.

  if( digitalRead(CHARGER_DETECT_PIN) ){  
    Serial.print("\nCharging.");
    pinMode(CHARGER_DETECT_PIN, FUNCTION_0);    //Returns to original state.
    return true;
  }
  else{ 
    Serial.print("\nNot charging.");
    pinMode(CHARGER_DETECT_PIN, FUNCTION_0);    //Returns to original state.
    return false; 
  } 
}

bool reedSwitchIsPressed(void){
  //Returns true if reed switch is pressed (when GPIO associated is in logic 1).
  if(digitalRead(REED_SWITCH_PIN)){  
    // digitalWrite(REED_SWITCH_PIN, LOW);   //Discharge capacitor  -> maybe this whould be implemented before deep sleep.
    // pinMode(REED_SWITCH_PIN, OUTPUT);     //Change pin to ouput.
    // pinMode(REED_SWITCH_PIN, INPUT);    //Put pin at the original type.
    return true;
  }
  else  return false;
}

void dischargeCapacitor(void){
  pinMode(REED_SWITCH_PIN, OUTPUT);     
  digitalWrite(REED_SWITCH_PIN, LOW);   //Put pin in 0 to discharge capacitor
  // delay(1);                      //waits a while till it discharges.
  pinMode(REED_SWITCH_PIN, INPUT);      //Put the pin as input again (to sense).
}

void LED_Color(uint32_t color){   //8-bit colors
  uint16_t r, g, b;
  // Serial.printf("\n color: %X", color);

  r = (color >> 16) & 0xFF;
  g = (color >> 8) & 0xFF;
  b = (color) & 0xFF;

  r = (r*100)/255;    //Convert to percentage for duty cycle.  -> Not needed
  g = (g*100)/255;
  b = (b*100)/255;
 
  r = 1023-((1023*r)/100);          //Invert value because LEDs are in high side (common annode).
  g = 1023-((1023*g)/100);
  b = 1023-((1023*b)/100);

  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, HIGH);    //Turn off leds.
  digitalWrite(LED_G_PIN, HIGH);
  digitalWrite(LED_B_PIN, HIGH);

  analogWrite(LED_R_PIN, r); 
  analogWrite(LED_G_PIN, g); 
  analogWrite(LED_B_PIN, b); 
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
    Serial.printf("\nArchive file size: %d bytes",file.size());
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
  delay(SENSOR_ON_TIME);  
  //GET MEASUREMENTS
  m.timestamp=rtcmem.getCurrentTime();  //Put current timestamp
  m.id_sensor[0] = config_globals.id_sensor_a;   //Read sensor number from configuration file.
  m.id_sensor[1] = config_globals.id_sensor_b; 
  m.id_sensor[2] = config_globals.id_sensor_c; 
  m.id_sensor[3] = config_globals.id_sensor_d;
  m.temperature[0] = generateMeasurementValue(TEMPERATURE, dht22_sensor_1.readTemperature());
  m.temperature[1] = generateMeasurementValue(TEMPERATURE, dht22_sensor_2.readTemperature());
  m.temperature[2] = generateMeasurementValue(TEMPERATURE, dht22_sensor_3.readTemperature());
  m.temperature[3] = generateMeasurementValue(TEMPERATURE, dht22_sensor_4.readTemperature());
  m.humidity[0] = generateMeasurementValue(HUMIDITY, dht22_sensor_1.readHumidity());
  m.humidity[1] = generateMeasurementValue(HUMIDITY, dht22_sensor_2.readHumidity());
  m.humidity[2] = generateMeasurementValue(HUMIDITY, dht22_sensor_3.readHumidity());
  m.humidity[3] = generateMeasurementValue(HUMIDITY, dht22_sensor_4.readHumidity()); 
  
  setSensorPower(OFF);

  for (uint8_t i = 0; i < 4; i++){    //Invalid measurements values are normalised into 200% RH and -200 ºC.
    if (m.humidity[i] == 255){
      m.humidity[i] = 255;
      m.temperature[i] = -2000;
    }
  }
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
  
  request += "id_transceiver="+(String)config_globals.id_transceiver;      //? symbol deleted as it is not needed into POST method.
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
  LED_Color(0); // Turn off leds.

  HTTPClient http;
  uint16_t start_connection_time;
  uint8_t request_retries=0;
  uint16_t sent_packets_amount = 0;
  // bool network_connection_timeout=false, request_timeout=false;

  wifiTurnOn(); //Turn on wifi.


  Serial.setDebugOutput(true);  //DEBUG
  if (config_globals.client_static_ip[0] != 0){    //if valid static IP

    WiFi.config(IPAddress(    //Local IP
      config_globals.client_static_ip[0],
      config_globals.client_static_ip[1],
      config_globals.client_static_ip[2],
      config_globals.client_static_ip[3]
      ),
      IPAddress(              //Gateway
        config_globals.client_gateway_ip[0],
        config_globals.client_gateway_ip[1],
        config_globals.client_gateway_ip[2],
        config_globals.client_gateway_ip[3]
      ),
      IPAddress(              //Subnet
        config_globals.client_subnet_mask[0],
        config_globals.client_subnet_mask[1],
        config_globals.client_subnet_mask[2],
        config_globals.client_subnet_mask[3]
      ),
      IPAddress(              //DNS1
        config_globals.client_dns1_ip[0],
        config_globals.client_dns1_ip[1],
        config_globals.client_dns1_ip[2],
        config_globals.client_dns1_ip[3]
      ),
      IPAddress(              //DNS2
        config_globals.client_dns2_ip[0],
        config_globals.client_dns2_ip[1],
        config_globals.client_dns2_ip[2],
        config_globals.client_dns2_ip[3]
      )
      //IPAddress(200,42,4,204)     //DNS2
    );
    Serial.print("\nStatic IP fixed: ");
    Serial.println(WiFi.localIP());
  }

  WiFi.begin(config_globals.network_ap_ssid, config_globals.network_ap_pass);
  LED_Color(COLOR_BLUE);
      
  Serial.print("\nConnecting to network...");
  start_connection_time = millis();
  //Blocks until Wifi is connected. 
  while(WiFi.status() != WL_CONNECTED && (millis() - start_connection_time < config_globals.network_connection_timeout)){ 
    yield(); 
  }
  LED_Color(COLOR_BLACK);
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());
  Serial.println(WiFi.dnsIP());

  if(WiFi.status() == WL_CONNECTED){
    
    uint8_t buf[MAX_PACKET_PER_REQUEST*sizeof(Measurement)]; //288 bytes.
    //Generate full URL.
    String url = "http://www."+(String)config_globals.server_ip+":"+(String)config_globals.server_port;
    url+=SEND_MEASUREMENTS_PATH;
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
    
    while(n > 0 && request_retries < config_globals.server_connection_retry){   //While there are pending packets...
      
      archiveRead(buf, i,n); //Read "n" packets starting from packet number "i"

      String request = generatePOSTRequest(buf, n); //Generate request with n elements.
      Serial.print("\n[HTTP] POST...:\n");  Serial.print(request);

      http.addHeader("Content-Type", "application/x-www-form-urlencoded");  //Specify content-type header.
      http.addHeader("Accept", "text/html");  //Specify content-type header.
      int httpCode = http.POST(request);   //Send the request.
      if(httpCode == 200){
        String payload = http.getString();  
        Serial.print("\n Response:"); Serial.println(payload);  
        http.end(); //for test.
 
        LED_Color(COLOR_GREEN);     //Blink GREEN led.
        delay(50);
        LED_Color(COLOR_BLACK);

        sent_packets_amount += n;   //Increase sent packets counter.
        rtcmem.var.archive_sent_pointer += n;

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
        LED_Color(COLOR_RED);     //Blink red led.
        delay(100);
        LED_Color(COLOR_BLACK);

        request_retries++;
        if(request_retries < config_globals.server_connection_retry);  //This semicolon should not be here as it inhibits the if...
          Serial.printf("\n HTTP client started: %d",  http.begin(url));   //Specify request destination
      }
      yield();
    }
    
    //Get current time.
    url = "http://"+(String)config_globals.server_ip+":"+(String)config_globals.server_port;
    url+=GET_TIME_PATH;
    Serial.print("\n url: ");   Serial.println(url);
    http.begin(url);
    if(http.GET() == 200){   //If return code is valid
      String payload = http.getString();
      payload = payload.substring(payload.indexOf("\n")+1); //Get only the numeric value.

      if( !payload.isEmpty() ){
        rtcmem.var.current_time = (uint32_t)payload.toInt();
        rtcmem.var.last_sync_time = rtcmem.var.current_time;
        rtcmem.rwVariables();
        Serial.printf("\n UTC time from server: %d\n", rtcmem.rwVariables().last_sync_time);
      }
      else
        Serial.print("\nInvalid time value.");
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
    LED_Color(COLOR_RED);     //Blink red led.
    yield();
    delay(200);
    LED_Color(COLOR_BLACK);
    LED_Color(COLOR_RED); //Blink red led.
    yield();
    delay(200);
    LED_Color(COLOR_BLACK);

    return 0;
    //Connection failed.
  }


  // Serial.printf("\n Time taken: %dms.", (int)(millis() - start_time)); */
}

uint32_t getServerTimeUnix(void){ //Obsolete!!
  //Cnnects to server, sends a request and returns unix time in UTC from server.
  return 0;
}

void scheduleConnect(void){
  // LED_Color(COLOR_CYAN);
  // Alarm.delay(750);
  // LED_Color(COLOR_BLACK);
  Serial.print("\n- - - - - - - Scheduled alarm. - - - - - - -\n");
  // if(sendMeasurements(0,0)) return true; else return false;
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
  // time is in milliseconds.
  
  if( time > 3600000 ) time = 3600000;  //Maximum 1h (3600 sec).
  
  uint32_t t = time/1000;
 
  rtcmem.setElapsedTime(t + millis()/1000);   //in seconds
  /*  If forced wake up, elapsed time should 
  */
  // rtcmem.setElapsedTime(t + millis()/1000);   //in seconds

  Serial.printf("\nGoing deep sleep for %d seconds... ", (t));

  ESP.deepSleep(time*1000, WAKE_RF_DISABLED);  //deep sleeps for 5 seconds...
}

void setSensorPower(unsigned char state){
  if (state == ON)
  {
    digitalWrite(PWR_SENSORS_PIN_P, LOW);  //Turn sensors supply on.
    digitalWrite(PWR_SENSORS_PIN_N, HIGH);  //Turn sensors supply on.
    // Serial.printf("\nSensors turned on.");
  }
  if (state == OFF)
  {
    digitalWrite(PWR_SENSORS_PIN_P, HIGH);  //Turn sensors supply off.
    digitalWrite(PWR_SENSORS_PIN_N, LOW);  //Turn sensors supply on.
    // Serial.printf("\nSensors turned off.");
  }
}

void wifiTurnOff(void){
  WiFi.disconnect(true);  //Turns off wifi module, so in the wake up it won\\\"t turn on automatically until required.
  delay(1);
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

  config_globals = readGlobals();
  
  return false;
}

bool log_event(uint16_t event_type){ 
  /*
    Log timestamp, battery status and event type into flash memory.
  */
  Logs l;
  l.battery_level = getBatteryLevel();    //Current battery level.
  l.timestamp = rtcmem.getCurrentTime();  //Current timestamp.
  l.event_type = event_type;

  File file = SPIFFS.open(LOGS_FILE_NAME, "a");   //Open file to append
  if (file){
    int bytesWritten = file.write((const char*)(&log), sizeof(Logs)); //Overwrites previous data.
    file.close();
    Serial.printf("\nEvent (type %d) logged.", l.event_type);
    return true;
  }
  return false;
}

uint16_t read_log(void* data, uint16_t index, uint16_t amount){
  /*  
    Read "index" amount of logs, starting from the given index.
      If amount is 0, it will return the number of logs saved.
  */

  File file = SPIFFS.open(LOGS_FILE_NAME, "r");   //Open file
  if(file){
    uint16_t size = file.size();

    if(amount == 0 || index >= size) return size;

    if(index+amount >= (size/sizeof(Logs))){   //If size exceeded, amount to read is modified.
      amount = size - index;
    }
    
    if(index + (amount * sizeof(Logs)) >= size){  //Avoid reading illegal values.
      return 0;
    }

    file.seek(index, fs::SeekSet);    //Move the cursor "index" bytes from the beginning.
    uint16 currpos = file.position();
    Serial.printf("\ncurrpos = %d\n", currpos);
    for (uint16 i = 0; i < (amount * sizeof(Logs)); i++)  //Read "bytes" number of bytes from the index position. 
    {
      ((uint8_t*)data)[i] = file.read(); 
    } 
    file.close();    //Close file.
    return amount;
  }
  else{
    Serial.printf("Failed to open [%s] file for reading", LOGS_FILE_NAME);
    return 0;
  }
}
 
bool saveGlobals(void){
  /*
    SEE HOW TO SAVE TO EEPROM... (?)
  */
  Config_globals* g = &config_globals;

  File file = SPIFFS.open(CONFIG_FILE_NAME, "w");   //Open file
  if (file){
    int bytesWritten = file.write((const char*)(g), sizeof(Config_globals)); //Overwrites previous data.
    file.close();
    return true;
  }
  return false;
}

Config_globals readGlobals(void){
  Config_globals g;

  uint8_t buf[sizeof(Config_globals)]; //temporary buffer.

  File file = SPIFFS.open(CONFIG_FILE_NAME, "r");   //Open file
  if (file){        //If file opens correctly...
    // Serial.print("\n  Config_globals: ");  
    for (uint16_t i = 0; i < sizeof(buf); i++)
    {
      buf[i] = file.read(); //Copies the read byte 
      // Serial.printf("%X ",  buf[i]);
    } 
    file.close();
    memcpy(&g, buf, sizeof(buf));  //Copies read data to the variable.
    // rwVariables();  //Stores read data to RTC memory.    ////////////////// REMOVED FOR TESTING ///////////
    return g;
  }
  else{     //Load default values.
    strcpy(config_globals.network_ap_ssid,"DATALOGGER SERVER");
    strcpy(config_globals.network_ap_pass,"!UBA12345!");
    strcpy(config_globals.server_ip, "192.168.123.123");
    strcpy(config_globals.local_ip,"192.168.123.2");
    strcpy(config_globals.wifi_security_type,"WPA2");
    strcpy(config_globals.send_measurements_path, SEND_MEASUREMENTS_PATH);
    strcpy(config_globals.get_time_path, GET_TIME_PATH);

    config_globals.client_static_ip[0]=0; //Def: 0.0.0.0
    config_globals.client_static_ip[0]=0;
    config_globals.client_static_ip[0]=0;
    config_globals.client_static_ip[0]=0;
    config_globals.server_port=8080;
    config_globals.server_connection_retry=2;
    config_globals.network_connection_timeout=10000;
    config_globals.server_connection_timeout=2000;
    config_globals.response_timeout=5000;
    config_globals.id_transceiver=0;
    config_globals.id_sensor_a=1;
    config_globals.id_sensor_b=2;
    config_globals.id_sensor_c=3;
    config_globals.id_sensor_d=4;
    config_globals.sample_time=3600;
    config_globals.connection_time[0]=13;  //13 a.m. UTC
    config_globals.connection_time[1]=0;

    saveGlobals();
  }
}


//////////////////////////// WEB SERVER FUNCTIONS /////////////////////////////////
unsigned char runWebServer(void){ 

  String ap_ssid="Datalogger";
  String ap_pssw="123456789!";
  
  IPAddress local_IP(192,168,0,170);  //192.168.0.170
  IPAddress gateway(192,168,0,1);   //192.168.0.1
  IPAddress subnet(255,255,255,0);  //255.255.255.255
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.mode(WIFI_AP);
  while(!WiFi.softAP(ap_ssid, ap_pssw))
  {
    Serial.println(".");
    delay(100);
  }
 
 
  // wifiTurnOn();
  // WiFi.config(local_IP, gateway, subnet);
  // WiFi.begin(config_globals.network_ap_ssid, config_globals.network_ap_pass);
  // while(WiFi.status() != WL_CONNECTED){ //Blocks until Wifi is connected. 
  //   digitalWrite(LED_B_PIN, LOW); //Blink GREEN led.
  //   delay(50);
  //   digitalWrite(LED_B_PIN, HIGH);  
  //   yield(); 
  // }

  LED_Color(COLOR_YELLOW); //Turns on red LED.
  //analogWriteFreq(5);   //See minimum frequency

  // start server
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
  server.on("/reset_archive_sent_pointer",handleResetSentPointer);
  server.on("/device_turn_off",handleTurnOffDevice);
  server.on("/get_parameters",handleGetParameters);
  server.on("/device_config",handleChangeConfig);

  return(1);
}

void handleHome(void){
  Serial.print("\n/");
  //Returns main page.
  server.send(200, "text/html", index_html);
  Serial.printf("\n\n    Number of clients connected: %d", wifi_softap_get_station_num());
  // log_event(Logs::CONFIG_MODE);

  // char buf[100*sizeof(Logs)];
  // Logs l;
  // int q = read_log(buf, 0, 0);
  // Serial.printf("\nLogs:");
  // for (int i = 0; i < q; i++)
  // {
  //   memcpy(&l, &buf[i*sizeof(Logs)], sizeof(Logs));
  //   Serial.printf("\ntimestamp: %d / battery_level:%d / event:%d",l.timestamp, l.battery_level, l.event_type);
  // }

}

void handleChangeNetworkConfig(void){
    Serial.print("/change_network_config");
    String client_local_ip, client_gateway_ip, client_subnet_mask, client_dns1_ip, client_dns2_ip;
    uint8_t index=0, octet;
    String new_ssid, new_password;
    
    if (server.hasArg("network_ap_ssid") && server.hasArg("network_ap_pass")){
        // new_ssid = server.arg("new_ssid");
        // new_password = server.arg("new_password");
        server.arg("network_ap_ssid").toCharArray(config_globals.network_ap_ssid, sizeof(config_globals.network_ap_ssid));
        server.arg("network_ap_pass").toCharArray(config_globals.network_ap_pass, sizeof(config_globals.network_ap_pass));
        client_local_ip = server.arg("client_static_ip");
        client_gateway_ip = server.arg("client_gateway_ip");
        client_subnet_mask = server.arg("client_subnet_mask");
        client_dns1_ip = server.arg("client_dns1_ip");
        client_dns2_ip = server.arg("client_dns2_ip");

        //If ip is not empty and has 4 periods...
        if(client_local_ip.length() && std::count(client_local_ip.begin(), client_local_ip.end(), '.') == 3  ){
          octet = client_local_ip.substring(index,client_local_ip.indexOf(".",index)).toInt();
          index = client_local_ip.indexOf(".",index)+1;
          config_globals.client_static_ip[0] = octet;
          octet = client_local_ip.substring(index,client_local_ip.indexOf(".",index)).toInt();
          index = client_local_ip.indexOf(".",index)+1;
          config_globals.client_static_ip[1] = octet;
          octet = client_local_ip.substring(index,client_local_ip.indexOf(".",index)).toInt();
          index = client_local_ip.indexOf(".",index)+1;
          config_globals.client_static_ip[2] = octet;
          octet = client_local_ip.substring(index).toInt();
          config_globals.client_static_ip[3] = octet;
        }

        if(client_gateway_ip.length() && std::count(client_gateway_ip.begin(), client_gateway_ip.end(), '.') == 3  ){
          index=0;
          octet = client_gateway_ip.substring(index,client_gateway_ip.indexOf(".",index)).toInt();
          index = client_gateway_ip.indexOf(".",index)+1;
          config_globals.client_gateway_ip[0] = octet;
          octet = client_gateway_ip.substring(index,client_gateway_ip.indexOf(".",index)).toInt();
          index = client_gateway_ip.indexOf(".",index)+1;
          config_globals.client_gateway_ip[1] = octet;
          octet = client_gateway_ip.substring(index,client_gateway_ip.indexOf(".",index)).toInt();
          index = client_gateway_ip.indexOf(".",index)+1;
          config_globals.client_gateway_ip[2] = octet;
          octet = client_gateway_ip.substring(index).toInt();
          config_globals.client_gateway_ip[3] = octet;
        }

        if(client_subnet_mask.length() && std::count(client_subnet_mask.begin(), client_subnet_mask.end(), '.') == 3  ){
          index=0;
          octet = client_subnet_mask.substring(index,client_subnet_mask.indexOf(".",index)).toInt();
          index = client_subnet_mask.indexOf(".",index)+1;
          config_globals.client_subnet_mask[0] = octet;
          octet = client_subnet_mask.substring(index,client_subnet_mask.indexOf(".",index)).toInt();
          index = client_subnet_mask.indexOf(".",index)+1;
          config_globals.client_subnet_mask[1] = octet;
          octet = client_subnet_mask.substring(index,client_subnet_mask.indexOf(".",index)).toInt();
          index = client_subnet_mask.indexOf(".",index)+1;
          config_globals.client_subnet_mask[2] = octet;
          octet = client_subnet_mask.substring(index).toInt();
          config_globals.client_subnet_mask[3] = octet;
        }

        if(client_dns1_ip.length() && std::count(client_dns1_ip.begin(), client_dns1_ip.end(), '.') == 3  ){
          index=0;
          octet = client_dns1_ip.substring(index,client_dns1_ip.indexOf(".",index)).toInt();
          index = client_dns1_ip.indexOf(".",index)+1;
          config_globals.client_dns1_ip[0] = octet;
          octet = client_dns1_ip.substring(index,client_dns1_ip.indexOf(".",index)).toInt();
          index = client_dns1_ip.indexOf(".",index)+1;
          config_globals.client_dns1_ip[1] = octet;
          octet = client_dns1_ip.substring(index,client_dns1_ip.indexOf(".",index)).toInt();
          index = client_dns1_ip.indexOf(".",index)+1;
          config_globals.client_dns1_ip[2] = octet;
          octet = client_dns1_ip.substring(index).toInt();
          config_globals.client_dns1_ip[3] = octet;
        }

        if(client_dns2_ip.length() && std::count(client_dns2_ip.begin(), client_dns2_ip.end(), '.') == 3  ){
          index=0;
          octet = client_dns2_ip.substring(index,client_dns2_ip.indexOf(".",index)).toInt();
          index = client_dns2_ip.indexOf(".",index)+1;
          config_globals.client_dns2_ip[0] = octet;
          octet = client_dns2_ip.substring(index,client_dns2_ip.indexOf(".",index)).toInt();
          index = client_dns2_ip.indexOf(".",index)+1;
          config_globals.client_dns2_ip[1] = octet;
          octet = client_dns2_ip.substring(index,client_dns2_ip.indexOf(".",index)).toInt();
          index = client_dns2_ip.indexOf(".",index)+1;
          config_globals.client_dns2_ip[2] = octet;
          octet = client_dns2_ip.substring(index).toInt();
          config_globals.client_dns2_ip[3] = octet;
        }

        Serial.printf("\nCLIENT STATIC IP:  %d.%d.%d.%d",  config_globals.client_static_ip[0],
                                                          config_globals.client_static_ip[1],
                                                          config_globals.client_static_ip[2],
                                                          config_globals.client_static_ip[3]);

        String cl_ip =  String(config_globals.client_static_ip[0],DEC)+"."+
                  String(config_globals.client_static_ip[1],DEC)+"."+
                  String(config_globals.client_static_ip[2],DEC)+"."+
                  String(config_globals.client_static_ip[3],DEC);
        Serial.print("\nConfig: Static IP:");
        Serial.println(cl_ip);


        //Change parameters in Flash config file.
        saveGlobals();

        // server.send(200, "text/plain", "Network credentials modified correctly.");
        server.send(200);   //Returns that everything is updated OK.

        // Serial.printf("\nNetwork credentials modified:\nSSID:%s\nPass:%s",new_ssid,new_password);
    }
    else{
        server.send(200, "text/html", "<h5>ERROR: Wrong parameters.</h5>");
    }
}

void handleChangeServerConfig(void){
  Serial.print("/change_server_config");
  String new_ip, new_port;

  if (server.hasArg("server_ip") && server.hasArg("server_port")){
      new_ip = server.arg("server_ip");
      new_port = server.arg("server_port");
      /* Validate ip and port number */
      server.arg("server_ip").toCharArray(config_globals.server_ip, sizeof(config_globals.server_ip));
      config_globals.server_port=server.arg("server_port").toInt();
      server.arg("send_measurements_path").toCharArray(config_globals.send_measurements_path, sizeof(config_globals.send_measurements_path));
      server.arg("get_time_path").toCharArray(config_globals.get_time_path, sizeof(config_globals.get_time_path));

      //Change parameters in Flash config file.
      saveGlobals();

      server.send(200);   //Returns that everything is updated OK.
  }
  else{
      server.send(200, "text/html", "<h5>ERROR: Wrong parameters.</h5>");
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
      // server.send(200, "text/plain", "Server parameters modified correctly.");
      server.send(200);   //Return code 200 (OK).

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
    SPIFFS.remove(LOGS_FILE_NAME);    //Delete logs file.
    // SPIFFS.remove(CONFIG_FILE_NAME);          //Delete 
    Serial.printf("\nFLASH Formatted.\n");

    rtcmem.clear();             //Write default values into rtc memory.
    rtcmem.safeDisconnect();    //Saves default values into NVM.

    server.send(200);   //Return code 200 (OK).

    return(1);  //Returns 1 to inform that FLASH has been erased correctly. This should drive to a reboot.
  }
  else return 0;

}

unsigned char handleResetSentPointer(void){
  Serial.print("\nresetting archive_sent_pointer...");
  rtcmem.var.archive_sent_pointer = 0;
  rtcmem.rwVariables();   //Save variables to RTC memory.
  server.send(200);   //Return code 200 (OK).
  Serial.printf("\n\n    archive_sent_pointer reset. New value: %d", rtcmem.var.archive_sent_pointer);
  Serial.printf("\n\n    archive_saved_pointer current value: %d", rtcmem.var.archive_saved_pointer);
}

unsigned char handleTurnOffDevice(void){
  Serial.print("\n\n Device set to turn off...");
  statem.setState(STATE_SEALED);
}

bool handleChangeConfig(void){
  /*
    Open configuration file and overwrite the values received. 
    If any entry is invalid or blank/null, don't overwrite it.
    Also data should be validated from both browser (Javascript code) 
    and this function too, in order to avoid errors.
  */
  //Number of args.
  Serial.print("\n\n (/device_config) - args:\n");
  if (server.args() > 0){
    for (uint16_t i = 0; i < server.args(); i++){
        Serial.printf("\n arg %d->", i);
        Serial.println(String(server.argName(i)));
        Serial.println(String(server.arg(i)));
    }

    /* server.arg("network_ap_ssid").toCharArray(config_globals.network_ap_ssid, sizeof(config_globals.network_ap_ssid));
    server.arg("network_ap_pass").toCharArray(config_globals.network_ap_pass, sizeof(config_globals.network_ap_pass));
    server.arg("server_ip").toCharArray(config_globals.server_ip, sizeof(config_globals.server_ip));
    // server.arg("local_ip").toCharArray(config_globals.local_ip, sizeof(config_globals.local_ip));
    server.arg("wifi_security_type").toCharArray(config_globals.wifi_security_type, sizeof(config_globals.wifi_security_type));
    config_globals.server_port=server.arg("server_port").toInt();
    server.arg("send_measurements_path").toCharArray(config_globals.send_measurements_path, sizeof(config_globals.send_measurements_path));
    server.arg("get_time_path").toCharArray(config_globals.get_time_path, sizeof(config_globals.get_time_path));
    */
  
    config_globals.network_connection_timeout=server.arg("network_connection_timeout").toInt();
    config_globals.server_connection_retry=server.arg("server_connection_retry").toInt();
    // config_globals.server_connection_timeout=server.arg("server_connection_timeout").toInt();
    // config_globals.response_timeout=server.arg("response_timeout").toInt();
    config_globals.id_transceiver=server.arg("id_transceiver").toInt();
    config_globals.id_sensor_a=server.arg("id_sensor_a").toInt();
    config_globals.id_sensor_b=server.arg("id_sensor_b").toInt();
    config_globals.id_sensor_c=server.arg("id_sensor_c").toInt();
    config_globals.id_sensor_d=server.arg("id_sensor_d").toInt();
    config_globals.sample_time=server.arg("sample_time").toInt();
    config_globals.connection_time[0]=server.arg("connection_time").toInt();  //6:30 a.m. UTC
    config_globals.connection_time[1]=0;  //00 minutes default value.


//Show modified values
    Serial.print("\n Configuration parameters modified:\n");
    Serial.print("\n network_ap_ssid:");
    Serial.println(config_globals.network_ap_ssid);
    Serial.print("\n network_ap_pass:");
    Serial.println(config_globals.network_ap_pass);
    Serial.print("\n server_ip:");
    Serial.println(config_globals.server_ip);

    Serial.printf("\n server_port: %d",config_globals.server_port);
    Serial.printf("\n network_connection_timeout: %d", config_globals.network_connection_timeout);
    Serial.printf("\n server_connection_retry: %d", config_globals.server_connection_retry);
    Serial.printf("\n id_transceiver: %d", config_globals.id_transceiver);
    Serial.printf("\n id_sensor_a: %d", config_globals.id_sensor_a);
    Serial.printf("\n id_sensor_b: %d", config_globals.id_sensor_b);
    Serial.printf("\n id_sensor_c: %d", config_globals.id_sensor_c);
    Serial.printf("\n id_sensor_d: %d", config_globals.id_sensor_d);
    Serial.printf("\n sample_time: %d", config_globals.sample_time);
    Serial.printf("\n connection_time: %d", config_globals.connection_time[0]);

    saveGlobals();
    server.send(200);   //Returns that everything is updated OK.
    return true;
  }
  else 
    return false;
}

unsigned char handleGetParameters(void){
  //Send configuration parameters to the client.

  //Generates IP address with each octet.
  String cl_ip, cl_gateway_ip, cl_subnet_mask,cl_dns1_ip,cl_dns2_ip;

  cl_ip =         String(config_globals.client_static_ip[0],DEC)+"."+
                  String(config_globals.client_static_ip[1],DEC)+"."+
                  String(config_globals.client_static_ip[2],DEC)+"."+
                  String(config_globals.client_static_ip[3],DEC);
  cl_gateway_ip = String(config_globals.client_gateway_ip[0],DEC)+"."+
                  String(config_globals.client_gateway_ip[1],DEC)+"."+
                  String(config_globals.client_gateway_ip[2],DEC)+"."+
                  String(config_globals.client_gateway_ip[3],DEC);
  cl_subnet_mask =String(config_globals.client_subnet_mask[0],DEC)+"."+
                  String(config_globals.client_subnet_mask[1],DEC)+"."+
                  String(config_globals.client_subnet_mask[2],DEC)+"."+
                  String(config_globals.client_subnet_mask[3],DEC);
  cl_dns1_ip =    String(config_globals.client_dns1_ip[0],DEC)+"."+
                  String(config_globals.client_dns1_ip[1],DEC)+"."+
                  String(config_globals.client_dns1_ip[2],DEC)+"."+
                  String(config_globals.client_dns1_ip[3],DEC);
  cl_dns2_ip =    String(config_globals.client_dns2_ip[0],DEC)+"."+
                  String(config_globals.client_dns2_ip[1],DEC)+"."+
                  String(config_globals.client_dns2_ip[2],DEC)+"."+
                  String(config_globals.client_dns2_ip[3],DEC);

  String parameters = "\"{";
  parameters += "\\\"network_ap_ssid\\\":\\\""+String(config_globals.network_ap_ssid)+"\\\"";
  parameters += ",\\\"network_ap_pass\\\":\\\""+String(config_globals.network_ap_pass)+"\\\"";
  parameters += ",\\\"client_static_ip\\\":\\\""+String(cl_ip)+"\\\"";

  parameters += ",\\\"client_gateway_ip\\\":\\\""+String(cl_gateway_ip)+"\\\"";
  parameters += ",\\\"client_subnet_mask\\\":\\\""+String(cl_subnet_mask)+"\\\"";
  parameters += ",\\\"client_dns1_ip\\\":\\\""+String(cl_dns1_ip)+"\\\"";
  parameters += ",\\\"client_dns2_ip\\\":\\\""+String(cl_dns2_ip)+"\\\"";

  parameters += ",\\\"server_ip\\\":\\\""+String(config_globals.server_ip)+"\\\"";
  parameters += ",\\\"server_port\\\":"+String(config_globals.server_port);
  parameters += ",\\\"send_measurements_path\\\":\\\""+String(config_globals.send_measurements_path)+"\\\"";
  parameters += ",\\\"get_time_path\\\":\\\""+String(config_globals.get_time_path)+"\\\"";
  parameters += ",\\\"network_connection_timeout\\\":"+String(config_globals.network_connection_timeout);
  parameters += ",\\\"server_connection_retry\\\":"+String(config_globals.server_connection_retry);
  parameters += ",\\\"id_transceiver\\\":"+String(config_globals.id_transceiver);
  parameters += ",\\\"id_sensor_a\\\":"+String(config_globals.id_sensor_a);
  parameters += ",\\\"id_sensor_b\\\":"+String(config_globals.id_sensor_b);
  parameters += ",\\\"id_sensor_c\\\":"+String(config_globals.id_sensor_c);
  parameters += ",\\\"id_sensor_d\\\":"+String(config_globals.id_sensor_d);
  parameters += ",\\\"sample_time\\\":"+String(config_globals.sample_time);
  parameters += ",\\\"connection_time\\\":"+String(config_globals.connection_time[0]);
  parameters += ",\\\"archive_saved_pointer\\\":"+String(rtcmem.var.archive_saved_pointer);
  parameters += ",\\\"archive_sent_pointer\\\":"+String(rtcmem.var.archive_sent_pointer);

  parameters += "}\"";

  Serial.print("Static IP:");
  Serial.println(cl_ip);
  Serial.print("\n\nparameters sent to web interface:\n");
  Serial.println(parameters);
  server.send(200, "text/plain", parameters);

  return true;
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
