#include <rtc_memory.hpp>
#include <user_interface.h>   //Functions to handle RTC memory.
#include <datalogger_config.h> 
#include <string.h>
#include <FS.h>


RtcMemory::RtcMemory(void){ //Constructor
  //Recover correct values to "var" from RTC memory.
  // readData(RTC_MEMORY_VARIABLES_START_BLOCK, &var, sizeof(var)); //Read stored data into rtc memory.

  
}

void RtcMemory::clearMeasurements(void){
  uint8_t initpos = RTC_MEMORY_MEASUREMENTS_START_BLOCK;
  int mem_pointer = RTC_MEMORY_MEASUREMENTS_POINTER_BLOCK;
  system_rtc_mem_write(mem_pointer, &initpos, sizeof(initpos)); //Restarts pointer value.
}

bool RtcMemory::saveMeasurements(void *data, unsigned short int bytes){
  /*  Get current pointer position (last_measurement_index + 1).
      Save new RTC_MEMORY_MEASUREMENT_BLOCK_SIZE blocks of data.
      Update pointer position (+= RTC_MEMORY_MEASUREMENT_BLOCK_SIZE)
  */
  uint8_t buffer[RTC_MEMORY_BLOCK_SIZE];  //Memory for pointer_obtained_measurement
  uint8_t pointer_obtained_measurement;
  system_rtc_mem_read(RTC_MEMORY_MEASUREMENTS_POINTER_BLOCK, buffer, RTC_MEMORY_BLOCK_SIZE);  //Read pointer position
  yield();  //Feeds watchdog.
  pointer_obtained_measurement = buffer[0];
  
  //Serial.printf("pointer_obtained_measurement: %X / ", pointer_obtained_measurement);


  if (pointer_obtained_measurement <= (RTC_MEMORY_MEASUREMENTS_END_BLOCK - RTC_MEMORY_MEASUREMENT_BLOCK_SIZE)) 
  { //If there is enough space for saving this measurement...
    system_rtc_mem_write(pointer_obtained_measurement,data,bytes);  //Writes measurements in the position the pointer is indexing.
    buffer[0] = pointer_obtained_measurement+RTC_MEMORY_MEASUREMENT_BLOCK_SIZE;
    system_rtc_mem_write(RTC_MEMORY_MEASUREMENTS_POINTER_BLOCK,buffer,RTC_MEMORY_BLOCK_SIZE); //Updates the pointer position.
    Serial.printf("new pointer value: %d / ", (unsigned int)(buffer[0]));

    ////////////////////////////  DEBUG  //////////////////////////////////
    Serial.printf("\nData written to RTC memory:  ");
    Serial.printf("address: %d / ", pointer_obtained_measurement);
    Serial.printf("bytes: %d \n", bytes);
    Serial.printf("data:");
    for (unsigned short x=0; x < bytes; x++){
      Serial.printf("%X", ((uint8_t*)(data))[x] );
    }   ///////////////////////////////////////////////////////////////////
    Serial.print("\n");

    return (true); //Returns 1 to notice that meausrements were correctly saved.
  }
  else    //If there is not enough memory to store new measurements...
  {
    Serial.print("Not enough RTC memory. ");
    return (false);  //Returns 0 to notice that measurements were not saved. The code flow should force current temporary measurements
                //to be saved into flash. 
                  /*Make a funcion to: Gather all measurements from temporary memory (RTC memory) and saves them to flash measurements.txt file.
                    Then measurements RAM gets cleared (pointer set to RTC_MEMORY_MEASUREMENTS_START_BLOCK).
                    After that, the current measurement is stored in the cleared RTC memory.*/
  }
}

bool RtcMemory::readMeasurements(uint8_t *data, unsigned short int amount){
  //  This method reads "amount" number of measurements from rtc memory, format into the
  //correct format (for flash storage) and put them into "data" pointer.

  uint8 buffer[RTC_MEMORY_MEASUREMENT_BLOCK_SIZE * 4]; //24B long buffer.
  Measurement m;
  for (uint8 i = RTC_MEMORY_MEASUREMENTS_START_BLOCK; 
      i < RTC_MEMORY_MEASUREMENTS_START_BLOCK + amount*RTC_MEMORY_MEASUREMENT_BLOCK_SIZE;
      i+= RTC_MEMORY_MEASUREMENT_BLOCK_SIZE){
        readData(i, &m, RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*4);

        arrangeData(m, buffer);   //Copies measurements arranged in the correct format to "buffer".
        
        // arraycpy(data+(sizeof(buffer) * (RTC_MEMORY_MEASUREMENTS_START_BLOCK-i)), buffer, sizeof(buffer));
        memcpy(data+(sizeof(buffer) * (RTC_MEMORY_MEASUREMENTS_START_BLOCK+i)), buffer, sizeof(buffer));

        Serial.printf("\n\nRead measurements:   (block: %d)  [%d bytes]  data: \n",i, RTC_MEMORY_MEASUREMENT_BLOCK_SIZE * 4);
        for (unsigned short x=0; x < 24; x++){
          Serial.printf("%X ", ((uint8_t*)(buffer))[x] );
        }
        Serial.printf("\ntimestamp: %d\n",m.timestamp);
        Serial.printf("id_sensor: [%d,%d,%d,%d] \n",m.id_sensor[0],m.id_sensor[1],m.id_sensor[2],m.id_sensor[3]);
        Serial.printf("temperature: [%d,%d,%d,%d] \n",m.temperature[0],m.temperature[1],m.temperature[2],m.temperature[3]);
        Serial.printf("humidity: [%d,%d,%d,%d] \n",m.humidity[0],m.humidity[1],m.humidity[2],m.humidity[3]);
  }


  return true;
}

void RtcMemory::readData(int address, void *data, unsigned short int bytes) {
  system_rtc_mem_read(address, data, bytes);
  yield();
  /////////////////// DEBUG ///////////////////////////
/* Measurement m;
  system_rtc_mem_read(address, &m, bytes);
  uint8_t buffer[288];
  system_rtc_mem_read(address, &buffer, bytes);
  Serial.printf("\n\nRead measurements:   (block: %d)   data: \n",address);
   for (unsigned short x=0; x < bytes; x++){
      Serial.printf("%X", ((uint8_t*)(buffer))[x] );
    }
  Serial.printf("\ntimestamp: %d\n",m.timestamp);
  Serial.printf("id_sensor: [%d,%d,%d,%d] \n",m.id_sensor[0],m.id_sensor[1],m.id_sensor[2],m.id_sensor[3]);
  Serial.printf("temperature: [%d,%d,%d,%d] \n",m.temperature[0],m.temperature[1],m.temperature[2],m.temperature[3]);
  Serial.printf("humidity: [%d,%d,%d,%d] \n",m.humidity[0],m.humidity[1],m.humidity[2],m.humidity[3]);
  /////////////////////////////////////////////////////
*/
  Serial.print("\n\nData read from RTC memory:");
  for (uint16 i = 0; i < bytes; i++)
  {
    Serial.printf("%X", ((uint8_t*)(data))[i] );
  }
  
}

void RtcMemory::writeData(int address, void* data, uint16_t bytes){
  //Writes data into rtc memory.
  system_rtc_mem_write(address, data, bytes);

  Serial.print("\n\nData written to RTC memory:");
  for (uint16 i = 0; i < bytes; i++)
  {
    Serial.printf("%X", ((uint8_t*)(data))[i] );
  }
}

uint8_t getPointer(void){
  uint8_t buf[RTC_MEMORY_BLOCK_SIZE];
  system_rtc_mem_read(RTC_MEMORY_MEASUREMENTS_POINTER_BLOCK, buf, RTC_MEMORY_BLOCK_SIZE);  //Read pointer position
  return(buf[0]);
}

void RtcMemory::clear(void){
  //CLEARS EVERYTHING!!  Initializes rtm memory. Clears all pointers.
  clearMeasurements();
  var.measurements_pointer = RTC_MEMORY_MEASUREMENTS_START_BLOCK; //Clears m
  var.statem_state = STATE_WAKE;
  var.powerdown_check = RTC_MEMORY_POWER_CHECK_VARIABLE;
  var.last_sync_time = 0;
  var.current_time = 0;
  var.archive_sent_pointer = 0;
  var.archive_saved_pointer = 0;
  rwVariables();  //Save modified variables.
}

Variables RtcMemory::rwVariables(void){
  //Method to read and write variables from object (RAM) to rtc memory in each operation!!!!!!
  Variables stored;   //temporal variable to read stored data.
  
  readData(RTC_MEMORY_VARIABLES_START_BLOCK, &stored, sizeof(stored)); //Read stored data into rtc memory.

  //Checks if any variable saved in RAM (RtcMemory::var) 
  //differs from the stored into the Rtc memory register.
  //If there is any modified value, overwrite the different values.

  //if   RAM value       !=    stored value
    if (var.statem_state != stored.statem_state ||
    var.measurements_pointer != stored.measurements_pointer ||
    var.last_sync_time != stored.last_sync_time ||
    var.current_time != stored.current_time     ||
    var.archive_sent_pointer!= stored.archive_sent_pointer ||
    var.archive_saved_pointer != stored.archive_saved_pointer)  //If there were any changes in RAM variables...
    {
  //then   write in RTC memory the RAM (new) value.
        writeData(RTC_MEMORY_VARIABLES_START_BLOCK, &var, sizeof(var));
    }
    else{   //If not changes in RAM variables...
        //nothing.
    }
    return(var);    //Returns actual value of RAM variables (same to RTC memory as these were written right before).
}

bool RtcMemory::checkPowerdown(void){
  if( rwVariables().powerdown_check != RTC_MEMORY_POWER_CHECK_VARIABLE)
    return true;  //
  else
    return false; //
}

bool RtcMemory::initialize(void){
  //This method is called only once when the supply has been connected to 
  //the device. It makes sure all the variables in RTC memory and RAM have 
  //the correct values for a safe operation.

  clearMeasurements();    //Clear measurements pointers.
  if(recoverVariables()){
    var.powerdown_check = RTC_MEMORY_POWER_CHECK_VARIABLE;
    rwVariables(); //Saves recovered variable into RTC memory.
    return true;
  }
  else return false;
}

bool RtcMemory::arrangeData(Measurement m, uint8_t* data){
  /*This method takes a packet of data read from rtc memory, extract all the members values
  inside of it (using the same struct type) and arrange them into a normalized 24B array.*/

  /*  typedef struct{
            uint32_t timestamp;                              -> 4B 
            uint16_t id_sensor[DATALOGGER_SENSOR_COUNT];     -> 8B
            int16_t  temperature[DATALOGGER_SENSOR_COUNT];   -> 8B
            uint8_t  humidity[DATALOGGER_SENSOR_COUNT];      -> 4B
    }Measurement;      //structure alias.                   -> (24B)
  */

  memcpy(data,    &m.timestamp, sizeof(m.timestamp));
  memcpy(data+4,  &m.timestamp, sizeof(m.id_sensor));
  memcpy(data+12, &m.timestamp, sizeof(m.temperature));
  memcpy(data+20, &m.timestamp, sizeof(m.humidity));

  // /////////////// DEBUG ////////////////////////
  // uint8_t buffer[RTC_MEMORY_MEASUREMENT_BLOCK_SIZE * 4];  //Buffer 24B long.    
  // memcpy(buffer,    &m.timestamp, sizeof(m.timestamp));
  // memcpy(buffer+4,  &m.timestamp, sizeof(m.id_sensor));
  // memcpy(buffer+12, &m.timestamp, sizeof(m.temperature));
  // memcpy(buffer+20, &m.timestamp, sizeof(m.humidity));

  // Serial.printf("\nArranged data: %d bytes  ", sizeof(buffer));
  // for (uint16 i = 0; i < sizeof(buffer); i++)
  // {
  //   Serial.printf("%X ", data[i]);
  // }
  // /////////////////////////////////////////////
  return true;
}

void RtcMemory::setElapsedTime(uint32_t time){
  var.current_time += time;   //Modifies actual value adding up the elapsed time.
  rwVariables();  //Saves to RTC memory.

  // writeData(RTC_MEMORY_VARIABLES_START_BLOCK, &var, sizeof(Variables));
}

uint32_t RtcMemory::getCurrentTime(void){
  //Returns the time calculated when the last deep sleep happened and adds up the elapsed since then.
  // readData(RTC_MEMORY_VARIABLES_START_BLOCK, &var, sizeof(Variables));

  return ((rwVariables().current_time)+(millis()/1000));
}

bool RtcMemory::safeDisconnect(void){
  //Saves current values for "var" (Variables struct type) into non-volatile memory.
  //File -> NVM_POINTERS_FILE_NAME
  Variables* p = &var;

  File file = SPIFFS.open(NVM_POINTERS_FILE_NAME, "w");
  if (file) {       //If file opened correctly...

      Serial.printf("\nsafeDiscinnect data being saved:");
      for (uint16 i = 0; i < sizeof(Variables); i++)
      {
        Serial.printf("%X ", ((const char*)p)[i]);
      }

      int bytesWritten = file.write((const char*)(p), sizeof(Variables)); //Overwrites previous data.
      file.close();
      return true;
  }

  return false;
}

bool RtcMemory::recoverVariables(void){
  //Recovers last saved data from flash and loads it to the "var" object variable.

  void* p = &var;
  uint8_t buf[sizeof(Variables)]; //temporary buffer.

  File file = SPIFFS.open(NVM_POINTERS_FILE_NAME, "r");   //Open file
  if (file){        //If file opens correctly...
     
    Serial.print("\n   Recovered data: ");  
    for (uint16_t i = 0; i < sizeof(Variables); i++)
    {
      buf[i] = file.read(); //Copies the read byte 
      Serial.printf("%X ",  buf[i]);
    } 
    memcpy(p, buf, sizeof(buf));  //Copies read data to the variable.
    // rwVariables();  //Stores read data to RTC memory.    ////////////////// REMOVED FOR TESTING ///////////
    return true;
  }

  return false;
}