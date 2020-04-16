#include <rtc_memory.hpp>
#include <user_interface.h>   //Functions to handle RTC memory.
#include <datalogger_config.h> 

RtcMemory::RtcMemory(void){ //Constructor
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

bool RtcMemory::readMeasurements(void *data, unsigned short int amount){
//  This method reads "amount" number of measurements from rtc memory, format into the
//correct format (for flash storage) and put them into "data" pointer.

  uint8 buffer[RTC_MEMORY_MEASUREMENT_BLOCK_SIZE * 4]; //24B long buffer.
  measurement m;
  for (uint8 i = RTC_MEMORY_MEASUREMENTS_START_BLOCK; 
      i <= RTC_MEMORY_MEASUREMENTS_START_BLOCK + amount*RTC_MEMORY_MEASUREMENT_BLOCK_SIZE;
      i+= RTC_MEMORY_MEASUREMENT_BLOCK_SIZE){
        readData(i, &m, RTC_MEMORY_MEASUREMENT_BLOCK_SIZE*4);

        arrangeData(m, buffer);   //Copies measurements arranged in the correct format to "buffer".
        
        arraycpy(data+(sizeof(buffer) * (RTC_MEMORY_MEASUREMENTS_START_BLOCK-i)), buffer, sizeof(buffer));
  }

  /////////////// DEBUG ////////////////////////
  Serial.printf("\n --- Read and arranged data:");
  for (uint16 i = 0; i < sizeof(buffer); i++)
  {
    Serial.printf("%X", ((uint8*)data)[i]);
  }
  /////////////////////////////////////////////
}

void RtcMemory::readData(int address, void *data, unsigned short int bytes) {
  system_rtc_mem_read(address, data, bytes);
  yield();
  /////////////////// DEBUG ///////////////////////////
  // measurement m;
  // system_rtc_mem_read(address, &m, bytes);
  uint8_t buffer[288];
  system_rtc_mem_read(address, &buffer, bytes);
  Serial.printf("\n\nRead measurements:   (block: %d)   data: \n",address);
   for (unsigned short x=0; x < bytes; x++){
      Serial.printf("%X", ((uint8_t*)(buffer))[x] );
    }
  // Serial.printf("timestamp: %d\n",m.timestamp);
  // Serial.printf("id_sensor: [%d,%d,%d,%d] \n",m.id_sensor[0],m.id_sensor[1],m.id_sensor[2],m.id_sensor[3]);
  // Serial.printf("temperature: [%d,%d,%d,%d] \n",m.temperature[0],m.temperature[1],m.temperature[2],m.temperature[3]);
  // Serial.printf("humidity: [%d,%d,%d,%d] \n",m.humidity[0],m.humidity[1],m.humidity[2],m.humidity[3]);
  //////////////////////////////////////////////////////

  //Data should be packed in a fixed size of 24B. For some reason there is inconsistancy between
  //the length of different measurements.
}

void RtcMemory::init(void){
  //Initializes rtm memory. Clears all pointers.
  clearMeasurements();
}

/*Create method to unallign struct type data.
  
*/

bool RtcMemory::arrangeData(measurement m, uint8_t* data){
  /*This method takes a packet of data read from rtc memory, extract all the members values
  inside of it (using the same struct type) and arrange them into a normalized 24B array.*/

/*  typedef struct{
            uint32_t timestamp;                              -> 4B 
            uint16_t id_sensor[DATALOGGER_SENSOR_COUNT];     -> 8B
            int16_t  temperature[DATALOGGER_SENSOR_COUNT];   -> 8B
            uint8_t  humidity[DATALOGGER_SENSOR_COUNT];      -> 4B
    }measurement;      //structure alias.                   -> (24B)
*/

  uint8 buffer[RTC_MEMORY_MEASUREMENT_BLOCK_SIZE * 4];  //Buffer 24B long.

  arraycpy(buffer,    &m.timestamp,   sizeof(m.timestamp));     //timestamp.
  arraycpy(buffer+4,  &m.id_sensor,   sizeof(m.id_sensor));     //id_sensor.
  arraycpy(buffer+12, &m.temperature, sizeof(m.temperature));   //temperature.
  arraycpy(buffer+20, &m.humidity,    sizeof(m.humidity));      //humidity.

  /////////////// DEBUG ////////////////////////
  Serial.printf("\n Arranged data:");
  for (uint16 i = 0; i < sizeof(buffer); i++)
  {
    Serial.printf("%X", buffer[i]);
  }
  /////////////////////////////////////////////



}

void arraycpy(uint8_t* result, uint8_t* origin, uint16_t len){
  for(uint16 i=0; i<len; i++){
    *(result + i) = *(origin + i);
  }
}