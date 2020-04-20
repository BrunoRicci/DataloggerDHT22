#include <Arduino.h>
#include <datalogger_config.h> 


typedef struct{   
        uint16_t rtcmem_pointer;             //rtcmemory pointer
        uint16_t archive_saved_pointer;      //measurement file last saved packet   
        uint16_t archive_sent_pointer;       //measurement file last sent packet   

        //TIME UNITS: seconds (s).
        uint32_t current_time;              //Estimated current time. Saved right before deepSleep.
        uint32_t last_sync_time;            //Last obtained time.
}Variables;      //structure alias.


class RtcMemory{
    public:
        RtcMemory(void);    //Constructor
        void clearMeasurements(void);
        bool saveMeasurements(void *data, unsigned short int bytes);
        bool readMeasurements(uint8_t *data, unsigned short int amount);
        void writeData(int address, void* data, uint16_t bytes);
        void readData(int address, void *data, unsigned short int bytes);
        void clear(void);
        bool safeDisconnect();
        bool recoverVariables(void);
        void setElapsedTime(uint32_t time);
        uint32_t getCurrentTime(void);
        uint8_t getPointer(void);

    private:
        Variables var;
        bool arrangeData(Measurement m, uint8_t* data);

};