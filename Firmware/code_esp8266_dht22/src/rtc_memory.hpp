#include <Arduino.h>
#include <datalogger_config.h> 


typedef struct{  
        uint16_t statem_state; 
        uint16_t measurements_pointer;             //rtcmemory pointer
        uint16_t archive_saved_pointer;      //measurement file last saved packet   
        uint16_t archive_sent_pointer;       //measurement file last sent packet   

        //TIME UNITS: seconds (s).
        uint32_t current_time;              //Estimated current time. Saved right before deepSleep.
        uint32_t last_sync_time;            //Last obtained time.

        uint8_t powerdown_check;        //Variable to compare with predetermined value (RTC_MEMORY_POWER_CHECK_VARIABLE)
}Variables;      //structure alias.


class RtcMemory{
    public:
        RtcMemory(void);    //Constructor
        uint8_t getPointer(void);
        void clearMeasurements(void);
        bool saveMeasurements(void *data, unsigned short int bytes);
        bool readMeasurements(uint8_t *data, unsigned short int amount);
        void writeData(int address, void* data, uint16_t bytes);
        void readData(int address, void *data, unsigned short int bytes);
        void setElapsedTime(uint32_t time);
        uint32_t getCurrentTime(void);
        bool checkPowerdown(void);
        bool initialize(void);
        bool safeDisconnect(void);
        bool recoverVariables(void);
        void clear(void);

        Variables var;
        Variables rwVariables(void);

    private:
        bool arrangeData(Measurement m, uint8_t* data);

};