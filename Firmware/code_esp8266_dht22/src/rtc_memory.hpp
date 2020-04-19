#include <Arduino.h>
#include <datalogger_config.h> 




class RtcMemory{
    public:
        RtcMemory(void);    //Constructor
        void clearMeasurements(void);
        bool saveMeasurements(void *data, unsigned short int bytes);
        bool readMeasurements(uint8_t *data, unsigned short int amount);
        void readData(int address, void *data, unsigned short int bytes);
        void init(void);

    private:
        void arraycpy(unsigned char* result, unsigned char* origin, unsigned short len);
        bool arrangeData(measurement m, uint8_t* data);


        typedef struct{   
        uint16_t rtcmem_pointer;                //rtcmemory pointer
        uint16_t archive_saved_pointer;      //measurement file last saved packet   
        uint16_t archive_sent_pointer;       //measurement file last sent packet     
        }variables;      //structure alias.
};