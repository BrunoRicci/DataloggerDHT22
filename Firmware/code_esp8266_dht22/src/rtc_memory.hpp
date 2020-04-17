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
};