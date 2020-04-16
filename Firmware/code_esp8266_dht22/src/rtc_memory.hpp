#include <Arduino.h>
// #include <datalogger_config.h> 



class RtcMemory{
    public:
        RtcMemory(void);    //Constructor
        void clearMeasurements(void);
        bool saveMeasurements(void *data, unsigned short int bytes);
        bool readMeasurements(void *data, unsigned short int amount);
        void readData(int address, void *data, unsigned short int bytes);
        void init(void);

    private:
        void arraycpy(void* result, void* origin, uint16_t len);
        bool arrangeData(measurement m, uint8_t* data);
};