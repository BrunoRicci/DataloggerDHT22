#include <Arduino.h>

#include <datalogger_config.h> 




class RtcMemory{
    public:
        RtcMemory(void);    //Constructor
        void clearMeasurements(void);
        bool saveMeasurements(void *data, unsigned short int bytes);
        void readData(int address, void *data, unsigned short int bytes);
        void init(void);

    private:
        
};