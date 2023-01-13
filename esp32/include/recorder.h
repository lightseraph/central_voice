#include <EEPROM.h>
#include "zmodule.h"
struct Record
{
    // uint8_t addr[2];
    uint16_t sendCount;
    uint16_t normalTransCount;
    uint16_t slowTransCount;
    uint16_t overtimeTransCount;
    tm totalTime_inSec;
    tm prevSentTime;
    uint16_t over1000Count;
    DeviceType type;
};

void initEEPROM();
void resetEEPROM();
void writeRecord(Record record);
void readRecord(Record &record);