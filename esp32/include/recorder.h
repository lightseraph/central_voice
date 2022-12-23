#include <EEPROM.h>

struct Record
{
    uint8_t addr[2];
    int sendCount;
    int normalTransCount;
    int slowTransCount;
    int overtimeTransCount;
    int totalTime_inSec;
};

void initEEPROM();
void resetEEPROM();
void writeRecord(Record record);
void readRecord(Record &record);