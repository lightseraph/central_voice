#include <EEPROM.h>

struct Record
{
    // uint8_t addr[2];
    int sendCount;
    int normalTransCount;
    int slowTransCount;
    int overtimeTransCount;
    tm totalTime_inSec;
    tm prevSentTime;
};

void initEEPROM();
void resetEEPROM();
void writeRecord(Record record);
void readRecord(Record &record);