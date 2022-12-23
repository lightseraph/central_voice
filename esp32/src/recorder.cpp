#include "recorder.h"

void initEEPROM()
{
    EEPROM.begin(256);
}