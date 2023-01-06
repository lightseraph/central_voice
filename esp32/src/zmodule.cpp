#include "zmodule.h"

void ZModule::initModule(void)
{
    byte rxBuff[64] = {0};
    int rxBuffLen = 0;

    Serial2.write(restartCmd, 4);
    delay(1000);
    Serial2.write(enterConfigCmd, 3);
    delay(1000);
    while (Serial2.available()) // 清空返回数据
    {
        ;
    }
    moduleMode = CONFIG;

    Serial2.write(queryAllParam, 4);
    delay(1000);
    if (Serial2.available())
    {
        rxBuffLen = Serial2.available();
        Serial2.read(rxBuff, rxBuffLen);
    }

    if (rxBuff[0] == 0xFB && rxBuff[1] == 0xFE)
    {
        moduleType = (DeviceType)rxBuff[2];
        networkStatus = (NetStatus)rxBuff[3];
        shortAddr = (uint16_t)rxBuff[6] << 8 | rxBuff[7];
        parentAddr = (uint16_t)rxBuff[16] << 8 | rxBuff[17];
        groupNum = rxBuff[26];
        transferMode = (TransferMode)rxBuff[42];
        recieveMode = (RecieveMode)rxBuff[43];
    }
}

DeviceType ZModule::getModuleType(void)
{
    return moduleType;
}

void ZModule::setModuleType(DeviceType type)
{
    byte cmd[5] = {0xFD, 0x01, 0x01, 0x00, 0xFF};
    cmd[3] = (byte)type;
    if (moduleMode == TRANSFER)
    {
        Serial2.write(enterConfigCmd, 3);
        moduleMode = CONFIG;
        delay(1000);
    }
    Serial2.write(cmd, 5);
    delay(1000);
    Serial2.write(restartCmd, 4);
    delay(1000);
    moduleType = type;
}

ModuleMode ZModule::getModuleMode(void)
{
    return moduleMode;
}

void ZModule::setModuleMode(ModuleMode mode)
{
    if (mode != moduleMode)
    {
        if (mode == CONFIG)
        {
            Serial2.write(enterConfigCmd, 3);
            delay(1000);
        }
        else if (mode == TRANSFER)
        {
            Serial2.write(enterTransferCmd, 3);
            delay(1000);
        }
    }
    moduleMode = mode;
    while (Serial2.available()) // 清空返回数据
    {
        ;
    }
}

TransferMode ZModule::getTransferMode(void)
{
    return transferMode;
}

void ZModule::setTransferMode(TransferMode mode)
{
    byte cmd[5] = {0xFD, 0x01, 0x26, 0x00, 0xFF};
    cmd[3] = (byte)mode;
    if (mode != transferMode)
    {
        if (moduleMode != CONFIG)
        {
            Serial2.write(enterConfigCmd, 3);
            delay(1000);
            Serial2.write(cmd, 5);
            delay(1000);
            Serial2.write(enterTransferCmd, 3);
            delay(1000);
        }
    }
    transferMode = mode;
    while (Serial2.available()) // 清空返回数据
    {
        ;
    }
}

RecieveMode ZModule::getRecieveMode(void)
{
    return recieveMode;
}

void ZModule::setRecieveMode(RecieveMode mode)
{
    byte cmd[5] = {0xFD, 0x01, 0x27, 0x00, 0xFF};
    cmd[3] = (byte)mode;
    if (mode != recieveMode)
    {
        if (moduleMode != CONFIG)
        {
            Serial2.write(enterConfigCmd, 3);
            delay(1000);
            Serial2.write(cmd, 5);
            delay(1000);
            Serial2.write(enterTransferCmd, 3);
            delay(1000);
        }
    }
    recieveMode = mode;
    while (Serial2.available()) // 清空返回数据
    {
        ;
    }
}

uint16_t ZModule::getShortAddr(void)
{
    return shortAddr;
}

uint16_t ZModule::getParentAddr(void)
{
    return parentAddr;
}

NetStatus ZModule::getNetStatus(void)
{
    return networkStatus;
}

void ZModule::restartModule(void)
{
    if (moduleMode != CONFIG)
    {
        Serial2.write(enterConfigCmd, 3);
        delay(1000);
    }
    Serial2.write(restartCmd, 4);
    delay(2000);
    if (moduleMode != CONFIG)
    {
        Serial2.write(enterTransferCmd, 3);
        delay(1000);
    }
}
