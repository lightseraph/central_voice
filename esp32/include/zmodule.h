#include "Arduino.h"

typedef enum
{
    COORDINATOR = 1,
    ROUTER,
    TERMINATOR,
    UNDEFINED
} DeviceType;

typedef enum
{
    BROADCAST,
    TEST,
    STOPPED,
    CHECKNETWORK
} WorkStatus;

typedef enum
{
    CONFIG,
    TRANSFER
} ModuleMode;

typedef enum
{
    NO_NETWORK,
    JOINING,
    IN_NETWORK,
    IN_NETWORK_LOST_PARENT,
    LEAVING
} NetStatus;

typedef enum
{
    BROADCAST_TRANS,
    GROUPCAST_TRANS,
    TRANS_SHORTADDR,
    TRANS_LONGADDR,
    PROTOCOL_SHORTADDR,
    PROTOCOL_GROUP
} TransferMode;

typedef enum
{
    SIMPLE_TRANSFER,
    WITH_SHORTADDR,
    WITH_LONGADDR,
    WITH_RSSI,
    WITH_SHORTADDR_RSSI,
    WITH_LONGADDR_RSSI
} RecieveMode;

class ZModule
{
private:
    DeviceType moduleType;
    WorkStatus moduleWorkStatus;
    ModuleMode moduleMode;
    uint16_t shortAddr;
    uint16_t parentAddr;
    uint8_t groupNum;
    NetStatus networkStatus;
    TransferMode transferMode;
    RecieveMode recieveMode;

    const uint8_t queryTypeCmd[4] = {0xfe, 0x01, 0x01, 0xff};           // 查询设备类型
    const uint8_t enterConfigCmd[3] = {0x2A, 0x2D, 0x2E};               // 进入配置模式
    const uint8_t enterTransferCmd[3] = {0x2F, 0x2C, 0x2B};             // 进入传输模式
    const uint8_t broadcastModeCmd[5] = {0xFD, 0x01, 0x26, 0x00, 0xFF}; // 广播模式
    const uint8_t p2pCmd[5] = {0xFD, 0x01, 0x26, 0x04, 0xFF};           // 协议点播模式
    const uint8_t registerReply[3] = {0x00, 0x00, 0xEE};                // 终端注册指令
    const uint8_t restartCmd[4] = {0xFD, 0x00, 0x12, 0xFF};             // 模块重启指令
    const uint8_t queryParent[4] = {0xFE, 0x02, 0x07, 0xFF};            // 查询父节点短地址
    const uint8_t queryAllParam[4] = {0xFE, 0x2F, 0xFE, 0xFF};          // 查询模块所有参数

public:
    void initModule(void);
    DeviceType getModuleType(void);
    void setModuleType(DeviceType type);
    ModuleMode getModuleMode(void);
    void setModuleMode(ModuleMode mode);
    TransferMode getTransferMode(void);
    void setTransferMode(TransferMode mode);
    RecieveMode getRecieveMode(void);
    void setRecieveMode(RecieveMode mode);
    uint16_t getShortAddr(void);
    uint16_t getParentAddr(void);
    NetStatus getNetStatus(void);
    void restartModule(void);
};