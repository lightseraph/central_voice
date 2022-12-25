#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <iostream>
#include <list>
#include <map>
#include "recorder.h"
// using namespace std;
#define NTP1 "cn.ntp.org.cn"
#define NTP2 "ntp3.aliyun.com"

const char *ssid = "LighthouseAP";                                                                                 // WIFI账户
const char *password = "letmethink";                                                                               // WIFI密码
const String WDAY_NAMES[] = {"星期天", "星期一", "星期二", "星期三", "星期四", "星期五", "星期六"};                // 星期
const String MONTH_NAMES[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"}; // 月份

// 指令集
const uint8_t queryTypeCmd[4] = {0xfe, 0x01, 0x01, 0xff};          // 查询设备类型
const uint8_t enterConfigCmd[] = {0x2A, 0x2D, 0x2E};               // 进入配置模式
const uint8_t enterTransferCmd[] = {0x2F, 0x2C, 0x2B};             // 进入传输模式
const uint8_t broadcastModeCmd[] = {0xFD, 0x01, 0x26, 0x00, 0xFF}; // 广播模式
const uint8_t p2pCmd[] = {0xFD, 0x01, 0x26, 0x04, 0xFF};           // 协议点播模式
const uint8_t registerReply[] = {0x00, 0x00, 0xEE};                // 终端注册指令
const uint8_t restartCmd[] = {0xFD, 0x00, 0x12, 0xFF};             // 模块重启指令

std::list<uint16_t> deviceAddr;
std::map<uint16_t, Record>::iterator addrIt;
std::map<uint16_t, Record> resultReport;

bool checkNTP = true;
bool registered = false;
typedef enum
{
  COORDINATOR = 1,
  ROUTER,
  TERMINATOR
} DeviceType;

typedef enum
{
  BROADCAST,
  TEST,
  STOPPED
} WorkStatus;

DeviceType type;
WorkStatus workStatus;
// put your main code here, to run repeatedly:
#define ONBOARD_LED 2
#define CONN_STATUS 4 // 检测模块联网状态
// 2<->0: 0; 2->0: 1
#define TRANSFER_MODE 2

// SemaphoreHandle_t h_SerialMutex;
// const byte E18_query_cmd[] = {0x55, 0x03, 0x00, 0x00, 0x00}; // 查询网络状态命令
// const byte E18_transfer_mode[] = {0x55, 0x07, 0x00, 0x11, 0x00, 0x03, 0x00, 0x01, 0x13};
// const byte E18_command_mode[] = {0x55, 0x07, 0x00, 0x11, 0x00, 0x03, 0x00, 0x00, 0x12};
u_int ReadSerial(HardwareSerial serial_instance, byte *buff); // 串口读取函数
bool XOR8_Checksum(byte *buff, int buff_len);                 // 检查数据校验是否正确
void Serial2_callback(void);
void Serial_callback(void);

template <class T>
int getArrayLen(T &array)
{
  return sizeof(array) / sizeof(array[0]);
}

bool XOR8_Checksum(byte *buff, int buff_len) // XOR8校验算法
{
  if (buff_len == 0)
    return false;

  byte sum = buff[2] ^ buff[3];
  if (buff_len > 5)
  {
    for (int i = 4; i < buff_len - 1; i++)
    {
      sum = sum ^ buff[i];
    }
  }
  if (sum == buff[buff_len - 1] && buff[0] == 0x55 && buff[1] == buff_len - 2) // 判断校验位，帧头是不是0x55, 实际收到数据包长度是否与数据内指示长度相等
    return true;
  else
    return false;
}

u_int ReadSerial(HardwareSerial serial_instance, byte *buff)
{
  u_int data_len = serial_instance.available();
  vTaskDelay(pdMS_TO_TICKS(100));
  if (data_len)
  {
    serial_instance.readBytes(buff, data_len);
    return data_len;
  }
  return 0;
}

void taskConnStatus_E180(void *parameter) // E180
{
  int connection_status;
  while (true)
  {
    connection_status = digitalRead(CONN_STATUS);
    if (connection_status == 0)
      digitalWrite(ONBOARD_LED, HIGH);
    else
      digitalWrite(ONBOARD_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  vTaskDelete(NULL);
}

void taskConnStatus_E18(void *parameter)
{
  byte *buff = new byte[80];
  int data_len;
  while (true)
  {
    // Serial2.write(E18_query_cmd, 5);
    vTaskDelay((pdMS_TO_TICKS(300)));
    data_len = ReadSerial(Serial2, buff);
    if (XOR8_Checksum(buff, data_len))
    {
      // Serial.println(XOR8_Checksum(buff,data_len));
      // Serial.write(buff,6);
      if (buff[4] == 0x00)
        digitalWrite(ONBOARD_LED, HIGH);
      else if (buff[4] == 0xff)
        digitalWrite(ONBOARD_LED, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
  Serial.println("Ending task_connection");
  vTaskDelete(NULL);
}

void initClock()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
  configTime(8 * 3600, 0, NTP1, NTP2);
  while (checkNTP) // 检查是否获取网络时间，正确获取后不再检查
  {
    struct tm timeInfo;
    if (getLocalTime(&timeInfo))
    {
      digitalWrite(ONBOARD_LED, HIGH);
      checkNTP = false;
      WiFi.disconnect(true); // 断开wifi网络
      WiFi.mode(WIFI_OFF);
    }
    else
    {
      for (int i = 0; i < 30; i++)
      {
        digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
        delay(100);
      }
    }
  }
  // Serial.println(&timeInfo, "%T");
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);
  // pinMode(CONN_STATUS, INPUT);

  // h_SerialMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(500);
  //  xTaskCreate(taskConnStatus_E18, "Task_Connection", 2048, NULL, 1, NULL);
  Serial2.write(restartCmd, 4);
  initClock();

  Serial.onReceive(Serial_callback);
  Serial2.onReceive(Serial2_callback);

  Serial2.write(enterConfigCmd, 3);
  delay(1000);
  Serial2.write(queryTypeCmd, 4);
  delay(1000);
  // Serial.print(type);
  if (type != COORDINATOR) // 如果不是协调器就进入传输模式
  {
    Serial2.write(p2pCmd, 5);
    workStatus = TEST;
    registered = false;
  }
  else // 是协调器就设置为广播模式,然后进入传输模式
  {
    Serial2.write(broadcastModeCmd, 5);
    workStatus = BROADCAST;
  }
  delay(1000);
  Serial2.write(enterTransferCmd, 3);
  delay(2000); // 此延时是防止进入loop后立即发送指令
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (type == COORDINATOR)
  {
    switch (workStatus)
    {
    case BROADCAST:
      Serial2.print("rft");
      delay(5000);
      break;

    case TEST: // 协调器进入测试模式，发送当前时间，前面加上终端地址
      struct tm timeInfo;
      char str[32] = {0};
      char strTime[32] = {0};

      for (addrIt = resultReport.begin(); addrIt != resultReport.end(); addrIt++)
      {
        if (getLocalTime(&timeInfo))
        {
          strftime(strTime, 32, "%Y-%m-%d %H:%M:%S", &timeInfo);
          str[0] = (addrIt->first >> 8) & 0xff;
          str[1] = addrIt->first & 0xff;
          for (int i = 2; i < 30; i++)
            str[i] = strTime[i - 2];

          addrIt->second.prevSentTime = timeInfo;
          Serial2.write(str);
          delay(10000);
        }
      }
      break;
    }
  }
}

void Serial_callback(void)
{
  // byte buff[64];
  char strBuff[16] = {0};
  int i;
  int s0_data_len = 0;
  s0_data_len = Serial.available();
  if (s0_data_len != 0)
  {
    // Serial.readBytes(buff, s0_data_len); // 读上位机数据
    Serial.read(strBuff, s0_data_len);
    delay(500);
    // Serial.println(strBuff);

    if (strcmp(strBuff, "qt") == 0) // 查询连接的终端列表
    {
      Serial.printf("total registered device: %d\r\n", resultReport.size());
      for (i = 1, addrIt = resultReport.begin(); addrIt != resultReport.end(); addrIt++, i++)
      {
        Serial.printf("Terminator %d address: %x\r\n", i, addrIt->first);
      }
      return;
    }

    if (strcmp(strBuff, "start") == 0) // 开始通信测试
    {
      Serial2.write(enterConfigCmd, 3);
      delay(1000);
      Serial2.write(p2pCmd, 5);
      delay(1000);
      Serial2.write(enterTransferCmd, 3);
      delay(1000);
      workStatus = TEST;
      return;
    }

    if (strcmp(strBuff, "qr") == 0) // 查询通信测试结果
    {
      return;
    }

    if (strcmp(strBuff, "stop") == 0) // 停止通信测试
    {
      workStatus = STOPPED;
      return;
    }
  }
}

void Serial2_callback(void)
{
  byte buff[32];
  int s2_data_len = 0;
  s2_data_len = Serial2.available();
  if (s2_data_len != 0)
  {
    Serial2.readBytes(buff, s2_data_len); // 读Zigbee数据
    delay(200);
    // Serial.write(buff, s2_data_len);
    if (buff[0] == 0xFB && buff[1] == 0x01)
    {
      type = (DeviceType)buff[2];
      // Serial.println(type);
    }
    // 非协调器设备收到"rft"，准备指令, 如果已经发送过就不回复
    if (type != COORDINATOR)
    {
      // 非协调器收到等待注册广播
      if (!registered && buff[0] == 0x72 && buff[1] == 0x66 && buff[2] == 0x74)
      {
        Serial.println("reply");
        delay(rand() % 6);               // 随机延迟0～5秒发送，防止多个终端同时发送产生拥堵
        Serial2.write(registerReply, 3); // 回复注册指令
        registered = true;
        // workStatus = TEST;
        return;
      }
      // 非协调器进入测试状态
      if (workStatus == TEST && registered && buff[0] == 0x32)
      {
        tm recvTimeInfo, localTimeInfo;
        int difTime = 0;
        char recvTime[32] = {0};

        sprintf(recvTime, "%s", buff);
        strptime(recvTime, "%Y-%m-%d %H:%M:%S", &recvTimeInfo);
        if (getLocalTime(&localTimeInfo))
          difTime = (int)difftime(mktime(&localTimeInfo), mktime(&recvTimeInfo));
        // Serial.println(&recvTimeInfo, "%Y-%m-%d %H:%M:%S");
        uint8_t replyBuff[4];
        replyBuff[0] = 0x00;
        replyBuff[1] = 0x00;
        replyBuff[2] = 0xE0;
        replyBuff[3] = (uint8_t)difTime;
        Serial2.write(replyBuff, 4);
        return;
      }
    }
    /*****************************************************************************************/
    // 协调器收到0xEE，解析出短地址加入列表
    if (type == COORDINATOR)
    {
      Record report;
      if (buff[0] == 0xEE && workStatus == BROADCAST) // 串口输出一次终端注册信息
      {
        uint16_t add = (uint16_t)buff[1] << 8 | buff[2];
        /*  for (addrIt = deviceAddr.begin(); addrIt != deviceAddr.end(); addrIt++)
         {
           if (add == *addrIt)
             return;
         } */
        auto it = resultReport.find(add);
        if (it == resultReport.end())
        {
          resultReport.insert(std::pair<uint16_t, Record>(add, report));
          Serial.printf("Terminator %x register in.\r\n", add);
        }

        // deviceAddr.push_back(add);
        return;
      }
      // 协调器收到0xE0，记录一次返回结果。
      if (buff[0] == 0xE0 && workStatus == TEST)
      {
        tm localTimeInfo;
        uint16_t addr = (uint16_t)buff[2] << 8 | buff[3];
        Serial.printf("Terminator %x replied, delay time is %d\r\n", addr, buff[1]);
        if (getLocalTime(&localTimeInfo))
        {
          int difTime = (int)difftime(mktime(&localTimeInfo), mktime(&(resultReport.find(addr)->second.prevSentTime)));
          Serial.printf("Echo time is:%d\r\n", difTime);
          if (difTime < 5)
            resultReport.find(addr)->second.normalTransCount++;
          if (difTime >= 5 && difTime < 10)
            resultReport.find(addr)->second.overtimeTransCount++;
        }
        return;
      }
    }
  }
}