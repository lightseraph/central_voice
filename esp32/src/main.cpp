#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <iostream>
#include <map>
#include "recorder.h"
// #include "zmodule.h"
//  using namespace std;
#define NTP1 "cn.ntp.org.cn"
#define NTP2 "ntp3.aliyun.com"

const char *ssid = "PDCN";           // WIFI账户
const char *password = "letmethink"; // WIFI密码

// 指令集
const uint8_t queryTypeCmd[4] = {0xfe, 0x01, 0x01, 0xff};          // 查询设备类型
const uint8_t enterConfigCmd[] = {0x2A, 0x2D, 0x2E};               // 进入配置模式
const uint8_t enterTransferCmd[] = {0x2F, 0x2C, 0x2B};             // 进入传输模式
const uint8_t broadcastModeCmd[] = {0xFD, 0x01, 0x26, 0x00, 0xFF}; // 广播模式
const uint8_t p2pCmd[] = {0xFD, 0x01, 0x26, 0x04, 0xFF};           // 协议点播模式
const uint8_t restartCmd[] = {0xFD, 0x00, 0x12, 0xFF};             // 模块重启指令
const uint8_t queryParent[] = {0xFE, 0x02, 0x07, 0xFF};            // 查询父节点短地址
const uint8_t dataOutputMode[] = {0xFD, 0x01, 0x27, 0x04, 0xFF};   // 数据输出模式设置为数据+短地址+RSSI
uint8_t registerReply[4] = {0x00, 0x00, 0xEE, 0x00};               // 终端注册指令

std::map<uint16_t, Record>::iterator addrIt;
std::map<uint16_t, Record> resultReport; // 终端短地址是键，测试结果是值

bool checkNTP = true;
bool registered = false;

hw_timer_t *tim1 = NULL;
uint16_t timer = 0;
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
bool XOR8_Checksum(byte *buff, int buff_len); // 检查数据校验是否正确
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

void IRAM_ATTR onTimer() // 定时器中断函数，每毫秒触发
{
  timer++;
}

/* void taskConnStatus_E180(void *parameter) // E180
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
} */

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
  Serial2.write(restartCmd, 4); // 重启模块
  initClock();

  Serial.onReceive(Serial_callback);
  Serial2.onReceive(Serial2_callback);

  tim1 = timerBegin(0, 80, true);
  timerAttachInterrupt(tim1, onTimer, true);
  timerAlarmWrite(tim1, 1000, true); // 每毫秒产生一个中断计数

  Serial2.write(enterConfigCmd, 3);
  delay(1000);
  while (Serial2.available()) // 清空返回数据
  {
    ;
  }
  Serial2.write(queryTypeCmd, 4);
  delay(1000);
  // Serial2.write(dataOutputMode, 5);
  // delay(1000);
  // Serial.println(type);

  // Serial.write(registerReply, 4);

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
  delay(2000);                // 此延时是防止进入loop后立即发送指令
  while (Serial2.available()) // 清空返回数据
  {
    ;
  }
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

    case TEST: // 协调器进入测试模式，发送当前时间 + 发送数据包计数，前面加上终端地址
      tm timeInfo;
      char txd_buff[24] = {0};
      char strTime[21] = {0};

      for (addrIt = resultReport.begin(); addrIt != resultReport.end(); addrIt++)
      {
        if (getLocalTime(&timeInfo))
        {
          strftime(strTime, 20, "%Y-%m-%d %H:%M:%S", &timeInfo);
          txd_buff[0] = (addrIt->first >> 8) & 0xff;
          txd_buff[1] = addrIt->first & 0xff;
          for (int i = 2; i < 21; i++)
            txd_buff[i] = strTime[i - 2];

          addrIt->second.prevSentTime = timeInfo;
          addrIt->second.sendCount++;
          txd_buff[21] = '\0';
          txd_buff[22] = addrIt->second.sendCount >> 8 & 0xff;
          txd_buff[23] = addrIt->second.sendCount & 0xff;
          // Serial.println();
          // Serial.write(txd_buff, 24);
          Serial2.write(txd_buff, 24);
          timer = 0;
          delay(6000); // 向终端发送数据包的间隔
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
    delay(100);
    // Serial.println(strBuff);

    if (strcmp(strBuff, "qt") == 0) // 查询连接的终端列表
    {
      Serial.printf("\r\nTotal registered device: %d\r\n", resultReport.size());
      for (i = 1, addrIt = resultReport.begin(); addrIt != resultReport.end(); addrIt++, i++)
      {
        Serial.printf("Terminal %d address: %04x    Type is: 0x%02x\r\n", i, addrIt->first, (uint8_t)addrIt->second.type);
      }
      return;
    }

    if (strcmp(strBuff, "start") == 0) // 开始通信测试
    {
      for (addrIt = resultReport.begin(); addrIt != resultReport.end(); addrIt++)
      {
        addrIt->second.normalTransCount = 0;
        addrIt->second.overtimeTransCount = 0;
        addrIt->second.sendCount = 0;
        addrIt->second.slowTransCount = 0;
        addrIt->second.over1000Count = 0;
      }
      Serial2.write(enterConfigCmd, 3);
      delay(1000);
      Serial2.write(p2pCmd, 5);
      delay(1000);
      Serial2.write(enterTransferCmd, 3);
      delay(1000);
      workStatus = TEST;
      timerAlarmEnable(tim1);
      return;
    }

    if (strcmp(strBuff, "qr") == 0) // 查询通信测试结果
    {
      uint8_t i = 2;
      Serial.println("\r\n*****************Transfer test report**********************");
      for (addrIt = resultReport.begin(); addrIt != resultReport.end(); addrIt++, i++)
      {
        Serial.printf("Terminal %04x, total sent %d package\r\n", addrIt->first, addrIt->second.sendCount);
        Serial.printf("Normal transfer: %d\r\n", addrIt->second.normalTransCount);
        Serial.printf("Slow transfer: %d\r\n", addrIt->second.slowTransCount);
        Serial.printf("Over 1s transfer: %d\r\n", addrIt->second.over1000Count);
        Serial.printf("Overtime transfer: %d\r\n", addrIt->second.overtimeTransCount);
        Serial.println("");
      }
      Serial.println("***********************************************************\r\n");
      return;
    }

    if (strcmp(strBuff, "stop") == 0) // 停止通信测试
    {
      timerAlarmDisable(tim1);
      workStatus = STOPPED;
      return;
    }

    if (strcmp(strBuff, "parent") == 0)
    {
      Serial2.write(enterConfigCmd, 3);
      delay(1000);
      Serial2.write(broadcastModeCmd, 5);
      delay(1000);
      Serial2.write(enterTransferCmd, 3);
      delay(1000);
      Serial2.write(0xE1);
      workStatus = STOPPED;
    }

    if (strcmp(strBuff, "search") == 0)
    {
      Serial2.write(enterConfigCmd, 3);
      delay(1000);
      Serial2.write(broadcastModeCmd, 5);
      delay(1000);
      Serial2.write(enterTransferCmd, 3);
      delay(1000);
      resultReport.clear();
      workStatus = BROADCAST;
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
    delay(50);
    // Serial.write(buff, s2_data_len);
    // 查询设备类型
    if (buff[0] == 0xFB && buff[1] == 0x01)
    {
      type = (DeviceType)buff[2];
      registerReply[3] = type;
      // Serial.println(type);
    }
    // 非协调器设备收到"rft"，准备指令, 回复注册信息
    if (type != COORDINATOR)
    {
      // 非协调器收到等待注册广播
      if (buff[0] == 0x72 && buff[1] == 0x66 && buff[2] == 0x74 && registerReply[3] != 0)
      {
        srand(millis());
        delay(((rand() + 20) % 20) * 200); // 随机延迟1～4秒发送，防止多个终端同时发送产生拥堵
        Serial2.write(registerReply, 4);   // 回复注册指令
        // registered = true;
        //  workStatus = TEST;
        return;
      }
      // 非协调器进入测试状态
      if (workStatus == TEST && buff[0] == 0x32)
      {
        tm recvTimeInfo, localTimeInfo;
        int difTime = 0;
        char recvTime[20] = {0};

        sprintf(recvTime, "%s", buff);
        strptime(recvTime, "%Y-%m-%d %H:%M:%S", &recvTimeInfo);
        if (getLocalTime(&localTimeInfo))
          difTime = (int)difftime(mktime(&localTimeInfo), mktime(&recvTimeInfo));
        // Serial.println(&recvTimeInfo, "%Y-%m-%d %H:%M:%S");
        uint8_t replyBuff[8];
        replyBuff[0] = 0x00; // 协调器地址0x0000
        replyBuff[1] = 0x00;
        replyBuff[2] = 0xE0;
        replyBuff[3] = (uint8_t)difTime;
        replyBuff[4] = buff[20]; // count
        replyBuff[5] = buff[21];
        replyBuff[6] = buff[24]; // RSSI
        replyBuff[7] = buff[25];
        Serial2.write(replyBuff, 8);
        return;
      }
      // 收到协调器发送的查询终端父节点指令
      if (buff[0] == 0xE1)
      {
        Serial2.write(enterConfigCmd, 3);
        delay(1000);
        Serial2.write(queryParent, 4);
        delay(1500);
        while (!Serial2.available())
        {
          delay(200);
        }
        Serial2.readBytes(buff, 7); // 读Zigbee数据
        delay(1000);
        Serial2.write(enterTransferCmd, 3);
        delay(1000);
        if (buff[3] == 0xFB && buff[4] == 0x07) // 前3位为上一条进入传输模式的指令，模块返回的信息
        {
          srand(millis());
          uint8_t txdBuff[7];
          txdBuff[0] = 0x00;
          txdBuff[1] = 0x00;
          txdBuff[2] = 0xE2;
          txdBuff[3] = buff[5];
          txdBuff[4] = buff[6];
          delay(((rand() + 20) % 20) * 200);
          Serial2.write(txdBuff, 5);
          delay(500);
        }
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
        uint16_t add = (uint16_t)buff[2] << 8 | buff[3];

        auto it = resultReport.find(add);
        if (it == resultReport.end())
        {
          report.type = (DeviceType)buff[1];
          resultReport.insert(std::pair<uint16_t, Record>(add, report));
          Serial.printf("Terminal %04x register in.   Type is: 0x%02x\r\n", add, buff[1]);
          Serial.printf("RSSI: %d\r\n", buff[5]);
        }
        // else
        //   Serial.printf("Terminal %04x rejoin in.\r\n", add);

        return;
      }
      // 协调器收到0xE0，记录一次返回结果。
      if (buff[0] == 0xE0 && workStatus == TEST)
      {
        tm localTimeInfo;
        uint16_t count = (uint16_t)buff[2] << 8 | buff[3];
        uint16_t addr = (uint16_t)buff[6] << 8 | buff[7];
        // 收到的回复信息不是当前发送的终端回复的，当成是上一次超时回复的。
        // 或者收到的数据包计数不是等待的，也是超时或者重发
        if (addr != addrIt->first || count != addrIt->second.sendCount)
        {
          Serial.printf("\r\nTerminal %04x replied package is overtime!\r\n", addr);
          addrIt->second.overtimeTransCount++;
          return;
        }

        Serial.printf("\r\nTerminal %04x replied, recieve delay time is %d\r\n", addr, buff[1]);

        Serial.printf("Echo delay time is:%d ms\r\n", timer);
        Serial.printf("Remote RSSI: %02d,     Local RSSI: %02d\r\n", (int8_t)buff[5], (int8_t)buff[9]);
        if (timer < 500)
          resultReport.find(addr)->second.normalTransCount++;
        if (timer >= 500 && timer < 1000)
          resultReport.find(addr)->second.slowTransCount++;
        if (timer >= 1000)
          resultReport.find(addr)->second.over1000Count++;

        timer = 0;
        return;
      }
      // 协调器接收到0xE2,输出终端的父节点查询结果
      if (buff[0] == 0xE2)
      {
        uint16_t addr, parentAddr;
        parentAddr = (uint16_t)buff[1] << 8 | buff[2];
        addr = (uint16_t)buff[3] << 8 | buff[4];
        Serial.printf("\r\nTerminal %04x parent is %04x   RSSI:%02d\r\n", addr, parentAddr, (int8_t)buff[6]);
      }
    }
  }
}