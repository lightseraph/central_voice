#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>

#define NTP1 "cn.ntp.org.cn"
#define NTP2 "pool.ntp.org"
#define NTP3 "ntp3.aliyun.com"

const char *ssid = "PDCN";                                                                                         // WIFI账户
const char *password = "letmethink";                                                                               // WIFI密码
const String WDAY_NAMES[] = {"星期天", "星期一", "星期二", "星期三", "星期四", "星期五", "星期六"};                // 星期
const String MONTH_NAMES[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"}; // 月份

// put your main code here, to run repeatedly:
#define ONBOARD_LED 2
#define CONN_STATUS 4 // 检测模块联网状态
// 2<->0: 0; 2->0: 1
#define TRANSFER_MODE 2

// SemaphoreHandle_t h_SerialMutex;
const byte E18_query_cmd[] = {0x55, 0x03, 0x00, 0x00, 0x00}; // 查询网络状态命令
const byte E18_transfer_mode[] = {0x55, 0x07, 0x00, 0x11, 0x00, 0x03, 0x00, 0x01, 0x13};
const byte E18_command_mode[] = {0x55, 0x07, 0x00, 0x11, 0x00, 0x03, 0x00, 0x00, 0x12};
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
    Serial2.write(E18_query_cmd, 5);
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

void getClock()
{
  struct tm timeInfo;
  if (!getLocalTime(&timeInfo))
  { // 一定要加这个条件判断，否则内存溢出
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeInfo, "%T");
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(ONBOARD_LED, OUTPUT);
  // pinMode(CONN_STATUS, INPUT);

  // h_SerialMutex = xSemaphoreCreateMutex();
  EEPROM.begin(8);
  EEPROM.write(0, 0);
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(500);
  // Serial.onReceive(Serial_callback);
  // Serial2.onReceive(Serial2_callback);

  //  xTaskCreate(taskConnStatus_E18, "Task_Connection", 2048, NULL, 1, NULL);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
  configTime(8 * 3600, 0, NTP1, NTP2, NTP3);
  getClock();
  delay(5000);

  WiFi.disconnect(true); // 断开wifi网络
  WiFi.mode(WIFI_OFF);   // 关闭网络
}

void loop()
{
  // put your main code here, to run repeatedly:
  getClock();
  delay(5000);
}

void Serial_callback(void)
{
  byte buff[64];
  int s0_data_len = 0;
  s0_data_len = Serial.available();
  if (s0_data_len != 0)
  {
    Serial.readBytes(buff, s0_data_len); // 读上位机数据
    delay(200);
    Serial2.write(buff, s0_data_len);
  }
}

void Serial2_callback(void)
{
  byte buff[64];
  int s2_data_len = 0;
  s2_data_len = Serial2.available();
  if (s2_data_len != 0)
  {
    Serial2.readBytes(buff, s2_data_len); // 读Zigbee数据
    delay(200);
    Serial.write(buff, s2_data_len);
    // Serial.println();
  }
}