#include "can_bus.h"
#include <SPI.h>
#include "mcp2518fd_can.h"

// ==================== 引脚与波特率 ====================
#define SPI_SCK 17
#define SPI_MOSI 23
#define SPI_MISO 5
#define SPI_CS_PIN 21
#define CAN_BAUD_RATE CAN_500K_2M  // 与例程保持一致的波特率

// 调试日志：1 = 每次发送在串口打印结果；0 = 关闭
#define CAN_DEBUG 1

// ==================== CAN 消息 ID ====================
const uint32_t Light_CAN_ID = 0x3FC;
const uint32_t Wind_CAN_ID  = 0x1DE;
const uint32_t Temp_CAN_ID  = 0x1DE;
const uint32_t Volume_CAN_ID = 0x4A8;
const uint32_t Seat_CAN_ID  = 0x200;
const uint8_t MAX_DATA_LEN = 8;

mcp2518fd CAN(SPI_CS_PIN);
static bool canReady = false;

// 通用发送函数
bool sendCANFrame(uint32_t id, uint8_t* data, uint8_t len) {
  if (!canReady) return false;
  int result = CAN.sendMsgBuf(id, 0, len, data);
  if (CAN_DEBUG) {
    if (result == CAN_OK) {
      Serial.print("发送成功: ID=0x");
      Serial.print(id, HEX);
      Serial.print(", 长度=");
      Serial.println(len);
    } else {
      Serial.print("发送失败: 错误码=");
      Serial.println(result);
    }
  }
  return result == CAN_OK;
}

enum Volume {
  Volume_INVALID = 0x00,
  Volume_1 = 0x01,
  Volume_4 = 0x04
};

// ==================== 音量 ====================
void sendVolumeMessage_add() {
  uint8_t data[MAX_DATA_LEN] = {0};
  data[0] = 1;
  data[1] = Volume_1;
  sendCANFrame(Volume_CAN_ID, data, MAX_DATA_LEN);
}

void sendVolumeMessage_sub() {
  uint8_t data[MAX_DATA_LEN] = {0};
  data[0] = 1;
  data[1] = Volume_4;
  sendCANFrame(Volume_CAN_ID, data, MAX_DATA_LEN);
}

// ==================== 氛围灯亮度 ====================
static uint8_t currentLightMessage[MAX_DATA_LEN] = {0};
static uint8_t Lightdata[MAX_DATA_LEN] = {0};
static uint8_t Colordata[MAX_DATA_LEN] = {0};

void sendLightMessage_add() {
  if (currentLightMessage[1] < 0x06) currentLightMessage[1] += 0x01;

  uint8_t data[MAX_DATA_LEN] = {0};
  data[1] = currentLightMessage[1] + 0x30; // 2.4-2.7  0x30 表示整车
  Lightdata[1] = data[1];
  data[0] = Colordata[0];
  sendCANFrame(Light_CAN_ID, data, MAX_DATA_LEN);
}

void sendLightMessage_sub() {
  if (currentLightMessage[1] > 0x01) currentLightMessage[1] -= 0x01;

  uint8_t data[MAX_DATA_LEN] = {0};
  data[1] = currentLightMessage[1] + 0x30;
  Lightdata[1] = data[1];
  data[0] = Colordata[0];
  sendCANFrame(Light_CAN_ID, data, MAX_DATA_LEN);
}

// ==================== 氛围灯颜色 ====================
static uint8_t ColorMessage[MAX_DATA_LEN] = {0};

void sendColorMessage_add() {
  if (ColorMessage[0] < 0x7F) ColorMessage[0] += 0x01;

  uint8_t data[MAX_DATA_LEN] = {0};
  data[1] = Lightdata[1];
  data[0] = ColorMessage[0];
  Colordata[0] = data[0];
  sendCANFrame(Light_CAN_ID, data, MAX_DATA_LEN);
}

void sendColorMessage_sub() {
  if (ColorMessage[0] > 0x01) ColorMessage[0] -= 0x01;
  uint8_t data[MAX_DATA_LEN] = {0};
  data[1] = Lightdata[1];
  data[0] = ColorMessage[0];
  Colordata[0] = data[0];
  sendCANFrame(Light_CAN_ID, data, MAX_DATA_LEN);
}

// ==================== 空调温度 ====================
static uint8_t TempMessage[MAX_DATA_LEN] = {0};

void sendTempMessage_add() {
  if (TempMessage[0] < 0x21) TempMessage[0] += 0x01;
  uint8_t data[8] = {0};
  data[4] = TempMessage[0];
  sendCANFrame(Temp_CAN_ID, data, 8);
}

void sendTempMessage_sub() {
  if (TempMessage[0] > 0x11) TempMessage[0] -= 0x01;
  uint8_t data[8] = {0};
  data[4] = TempMessage[0];
  sendCANFrame(Temp_CAN_ID, data, 8);
}

// ==================== 空调风量 ====================
static uint8_t WindMessage[MAX_DATA_LEN] = {0};

void sendWindMessage_add() {
  if (WindMessage[0] < 0x70) WindMessage[0] += 0x10;
  uint8_t data[8] = {0};
  data[0] = WindMessage[0];
  sendCANFrame(Wind_CAN_ID, data, 8);
}

void sendWindMessage_sub() {
  if (WindMessage[0] > 0x10) WindMessage[0] -= 0x10;
  uint8_t data[8] = {0};
  data[0] = WindMessage[0];
  sendCANFrame(Wind_CAN_ID, data, 8);
}

// ==================== 座椅分区 ====================
void sendSeatpartMessage_add() {
  uint8_t data[8] = {0};
  data[7] = 0x01;
  sendCANFrame(Seat_CAN_ID, data, 8);
}

void sendSeatpartMessage_sub() {
  uint8_t data[8] = {0};
  data[7] = 0x04;
  sendCANFrame(Seat_CAN_ID, data, 8);
}

// ==================== 座椅强度 ====================
void sendSeatMessage_add() {
  uint8_t data[8] = {0};
  data[7] = 0x10;
  sendCANFrame(Seat_CAN_ID, data, 8);
}

void sendSeatMessage_sub() {
  uint8_t data[8] = {0};
  data[7] = 0x40;
  sendCANFrame(Seat_CAN_ID, data, 8);
}

// ==================== 初始化 ====================
void canfd_init() {
  Serial.println("开始CAN初始化...");

  // SPI 初始化（严格按照例程的引脚顺序）
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Serial.println("SPI初始化完成");

  // CAN 初始化（最多重试 5 次）
  int initResult;
  for (int i = 0; i < 5; i++) {
    initResult = CAN.begin(CAN_BAUD_RATE);
    if (initResult == CAN_OK) break;
    Serial.print("初始化失败，重试...(");
    Serial.print(i);
    Serial.println(")");
    delay(100);
  }

  if (initResult == CAN_OK) {
    CAN.setMode(CAN_NORMAL_MODE);
    delay(10);
    Serial.println("CAN初始化成功！");
    canReady = true;
  } else {
    Serial.print("CAN初始化失败，错误码: ");
    Serial.println(initResult);
    canReady = false;
  }
}
