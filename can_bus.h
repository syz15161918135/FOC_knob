#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <Arduino.h>

// CAN 初始化（SPI + MCP2518FD）
void canfd_init();

// 音量（TITLE_1_1 / TITLE_1_2）
void sendVolumeMessage_add();
void sendVolumeMessage_sub();

// 氛围灯亮度（TITLE_3_1）
void sendLightMessage_add();
void sendLightMessage_sub();

// 氛围灯颜色（TITLE_3_2）
void sendColorMessage_add();
void sendColorMessage_sub();

// 空调温度（TITLE_2_1）
void sendTempMessage_add();
void sendTempMessage_sub();

// 空调风量（TITLE_2_2）
void sendWindMessage_add();
void sendWindMessage_sub();

// 座椅分区（TITLE_8_1）
void sendSeatpartMessage_add();
void sendSeatpartMessage_sub();

// 座椅强度（TITLE_8_2）
void sendSeatMessage_add();
void sendSeatMessage_sub();

#endif
