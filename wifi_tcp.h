#ifndef WIFI_TCP_H
#define WIFI_TCP_H

#include <Arduino.h>

// 初始化：开启 AP 热点并启动后台网络任务
// 上电流程：先开热点 -> 电机再启动进入锁定模式 -> 后台任务扫描网段
// 后台任务会自动发现并连接本网络所有监听 5001 端口的 TCP 服务器（最多 8 台），
// sendToHost() 的消息会同时发给所有已连接的服务器。
void wifi_tcp_init();

// 需要放在 loop() 中循环调用：连接/扫描/发送都由后台任务完成，这里保留空实现
void wifi_tcp_loop();

// 是否已连上至少一台服务器
bool tcp_host_connected();

// 向所有已连接的服务器发送一行文本（等效于原来的 Serial.println）
// msg 不需要带换行符，函数会自动补 \r\n；发送由后台任务完成，不会阻塞主循环
void sendToHost(const String& msg);

#endif
