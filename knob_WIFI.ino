/*
 * knob8.10 —— FOC 旋钮固件（ESP32 WROOM）
 *
 * 通信架构：
 *   - ESP32 开启 AP 热点 "ESP32-AP"
 *   - 树莓派连接该热点（IP 通常为 192.168.4.2）并运行 TCP 服务器
 *   - ESP32 作为 TCP 客户端连接 192.168.4.2:5001
 *   - 旋钮状态变化时，通过 TCP 发送与原来 Serial.println 等效的文本行
 *     （消息协议见 README.md）
 *
 * 上电时序：
 *   1. 先开启 AP 热点（wifi_tcp_init，只开热点不阻塞）
 *   2. 电机再上电、校准并进入锁定模式
 *   3. loop 中后台连接树莓派 TCP 服务器（wifi_tcp_loop）
 *
 * 模块划分：
 *   motor_function.*  旋钮模式与手感控制
 *   can_bus.*         CAN 报文发送（MCP2518FD）
 *   wifi_tcp.*        AP 热点 + TCP 客户端
 */
#include "FOC.h"
#include "motor_function.h"
#include "can_bus.h"
#include "wifi_tcp.h"

// 电机参数
int Sensor_DIR = 1; // 传感器方向，若电机运动不正常，将此值取反
int Motor_PP = 7;   // 电机极对数

void setup() {
  Serial.begin(115200);

  // LED4 (WS2812B) 数据线 GPIO2 钳位到低电平，防止 WiFi 射频噪声导致误亮
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  // 1. 先开 AP 热点：热点起来后再启动电机
  wifi_tcp_init();

  // 2. 电机上电、校准，进入锁定模式
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);           // 使能，一定要放在校准电机前
  FOC_Vbus(12);                    // 设定驱动器供电电压，初始化编码器和电流传感器
  FOC_alignSensor(Motor_PP, Sensor_DIR);

  knob_init();                      // 记录零角度
  button_event_init();              // 按钮事件初始化
  canfd_init();                     // CAN 初始化

  FOC_M0_SET_ANGLE_PID(0.13, 0, 0, 100000, 5.5);   // 角度环 PID
  FOC_M0_SET_VEL_PID(0.13, 0, 0, 100000, 5.5);    // 速度环 PID
  FOC_M0_SET_CURRENT_PID(0.5, 0.01, 0, 100000);   // 电流环 PID
}

void loop() {
  runFOC();
  Mode_Change();
  button_and_time();
  wifi_tcp_loop();                  // 后台保持/重连到树莓派的 TCP 服务器
  sync_send_position();             // 周期上报当前角度给 knob_sync（50Hz）
}
