#include <Arduino.h>

#ifndef MOTOR_FUNCTION_H
#define MOTOR_FUNCTION_H

#define BUTTON1 4   // 按键引脚
#define BUTTON2 -1
#define BUTTON3 -1

#define _2PI 6.28318530718f

extern float pre_speed;
extern float pre_angle;
extern float zero_angle;

/**************按键函数声明*****************/
void Mode_Change();           // 旋钮功能模式切换
void attachClick();           // 单击
void attachDoubleClick();     // 双击
void attachLongPressStart();  // 长按开始
void attachDuringLongPress(); // 长按过程
void attachLongPressStop();   // 长按结束
void attachMultiClick();      // 打印按下多次次数
void button_event_init();     // 按键事件初始化
void button_vibrate();        // 单击震动
void knob_init();             // 重置零角度

/**************旋钮手感函数***************/
void Check_Rotate(int target_angle, int min_count, int max_count); // 档位计数
void Adj_Detents_Mode(int target_angle, int min_count, int max_count); // 棘轮
void Adj_Sin_Mode(int target_angle, int min_count, int max_count); // 正弦
void Adj_Damp_Mode(int ratio);                // 阻尼，顺滑
void Adj_Damp_Mode(int ratio, int target_angle, int min_count, int max_count); // 阻尼+档位
void Smooth_Boundary_Mode(float angle_rang);  // 有界——平滑模式
void Detents_Boundary_Mode(int target_angle, int max_count); // 有界——棘轮模式

/**************旋钮状态更新***************/
void update_angle_speed();
void button_and_time(); // 检测按键 无BLE

/**************同步位置上报（knob_sync）***************/
float sync_unwrap_deg();   // 获取从开机起的连续角度（度）
void sync_send_position(); // 周期上报当前角度给 knob_sync（50Hz）

#endif
