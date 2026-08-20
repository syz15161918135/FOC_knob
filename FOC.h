#include <Arduino.h>
#include "pid.h"

/************************ 硬件初始化 ************************/
volatile extern uint32_t tim_cnt;
volatile extern bool run;
extern hw_timer_t *timer0;
void IRAM_ATTR onTimer();

void setPwm(float Ua, float Ub, float Uc);
void setTorque(float Uq, float angle_el);
void FOC_Vbus(float power_supply);
void FOC_alignSensor(int _PP, int _DIR);
float _electricalAngle();
void Timer_Init();
// 传感器读取
float FOC_M0_Velocity();
float FOC_M0_Angle();
float FOC_M0_Current();

// PID
void FOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit);
void FOC_M0_SET_VEL_PID(float P, float I, float D, float ramp, float limit);
void FOC_M0_SET_CURRENT_PID(float P, float I, float D, float ramp);
float FOC_M0_VEL_PID(float error);
float FOC_M0_ANGLE_PID(float error);

// 接口函数
void FOC_M0_set_Velocity_Angle(float Target);
void FOC_M0_setVelocity(float Target);
void FOC_M0_set_Force_Angle(float Target);
void FOC_M0_setTorque(float Target);
void FOC_M0_setTorque_current(float Target);//电流环力矩
void FOC_M0_set_Velocity_current(float Target);

// runFOC 循环函数
void runFOC();