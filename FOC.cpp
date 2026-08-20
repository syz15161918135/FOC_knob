#include <Arduino.h>
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "InlineCurrent.h" //引入在线电流检测
#include "motor_function.h"
#include "FOC.h"

#define _3PI_2 4.71238898038f
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))


float voltage_power_supply;
float Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0;
float zero_electric_angle = 0;
int PP = 7, DIR = 1;
int pwmA = 32;
int pwmB = 33;
int pwmC = 25;

// 定时器中断变量
volatile bool run = false;  // 中断标志
volatile uint32_t tim_cnt = 0; // 中断计数
hw_timer_t *timer0 = NULL;  // 定时器中断全局变量（句柄）

/************************ 全局控制变量 ************************/
volatile float target_angle   = 0;       // 位置目标
float target_vel     = 0;       // 速度目标
float target_current = 0;       // 电流目标

// 低通滤波初始化
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01);  // Tf = 10ms 0.01  //M0速度环
LowPassFilter M0_Curr_Flt = LowPassFilter(0.05); // Tf = 5ms 0.05  //M0电流环

// PID
PIDController vel_loop_M0(2, 0, 0, 100000, voltage_power_supply / 2);
PIDController angle_loop_M0(2, 0, 0, 100000, 100);
PIDController current_loop_M0(1.2, 0, 0, 100000, 12.6);

// AS5600
Sensor_AS5600 S0 = Sensor_AS5600(0);
TwoWire S0_I2C = TwoWire(0);

// 初始化电流闭环
CurrSense CS_M0 = CurrSense(0);

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

//=================PID 设置函数=================
// 速度PID参数初始化
void FOC_M0_SET_VEL_PID(float P, float I, float D, float ramp, float limit) // M0角度环PID设置
{
  vel_loop_M0.P = P;
  vel_loop_M0.I = I;
  vel_loop_M0.D = D;
  vel_loop_M0.output_ramp = ramp;
  vel_loop_M0.limit = limit;
}

// 角度PID参数初始化
void FOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit) // M0角度环PID设置
{
  angle_loop_M0.P = P;
  angle_loop_M0.I = I;
  angle_loop_M0.D = D;
  angle_loop_M0.output_ramp = ramp;
  angle_loop_M0.limit = limit;
}

// 电流PID参数初始化
void FOC_M0_SET_CURRENT_PID(float P, float I, float D, float ramp) // M0电流环PID设置
{
  current_loop_M0.P = P;
  current_loop_M0.I = I;
  current_loop_M0.D = D;
  current_loop_M0.output_ramp = ramp;
}

// 设置M0速度PID参数 返回值：pid计算结果
float FOC_M0_VEL_PID(float error) // M0速度环
{
  return vel_loop_M0(error);
}

// 设置M0角度PID参数 返回值：pid计算结果
float FOC_M0_ANGLE_PID(float error) // M0角度环
{
  return angle_loop_M0(error);
}


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2 * PI); // 取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2 * PI);
}

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc)
{
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // 写入PWM到PWM 0 1 2 通道
  ledcWrite(pwmA, dc_a * 255);
  ledcWrite(pwmB, dc_b * 255);
  ledcWrite(pwmC, dc_c * 255);
}
void setTorque(float Uq, float angle_el)
{
  Uq = _constrain(Uq, -(voltage_power_supply) / 2, (voltage_power_supply) / 2);
  float Ud = 0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha = -Uq * sin(angle_el);
  Ubeta = Uq * cos(angle_el);

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (- Ualpha + sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  setPwm(Ua, Ub, Uc);
}

void FOC_Vbus(float power_supply)
{
  voltage_power_supply = power_supply;
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  ledcAttach(pwmA, 30000, 8);
  ledcAttach(pwmB, 30000, 8);
  ledcAttach(pwmC, 30000, 8);
  ledcWrite(pwmA, 0);
  ledcWrite(pwmB, 0);
  ledcWrite(pwmC, 0);
  Serial.println("完成PWM初始化设置");

  // AS5600
  S0_I2C.begin(19, 18, 400000UL);
  S0.Sensor_init(&S0_I2C);
  Serial.println("编码器加载完毕");

  // 速度环 PID 加载
  vel_loop_M0 = PIDController{2, 0, 0, 100000, voltage_power_supply / 2};
  
  // 初始化电流传感器
  CS_M0.init();
}

float _electricalAngle()
{
  return _normalizeAngle((float)(DIR * PP) * S0.getMechanicalAngle() - zero_electric_angle);
}

void FOC_alignSensor(int _PP, int _DIR)
{ 
  PP = _PP;
  DIR = _DIR;
  setTorque(3, _3PI_2); // 起劲
  delay(1000);
  S0.Sensor_update(); // 更新角度，方便下面电角度读取
  zero_electric_angle = _electricalAngle();
  setTorque(0, _3PI_2); // 松劲（解除校准）
  Serial.print("0电角度：");
  Serial.println(zero_electric_angle);
}

//=========================电流读取=========================

// 通过Ia,Ib,Ic计算Iq,Id(目前仅输出Iq)
float cal_Iq_Id(float current_a, float current_b, float angle_el)
{
  float I_alpha = current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  float ct = cos(angle_el);
  float st = sin(angle_el);
  // float I_d = I_alpha * ct + I_beta * st;
  float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}

float FOC_M0_Current()
{
  float I_q_M0_ori = cal_Iq_Id(CS_M0.current_a, CS_M0.current_b, _electricalAngle());
  float I_q_M0_flit = M0_Curr_Flt(I_q_M0_ori);
  return I_q_M0_flit;
}

//=========================编码器读取=========================
// 获得编码器弧度
float FOC_M0_Angle()
{
  return DIR * S0.getAngle();
}

// 获得编码器弧度速度，有滤波
float FOC_M0_Velocity()
{
  float vel_M0_ori = S0.getVelocity();
  float vel_M0_flit = M0_Vel_Flt(DIR * vel_M0_ori);
  return vel_M0_flit; // 考虑方向
}

//================力矩设置函数================
// 参数：设定电压 结果：运行
void FOC_M0_setTorque(float Target)
{
  // setTorque(current_loop_M0(Target - FOC_M0_Current()), _electricalAngle());// 改进后+电流环
  setTorque(Target, _electricalAngle()); // 启动
}

void FOC_M0_set_Velocity_Angle(float Target) // 速度+位置闭环
{
  FOC_M0_setTorque(FOC_M0_VEL_PID(FOC_M0_ANGLE_PID((Target - FOC_M0_Angle()) * 180 / PI) - FOC_M0_Velocity()));
}

void FOC_M0_setVelocity(float Target) // 速度闭环
{
  FOC_M0_setTorque(FOC_M0_VEL_PID((Target - FOC_M0_Velocity()) * 180 / PI));
}

void FOC_M0_set_Force_Angle(float Target) // 位置闭环
{
  FOC_M0_setTorque(FOC_M0_ANGLE_PID((Target - FOC_M0_Angle()) * 180 / PI));
}

//=============电流力矩环===================
void FOC_M0_setTorque_current(float Target) // 电流力矩环
{
  setTorque(current_loop_M0(Target - FOC_M0_Current()), _electricalAngle()); //电流环
}

//=============电流速度环===================
void FOC_M0_set_Velocity_current(float Target) // 电流力矩环
{
  FOC_M0_setTorque(current_loop_M0(FOC_M0_VEL_PID((Target - FOC_M0_Velocity()) * 180 / PI) - FOC_M0_Current()));
}

//=============10kHz定时器中断===================
// ESP32 240MHz 10kHz定时器初始化
void Timer_Init(void){
  timer0 = timerBegin(100000UL); // 分频为100kHz，1us
  if (timer0 == NULL) {
    Serial.println("错误：定时器初始化失败 (Core 3.0)!");
    return;
  }
  timerAttachInterrupt(timer0, &onTimer);
  timerAlarm(timer0, 100, true, 0); // 100kHz/100 目前1kHz
  Serial.println("定时器已启动 (Core 3.0)，频率：10kHz");
}
/************************ 核心中断（无中间变量 + 你的定制逻辑） ************************/
void IRAM_ATTR onTimer()
{
  tim_cnt++;
  run = true;
}

void runFOC()
{
  //====角度传感器更新====
  S0.Sensor_update();
  //====电流传感器更新====
  CS_M0.getPhaseCurrents();
}