#include <stdint.h>
#include "FOC.h"
#include "motor_function.h"
#include "OneButton.h"
#include "lowpass_filter.h"
#include "wifi_tcp.h"
#include "can_bus.h"

// 按键调试日志：1 = 在串口打印按键事件；0 = 关闭
#define BUTTON_DEBUG 1

// ==================== 树莓派 TCP 消息协议 ====================
// 音量:        1—40   对应音量大小
// 空调风量:    1—8    对应风量大小
// 空调温度:    1—17   对应温度高低
// 氛围灯亮度:  1—7    对应氛围灯亮度
// 氛围灯颜色:  1—128  对应氛围灯颜色
// 座椅分区:    1—3    对应分区
// 座椅强度:    1—5    对应强度
// 状态切换:    select / back / 1.1 / 1.2 / 2.1 / 2.2 / 3.1 / 3.2 / 8.1 / 8.2
// 锁定态进入9号: 先发 9 → select → 0.1，进入9号模式阻尼子模式
// 9号子模式:   0.1(阻尼) / 0.2(10°小棘轮) / 0.3(30°中棘轮) / 0.4(60°正弦)
//              进入/切换到哪个子模式就发送对应编号；双击发 back 回标题页（指针停4号，不进入）
// 音乐盒:      111 播放拧发条音乐 / 333 播放音乐盒音乐 / 444 停止播放

// ============================ 旋钮模式 ============================
typedef enum {
  LOCK_MODE = 0,
  TITLE_1_1,
  TITLE_2_1,
  TITLE_3_1,
  TITLE_4,
  TITLE_5,
  TITLE_6,
  TITLE_7,
  TITLE_8_1,
  TITLE_9,
  TITLE_1_2,
  TITLE_2_2,
  TITLE_3_2,
  TITLE_8_2,
  TITLE_0
} KEY_STATE;
int last_state, state = LOCK_MODE;

// ============================ 9号模式子模式 ============================
// 9号模式内置四个子模式：阻尼 / 10°小棘轮 / 30°中棘轮 / 60°正弦
#define MODE9_SUB_DAMPING      1  // 阻尼
#define MODE9_SUB_RATCHET_10   2  // 10°小棘轮
#define MODE9_SUB_RATCHET_30   3  // 30°中棘轮
#define MODE9_SUB_RATCHET_60   4  // 60°正弦
int mode9_sub = MODE9_SUB_DAMPING; // 当前9号模式子模式

// ============================ 全局状态 ============================
bool clockwork = false, player = false;  // player=true 表示正在播放音乐盒音乐
bool flag_target = true;                 // true 表示按键切换了模式
bool click_flag = false;
bool clickChangeMode = false;
bool doubleClickChangeMode = false;
int cnt_music = 0;
float target = 0, last_target = 0;
static int angle_rest = 0;
int time_flag = 0;
float pre_angle = 0, pre_speed = 0, zero_angle = 0;
int clock_time = 0;
int last_clock_time = 0;
int Music_diff = 0;

// 每个档位的计数值：[档位][0] = 当前值，[档位][1] = 上一次的值
int title_cnt[13][2] = {
    {1, 1}, // 0   TITLE_0 模式选择
    {1, 1}, // 1_1 音量
    {1, 1}, // 1_2 音量
    {1, 1}, // 2_1 空调温度
    {1, 1}, // 2_2 空调风量
    {1, 1}, // 3_1 氛围灯亮度
    {1, 1}, // 3_2 氛围灯颜色
    {1, 1}, // 4   音乐盒
    {1, 1}, // 5   指尖陀螺
    {1, 1}, // 6   掉头
    {1, 1}, // 7   悬架
    {1, 1}, // 8_1 座椅分区
    {1, 1}  // 8_2 座椅强度
};

OneButton button(BUTTON1, true, true);

// ============================ 小工具 ============================
// 向树莓派发送档位数值（等效原 Serial.println(数值)）
static void hostSendValue(int value) {
  sendToHost(String(value));
}

// ============================ 档位计算 ============================
void Check_Rotate(int target_angle, int min_count, int max_count) {
  float attractor_distance = target_angle * 3.1415926 / 180.0; // 角度转弧度
  if (flag_target == true) {
    last_target = round(FOC_M0_Angle() / attractor_distance) * attractor_distance;
    flag_target = false;
  }
  // 四舍五入取整 attractor_distance 倍数
  target = round(FOC_M0_Angle() / attractor_distance) * attractor_distance;

  int *p_cnt = nullptr;
  switch (state) {
    case TITLE_0:   p_cnt = &title_cnt[0][0];  break;
    case TITLE_1_1: p_cnt = &title_cnt[1][0];  break;
    case TITLE_1_2: p_cnt = &title_cnt[2][0];  break;
    case TITLE_2_1: p_cnt = &title_cnt[3][0];  break;
    case TITLE_2_2: p_cnt = &title_cnt[4][0];  break;
    case TITLE_3_1: p_cnt = &title_cnt[5][0];  break;
    case TITLE_3_2: p_cnt = &title_cnt[6][0];  break;
    case TITLE_4:   p_cnt = &title_cnt[7][0];  break;
    case TITLE_5:   p_cnt = &title_cnt[8][0];  break;
    case TITLE_6:   p_cnt = &title_cnt[9][0];  break;
    case TITLE_7:   p_cnt = &title_cnt[10][0]; break;
    case TITLE_8_1: p_cnt = &title_cnt[11][0]; break;
    case TITLE_8_2: p_cnt = &title_cnt[12][0]; break;
    default:        p_cnt = nullptr;           break;
  }

  int steps = (int)round((target - last_target) / attractor_distance);
  if (p_cnt != nullptr) {
    *(p_cnt + 1) = *p_cnt;   // 保存上一次的值
    if (steps != 0) {
      (*p_cnt) += steps;
      if (*p_cnt > max_count) {
        if (state == TITLE_0) {
          while (*p_cnt > max_count) {
            *p_cnt -= max_count;
          }
        } else {
          *p_cnt = max_count;
        }
      }
      if (*p_cnt < min_count) {
        if (state == TITLE_0) {
          while (*p_cnt < min_count) {
            *p_cnt += max_count;
          }
        } else {
          *p_cnt = min_count;
        }
      }
    }
  }
  last_target = target;
}

// ============================ 手感模式 ============================
// 参数：目标角度，目标力矩，棘轮手感（target_angle = 15 表示 15° 变化力矩）
// 范围 [min_count, max_count]
void Adj_Detents_Mode(int target_angle, int min_count, int max_count) {
  if (angle_rest == 0) {
    zero_angle = FOC_M0_Angle();
    angle_rest = 1;
  }
  Check_Rotate(target_angle, min_count, max_count);
  FOC_M0_setTorque(FOC_M0_ANGLE_PID((target - FOC_M0_Angle()) * 180 / PI)); // 角度 PID
}

// 参数：目标角度，档位范围，正弦手感（target_angle = 30°）
// 每个 target_angle 周期分成 4 段：力矩大小 = 到段边界的距离，
// 方向按段相位交替（0、1 段为负，2、3 段为正），与原四分支逻辑等价
void Adj_Sin_Mode(int target_angle, int min_count, int max_count) {
  if (angle_rest == 0) {
    zero_angle = FOC_M0_Angle();
    angle_rest = 1;
  }
  Check_Rotate(target_angle, min_count, max_count); // 档位计数，与 Adj_Detents_Mode 一致

  float quarter = target_angle * PI / 180.0f / 4.0f; // 1/4 目标角度周期（弧度）
  float rel = FOC_M0_Angle() - zero_angle;          // 相对进入模式时的角度，避免多圈累计漂移
  int n = (int)floor(rel / quarter); // 所在四分之一段
  int phase = n % 4;                 // 段内相位
  float frac = rel - n * quarter;    // 段内位置 [0, quarter)

  // 偶数段(0、2)取段首距离，奇数段(1、3)取段尾距离
  float dist = (phase == 0 || phase == 2) ? frac : (quarter - frac);
  // 前两段(0、1)拉向负方向，后两段(2、3)拉向正方向
  float err = (phase == 0 || phase == 1) ? -dist : dist;

  FOC_M0_setTorque(FOC_M0_ANGLE_PID(err * 180 / PI));
}

LowPassFilter angleflt = LowPassFilter(0.01);

// 阻尼，顺滑
void Adj_Damp_Mode(int ratio) {
  FOC_M0_SET_ANGLE_PID(ratio, 0, 0.00005, 100000, 5.5); // 角度环 PID，设置力矩大小
  FOC_M0_setTorque(FOC_M0_ANGLE_PID((angleflt(FOC_M0_Angle()) - FOC_M0_Angle())));
}

// 阻尼，顺滑，带角度档位
void Adj_Damp_Mode(int ratio, int target_angle, int min_count, int max_count) {
  Check_Rotate(target_angle, min_count, max_count);
  FOC_M0_SET_ANGLE_PID(ratio, 0, 0.00005, 100000, 5.5);
  FOC_M0_setTorque(FOC_M0_ANGLE_PID((angleflt(FOC_M0_Angle()) - FOC_M0_Angle())));
}

// 有界——阻尼模式
void Smooth_Boundary_Mode(float angle_rang) {
  if (angle_rest == 0) {
    zero_angle = FOC_M0_Angle();
    angle_rest = 1;
  }
  float angle = FOC_M0_Angle();
  float limit_rang = angle_rang / 360.0f / 2 * _2PI;
  if (angle >= zero_angle - limit_rang && angle <= zero_angle + limit_rang) {
    Adj_Damp_Mode(50);
  } else {
    if (angle < zero_angle - limit_rang) {
      FOC_M0_set_Force_Angle(zero_angle - limit_rang);
    }
    if (angle > zero_angle + limit_rang) {
      FOC_M0_set_Force_Angle(zero_angle + limit_rang);
    }
  }
}

// 有界——棘轮模式（target_angle = 45°，5 个档位：±limit_rang、±limit_rang/2、0）
void Detents_Boundary_Mode(int target_angle, int max_count) {
  if (angle_rest == 0) {
    zero_angle = FOC_M0_Angle();
    angle_rest = 1;
  }
  int *p_cnt = nullptr;
  switch (state) {
    case TITLE_1_1: p_cnt = &title_cnt[1][0];  break;
    case TITLE_1_2: p_cnt = &title_cnt[2][0];  break;
    case TITLE_2_1: p_cnt = &title_cnt[3][0];  break;
    case TITLE_2_2: p_cnt = &title_cnt[4][0];  break;
    case TITLE_3_1: p_cnt = &title_cnt[5][0];  break;
    case TITLE_3_2: p_cnt = &title_cnt[6][0];  break;
    case TITLE_4:   p_cnt = &title_cnt[7][0];  break;
    case TITLE_5:   p_cnt = &title_cnt[8][0];  break;
    case TITLE_6:   p_cnt = &title_cnt[9][0];  break;
    case TITLE_7:   p_cnt = &title_cnt[10][0]; break;
    case TITLE_8_1: p_cnt = &title_cnt[11][0]; break;
    case TITLE_8_2: p_cnt = &title_cnt[12][0]; break;
    default:        p_cnt = nullptr;           break;
  }

  float limit_rang = target_angle / 180.0f * _2PI; // 半程范围（弧度），如 45° → ±90°
  float step = limit_rang / 4;                     // 相邻档位间隔
  // 归一化到档位刻度（-4,-2,0,2,4 对应 5 个档位），吸附到最近档位并限幅
  int detent = constrain((int)round((FOC_M0_Angle() - zero_angle) / step / 2) * 2, -4, 4);

  FOC_M0_set_Force_Angle(zero_angle + detent * step);
  if (p_cnt != nullptr) {
    *p_cnt = detent / 2 + 3; // -4→1, -2→2, 0→3, 2→4, 4→5
  }
}

// ============================ 各模式处理 ============================
// LOCK_MODE：锁定模式
static void mode_lock() {
  FOC_M0_set_Force_Angle(zero_angle); // 位置闭环固定角度，0 30° 时达到最大力 5.5V
  if (clickChangeMode) {
    // 锁定态单击：wifi 发“9”→“select”→“0.1”，进入9号模式的阻尼子模式
    sendToHost("9");
    sendToHost("select");
    mode9_sub = MODE9_SUB_DAMPING;
    sendToHost("0.1");            // 进入阻尼子模式，上报 0.1
    flag_target = true;
    state = TITLE_9;
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    doubleClickChangeMode = false;
  }
}

// TITLE_0：主标题，选择模式 1—9
static void mode_title_0() {
  FOC_M0_SET_ANGLE_PID(0.4, 0, 0, 100000, 5.5);
  Adj_Detents_Mode(30, 1, 9);
  if (title_cnt[0][0] != title_cnt[0][1]) {
    hostSendValue(title_cnt[0][0]);
    title_cnt[0][1] = title_cnt[0][0];
  }
  if (clickChangeMode) {
    state = title_cnt[0][0];
    sendToHost("select");
    if (state == TITLE_9) {
      mode9_sub = MODE9_SUB_DAMPING; // 从标题页进入9号也默认阻尼子模式
      sendToHost("0.1");             // 进入阻尼子模式，上报 0.1
    }
    flag_target = true;
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    doubleClickChangeMode = false;
  }
}

// TITLE_1_1：音量 1—40
static void mode_1_1() {
  FOC_M0_SET_ANGLE_PID(0.4, 0, 0, 100000, 5.5);
  Adj_Detents_Mode(10, 1, 40);
  if (title_cnt[1][0] > title_cnt[1][1]) {
    sendVolumeMessage_add();
    hostSendValue(title_cnt[1][0]);
  } else if (title_cnt[1][0] < title_cnt[1][1]) {
    sendVolumeMessage_sub();
    hostSendValue(title_cnt[1][0]);
  }
  title_cnt[1][1] = title_cnt[1][0];

  if (clickChangeMode) {
    flag_target = true;
    state = TITLE_1_2;
    sendToHost("1.2");
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_1_2：音量 1—40
static void mode_1_2() {
  FOC_M0_SET_ANGLE_PID(0.4, 0, 0, 100000, 5.5);
  Adj_Detents_Mode(10, 1, 40);
  if (title_cnt[2][0] > title_cnt[2][1]) {
    sendVolumeMessage_add();
    hostSendValue(title_cnt[2][0]);
  } else if (title_cnt[2][0] < title_cnt[2][1]) {
    sendVolumeMessage_sub();
    hostSendValue(title_cnt[2][0]);
  }
  title_cnt[2][1] = title_cnt[2][0];

  if (clickChangeMode) {
    flag_target = true;
    state = TITLE_1_1;
    sendToHost("1.1");
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_2_1：空调温度 1—17
static void mode_2_1() {
  FOC_M0_SET_ANGLE_PID(0.2, 0, 0, 100000, 5.5);
  Adj_Detents_Mode(20, 1, 17);
  if (title_cnt[3][0] > title_cnt[3][1]) {
    sendTempMessage_add();
    hostSendValue(title_cnt[3][0]);
  } else if (title_cnt[3][0] < title_cnt[3][1]) {
    sendTempMessage_sub();
    hostSendValue(title_cnt[3][0]);
  }
  title_cnt[3][1] = title_cnt[3][0];

  if (clickChangeMode) {
    flag_target = true;
    state = TITLE_2_2;
    sendToHost("2.2");
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_2_2：空调风量 1—8
static void mode_2_2() {
  FOC_M0_SET_ANGLE_PID(0.2, 0, 0, 100000, 5.5);
  Adj_Detents_Mode(30, 1, 8);
  if (title_cnt[4][0] > title_cnt[4][1]) {
    sendWindMessage_add();
    hostSendValue(title_cnt[4][0]);
  } else if (title_cnt[4][0] < title_cnt[4][1]) {
    sendWindMessage_sub();
    hostSendValue(title_cnt[4][0]);
  }
  title_cnt[4][1] = title_cnt[4][0];

  if (clickChangeMode) {
    flag_target = true;
    state = TITLE_2_1;
    sendToHost("2.1");
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_3_1：氛围灯亮度 1—7
static void mode_3_1() {
  FOC_M0_SET_ANGLE_PID(0.2, 0, 0, 100000, 5.5);
  Adj_Detents_Mode(30, 1, 7);
  if (title_cnt[5][0] > title_cnt[5][1]) {
    sendLightMessage_add();
    hostSendValue(title_cnt[5][0]);
  } else if (title_cnt[5][0] < title_cnt[5][1]) {
    sendLightMessage_sub();
    hostSendValue(title_cnt[5][0]);
  }
  title_cnt[5][1] = title_cnt[5][0];

  if (clickChangeMode) {
    flag_target = true;
    state = TITLE_3_2;
    sendToHost("3.2");
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_3_2：氛围灯颜色 1—128
static void mode_3_2() {
  FOC_M0_SET_ANGLE_PID(0.2, 0, 0.001, 100000, 5.5);
  Adj_Detents_Mode(30, 1, 128);
  if (title_cnt[6][0] > title_cnt[6][1]) {
    sendColorMessage_add();
    hostSendValue(title_cnt[6][0]);
  } else if (title_cnt[6][0] < title_cnt[6][1]) {
    sendColorMessage_sub();
    hostSendValue(title_cnt[6][0]);
  }
  title_cnt[6][1] = title_cnt[6][0];

  if (clickChangeMode) {
    flag_target = true;
    state = TITLE_3_1;
    sendToHost("3.1");
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_4：音乐盒（拧发条后自动播放）
static void mode_4() {
  if (cnt_music == 0) {
    last_clock_time = millis();
  }

  // 上发条阶段：旋转累计发条量
  if (player == false) {
    FOC_M0_SET_ANGLE_PID(0.4, 0, 0, 100000, 5.5);
    Adj_Detents_Mode(30, -99999, 99999);
    if (title_cnt[7][0] != title_cnt[7][1]) {
      if (title_cnt[7][0] > title_cnt[7][1]) {
        cnt_music = cnt_music + 1000; // 计算总的拧动时长
      } else if (title_cnt[7][0] < title_cnt[7][1]) {
        cnt_music = cnt_music - 1000;
      }
      title_cnt[7][1] = title_cnt[7][0]; // 上一次旋转位置更新
      last_clock_time = millis();
      sendToHost("111"); // 播放拧发条音乐
    }
  }

  // 播放阶段：按发条量转动
  if (player == true) {
    flag_target = false;
    Check_Rotate(30, -99999, 99999);
    Music_diff = title_cnt[7][0] - title_cnt[7][1];
    if (cnt_music * Music_diff > 0) {
      last_clock_time = millis();
      player = false;
      return;
    }

    FOC_M0_SET_VEL_PID(0.006, 0, 0.000001, 100000, 5.5); // 速度环 PID
    if (cnt_music > 0) {
      FOC_M0_setVelocity(-4);
      cnt_music--;
    } else if (cnt_music < 0) {
      FOC_M0_setVelocity(4);
      cnt_music++;
    } else if (cnt_music == 0) {
      sendToHost("444");
      player = false;
      flag_target = true;
    }
  }

  // 按键控制
  if (clickChangeMode && cnt_music != 0) {
    cnt_music = 0;
    sendToHost("444"); // 停止播放音乐盒音乐
    player = false;
    flag_target = true;
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    cnt_music = 0;
    sendToHost("444");
    player = false;
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }

  // 停止拧动 900ms 后自动开始播放
  clock_time = millis();
  if (clock_time - last_clock_time > 900 && cnt_music != 0) {
    if (player == false) {
      sendToHost("333");
    }
    player = true;
    flag_target = true;
  }
}

// TITLE_5：指尖陀螺
static void mode_5() {
  Adj_Damp_Mode(-6);
  if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_6：掉头
static void mode_6() {
  // FOC_M0_SET_ANGLE_PID(0.13, 0, 0, 100000, 5.5);
  Adj_Damp_Mode(50, 5, 1, 360);
  if (title_cnt[9][0] != title_cnt[9][1]) {
    hostSendValue(title_cnt[9][0]);
    title_cnt[9][1] = title_cnt[9][0];
  }
  if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_7：悬架
static void mode_7() {
  FOC_M0_SET_ANGLE_PID(0.2, 0, 0, 100000, 5.5);
  Detents_Boundary_Mode(45, 5);
  if (title_cnt[10][0] != title_cnt[10][1]) {
    hostSendValue(title_cnt[10][0]);
    title_cnt[10][1] = title_cnt[10][0];
  }
  if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_8_1：座椅分区 1—3
static void mode_8_1() {
  FOC_M0_SET_ANGLE_PID(0.1, 0, 0, 100000, 5.5);
  Adj_Detents_Mode(120, 1, 3);
  if (title_cnt[11][0] > title_cnt[11][1]) {
    sendSeatpartMessage_add();
    hostSendValue(title_cnt[11][0]);
  } else if (title_cnt[11][0] < title_cnt[11][1]) {
    sendSeatpartMessage_sub();
    hostSendValue(title_cnt[11][0]);
  }
  title_cnt[11][1] = title_cnt[11][0];

  if (clickChangeMode) {
    flag_target = true;
    state = TITLE_8_2;
    sendToHost("8.2");
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_8_2：座椅强度 1—5
static void mode_8_2() {
  FOC_M0_SET_ANGLE_PID(0.2, 0, 0, 100000, 5.5);
  Adj_Detents_Mode(72, 1, 5);
  if (title_cnt[12][0] > title_cnt[12][1]) {
    sendSeatMessage_add();
    hostSendValue(title_cnt[12][0]);
  } else if (title_cnt[12][0] < title_cnt[12][1]) {
    sendSeatMessage_sub();
    hostSendValue(title_cnt[12][0]);
  }
  title_cnt[12][1] = title_cnt[12][0];

  if (clickChangeMode) {
    flag_target = true;
    state = TITLE_8_1;
    sendToHost("8.1");
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    flag_target = true;
    state = TITLE_0;
    sendToHost("back");
    doubleClickChangeMode = false;
  }
}

// TITLE_9：9号模式（阻尼 / 10°小棘轮 / 30°中棘轮 / 60°正弦）
static void mode_9() {
  // 按当前子模式施加不同手感
  switch (mode9_sub) {
    case MODE9_SUB_DAMPING:
      Adj_Damp_Mode(50);                   // 阻尼手感
      break;
    case MODE9_SUB_RATCHET_10:
      FOC_M0_SET_ANGLE_PID(0.6, 0, 0.0002, 100000, 5.5);
      Adj_Detents_Mode(10, -99999, 99999);  // 10°小棘轮
      break;
    case MODE9_SUB_RATCHET_30:
      FOC_M0_SET_ANGLE_PID(0.36, 0, 0, 100000, 5.5);
      Adj_Detents_Mode(30, -99999, 99999); // 30°中棘轮
      break;
    case MODE9_SUB_RATCHET_60:
      FOC_M0_SET_ANGLE_PID(0.3, 0, 0, 100000, 5.5);
      Adj_Sin_Mode(60, -99999, 99999);     // 60°正弦手感
      break;
    default:
      mode9_sub = MODE9_SUB_DAMPING;       // 异常兜底
      break;
  }

  // 单击：按顺序切换到下一个子模式，并发送“新进入”子模式的编号（0.1~0.4）
  if (clickChangeMode) {
    switch (mode9_sub) {
      case MODE9_SUB_DAMPING:
        mode9_sub = MODE9_SUB_RATCHET_10;
        sendToHost("0.2");                 // 进入10°小棘轮
        break;
      case MODE9_SUB_RATCHET_10:
        mode9_sub = MODE9_SUB_RATCHET_30;
        sendToHost("0.3");                 // 进入30°中棘轮
        break;
      case MODE9_SUB_RATCHET_30:
        mode9_sub = MODE9_SUB_RATCHET_60;
        sendToHost("0.4");                 // 进入60°正弦
        break;
      case MODE9_SUB_RATCHET_60:
        mode9_sub = MODE9_SUB_DAMPING;
        sendToHost("0.1");                 // 循环回阻尼
        break;
    }
    flag_target = true;   // 重新锚定棘轮目标角度
    clickChangeMode = false;
  } else if (doubleClickChangeMode) {
    // 双击：发 back 返回标题选择页，指针停在4号模式但不进入
    flag_target = true;
    state = TITLE_0;
    title_cnt[0][0] = 4;  // 标题页指针停在4号模式
    sendToHost("back");
    sendToHost("4");      // 上报当前选中模式为4（只停在4号，不进入）
    doubleClickChangeMode = false;
  }
}

// ============================ 模式调度 ============================
void Mode_Change() {
  update_angle_speed(); // 获得编码器角度、速度
  switch (state) {
    case LOCK_MODE: mode_lock();    break;
    case TITLE_0:   mode_title_0(); break;
    case TITLE_1_1: mode_1_1();     break;
    case TITLE_1_2: mode_1_2();     break;
    case TITLE_2_1: mode_2_1();     break;
    case TITLE_2_2: mode_2_2();     break;
    case TITLE_3_1: mode_3_1();     break;
    case TITLE_3_2: mode_3_2();     break;
    case TITLE_4:   mode_4();       break;
    case TITLE_5:   mode_5();       break;
    case TITLE_6:   mode_6();       break;
    case TITLE_7:   mode_7();       break;
    case TITLE_8_1: mode_8_1();     break;
    case TITLE_8_2: mode_8_2();     break;
    case TITLE_9:   mode_9();       break;
    default:                        break;
  }
}

// ============================ 按键处理 ============================
void attachClick() {
  if (BUTTON_DEBUG) Serial.println("检测到单击");
  click_flag = true;
  clickChangeMode = true;
}

void attachDoubleClick() {
  if (BUTTON_DEBUG) Serial.println("检测到双击");
  doubleClickChangeMode = true;
}

void attachLongPressStart() {
  if (BUTTON_DEBUG) Serial.println("检测到长按开始");
}

void attachLongPressStop() {
  if (BUTTON_DEBUG) Serial.println("检测到长按结束");
}

void attachDuringLongPress() {
  if (button.isLongPressed()) {
    if (BUTTON_DEBUG) Serial.println("长按中...");
  }
}

void attachMultiClick() {
  if (BUTTON_DEBUG) Serial.printf("检测到%d次连击\n", button.getNumberClicks());
}

void button_vibrate() {
  if (click_flag) {
    FOC_M0_setTorque(0.6);
    for (uint8_t i = 0; i < 15; i++) {
      runFOC();
      delay(1);
    }
    FOC_M0_setTorque(-0.6);
    for (uint8_t i = 0; i < 15; i++) {
      runFOC();
      delay(1);
    }
    FOC_M0_setTorque(0);
    click_flag = false;
  }
}

void button_event_init() {
  button.reset();
  button.setDebounceTicks(80);
  button.setClickTicks(400);
  button.setPressTicks(1000);

  button.attachClick(attachClick);
  button.attachDoubleClick(attachDoubleClick);
  button.attachLongPressStart(attachLongPressStart);
  button.attachDuringLongPress(attachDuringLongPress);
  button.attachLongPressStop(attachLongPressStop);
  button.attachMultiClick(attachMultiClick);
}

unsigned long prev_time = 0, now_time = 0;

void button_and_time() {
  now_time = millis();
  button.tick();
  button_vibrate();

  if (now_time - prev_time > 1000) {
    prev_time = now_time;
    time_flag = 1;
  } else {
    time_flag = 0;
  }
}

// ============================ 初始化与状态更新 ============================
void knob_init() {
  zero_angle = FOC_M0_Angle();
}

void update_angle_speed() {
  if (state != last_state) {
    angle_rest = 0;
  }
  last_state = state;
  pre_speed = FOC_M0_Velocity();
  pre_angle = FOC_M0_Angle() * 180 / 3.1415926;
}

// ============================ 同步位置上报（knob_sync） ============================
// 把 0~2π 的编码器读数展开成连续角度，初始为 0（度）
static bool  sync_first = true;
static float sync_last_raw = 0;
static float sync_unwrapped = 0;

float sync_unwrap_deg() {
  float raw = FOC_M0_Angle();
  if (sync_first) {
    sync_last_raw = raw;
    sync_unwrapped = 0;
    sync_first = false;
    return 0;
  }
  float delta = raw - sync_last_raw;
  if (delta > PI)       delta -= _2PI;
  else if (delta < -PI) delta += _2PI;
  sync_last_raw = raw;
  sync_unwrapped += delta;
  return sync_unwrapped * 180.0f / PI;
}

void sync_send_position() {
  static unsigned long lastSync = 0;
  if (millis() - lastSync >= 20) {          // 50Hz
    lastSync = millis();
    sendToHost("P" + String(sync_unwrap_deg(), 2));  // 协议：P<连续角度，度>
  }
}
