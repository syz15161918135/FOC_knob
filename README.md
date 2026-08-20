# knob_WIFI —— 力反馈旋钮固件（ESP32 WROOM）

基于 FOC 电机控制，实现带力反馈的旋钮交互。旋钮旋转时通过 CAN 总线控制车辆功能（音量、空调、氛围灯、座椅），并通过 WiFi/TCP 向树莓派等服务器上报状态，同时可向 knob_sync 从动旋钮同步电机位置。

## 项目特点

- **模块化结构**：CAN 总线、WiFi/TCP、旋钮逻辑、主程序各自独立
- **模式函数化**：每个手感模式独立一个函数，方便单独维护
- **后台网络任务**：TCP 扫描、连接、发送由 FreeRTOS 后台任务完成，不阻塞主循环的电机控制
- **上电时序**：先开 AP 热点，再启动电机进入锁定模式；TCP 连接在后台处理，不耽误电机启动
- **断线自动重连**：周期扫描网段，自动发现并连接所有 TCP 服务器，断线自动清理、新设备自动接入

## 目录结构

```
knob_WIFI/
├── knob_WIFI.ino          主程序（setup / loop）
├── motor_function.h/.cpp  旋钮模式、手感控制、按键处理、位置同步上报
├── can_bus.h/.cpp         CAN 报文发送（MCP2518FD）
├── wifi_tcp.h/.cpp        AP 热点 + TCP 客户端（后台任务）
└── FOC.h/.cpp             电机 FOC 控制及 AS5600、电流、PID 等库
```

## 通信架构

1. ESP32 上电后**先开启 AP 热点**（默认 `ESP32-AP` / `12345678`），只开热点、不做阻塞连接
2. 热点起来后，电机再上电、校准并进入锁定模式
3. 各服务器（树莓派等）连接该热点，由 DHCP 分配 `192.168.4.x` 地址
4. 每台服务器运行 TCP 服务器（端口 `5001`）
5. ESP32 后台任务周期性扫描地址段 `.2 ~ .16`（`SCAN_IP_END` 可调），自动发现并连接所有监听 `5001` 的服务器（最多 `MAX_SERVERS` 台）
6. 旋钮状态变化时，ESP32 通过 TCP 发送一行文本（`\r\n` 结尾），同时发给所有已连接的服务器

所有配置都在 [wifi_tcp.cpp](wifi_tcp.cpp) 顶部，按需修改：

```cpp
const char* AP_SSID             = "ESP32-AP";
const char* AP_PASSWORD         = "12345678";   // WPA2 至少 8 位；留空为开放热点
const uint16_t SERVER_PORT      = 5001;         // 所有服务器统一监听端口
const uint32_t FIRST_SCAN_DELAY_MS = 3000;      // 上电后多久开始第一次扫描
const uint32_t SCAN_INTERVAL_MS   = 30000;      // 两轮完整扫描的间隔
const uint32_t SCAN_RETRY_FAST_MS = 5000;       // 一台服务器都没连上时的快速重扫间隔
const uint32_t PROBE_TIMEOUT_MS   = 200;        // 单个 IP 探测超时
const int      MAX_SERVERS        = 8;          // 最多同时连接的服务器数
const uint8_t  SCAN_IP_END        = 16;         // 扫描 .2 ~ .16 共 15 个地址
```

CAN 引脚和波特率在 [can_bus.cpp](can_bus.cpp) 顶部；按键引脚 `BUTTON1` 在 [motor_function.h](motor_function.h)。

## TCP 消息协议

| 模式 | 消息内容 | 说明 |
| --- | --- | --- |
| 位置同步 | `P<角度>` | 每 20ms 上报一次当前角度（连续角度，度，如 `P123.45`），供 knob_sync 跟随 |
| 主标题 TITLE_0 | `1`—`9` | 当前选择的模式编号 |
| 主标题确认 | `select` | 单击确认进入所选模式 |
| 所有模式返回 | `back` | 双击返回主标题 |
| 锁定态进入9号 | `9` → `select` → `0.1` | 锁定态单击：先发 `9`、`select`，再发 `0.1`，进入9号模式阻尼子模式 |
| 9号子模式切换 | `0.1` / `0.2` / `0.3` / `0.4` | 0.1阻尼 / 0.2 10°小棘轮 / 0.3 30°中棘轮 / 0.4 60°正弦（循环） |
| 9号返回标题 | `back` + `4` | 双击返回标题选择页，指针停在4号模式但不进入 |
| 音量 1_1 / 1_2 | `1`—`40` | 音量大小（同时发 CAN） |
| 空调温度 2_1 | `1`—`17` | 温度高低（同时发 CAN） |
| 空调风量 2_2 | `1`—`8` | 风量大小（同时发 CAN） |
| 氛围灯亮度 3_1 | `1`—`7` | 亮度（同时发 CAN） |
| 氛围灯颜色 3_2 | `1`—`128` | 颜色（同时发 CAN） |
| 座椅分区 8_1 | `1`—`3` | 分区（同时发 CAN） |
| 座椅强度 8_2 | `1`—`5` | 强度（同时发 CAN） |
| 子模式切换 | `1.1` `1.2` `2.1` `2.2` `3.1` `3.2` `8.1` `8.2` | 单击在子模式间切换 |
| 音乐盒 4 | `111` / `333` / `444` | 播放拧发条音乐 / 播放音乐盒 / 停止 |
| 掉头 6 / 悬架 7 | 数值 | 当前角度/档位 |

## 树莓派端 TCP 服务器示例

```python
import socket

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("0.0.0.0", 5001))
server.listen(1)
print("等待 ESP32 连接...")

while True:
    conn, addr = server.accept()
    print(f"ESP32 已连接: {addr}")
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            for line in data.decode().splitlines():
                print(line)
    except OSError:
        pass
    finally:
        conn.close()
        print("连接断开")
```

## 烧录

- Arduino IDE 选择开发板 **ESP32 Dev Module**（WROOM），已安装 ESP32 核心
- 直接打开本文件夹内的 `knob_WIFI.ino` 烧录
- 服务器需先运行上面的 TCP 服务器；ESP32 开机会自动重连，不用重启
