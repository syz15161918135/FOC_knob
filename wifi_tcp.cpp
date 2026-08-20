#include "wifi_tcp.h"
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ==================== AP 热点配置 ====================
const char* AP_SSID     = "ESP32-AP";  // 热点名称（服务器设备搜索这个 WiFi）
const char* AP_PASSWORD = "12345678";  // WPA2 密码至少 8 位；留空字符串表示开放热点

// ==================== 多服务器发现与连接 ====================
// ESP32 作为 AP，服务器（树莓派等）连上热点后由 DHCP 分配 192.168.4.x 地址
//（从 .2 开始顺序分配）。后台任务周期性扫描 SCAN_IP_END 以内的地址段，
// 凡是 SERVER_PORT 端口有 TCP 服务器监听的设备全部连接（最多 MAX_SERVERS 台）；
// sendToHost() 的消息会同时发给所有已连接服务器。
const uint16_t SERVER_PORT          = 5001;   // 所有服务器统一监听端口
const uint32_t FIRST_SCAN_DELAY_MS  = 3000;   // 上电后多久开始第一次扫描
const uint32_t SCAN_INTERVAL_MS     = 30000;  // 两轮完整扫描的间隔
const uint32_t SCAN_RETRY_FAST_MS   = 5000;   // 一台服务器都没连上时，用这个短间隔快速重扫
const uint32_t PROBE_TIMEOUT_MS     = 200;    // 单个 IP 探测超时（手机等设备响应可能较慢）
const int      MAX_SERVERS          = 8;      // 最多同时连接的服务器数
const uint8_t  SCAN_IP_END          = 16;     // 扫描 .2 ~ .16 共 15 个地址；设备多了再调大
const bool     MIRROR_TO_SERIAL     = false;  // 调试：同时打印到串口监视器

// 发送消息队列（主循环入队，后台任务统一发给所有服务器）
#define TX_QUEUE_LEN      16
#define TX_QUEUE_MSG_LEN  128
static QueueHandle_t txQueue = nullptr;

// 服务器连接表
static WiFiClient servers[MAX_SERVERS];
static bool serverUsed[MAX_SERVERS] = {false};

// ---------- 小工具 ----------
static int countServers() {
  int n = 0;
  for (int i = 0; i < MAX_SERVERS; i++) {
    if (serverUsed[i]) n++;
  }
  return n;
}

static bool addrInList(IPAddress ip) {
  for (int i = 0; i < MAX_SERVERS; i++) {
    if (serverUsed[i] && servers[i].remoteIP() == ip) return true;
  }
  return false;
}

// 清理已断开的服务器连接，空出槽位
static void dropDeadServers() {
  for (int i = 0; i < MAX_SERVERS; i++) {
    if (serverUsed[i] && !servers[i].connected()) {
      Serial.printf("服务器断开: %s\n", servers[i].remoteIP().toString().c_str());
      servers[i].stop();
      serverUsed[i] = false;
    }
  }
}

// 把队列里积压的消息发给所有已连接的服务器
static void drainQueue() {
  char buf[TX_QUEUE_MSG_LEN];
  while (xQueueReceive(txQueue, buf, 0) == pdTRUE) {
    for (int i = 0; i < MAX_SERVERS; i++) {
      if (serverUsed[i] && servers[i].connected()) {
        servers[i].print(buf);
        servers[i].print("\r\n");
      }
    }
  }
}

// ==================== 后台网络任务 ====================
// 职责：1) 尽快把主循环排队的消息发出去；2) 逐 IP 扫描网段发现服务器。
// 扫描用“游标”状态机，每次任务循环只探测一个 IP，避免长时间占用任务；
// 所有网络 I/O 都在本任务内完成，主循环（电机控制）不受阻塞影响。
static int scanCursor = 0;               // 0=空闲；2~254=正在探测该末位
static unsigned long nextScanDue = 0;

static void networkTask(void* arg) {
  nextScanDue = millis() + FIRST_SCAN_DELAY_MS;
  for (;;) {
    drainQueue();  // 优先处理主循环发来的消息

    if (scanCursor == 0) {
      if (millis() >= nextScanDue) {
        dropDeadServers();
        scanCursor = 2;  // 从 .2 开始（.1 是 ESP32 自己）
      }
    } else {
      IPAddress ip = WiFi.softAPIP();  // 例如 192.168.4.1
      ip[3] = (uint8_t)scanCursor;

      if (!addrInList(ip) && countServers() < MAX_SERVERS) {
        for (int i = 0; i < MAX_SERVERS; i++) {
          if (!serverUsed[i]) {
            servers[i].stop();
            if (servers[i].connect(ip, SERVER_PORT, PROBE_TIMEOUT_MS)) {
              serverUsed[i] = true;
              Serial.printf("已连接服务器: %s\n", ip.toString().c_str());
            }
            break;  // 每个 IP 只尝试一个空闲槽位
          }
        }
      }

      scanCursor++;
      if (scanCursor > SCAN_IP_END) {
        scanCursor = 0;
        // 一台都没连上时快速重扫，方便首次连接；连上后再放宽扫描间隔
        nextScanDue = millis() + (countServers() > 0 ? SCAN_INTERVAL_MS : SCAN_RETRY_FAST_MS);
        Serial.printf("本轮扫描完成，当前连接 %d 台服务器\n", countServers());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== 对外接口 ====================
void wifi_tcp_init() {
  // 上电先开 AP 热点：只开热点就返回，不阻塞电机启动
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("AP 启动失败！");
  }
  Serial.println("WiFi AP 已启动");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP:   ");
  Serial.println(WiFi.softAPIP());

  // 启动后台任务：扫描网段 + 连接所有服务器 + 发送消息
  txQueue = xQueueCreate(TX_QUEUE_LEN, TX_QUEUE_MSG_LEN);
  if (txQueue != nullptr) {
    xTaskCreatePinnedToCore(networkTask, "netTask", 6144, nullptr, 1, nullptr, 0);
    Serial.println("多服务器扫描任务已启动");
  }
}

void wifi_tcp_loop() {
  // 连接、扫描、发送全部由后台网络任务完成，这里无需额外处理
}

// 是否已连上至少一台服务器
bool tcp_host_connected() {
  return countServers() > 0;
}

// 向所有已连接的服务器发送一行文本（等效于原来的 Serial.println）
// msg 不需要带换行符，函数会自动补 \r\n
void sendToHost(const String& msg) {
  if (MIRROR_TO_SERIAL) {
    Serial.println(msg);
  }
  if (txQueue == nullptr) return;

  char buf[TX_QUEUE_MSG_LEN];
  int n = msg.length();
  if (n > TX_QUEUE_MSG_LEN - 1) n = TX_QUEUE_MSG_LEN - 1;
  memcpy(buf, msg.c_str(), n);
  buf[n] = '\0';
  // 队列满时丢弃新消息（状态消息，丢一条无妨，避免阻塞主循环）
  xQueueSend(txQueue, buf, 0);
}
