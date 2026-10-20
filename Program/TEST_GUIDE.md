# 测试指南 - NUC 与 STM32 UART8 通信

## 硬件连接

**UART8 (STM32H743):**
- TX: PE1 → NUC RX
- RX: PE0 ← NUC TX
- GND: 共地
- 波特率: 115200

## 测试步骤

### 1. STM32 端验证

程序已编译并烧录到芯片，包含以下功能：
- UART8 @ 115200 初始化
- 8 字节帧接收：`AA BB XH XL YH YL CHK 55`
- 200ms 超时检测

**API:**
```c
int16_t x = NUC_GetObstacleX();      // 前向距离 mm
int16_t y = NUC_GetObstacleY();      // 横向偏移 mm
uint8_t valid = NUC_IsObstacleValid(); // 数据有效性
```

### 2. NUC 端确认

NUC 端代码位于：`NUC_MID360/2_ROS2包_obstacle_detector/`

**启动 ROS2 节点:**
```bash
cd NUC_MID360/2_ROS2包_obstacle_detector
ros2 run obstacle_detector obstacle_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

**检查串口连接:**
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
# 确认设备存在并有读写权限
sudo chmod 666 /dev/ttyUSB0  # 如需要
```

### 3. 数据监控（可选）

使用监控脚本查看 UART8 数据：

```bash
cd Program
python test_uart_monitor.py
```

脚本会：
- 列出可用串口
- 实时解析 NUC 数据帧
- 显示 X/Y 坐标和校验状态

**预期输出:**
```
[0001] ✓ X=  450mm  Y= -120mm  [aa bb 01 c2 ff 88 4b 55]
[0002] ✓ X=  455mm  Y= -115mm  [aa bb 01 c7 ff 8d 50 55]
```

### 4. 故障排查

**UART8 无数据:**
1. 检查硬件连接（TX/RX 交叉，共地）
2. 万用表测量电平（3.3V TTL）
3. NUC 端检查串口设备存在性
4. 逻辑分析仪抓取波形

**校验错误:**
- 检查波特率是否匹配（115200）
- 检查接线是否有干扰
- 确认 NUC 发送格式正确

**超时失效:**
- NUC 端是否正常运行
- 激光雷达是否检测到障碍物
- 检查 `send_interval` 参数（默认 50ms）

## 数据协议细节

**发送方 (NUC):**
```python
x_mm = int(round(x * 1000))  # 米转毫米
y_mm = int(round(y * 1000))
payload = struct.pack(">hh", x_mm, y_mm)  # 大端序
checksum = sum(payload) & 0xFF
frame = [0xAA, 0xBB] + payload + [checksum, 0x55]
```

**接收方 (STM32):**
```c
int16_t x = (rx_buf[2] << 8) | rx_buf[3];  // 大端序
int16_t y = (rx_buf[4] << 8) | rx_buf[5];
uint8_t chk_calc = rx_buf[2] + rx_buf[3] + rx_buf[4] + rx_buf[5];
if (chk_calc == rx_buf[6]) { /* 数据有效 */ }
```

## 集成到导航系统

在 `Navigation.c` 中调用：

```c
#include "NUC_Obstacle.h"

void Navigation_Task(void) {
    if (NUC_IsObstacleValid()) {
        int16_t obstacle_x = NUC_GetObstacleX();  // mm
        int16_t obstacle_y = NUC_GetObstacleY();  // mm
        
        // 避障逻辑
        if (obstacle_x < 500 && abs(obstacle_y) < 200) {
            // 检测到前方 500mm 内有障碍物
            // 执行避障动作
        }
    }
}
```

## 已删除的模块

- ✗ GY614 (红外温度) - 原 UART8 @ 9600
- ✗ MAX30102 (心率) - 原 UART4 @ 57600

相关代码已从工程中移除。
