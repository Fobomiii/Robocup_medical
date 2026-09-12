# NUC通信迁移说明

## 概述

本次修改将原有的 GY614（温度传感器）和 MAX30102（心率传感器）模块从 UART8 和 UART4 移除，并将 UART8 重新配置用于与 NUC 通信，接收激光雷达障碍物检测数据。

## 主要修改

### 1. 删除的文件
- `Program/Core/Src/GY614.c` - GY614温度传感器驱动
- `Program/Core/Inc/GY614.h`
- `Program/Core/Src/MAX30102.c` - MAX30102心率传感器驱动
- `Program/Core/Inc/MAX30102.h`

### 2. 新增的文件
- `Program/Core/Src/NUC_Obstacle.c` - NUC障碍物数据接收模块
- `Program/Core/Inc/NUC_Obstacle.h`

### 3. 修改的文件

#### main.c
- 移除了 `#include "MAX30102.h"` 和 `#include "GY614.h"`
- 添加了 `#include "NUC_Obstacle.h"`
- 移除了 `MAX30102_Init()` 和 `GY614_Init()` 初始化调用
- 添加了 `NUC_Obstacle_Init()` 初始化调用
- 移除了 `defaultTask` 中的 `MAX30102_Update()` 心率更新代码
- 简化了 `oledTask` 显示内容（移除温度和心率显示）；OLED 始终显示最近一次合法 NUC 帧的 `X/Y` 坐标，无新帧时保留上一帧
- **UART8 波特率从 9600 改为 115200**

#### OPS.c
- 移除了 `#include "MAX30102.h"` 和 `#include "GY614.h"`
- 添加了 `#include "NUC_Obstacle.h"`
- `HAL_UART_RxCpltCallback()` 中移除了 UART4 和原 UART8 的中断处理
- 添加了新的 UART8 中断处理，调用 `NUC_Obstacle_OnUartRxCplt()`

#### Program.uvprojx（Keil项目文件）
- 移除了 `MAX30102.c` 和 `GY614.c` 的编译条目
- 添加了 `NUC_Obstacle.c` 的编译条目

## UART8 配置变更

### 旧配置（GY614温度传感器）
- 波特率：9600
- 数据位：8
- 停止位：1
- 校验位：无
- 功能：接收红外温度计数据

### 新配置（NUC通信）
- 波特率：**115200**
- 数据位：8
- 停止位：1
- 校验位：无
- 功能：接收激光雷达障碍物坐标

## NUC通信协议

### 数据帧格式（8字节）
```
AA BB | XH XL | YH YL | CHK | 55
```

| 字段 | 长度 | 说明 |
|------|------|------|
| AA BB | 2字节 | 帧头 |
| XH XL | 2字节 | 前向距离，int16大端序，单位mm（正=前方）|
| YH YL | 2字节 | 横向偏移，int16大端序，单位mm（正=左，负=右）|
| CHK | 1字节 | 校验和，(XH+XL+YH+YL) & 0xFF |
| 55 | 1字节 | 帧尾 |

### 坐标系
- **X轴**：前向距离，正值表示前方，单位毫米（mm）
- **Y轴**：横向偏移，正值表示左侧，负值表示右侧，单位毫米（mm）
- **检测范围**：X = 400–500mm，|Y| < 300mm

### 发送时机
- 按点云回调周期发送帧，频率约 20Hz
- 检测到障碍物时发送其坐标；无障碍物或过滤失败时发送 `X=0,Y=0`

修改 NUC 端代码后，需要在 NUC 工作空间重新构建并安装节点，systemd 才会使用新的零坐标心跳逻辑：
```bash
cd ~/livox_ws
colcon build --packages-select obstacle_detector --symlink-install
sudo systemctl restart obstacle-detector.service
```

## API 使用说明

### 初始化
```c
#include "NUC_Obstacle.h"

// 在 main() 中调用
NUC_Obstacle_Init();

// 启动时验证 NUC 双向通信；未收到回复时持续等待
while (!NUC_IsOnline())
{
    NUC_Obstacle_SendPing();
    HAL_Delay(100);
}
```

握手帧为 `A5 5A 01 00`（STM32 -> NUC）和 `A5 5A 01 01`（NUC -> STM32）。NUC 端在收到探测帧后立即回复，不依赖是否检测到障碍物。运行中可用 `NUC_IsOnline()` 查询握手是否成功，也可用 `NUC_Obstacle_SendPing()` 手动发送一次探测。若需要有限时等待，也可调用 `NUC_Obstacle_WaitForOnline(timeout_ms)`。

### 读取障碍物数据
```c
// 获取前向距离（mm）
int16_t x = NUC_GetObstacleX();

// 获取横向偏移（mm）
int16_t y = NUC_GetObstacleY();

// 检查数据是否有效（200ms超时）
if (NUC_IsObstacleValid())
{
    // 数据有效，可以使用 x 和 y
}
else
{
    // 数据超时或无效
}
```

### 数据有效性
- 如果超过 **200ms** 未收到新数据，`NUC_IsObstacleValid()` 返回 0
- `NUC_GetObstacleX()` 和 `NUC_GetObstacleY()` 在数据无效时返回 0

## NUC端代码位置

NUC端的障碍物检测代码位于：
```
NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/obstacle_node.py
```

**注意**：根据项目记忆文件，`obstacle_node.py` 不属于本项目的 STM32 代码部分，仅供参考。

## 注意事项

1. **UART8 引脚**：PE0=RX, PE1=TX（硬件连接需确认）
2. **波特率匹配**：STM32端和NUC端都必须使用 115200 波特率
3. **数据超时**：200ms无数据则认为障碍物数据失效
4. **大小端序**：X和Y坐标使用大端序（Big-Endian）编码
5. **中断优先级**：确保 UART8 中断优先级配置正确

## 测试建议

1. 使用串口调试助手验证 UART8 能正常接收数据
2. 发送测试帧验证解析是否正确：
   ```
   AA BB 01 F4 00 64 59 55
   // X = 500mm (0x01F4)
   // Y = 100mm (0x0064)
   // CHK = 0x59
   ```
3. 检查 `NUC_IsObstacleValid()` 在超时后是否正确返回 0
4. 验证校验和计算是否正确拒绝错误帧

## 回滚方案

如需恢复原有的温度和心率传感器功能：
1. 从Git历史恢复删除的文件
2. 撤销 main.c 和 OPS.c 的修改
3. 将 UART8 波特率改回 9600
4. 更新 Program.uvprojx 项目文件

## 编译注意

重新打开 Keil MDK 项目后，需要：
1. 确认 `NUC_Obstacle.c` 已添加到编译列表
2. 确认 `GY614.c` 和 `MAX30102.c` 已移除
3. 重新编译整个项目
