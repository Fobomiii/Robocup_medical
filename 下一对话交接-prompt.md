# RoboCup 医疗机器人 — 下一对话交接 Prompt

将下面整段复制到新对话开头，便于 AI 快速接手本工程。

---

## 复制起点

```
你是接续开发 RoboCup 医疗机器人 STM32 固件的助手。请先阅读工作区根目录 `工程概述.md`、`工程资源占用.md`，工程在 `Program/`（STM32H743IIT6 + Keil MDK-ARM + FreeRTOS）。

### 硬件平台
- MCU：STM32H743IIT6，LQFP176，400 MHz，HSE 25 MHz
- 上位/感知：RDK X5（Mid-360 避障、后续摄像头识二维码）；与 MCU 规划 **USART2 UART**，不用 CAN 传 Mid-360 摘要
- STP23L ×3：规划 MCU **直连 3 路 UART**（不用官方 USB/CAN 转换板接 MCU）

### 已完成功能（驱动在 Program/Core/Src）
| 模块 | 接口 | API/变量 |
|------|------|----------|
| 底盘/臂电机 | FDCAN1 PB8/9 1Mbps | `DJI_Chassis_SetCommand`、`DJI_Arm_CtrlAngle` |
| OPS 全场定位 | USART3 PC10/11 115200 | `pos_x/y`、`OPS_Task`（XY 场坐标） |
| HWT101CT | USART1 PA9/10 **115200** DMA+IDLE | `hwt_zangle`、`hwt_online`、`HWT101_GetYaw()`（航向，°） |
| GM65 | UART7 PE8/7 9600 DMA+IDLE | `GM65_GetLastCode()` |
| MAX30102 | UART4 PD0/1 57600 | `getBPM()`、`MAX30102_Update()` @1Hz |
| GY-614 | UART8 PE0/1 9600 | `getTemp()` |
| 舵机×2 | TIM3 PA6/7 50Hz PWM | `SERVO1_ANGLE` / `SERVO2_ANGLE` |
| **位姿 PID** | C++ `PID.cpp` | `PID_SetX/Y/Z`、`PID_UpdateX/Y/Z`、`PID_Reset`、`PID_ResetXY` |
| **底盘外环** | C `ChassisCtrl.c` | `ChassisCtrl_Enable`、`ChassisCtrl_MoveTarget`、`ChassisCtrl_Update` |

### 底盘控制分层（重要）
- **外环** `ChassisCtrl`：OPS 位置 + HWT101 航向 → 位姿 PID + 梯形速度规划 → `DJI_Chassis_SetCommand(Vx,Vy,W)`
- **内环** `DJIMotorCtrlSTM32`：`chassisTask` 1kHz 电机速度环（3508）
- 坐标：目标/反馈 X/Y = OPS 场坐标；航向 Z = **`hwt_zangle`（HWT101）**，不是 OPS 的 `zangle`
- 场→体旋转：`cos_f32/sin_f32 = cos/sin(pos_z * π/180)`，在 `ChassisCtrl_Update` 每帧计算

### ChassisCtrl 算法（已实现，参考 RC_Old `Location.c`）
**文件**：`Program/Core/Src/ChassisCtrl.c`、`ChassisCtrl.h`、`PID.cpp`、`PID.h`

**调用流程**：
1. `ChassisCtrl_Enable(1)` → `ChassisCtrl_MoveTarget(x,y,z, pos_x,pos_y,pos_z)` → 循环 `ChassisCtrl_Update(pos_x,pos_y,hwt_zangle)` @ **100Hz**（`osDelay(10)`）
2. `MoveTarget` 时算一次 `target_distance` 和场方向 `(cosf_Vx, sinf_Vy)`；复位 `result_vel`、`velPIDCtrlFlag`、`PID_Reset()`

**分段逻辑**（`long_distance_thd = 350`）：
| 条件 | 行为 |
|------|------|
| `target_distance ≤ 350` | 短距离，直接 `velPIDCtrlFlag=1` 纯 PID |
| `remain > 350` | 加速：`result_vel += 10`（每帧），沿固定场方向 |
| `remain ≤ 350` 且 `result_vel > \|PID\|` | 减速：`result_vel -= 10`，方向 `(cosf_Vx,sinf_Vy)` 向 PID 单位方向 ±`angle_step`(0.01) 渐变 |
| `result_vel ≤ \|PID\|` | `velPIDCtrlFlag=1`，切纯 PID |

**关键变量**：
- `velPIDCtrlFlag`：锁存，**不在每帧清 0**；`MoveTarget`/到位时清 0
- 梯形段（`velPIDCtrlFlag==0`）：每帧 `PID_ResetXY()` 防 XY 积分 windup；**Z 不受影响**
- `W = newW` 全程独立航向闭环，与 XY 梯形无关
- **无 HAL_GetTick 时间戳**：步进绑在调用频率（100Hz），快慢调 `result_vel` 步长(10)、`max_vel`(200)、`angle_step`(0.01)

**到达判断**：`|Δx|<2`、`|Δy|<2`、`|Δz|<0.5°` → 停车、`PID_Reset()`、返回 1

**减速方向衔接 trade-off**（已讨论，实车调参）：
- `angle_step` 小：刚进减速若角度差大可能横甩；handoff 进纯 PID 更顺
- `angle_step` 大：进减速更顺；handoff 时可能剩一步方向差而抖
- 备选优化：首次减速限速 / 按角度限速 / handoff 加方向对齐条件 / 末段直接 `cosf_Vx=cos_pid`

### main 初始化顺序（要点）
`MX_GPIO/DMA/FDCAN1/2` → `MX_I2C2_Init`（Generate 后）→ `USART1/3` → `UART4/7/8` → `SERVO_Init()` → `HWT101_Init()` →（可选）等 `HWT101_IsOnline()` 后 `HWT101_CaliYaw()` → 其他传感器 Init → **`PID_SetX/Y/Z(...)`**（待加）→ `osKernelStart()`。

HWT101 等待在线循环当前可能被注释（仅测其它模块时）。

### 关键实现细节（避免重复踩坑）
1. **舵机 TIM3**：`Servo.c` 在 `MX_TIM3_Init()` 里调用 `tim3_hw_init()` 显式 `__HAL_RCC_TIM3_CLK_ENABLE()` + PA6/PA7 AF2；**不要**只依赖 `HAL_TIM_MspInit`（曾被 Keil 链接器删掉，导致 TIM3 寄存器全 0）。
2. **UART MSP**：USART1/3/4/7/8 等 MSP 在 `stm32h7xx_hal_msp.c`；勿在传感器 .c 里重复 `HAL_UART_MspInit`（已清理 OPS 等重复项）。
3. **HWT 航向**：闭环用 `hwt_zangle` / `HWT101_GetYaw()`，不是 OPS 的 `zangle`。
4. **MAX30102**：`defaultTask` 里 `MAX30102_Update()` 会阻塞；单测时可注释。
5. **ChassisCtrl 与 canHostTask 冲突**：`main.c` 里 `canHostTask` 当前有测试代码 `DJI_Chassis_SetCommand(10,0,0)`，接入 `ChassisCtrl_Update` 前须删掉或改占位，否则与底盘外环抢指令。
6. **Keil 工程**：`PID.cpp` 已在 `Program.uvprojx`；**`ChassisCtrl.c` 尚未加入工程**，需手动 Add。

### CubeMX Program.ioc 已配置但固件未实现
请在 Generate Code 后合并 USER CODE，勿覆盖已有驱动：

| 功能 | 引脚/外设 |
|------|-----------|
| 按钮 A/B/C | PB11 / PB10 / PH8，GPIO 输入上拉 |
| FDCAN2 保留 | PB12 RX / PB13 TX，1Mbps Classic |
| RDK X5 | **USART2** PD5 TX / PD6 RX，115200 |
| STP23L A | USART6 PC6 TX / PC7 RX，115200 |
| STP23L B | UART5 PC12 TX / PD2 RX，115200 |
| STP23L C | **LPUART1** PB6 TX / PB7 RX，115200（HAL_LPUART） |
| OLED | **I2C2** PH4 SCL / PH5 SDA，400 kHz |
| 蜂鸣器 | **PE9** GPIO 输出，标签 `BUZZER`，默认低电平关 |

### FreeRTOS 任务
- `chassisTask` 1kHz → `DJI_Motor_ChassisTask()`（内环电机速度）
- `armTask` 臂
- `OPSUartTask` → `OPS_Task()`
- `defaultTask` MAX30102 1Hz（可改）
- `canHostTask` **当前有测试 SetCommand，应改**；可放 `ChassisCtrl_Update` @100Hz 或独立 `navTask`
- `oledTask`：SSD1309（`OLED_SSD1309`，I2C2）

### 用户架构决策（请遵守）
- Mid-360 在 X5 处理；MCU 只收压缩障碍物消息，UART 带宽足够，**不必为 RDK 上 CAN**。
- FDCAN2 引脚保留，当前 RDK 走 USART2。
- STP23L 三路独立 UART，不用转换板 USB 接 MCU。
- OPS 保持字节中断即可，不必强行改 DMA。
- 底盘外环与内环分离：`ChassisCtrl` 发 Vx/Vy/W，`DJIMotorCtrlSTM32` 做电机速度 PID。

### 建议下一步（按优先级）
1. **接入底盘外环**：Keil 加入 `ChassisCtrl.c`；`main` 或新任务 100Hz 调 `ChassisCtrl_Update(OPS_x, OPS_y, hwt_zangle)`；初始化 `PID_SetX/Y/Z`
2. 删掉/改掉 `canHostTask` 里测试 `DJI_Chassis_SetCommand(10,0,0)`
3. CubeMX Generate Code → 合并 `main.c` / MSP / IT（注意与已有驱动冲突）
4. `Button.c`、`RDK_Link.c`（USART2）、`STP23L.c`、`Buzzer.c`
5. 实车调参：`PID` 增益、`max_vel`、`result_vel` 步长、`angle_step`、到达阈值
6. 可选：handoff 加方向对齐条件；`large_angle_thd` 航向加减速（已注释）

### 文档路径
- `工程概述.md` — 模块与任务总览
- `工程资源占用.md` — 引脚/串口/CAN/DMA 全表
- 参考：`RC_Old` 主机 `Location.c`（梯形+减速方向衔接）
- 工程：`c:\Users\ASUS\Desktop\Robocup医疗\Program\`

请用中文回复。改代码前先读相关 .c/.h。只在我要求时 git commit。
```

---

## 复制终点

---

## 本地备注（不必发给 AI）

- **2026-08-28**：`ChassisCtrl.c` + `PID.cpp` 外环位姿闭环主体完成；`PID_ResetXY` 梯形段只清 XY 积分；无时间戳、100Hz 步进；`angle_step` trade-off 已讨论。
- **待接**：`ChassisCtrl.c` 进 Keil、`PID_Set` 初始化、100Hz 任务调用、`canHostTask` 测试代码冲突。
- 上次会话已验证：HWT101 读角、舵机 PWM（`tim3_hw_init`）正常。
- `Program.ioc` 外设引脚已配置；若未 Generate Code，`main.c` 可能缺 `MX_I2C2_Init` / `MX_USART2` 等。
- 编译曾 0 Error（接入 ChassisCtrl 前）；改工程后需再编一次。
