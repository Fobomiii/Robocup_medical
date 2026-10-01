# ESP32-S3 生命体征终端

`ESP32.ino` 通过 UART 读取血氧模块的连续 PPG、血氧和脉率，通过另一路 UART
读取 GY614 体温，并通过 BLE GATT 向 NUC 发送心率和体温。SH1106 OLED 会以
`H:心率  T:体温` 的格式显示测量值，并显示连续滚动的 PPG 曲线；血氧值不在界面显示。

## 接线

| 设备 | ESP32-S3 引脚 |
| --- | --- |
| SH1106 SDA / SCL | GPIO16 / GPIO15 |
| 血氧模块 TX / RX | GPIO42 / GPIO41 |
| GY614 TX / RX | GPIO6 / GPIO7 |
| 测温按钮 | GPIO39，按下接地 |
| 有源蜂鸣器 | GPIO38 |

血氧模块的 TX 接 ESP32 RX（GPIO42），血氧模块的 RX 接 ESP32 TX（GPIO41），
串口参数为 `115200 8N1`。固件上电发送 `AT+MD:0`，进入连续数据模式：标准
`U1:<raw>` 和部分新版模块的无逗号 `U2:<raw>` 会作为 PPG 原始值；
`U2:<血氧>,<脉率>,<灌注指数>` 用于更新测量结果；仅输出 `S1:<raw>` 的模块也可兼容。
不同标签的原始值不会混入同一段曲线，避免量程突变。

OLED 对 PPG 做轻度 EMA 平滑，然后根据最近128点的幅值范围自动缩放并逐点连线，
不会再显示原 GPIO8 输入产生的高低电平方波。3秒收不到 PPG 时显示 `PPG WAIT`，
并自动重发模式0命令。

GY614 的 TX 接 ESP32 RX（GPIO6），GY614 的 RX 接 ESP32 TX（GPIO7）。如果实物
已经使用其他引脚，只修改 `ESP32.ino` 顶部对应的 RX/TX 引脚定义。

按下 GPIO39 按钮后开始采集7个新的 GY614 数据帧。程序以中位数为
中心剔除偏差超过 `0.5°C` 的异常点，再对剩余数据求平均，结果同时更新到 OLED 和
NUC，并统一显示两位小数。测量成功后蜂鸣器连续鸣叫200ms；5秒内无法获得足够有效帧时执行
`100ms响 → 100ms停 → 100ms响 → 停止`。测量期间不会向 NUC 发布未滤波温度，
超时时保持温度无效，避免显示旧数据。

按住按钮超过 `800ms` 会取消上述7帧批量测量并进入实时模式。OLED 使用 `T*` 标识，
每收到一个有效 GY614 新帧便同步更新 OLED 和 NUC；松开按钮后退出实时模式并保留
最后一次有效温度。实时模式进入、退出均不触发蜂鸣器。

## Arduino 设置

- 开发板包：`esp32 by Espressif Systems`
- 开发板：`ESP32S3 Dev Module`
- N16R8：Flash Size 选择 `16MB`，PSRAM 选择 `OPI PSRAM`
- 外部库：`Adafruit GFX Library`、`Adafruit SH110X`
- BLE 库由 ESP32 Arduino Core 自带

设备广播名为 `MedicalVitals-S3`。这是 BLE GATT，不是经典蓝牙串口；NUC 不需要
人工输入配对码，后台节点会自动扫描、连接并在断线后重连。

## NUC验证

一键部署会安装并启动 BlueZ/Bleak，同时启动 `health_ble_bridge`。ESP32 上电后执行：

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash

bluetoothctl scan on
ros2 node list | grep health_ble_bridge
ros2 topic echo /medical_nav/health_status
```

正常状态中应出现 `"connected":true`、`heart_rate_bpm` 和 `temperature_c`。超过
2秒没有新数据，界面会自动恢复为 `-- bpm` 和 `-- °C`，不会保留过期测量值。

如果赛场有多台同名设备，可先用 `bluetoothctl devices` 查询地址，然后在 Windows
部署前设置 `MEDICAL_HEALTH_BLE_DEVICE_ADDRESS`；留空时按设备名自动发现。
