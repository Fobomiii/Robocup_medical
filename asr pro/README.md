# ASR Pro 医疗送药车语音工程

## 功能

- ASR Pro 上电保持静音，等待 STM32 串口命令。
- 收到 `AA 55 01 FE` 后播报“1床病人请取药”。
- 收到 `AA 55 03 FC` 后播报“3床病人请取药”。
- 串口为 `Serial`：PB5 TX、PB6 RX、9600 8N1。

## 编译并烧录

用天问 Block AI 版打开 `niulaipro.hd`，编译并烧录。当前工程已删除欢迎语和 setup 中的主动播放块，模块自身上电不会播音。

STM32 初始化 UART4 后等待 ASR Pro 启动 3 秒，发送 1 床命令；再等待 3 秒发送 3 床命令，用于验证串口和两条播报音。该测试不依赖 OPS、HWT101 或其他传感器在线。

`flash_asr_pro.cmd` 和自定义音频脚本会重新注入 `niulai.wav`，本次静音开机测试不要运行这些脚本。

PB5/PB6 也是 ASR Pro 的程序下载串口。烧录 ASR Pro 时先断开 `STM32 PD1 -> ASR PB6`，烧录完成后再接回，避免 STM32 TX 与下载器 TX 同时驱动 ASR RX。

目录中的 `niulai.wav` 和相关脚本暂时保留，后续需要恢复自定义开机音时仍可使用。

## 接线

| ASR Pro | STM32H743 | 说明 |
|---|---|---|
| PB5 TX | PD0 UART4 RX | ASR 发、STM32 收（当前协议不依赖此方向） |
| PB6 RX | PD1 UART4 TX | STM32 向 ASR 发送播报命令 |
| GND | GND | 必须共地 |

两端均使用 3.3 V TTL 电平，不要接 RS-232 电平。
