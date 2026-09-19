# ASR Pro 医疗送药车语音工程

## 功能

- ASR Pro 上电播放 `niulai.wav`。
- 收到 `AA 55 01 FE` 后播报“1床病人请取药”。
- 收到 `AA 55 03 FC` 后播报“3床病人请取药”。
- 串口为 `Serial`：PB5 TX、PB6 RX、9600 8N1。

## 准备并直接烧录

首次使用时，用天问 Block AI 版打开 `niulaipro.hd` 并编译一次，让软件生成程序和两条床位播报音。需要保留完整自定义开机音时，直接双击：

```text
flash_asr_pro.cmd
```

工具始终读取同目录的原始 `niulai.wav`，不会使用文字合成替代。它会转换完整音轨、解码检查时长，然后打开官方 ASR Pro 烧录器。选择下载串口并开始烧录即可。天问离线工程把语音资源合并与烧录绑定在官方工具中，不会预先导出包含自定义音频的独立 BIN。

若后来在天问 Block 中修改程序，需要先在天问 Block 中编译一次，再双击 `flash_asr_pro.cmd`。打开烧录器后不要再次点击天问 Block 的编译按钮，否则自定义 MP3 可能被文字合成结果覆盖。

PB5/PB6 也是 ASR Pro 的程序下载串口。烧录 ASR Pro 时先断开 `STM32 PD1 -> ASR PB6`，烧录完成后再接回，避免 STM32 TX 与下载器 TX 同时驱动 ASR RX。

天问 Block 的 `.hd` 文件不能直接嵌入 WAV。工具只把 `niulai.wav` 转成芯片所需的 16 kHz、16 kbps MP3，原始声音、背景音乐和完整时长均保留；源 WAV 文件不会被修改。MP3 是有损编码，因此二进制波形不会与 WAV 完全相同，但不会变成合成语音。上电欢迎音使用 ID `10001`。

## 接线

| ASR Pro | STM32H743 | 说明 |
|---|---|---|
| PB5 TX | PD0 UART4 RX | ASR 发、STM32 收（当前协议不依赖此方向） |
| PB6 RX | PD1 UART4 TX | STM32 向 ASR 发送播报命令 |
| GND | GND | 必须共地 |

两端均使用 3.3 V TTL 电平，不要接 RS-232 电平。
