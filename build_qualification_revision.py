from __future__ import annotations

import subprocess
from pathlib import Path

import fitz


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "资格认证.pdf"
OUTPUT = ROOT / "资格认证_修订版.pdf"
WORK = ROOT / "_qualification_revision_text"
HTML = WORK / "revision-pages.html"
CHROME = Path(r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe")


HTML_CONTENT = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<style>
@page { size: Letter; margin: 0; }
* { box-sizing: border-box; }
html, body { margin: 0; padding: 0; background: #fff; }
body { font-family: "SimSun", "宋体", "Microsoft YaHei", Arial, sans-serif; color: #26343e; }
.page { width: 8.5in; height: 11in; padding: 0.45in 0.58in 0.35in; page-break-after: always; position: relative; overflow: hidden; }
.page:last-child { page-break-after: auto; }
.topbar { height: 0.12in; background: #147080; margin: -0.45in -0.58in 0.3in; }
.kicker { color: #147080; font-family: Arial, sans-serif; font-weight: 700; font-size: 21pt; margin-bottom: 0.08in; }
h1 { font-family: "Microsoft YaHei", "SimHei", sans-serif; font-size: 31pt; line-height: 1.15; margin: 0; color: #182832; }
.subtitle { color: #5d6f7c; font-family: "Microsoft YaHei", sans-serif; font-size: 14pt; margin-top: 0.08in; padding-bottom: 0.18in; border-bottom: 1px solid #cbd7dc; }
h2 { font-family: "Microsoft YaHei", "SimHei", sans-serif; font-size: 19pt; line-height: 1.1; margin: 0.22in 0 0.1in; padding-left: 0.14in; border-left: 0.08in solid #147080; color: #243943; }
.panel { border: 1px solid #b8cbd1; border-radius: 9px; background: #f3f8f9; padding: 0.16in 0.2in; margin-bottom: 0.12in; }
.panel.gray { background: #fafbfc; border-color: #c6d0d6; }
.panel.blue { background: #f1f7fb; border-color: #adc8dc; }
p { font-size: 12pt; line-height: 1.5; margin: 0 0 0.12in; }
p:last-child { margin-bottom: 0; }
.label { color: #147080; font-family: "Microsoft YaHei", "SimHei", sans-serif; font-weight: 700; font-size: 12.3pt; white-space: nowrap; }
table { width: 100%; border-collapse: collapse; font-size: 10.5pt; line-height: 1.3; }
th { text-align: left; color: #147080; font-family: "Microsoft YaHei", "SimHei", sans-serif; font-size: 11.2pt; padding: 0.07in 0.04in; border-bottom: 1px solid #b8cbd1; }
td { padding: 0.055in 0.04in; border-bottom: 1px solid #e0e6e8; vertical-align: top; }
.iface td:first-child { width: 1.28in; color: #147080; font-weight: 700; font-family: "Microsoft YaHei", "SimHei", sans-serif; }
.step { display: grid; grid-template-columns: 0.42in 0.58in 1fr; gap: 0.1in; align-items: start; margin: 0.08in 0; }
.num { width: 0.34in; height: 0.34in; border-radius: 50%; background: #147080; color: white; font: 700 10pt Arial, sans-serif; text-align: center; padding-top: 0.08in; }
.step strong { font-family: "Microsoft YaHei", "SimHei", sans-serif; font-size: 12pt; color: #245065; }
.step span { font-size: 10.8pt; line-height: 1.42; }
.flow { display: grid; grid-template-columns: 1.27in 0.26in 1.27in 0.26in 1.27in 0.26in 1.38in; gap: 0.06in; align-items: center; margin-bottom: 0.12in; }
.node { min-height: 0.74in; border-radius: 8px; color: white; text-align: center; padding: 0.11in 0.04in; font-family: Arial, "Microsoft YaHei", sans-serif; font-weight: 700; font-size: 10pt; line-height: 1.35; }
.node.teal { background: #147080; } .node.blue { background: #347c99; } .node.green { background: #467a6d; } .node.purple { background: #706499; font-size: 8.5pt; }
.arrow { color: #315462; font: 700 20pt Arial, sans-serif; text-align: center; }
.small { font-size: 10.2pt; line-height: 1.4; }
.safety { display: grid; grid-template-columns: 1.1in 1fr; gap: 0.07in 0.1in; font-size: 10.7pt; line-height: 1.42; margin-top: 0.13in; }
.safety strong { color: #b14d3d; font-family: "Microsoft YaHei", "SimHei", sans-serif; }
.foot { position: absolute; left: 0.58in; right: 0.58in; bottom: 0.18in; color: #6a7881; font-size: 8.5pt; }
.page:nth-of-type(2) h2 { font-size: 18pt; margin-top: 0.13in; margin-bottom: 0.06in; }
.page:nth-of-type(2) .panel { padding: 0.1in 0.16in; margin-bottom: 0.08in; }
.page:nth-of-type(2) p { font-size: 12pt; line-height: 1.5; margin-bottom: 0.07in; }
.page:nth-of-type(2) .small { font-size: 10.2pt; line-height: 1.4; }
.page:nth-of-type(2) .flow { margin-bottom: 0.07in; }
.page:nth-of-type(2) .node { min-height: 0.64in; padding-top: 0.08in; font-size: 9pt; }
.page:nth-of-type(2) .node.purple { font-size: 7.6pt; }
.page:nth-of-type(2) table { font-size: 10.2pt; line-height: 1.25; }
.page:nth-of-type(2) th { font-size: 10.8pt; padding-top: 0.035in; padding-bottom: 0.035in; }
.page:nth-of-type(2) td { padding-top: 0.035in; padding-bottom: 0.035in; }
.page:nth-of-type(2) .safety { font-size: 10.5pt; line-height: 1.38; margin-top: 0.07in; }
.appendix h1 { font-size: 27pt; }
.appendix h2 { font-size: 17pt; margin-top: 0.15in; margin-bottom: 0.08in; }
.appendix p { font-size: 12pt; line-height: 1.5; margin-bottom: 0.08in; }
.appendix .panel { padding: 0.11in 0.16in; margin-bottom: 0.08in; }
.appendix table { font-size: 9.7pt; line-height: 1.25; }
.appendix th { font-size: 10.1pt; padding: 0.045in 0.04in; }
.appendix td { padding: 0.04in; }
.appendix .diagram { display: grid; grid-template-columns: 1fr 0.25in 1fr 0.25in 1fr; gap: 0.06in; align-items: center; margin: 0.08in 0 0.12in; }
.appendix .box { min-height: 0.58in; border: 1px solid #7ba5ad; background: #f1f7f8; border-radius: 6px; padding: 0.08in 0.06in; text-align: center; font-size: 10pt; line-height: 1.28; }
.appendix .arrow { font-size: 17pt; }
.appendix .code { font-family: "Consolas", "Courier New", monospace; font-size: 8.9pt; line-height: 1.35; white-space: pre-wrap; background: #f5f7f8; border: 1px solid #cbd6da; padding: 0.09in 0.12in; margin: 0.06in 0; }
.appendix .twocol { display: grid; grid-template-columns: 1fr 1fr; gap: 0.1in; }
.appendix .note { color: #5d6f7c; font-size: 9.4pt; }
</style>
</head>
<body>
<section class="page">
<div class="topbar"></div><div class="kicker">10</div>
<h1>医疗生命体征终端（ESP32-S3）</h1>
<div class="subtitle">手腕/手部心率与额头体温采集，面向机器人本体实时上报</div>

<h2>系统组成与测量原理</h2>
<div class="panel">
<p>主控：ESP32-S3。心率采用血氧模块的 PPG 光电容积脉搏波：传感器贴合手部/手腕，依据组织血容量随心搏变化产生的光学波动计算脉率；项目不使用“指透式 PPG”表述。模块同时输出血氧和灌注指数，供终端状态判断。</p>
<p>体温采用 GY614/GY615V3 非接触式红外体温模块，对准额头测量。按下 GPIO39 测温键后采集 7 帧新数据，以中位数为中心剔除偏差超过 0.5 °C 的异常帧，再对剩余帧求平均；25–45 °C 为有效范围，5 s 内不足有效帧则判无效。</p>
</div>

<h2>接口与人机交互</h2>
<div class="panel gray"><table class="iface">
<tr><td>血氧/PPG UART</td><td>115200 8N1；模块 TX→GPIO42，RX→GPIO41；上电发送 AT+MD:0，兼容 U1/U2/S1 原始波形及 U2:血氧,脉率,灌注指数。</td></tr>
<tr><td>GY614 UART</td><td>9600；模块 TX→GPIO6，RX→GPIO7；温度按批量 7 帧或长按 800 ms 进入实时模式。</td></tr>
<tr><td>显示与提示</td><td>SH1106 OLED：SDA GPIO16、SCL GPIO15，显示 H/T 数值和 PPG 波形；GPIO38 蜂鸣器提示测量成功或超时。</td></tr>
</table></div>

<h2>端侧数据处理与有效性</h2>
<div class="panel blue">
<div class="step"><div class="num">01</div><strong>采集</strong><span>UART 解析心率/血氧结果和连续 PPG；对 PPG 波形使用 EMA 平滑，仅使用当前数据源，避免切换时的波形突变。</span></div>
<div class="step"><div class="num">02</div><strong>判定</strong><span>心率 30–250 bpm、血氧 50–100% 作为模块结果有效性边界；超过 3 s 无新结果，心率/血氧标志清零。</span></div>
<div class="step"><div class="num">03</div><strong>温度</strong><span>7 帧中位数 + 0.5 °C 异常值剔除 + 平均值；未完成或超时不发布未滤波温度，避免机器人显示旧值。</span></div>
<div class="step"><div class="num">04</div><strong>输出</strong><span>OLED 显示心率和体温；BLE 每 200 ms 发布 14 字节生命体征包。</span></div>
</div>
<div class="foot">注：血氧模块结果在终端解析并用于状态判断；当前机器人 BLE 数据包按赛事任务需要上报心率和体温。</div>
</section>

<section class="page">
<div class="topbar"></div><div class="kicker">11</div>
<h1>生命体征通信与机器人安全闭环</h1>
<div class="subtitle">BLE GATT → NUC/ROS 2 → 机器人状态与仪表盘</div>

<h2>数据流与通信接口</h2>
<div class="panel">
<div class="flow">
<div class="node teal">ESP32-S3<br>MedicalVitals-S3</div><div class="arrow">→</div>
<div class="node blue">BLE GATT<br>Notify 200 ms</div><div class="arrow">→</div>
<div class="node green">NUC<br>health_ble_bridge</div><div class="arrow">→</div>
<div class="node purple">ROS 2<br>/medical_nav/<br>health_status</div>
</div>
<p class="small">NUC 自动扫描并在断线后重连；桥接节点校验版本、长度和 CRC16-CCITT；仪表盘显示心率和体温，ROS 话题供医疗任务订阅。</p>
</div>

<h2>BLE 14 字节数据包（小端序）</h2>
<div class="panel gray"><table>
<tr><th>字段</th><th>类型</th><th>字节数</th><th>含义</th><th>有效标志</th><th>校验</th></tr>
<tr><td>version</td><td>uint8</td><td>1</td><td>协议版本=1</td><td>—</td><td>—</td></tr>
<tr><td>sequence</td><td>uint16</td><td>2</td><td>递增序号</td><td>—</td><td>—</td></tr>
<tr><td>heart_rate_bpm</td><td>uint16</td><td>2</td><td>心率 bpm</td><td>bit0</td><td>—</td></tr>
<tr><td>temperature_centi</td><td>int16</td><td>2</td><td>温度×100 °C</td><td>bit1</td><td>—</td></tr>
<tr><td>flags</td><td>uint8</td><td>1</td><td>数据有效标志</td><td>bit0/bit1</td><td>—</td></tr>
<tr><td>uptime_ms</td><td>uint32</td><td>4</td><td>ESP32 运行时间</td><td>—</td><td>—</td></tr>
<tr><td>crc</td><td>uint16</td><td>2</td><td>CRC16-CCITT</td><td>—</td><td>前 12 字节</td></tr>
</table><p class="small">发送周期：200 ms。NUC 默认 2 s 未收到新包即将数据标记为过期，界面恢复为“-- bpm / -- °C”，不沿用旧测量值。</p></div>

<h2>机器人端任务与安全闭环</h2>
<div class="panel blue">
<p><strong>NUC / ROS 2 Humble + Nav2</strong></p>
<p>NUC 接收生命体征后与 Mid-360 环境感知、OPS9 位移、HWT101CT 航向、STP23L 精确靠站和 DECXIN/GM65 扫码链路并行工作。医疗任务状态机覆盖初始化、前往护士台、扫描任务码、前往 1/3 号床、扫描床头码、放药、返回起点和完成/错误状态。</p>
<div class="safety">
<strong>赛事测量要求</strong><span>心率在手部/手腕测量；体温对准额头测量；技术文档明确 PPG 与红外测温原理，并通过机器人本体通信。</span>
<strong>误差目标</strong><span>按赛事满分指标进行验证：心率误差 ±3 bpm；体温误差 ±0.5 °C。终端保留有效性标志和时间戳，便于复核。</span>
<strong>安全停车</strong><span>机器人保留启动按钮和急停按钮；NUC 紧急停止、OPS/HWT 掉线、四轮反馈不完整或关键通信异常时立即归零；合法速度命令约 600 ms 超时自动归零。</span>
</div></div>
<div class="foot">接口依据：ESP32/ESP32.ino、ESP32/README.md、NUC_MID360/health_ble_bridge.py、Program/Core/Src/MedicalTask.c。</div>
</section>

<section class="page appendix">
<div class="topbar"></div><div class="kicker">12</div>
<h1>附录 A：系统实现证据</h1>
<div class="subtitle">以工程文件为依据的硬件、软件与接口索引</div>

<h2>附录说明</h2>
<div class="panel">
<p>本附录把可复核的实现信息用文字和表格列出，便于技术委员会检索。图示使用本 PDF 内的排版元素，不依赖整页截图；原始工程文件路径与关键接口均在表中注明。</p>
<p class="note">主要依据：ESP32/ESP32.ino、ESP32/README.md、NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/health_ble_core.py、health_ble_bridge.py、NUC_MID360/README.md、Program/Core/Src/MedicalTask.c。</p>
</div>

<h2>上下位机分工</h2>
<div class="diagram">
<div class="box"><strong>ESP32-S3</strong><br>采集心率、血氧、体温<br>OLED 与蜂鸣器提示</div><div class="arrow">→</div>
<div class="box"><strong>BLE GATT</strong><br>14 字节结构化数据包<br>200 ms 通知</div><div class="arrow">→</div>
<div class="box"><strong>NUC / ROS 2</strong><br>health_ble_bridge<br>发布状态并驱动界面</div>
</div>
<div class="panel gray"><table>
<tr><th>层级</th><th>职责</th><th>可复核文件</th></tr>
<tr><td>生命体征端</td><td>UART 解析 PPG/血氧和 GY614；温度滤波；OLED 显示；BLE 通知。</td><td>ESP32/ESP32.ino、GY614.cpp、GY614.h</td></tr>
<tr><td>机器人上位机</td><td>扫描、连接、CRC 校验、过期清除；向 ROS 2 话题发布 JSON 状态。</td><td>health_ble_core.py、health_ble_bridge.py</td></tr>
<tr><td>机器人底层</td><td>启动按钮防抖、医疗任务状态机、扫码确认、靠站与放药时序。</td><td>Program/Core/Src/MedicalTask.c、MedicalTask.h</td></tr>
</table></div>

<h2>从采集到机器人显示</h2>
<div class="panel blue"><p>传感器输出先在 ESP32-S3 上完成有效性判断，再封装为 BLE 数据包。NUC 只接受长度、版本和 CRC 均正确的数据；超过 2 s 没有新包时，桥接节点发布空值，仪表盘显示“--”，不沿用旧测量结果。</p></div>
<div class="foot">附录 A / 文字型技术证据页</div>
</section>

<section class="page appendix">
<div class="topbar"></div><div class="kicker">13</div>
<h1>附录 B：硬件接口与测量链路</h1>
<div class="subtitle">ESP32-S3 生命体征终端的接线、采集和交互</div>

<h2>引脚与串口参数</h2>
<div class="panel gray"><table>
<tr><th>设备</th><th>接口</th><th>ESP32-S3 引脚</th><th>用途</th></tr>
<tr><td>血氧/PPG 模块</td><td>UART2，115200 8N1</td><td>TX→GPIO42；RX→GPIO41</td><td>连续 PPG；血氧、脉率、灌注指数结果</td></tr>
<tr><td>GY614/GY615V3</td><td>UART1，9600</td><td>TX→GPIO6；RX→GPIO7</td><td>额头非接触红外温度</td></tr>
<tr><td>SH1106 OLED</td><td>I2C，地址 0x3C</td><td>SDA GPIO16；SCL GPIO15</td><td>心率、温度、PPG 波形和测量状态</td></tr>
<tr><td>测温按钮 / 蜂鸣器</td><td>数字 GPIO</td><td>按钮 GPIO39；蜂鸣器 GPIO38</td><td>短按批量测温；长按 800 ms 实时测温；声音反馈</td></tr>
</table></div>

<h2>两类测量原理与软件处理</h2>
<div class="twocol">
<div class="panel"><p><span class="label">心率：PPG 光电容积脉搏波</span></p><p>传感器贴合手部或手腕，利用组织血容量随心搏变化产生的反射光强变化估计脉率。模块返回 PPG 原始波形及血氧/脉率结果；固件对当前波形做 EMA 平滑，最近 128 点自动缩放显示。</p></div>
<div class="panel"><p><span class="label">体温：额头红外测温</span></p><p>GY614/GY615V3 对准额头采集非接触红外温度。一次测量取 7 帧，以中位数为中心剔除偏差超过 0.5 °C 的帧，再求平均；25–45 °C 之外的数据无效。</p></div>
</div>

<h2>人机交互状态</h2>
<div class="panel blue"><table>
<tr><th>操作</th><th>状态变化</th><th>输出</th></tr>
<tr><td>短按 GPIO39</td><td>开始 7 帧批量测温，5 s 超时即无效。</td><td>成功鸣叫 200 ms；超时为 100 ms 响/停/响。</td></tr>
<tr><td>长按超过 800 ms</td><td>进入实时测温；松开退出并保留最后有效值。</td><td>OLED 使用 T* 标识；每个有效帧同步 BLE。</td></tr>
<tr><td>心率数据超过 3 s 未更新</td><td>心率有效标志清零并重发模式 0 请求。</td><td>OLED 与机器人端显示 --。</td></tr>
</table></div>
<div class="foot">附录 B / 接口信息可由 ESP32/README.md 与 ESP32.ino 逐项复核</div>
</section>

<section class="page appendix">
<div class="topbar"></div><div class="kicker">14</div>
<h1>附录 C：通信协议与上位机实现</h1>
<div class="subtitle">BLE GATT、CRC 校验和 ROS 2 状态发布</div>

<h2>14 字节数据包（小端序）</h2>
<div class="panel gray"><table>
<tr><th>字段</th><th>类型</th><th>长度</th><th>说明</th></tr>
<tr><td>version</td><td>uint8</td><td>1</td><td>协议版本，当前为 1。</td></tr>
<tr><td>sequence</td><td>uint16</td><td>2</td><td>递增序号，用于判断新旧包。</td></tr>
<tr><td>heart_rate_bpm</td><td>uint16</td><td>2</td><td>心率；flags bit0 有效。</td></tr>
<tr><td>temperature_centi</td><td>int16</td><td>2</td><td>温度乘以 100；flags bit1 有效。</td></tr>
<tr><td>flags</td><td>uint8</td><td>1</td><td>有效标志：0x01 心率，0x02 体温。</td></tr>
<tr><td>uptime_ms</td><td>uint32</td><td>4</td><td>ESP32 运行时间戳。</td></tr>
<tr><td>crc</td><td>uint16</td><td>2</td><td>前 12 字节的 CRC16-CCITT，初值 0xFFFF。</td></tr>
</table></div>

<h2>协议校验关键逻辑</h2>
<div class="code">health_packet_size = 14
version == 1
crc16_ccitt(packet[:-2]) == received_crc
heart_rate = bpm if flags & 0x01 and 30 &lt;= bpm &lt;= 240 else None
temperature = centi / 100.0 if flags &amp; 0x02 and 0 &lt;= centi &lt;= 6000 else None</div>
<p class="note">上述逻辑对应 NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/health_ble_core.py；ESP32 端使用相同多项式生成校验值，避免端到端字段解释不一致。</p>

<h2>NUC 连接与 ROS 2 输出</h2>
<div class="panel blue"><p>health_ble_bridge 使用 Bleak 扫描设备名 MedicalVitals-S3 或服务 UUID，连接后订阅特征通知，并发布到 <strong>/medical_nav/health_status</strong>。设备断线后自动重连；ROS 2 状态 JSON 包含 connected、phase、heart_rate_bpm 和 temperature_c。默认状态频率为 5 Hz，过期窗口为 2 s。</p></div>
<div class="foot">附录 C / 通信协议与实现均为可提取文字</div>
</section>

<section class="page appendix">
<div class="topbar"></div><div class="kicker">15</div>
<h1>附录 D：机器人任务状态与安全验证</h1>
<div class="subtitle">NUC / Nav2 与 STM32 医疗任务的可复核状态机</div>

<h2>任务状态序列</h2>
<div class="panel"><div class="diagram">
<div class="box">INIT / WAIT_START<br>等待机器人本体启动按钮</div><div class="arrow">→</div>
<div class="box">NAV_NURSE<br>扫码确认任务码</div><div class="arrow">→</div>
<div class="box">NAV_BED1 / BED3<br>床头码确认与靠站</div>
</div><div class="diagram">
<div class="box">DISPENSE_BED1 / BED3<br>打开对应药箱并保持</div><div class="arrow">→</div>
<div class="box">NAV_HOME<br>返回起点</div><div class="arrow">→</div>
<div class="box">COMPLETE / NAV_ERROR<br>完成或安全停机</div>
</div></div>

<h2>底层安全门控</h2>
<div class="panel gray"><table>
<tr><th>检查项</th><th>实现</th><th>异常动作</th></tr>
<tr><td>启动 / 急停</td><td>机器人本体设置启动按钮和急停按钮；启动事件做 50 ms 防抖。</td><td>未启动不接受任务运动；急停后停止执行机构。</td></tr>
<tr><td>通信超时</td><td>NUC 与 STM32 采用带 ACK/重发的串口消息；速度命令有约 600 ms 超时。</td><td>底层将速度归零，不执行最后一次旧命令。</td></tr>
<tr><td>定位与靠站</td><td>OPS9/HWT101CT 提供位姿；STP23L 以 8 帧采样、去极值和姿态门限校正床边距离。</td><td>数据不足、偏差超限或超时进入错误状态。</td></tr>
<tr><td>扫码与放药</td><td>NUC 先识别 QR/CODE128，STM32 再校验任务状态和赛事白名单。</td><td>不一致时不切换药箱，不进入放药状态。</td></tr>
</table></div>

<h2>验证记录入口</h2>
<div class="panel blue"><p>可按以下入口复核：<strong>MedicalTask.c / MedicalTask.h</strong>（任务枚举、启动按钮和靠站逻辑）；<strong>NUC_MID360/README.md</strong>（Nav2、Mid-360、扫码与速度门控）；<strong>ESP32/README.md</strong>（BLE 与 2 s 过期显示）。本页将原“程序/电路图截图”补充为可检索的接口、状态和安全说明。</p></div>
<div class="foot">附录 D / 机器人任务与安全闭环</div>
</section>
</body>
</html>'''


# Plain-text replacement pages.  The qualification document uses ordinary
# Songti paragraphs; no screenshots, panels, diagrams, or image-only pages are
# used for the technical material.
PLAIN_HTML_CONTENT = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<style>
@page { size: 612pt 792pt; margin: 0; }
html, body { margin: 0; padding: 0; background: #fff; }
body { font-family: "SimSun", "宋体", serif; color: #000; font-size: 12pt; line-height: 23.34pt; }
.page { position: relative; box-sizing: border-box; width: 481pt; height: 752pt; margin: 20pt 0 0 73.05pt; padding: 9.7pt 0 0 0; border: 0.24pt solid #000; page-break-after: always; }
.page:last-child { page-break-after: auto; }
.page-number { display: none; }
h1 { font-family: "SimSun", "宋体", serif; font-size: 12pt; line-height: 23.34pt; text-align: left; margin: 0 0 14pt; font-weight: normal; }
h2 { font-family: "SimSun", "宋体", serif; font-size: 12pt; line-height: 23.34pt; margin: 0 0 14pt; font-weight: bold; }
p { font-size: 12pt; line-height: 23.34pt; margin: 0 0 14pt; text-align: left; text-indent: 24pt; }
p.no-indent { text-indent: 0; }
ol, ul { font-size: 12pt; line-height: 23.34pt; margin: 0 0 14pt; padding-left: 24pt; }
li { font-size: 12pt; line-height: 23.34pt; margin: 0; text-align: left; }
pre { font-family: "SimSun", "宋体", serif; font-size: 12pt; line-height: 23.34pt; white-space: pre-wrap; margin: 0 0 14pt; }
</style>
</head>
<body>
<section class="page">
<div class="page-number">10</div>
<h1>医疗生命体征终端（ESP32-S3）</h1>
<h2>1. 系统组成与测量原理</h2>
<p>生命体征终端的主控采用 ESP32-S3。心率采用血氧模块的光电容积脉搏波（PPG）方法，传感器贴合手部或手腕，利用组织血容量随心搏变化产生的光学波动计算脉率。血氧模块同时返回血氧和灌注指数，终端对数据进行有效性判断。</p>
<p>体温采用 GY614/GY615V3 非接触式红外体温模块，测量时将模块对准额头。按下测温按钮后采集 7 帧新数据，以中位数为中心剔除偏差超过 0.5 ℃ 的异常帧，再对剩余数据求平均。有效温度范围为 25 ℃ 至 45 ℃，5 s 内不能获得足够有效数据时本次测量无效。</p>
<h2>2. 接口与人机交互</h2>
<ol>
<li>血氧/PPG 模块使用 UART，参数为 115200 8N1，模块 TX 连接 ESP32-S3 GPIO42，模块 RX 连接 GPIO41。上电后发送 AT+MD:0，读取 U1、U2 或 S1 格式的原始 PPG 数据，并读取 U2 格式的血氧、脉率和灌注指数。</li>
<li>GY614/GY615V3 使用 UART，参数为 9600，模块 TX 连接 GPIO6，模块 RX 连接 GPIO7。</li>
<li>SH1106 OLED 使用 I2C，SDA 为 GPIO16，SCL 为 GPIO15，用于显示心率、体温和 PPG 波形。GPIO39 为测温按钮，GPIO38 为有源蜂鸣器。</li>
</ol>
<h2>3. 端侧数据处理</h2>
<ol>
<li>固件对连续 PPG 进行 EMA 平滑，只使用当前数据源，避免切换数据标签时出现波形突变。</li>
<li>心率有效范围为 30 至 250 bpm，血氧有效范围为 50% 至 100%。超过 3 s 没有新的有效结果时，心率和血氧有效标志清零。</li>
<li>体温使用 7 帧中位数、异常值剔除和平均值计算。测量未完成或超时时不发布未滤波温度，避免机器人显示旧值。</li>
<li>终端每 200 ms 发送一次生命体征数据包，机器人端显示当前有效的心率和体温。</li>
</ol>
</section>

<section class="page">
<div class="page-number">11</div>
<h1>生命体征通信与机器人安全闭环</h1>
<h2>1. 数据流与通信接口</h2>
<p>生命体征数据流为：ESP32-S3（设备名 MedicalVitals-S3）通过 BLE GATT 发送通知，NUC 上的 health_ble_bridge 接收并校验数据，随后发布 ROS 2 话题 /medical_nav/health_status，供机器人任务和仪表盘订阅。</p>
<p>NUC 自动扫描目标设备，断线后自动重连。桥接节点检查数据包的版本、长度和 CRC16-CCITT 校验值。默认 2 s 没有收到新数据时，将心率和体温标记为过期，界面恢复为“--”，不沿用旧测量值。</p>
<h2>2. BLE 数据包</h2>
<p>数据包采用小端序，共 14 字节，依次包括 version（uint8，1 字节）、sequence（uint16，2 字节）、heart_rate_bpm（uint16，2 字节）、temperature_centi（int16，2 字节）、flags（uint8，1 字节）、uptime_ms（uint32，4 字节）和 crc（uint16，2 字节）。crc 是前 12 字节的 CRC16-CCITT 校验值，初值为 0xFFFF。</p>
<p>flags 的 bit0 表示心率有效，bit1 表示体温有效。NUC 只接受版本为 1、长度为 14 且 CRC 正确的数据包，并将有效字段转换为 ROS 2 状态消息。</p>
<h2>3. 机器人任务与安全闭环</h2>
<p>NUC 接收生命体征数据的同时，运行 Mid-360 环境感知、OPS9 位移、HWT101CT 航向、STP23L 精确靠站和 DECXIN/GM65 扫码链路。医疗任务状态依次覆盖初始化、前往护士台、扫描任务码、前往 1 号床或 3 号床、扫描床头码、放药、返回起点和完成或错误状态。</p>
<p>按照赛事要求，心率测量位置为手部或手腕，体温测量位置为额头；技术说明中给出 PPG 和红外测温原理，并通过机器人本体通信。机器人设置启动按钮和急停按钮；NUC 紧急停止、OPS/HWT 掉线、四轮反馈不完整、关键通信异常或合法速度命令超时约 600 ms 时，底层立即将速度归零。</p>
</section>

<section class="page">
<div class="page-number">12</div>
<h1>附录 A：系统实现说明</h1>
<h2>1. 实现文件与职责</h2>
<p>ESP32/ESP32.ino 负责读取血氧模块和 GY614/GY615V3，执行心率、血氧和体温有效性判断，驱动 SH1106 OLED，并通过 BLE GATT 发送数据。ESP32/GY614.cpp 和 GY614.h 负责 GY614 串口帧解析。</p>
<p>NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/health_ble_core.py 负责 BLE 数据包格式、CRC16-CCITT 校验和字段解码。health_ble_bridge.py 负责扫描设备、建立连接、订阅通知、自动重连并发布 ROS 2 状态。</p>
<p>Program/Core/Src/MedicalTask.c 和 Program/Core/Inc/MedicalTask.h 负责机器人医疗任务状态机、启动按钮防抖、扫码结果确认、药箱控制、床边靠站和错误状态处理。</p>
<h2>2. 端到端处理流程</h2>
<ol>
<li>传感器在 ESP32-S3 上产生 PPG、血氧、脉率和额头温度数据。</li>
<li>ESP32-S3 完成范围判断、温度异常值剔除和时间戳记录，设置有效标志并计算 CRC。</li>
<li>BLE GATT 以 200 ms 周期发送 14 字节数据包。</li>
<li>NUC 校验版本、长度和 CRC，解析有效心率和体温，发布 /medical_nav/health_status。</li>
<li>机器人任务和仪表盘只使用有效且未过期的数据；断线或超时后显示无效状态。</li>
</ol>
<h2>3. 复核方式</h2>
<p>可按照上述文件路径查看接口定义和处理逻辑。工程文件中的设备名、服务 UUID、特征 UUID、数据包长度、有效标志和超时策略与本技术说明保持一致。</p>
</section>

<section class="page">
<div class="page-number">13</div>
<h1>附录 B：硬件接口与测量链路</h1>
<h2>1. ESP32-S3 接线</h2>
<ol>
<li>血氧/PPG 模块：UART2，115200 8N1，TX 接 GPIO42，RX 接 GPIO41。</li>
<li>GY614/GY615V3：UART1，9600，TX 接 GPIO6，RX 接 GPIO7。</li>
<li>SH1106 OLED：I2C 地址 0x3C，SDA 接 GPIO16，SCL 接 GPIO15。</li>
<li>测温按钮接 GPIO39，按下接地；有源蜂鸣器接 GPIO38。</li>
</ol>
<h2>2. 心率测量</h2>
<p>心率传感器贴合手部或手腕，利用血液容积变化引起的反射光强变化形成 PPG 波形。固件兼容血氧模块的 U1、U2 和 S1 原始波形格式，对波形使用 EMA 平滑并保存最近 128 个采样点。模块返回的脉率在 30 至 250 bpm 范围内才被标记为有效。</p>
<h2>3. 体温测量</h2>
<p>GY614/GY615V3 对准额头进行非接触式红外测温。短按按钮后，固件读取 7 帧新数据；以 7 帧中位数为中心，剔除与中位数相差超过 0.5 ℃ 的帧，再求平均。有效范围为 25 ℃ 至 45 ℃，5 s 内数据不足时测量失败。</p>
<p>按钮按下超过 800 ms 后进入实时测温模式，OLED 使用 T* 标识；每个有效温度帧同步更新机器人端。松开按钮后退出实时模式，并保留最后一次有效温度。</p>
<h2>4. 提示与失效处理</h2>
<p>温度测量成功后蜂鸣器鸣叫 200 ms；5 s 内无法得到足够有效帧时执行两次 100 ms 提示音。连续 PPG 超过 3 s 没有更新时，OLED 显示 PPG WAIT，固件重发模式 0 请求，机器人端不保留过期心率。</p>
</section>

<section class="page">
<div class="page-number">14</div>
<h1>附录 C：通信协议与上位机实现</h1>
<h2>1. BLE 生命体征数据包</h2>
<ol>
<li>version：uint8，占 1 字节，协议版本为 1。</li>
<li>sequence：uint16，占 2 字节，递增序号，用于判断新旧数据。</li>
<li>heart_rate_bpm：uint16，占 2 字节，心率数值；flags 的 bit0 为 1 时有效。</li>
<li>temperature_centi：int16，占 2 字节，温度值乘以 100；flags 的 bit1 为 1 时有效。</li>
<li>flags：uint8，占 1 字节，记录心率和体温有效状态。</li>
<li>uptime_ms：uint32，占 4 字节，ESP32-S3 运行时间。</li>
<li>crc：uint16，占 2 字节，为前 12 字节的 CRC16-CCITT 校验值。</li>
</ol>
<h2>2. NUC 解码规则</h2>
<p>health_ble_core.py 先检查数据长度是否为 14 字节，再检查版本和 CRC。心率只有在有效标志置位且数值处于 30 至 240 bpm 时才写入 ROS 2 状态；体温只有在有效标志置位且换算值处于 0 ℃ 至 60 ℃ 时才写入状态。</p>
<p>health_ble_bridge.py 默认扫描设备名 MedicalVitals-S3 或服务 UUID，连接后订阅特征通知。连接失败时自动重试；设备连接成功后以 5 Hz 发布 JSON 状态，状态中包含 connected、phase、heart_rate_bpm 和 temperature_c。</p>
<h2>3. 过期数据处理</h2>
<p>桥接节点记录最后一次有效数据的接收时间。超过 2 s 未收到新包时，发布的心率和体温字段置为空值，仪表盘显示“--”。重新收到通过校验的新包后，状态恢复为有效。</p>
<h2>4. 复核命令</h2>
<pre>source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
ros2 node list | grep health_ble_bridge
ros2 topic echo /medical_nav/health_status</pre>
</section>

<section class="page">
<div class="page-number">15</div>
<h1>附录 D：机器人任务状态与安全验证</h1>
<h2>1. 医疗任务状态</h2>
<ol>
<li>INIT 或 WAIT_START：初始化任务，等待机器人本体启动按钮。</li>
<li>NAV_NURSE：使用 Nav2 前往护士台，读取任务二维码。</li>
<li>NAV_BED1 或 NAV_BED3：前往对应病床，使用床头码确认床位。</li>
<li>DISPENSE_BED1 或 DISPENSE_BED3：打开对应储药箱并保持放药时间。</li>
<li>NAV_HOME：任务完成后返回起始区域。</li>
<li>COMPLETE 或 NAV_ERROR：完成任务，或因扫码、通信、定位和安全检查失败而停止。</li>
</ol>
<h2>2. 底层安全门控</h2>
<ol>
<li>启动按钮事件使用 50 ms 防抖；机器人未通过本体按钮启动时，不接受任务运动。</li>
<li>NUC 与 STM32 的串口消息使用确认和重发机制。合法速度命令约 600 ms 未更新时，底层将速度归零。</li>
<li>OPS9 和 HWT101CT 提供位姿；STP23L 在床边以 8 帧采样、去除异常值并检查位置和航向门限。</li>
<li>DECXIN 相机识别 QR 或 CODE128，GM65 作为备用源。STM32 再次检查任务状态和赛事码白名单，未通过校验时不切换药箱。</li>
<li>急停、关键定位传感器掉线、四轮反馈不完整、上位机退出或通信异常时，底层停止执行机构并进入错误状态。</li>
</ol>
<h2>3. 与赛事要求的对应关系</h2>
<p>机器人设置启动按钮和急停按钮；生命体征测量时心率位置为手部或手腕、体温位置为额头；测量原理在正文中明确说明；生命体征数据通过 BLE GATT、NUC 和 ROS 2 进入机器人本体。目标误差按赛事指标进行验证，心率为 ±3 bpm，体温为 ±0.5 ℃。</p>
<h2>4. 工程复核入口</h2>
<p>任务枚举和底层控制见 Program/Core/Inc/MedicalTask.h；任务状态、启动按钮、扫码和靠站逻辑见 Program/Core/Src/MedicalTask.c；Nav2、Mid-360、扫码和速度门控见 NUC_MID360/README.md；生命体征 BLE 和过期显示见 ESP32/README.md。</p>
</section>
</body>
</html>'''


def main() -> None:
    WORK.mkdir(exist_ok=True)
    HTML.write_text(PLAIN_HTML_CONTENT, encoding="utf-8")
    chrome_pdf = WORK / "revision-pages.pdf"
    if chrome_pdf.exists():
        chrome_pdf.unlink()
    subprocess.run(
        [
            str(CHROME),
            "--headless=new",
            "--disable-gpu",
            "--no-sandbox",
            "--no-pdf-header-footer",
            f"--print-to-pdf={chrome_pdf}",
            HTML.as_uri(),
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    replacement = fitz.open(chrome_pdf)
    if replacement.page_count != 6:
        raise RuntimeError(f"expected 6 replacement pages, got {replacement.page_count}")
    # Use PDF highlight annotations so the marker stays visible above the page background.
    for page in replacement:
        for block in page.get_text("dict")["blocks"]:
            for line in block.get("lines", []):
                spans = line.get("spans", [])
                if not spans:
                    continue
                left = min(span["bbox"][0] for span in spans)
                top = min(span["bbox"][1] for span in spans) - 1.5
                right = max(span["bbox"][2] for span in spans)
                bottom = max(span["bbox"][3] for span in spans) + 1.5
                annotation = page.add_highlight_annot(fitz.Rect(left, top, right, bottom))
                annotation.set_colors(stroke=(1, 1, 0))
                annotation.set_opacity(0.62)
                annotation.update()
    source = fitz.open(SOURCE)
    output = fitz.open()
    output.insert_pdf(source, from_page=0, to_page=10)
    output.insert_pdf(replacement)
    output.insert_pdf(source, from_page=17, to_page=17)
    if OUTPUT.exists():
        OUTPUT.unlink()
    output.save(OUTPUT, garbage=4, deflate=True)
    output.close()
    source.close()
    replacement.close()
    check = fitz.open(OUTPUT)
    print(f"wrote {OUTPUT} ({check.page_count} pages)")
    print("replacement text chars:", len(check[11].get_text()) + len(check[12].get_text()))
    check.close()


if __name__ == "__main__":
    main()
