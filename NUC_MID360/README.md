# NUC + Livox Mid360 障碍物检测系统

## 系统概述

本系统运行在 ASUS NUC (Ubuntu 22.04, ROS2 Humble) 上，驱动 Livox Mid360 激光雷达，
检测前方 40–50 cm 范围内的障碍物（雪糕筒），并通过 USB-TTL 串口将障碍物坐标发送给 STM32H7。

---

## 设备信息

| 项目 | 值 |
|------|----|
| NUC IP（WiFi） | 192.168.50.63 |
| NUC IP（有线，接雷达） | 192.168.1.50 |
| SSH 登录 | `ssh gp-pcie@192.168.50.63` 密码 `FZUREA2026` |
| Mid360 IP | 192.168.1.107 |
| 工作空间 | `/home/gp-pcie/livox_ws` |
| **USB-TTL 串口设备** | **`/dev/ttyUSB0`**（CH340）|

---

## 目录说明

```
NUC_MID360/
├── README.md                        ← 本文件
├── 1_配置文件/
│   ├── MID360_config.json           ← 雷达网络配置（已填好 IP）
│   └── my_mid360_launch.py          ← ROS2 launch 文件（PointCloud2 输出格式）
├── 2_ROS2包_obstacle_detector/      ← 障碍物检测 ROS2 包（完整源码）
│   ├── obstacle_detector/
│   │   └── obstacle_node.py         ← 核心检测节点
│   ├── launch/obstacle.launch.py
│   ├── config/params.yaml           ← 检测阈值配置（最常调整）
│   ├── package.xml / setup.py       ← ROS2 包定义
├── 3_可视化工具/
│   └── lidar_viz.py                 ← PyQt5 可视化调试工具（桌面图标启动）
├── 4_启动脚本/
│   ├── start_mid360.sh              ← 启动 Livox 驱动
│   ├── start_obstacle.sh            ← 启动障碍物检测（串口发送）
│   └── start_viz.sh                 ← 启动可视化工具
└── 5_协议与说明/
    └── 串口协议.md                   ← STM32 串口通信帧格式
```

---

## 快速上手

### 比赛模式（开机自动运行，无需操作）

两个 systemd 服务已配置为开机自启：
- `livox-mid360.service`：驱动 Mid360
- `obstacle-detector.service`：障碍物检测 + 串口发送（延迟 5 秒等驱动就绪）

检查服务状态：
```bash
sudo systemctl status livox-mid360.service
sudo systemctl status obstacle-detector.service
```

查看实时日志：
```bash
journalctl -fu obstacle-detector.service
```

### 调试模式（可视化工具）

双击桌面图标 **「雷达可视化调试」** 即可打开 GUI 工具，功能包括：
- 极坐标实时点云显示（原始 / 过滤 / 叠加）
- 左侧滑块实时调整所有检测阈值
- 右侧说明面板解释每个参数含义
- 右侧一键启动/停止两个服务

---

## 话题说明

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 驱动发布的点云（x=前, y=左, z=上）|
| `/livox/imu` | `sensor_msgs/Imu` | IMU 数据（未使用）|

> **注意**：实际发布在 `/livox/lidar`，不是 `/livox/lidar_PointCloud2`。

---

## 最常调整的参数

编辑 `2_ROS2包_obstacle_detector/config/params.yaml`，修改后重启服务：
```bash
sudo systemctl restart obstacle-detector.service
```

| 参数 | 含义 | 推荐值 |
|------|------|--------|
| `dist_min` / `dist_max` | 前向检测距离范围 | 0.40 / 0.50 m |
| `y_span_max` | 横向点云宽度上限，超过认为是大物体 | 0.20–0.25 m |
| `min_points` | 最少有效点数 | 5–10 |
| `max_points` | 最多有效点数，超过认为是大物体 | 60–100 |

---

## 重新部署（若换机器）

```bash
# 1. 将 2_ROS2包_obstacle_detector/ 复制到目标机器
scp -r 2_ROS2包_obstacle_detector gp-pcie@192.168.50.63:~/livox_ws/src/obstacle_detector

# 2. 编译
ssh gp-pcie@192.168.50.63
source /opt/ros/humble/setup.bash
cd ~/livox_ws
colcon build --packages-select obstacle_detector --symlink-install

# 3. 安装 systemd 服务（参考 4_启动脚本/）
```
