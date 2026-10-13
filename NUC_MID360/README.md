# NUC + Mid360 + Nav2 医疗机器人导航

## 当前导航链路

本目录已经切换到真正的 ROS 2 Nav2。旧的 `navigation_node.py`、自写 A*、人工航点跟随、
STM32 侧 OA 和 `ChassisCtrl` 位置环仅保留作历史参考，不再由启动入口或实时底盘任务调用。

```text
OPS9 XY + HWT101CT yaw
        ↓ UART
stm32_bridge
        ├─ /odom
        ├─ map -> odom -> base_link TF
        ├─ /cmd_vel_safe -> STM32 wheel velocity loop
        └─ /medical_nav/bridge_cmd_debug (event-driven diagnostic only)

Mid360 /livox/lidar (livox_frame, raw diagnostics)
        -> robot_state_publisher TF + lidar_self_filter
        -> /livox/lidar_filtered (base_link, chassis returns removed)
Nav2 global/local costmap -> NavFn + MPPI(Omni) -> Collision Monitor
```

定位主体使用 OPS9 的 X/Y 和 HWT101CT 航向，不使用 Mid360 里程计或旧算法估计位置。
当前桥接节点不使用 EKF：`/odom`、机器人位姿和 TF 的 X/Y 直接取经过现有坐标变换及
STP23L 偏置校正的 OPS，yaw 直接取 HWT101CT；轮速不参与位置积分。
`/odom` 的速度优先使用四个电机均在线且最近 `0.15 s` 内收到的实测轮速换算值，
轮速失效时才使用独立滤波的 OPS/HWT 差分估计。轮速能更快反映电机速度变化，但打滑时
不等于真实地面速度；OPS 差分受位置噪声及接收时间抖动影响，也不是无延迟的地面测速。
桥接状态分别报告 `odom_source=ops_hwt_sample_timed` 和 `twist_source`，便于核对运行数据来源。
新版 STM32 `POSE` 包附带 OPS、HWT 航向的采样 tick 和发送 tick；NUC 根据包内相对
时间恢复 ROS 测量时间，并将异步 XY/yaw 用当时轮速短时传播到较新的采样时刻。NUC 仍
兼容旧 15 字节位姿包，旧固件运行时自动回退到接收时刻，因此应先部署 NUC 再烧录 STM32。

两个 Mid360 过滤实例会读取 `timestamp` 或 `offset_time`，用 `/odom` 中的 OPS/HWT 位姿
历史、实测轮速和 yaw 历史将一帧内各点补偿到最后点时刻。输出继续携带绝对
`timestamp`；时间字段异常、跨度超过 `0.20 s` 或里程历史不足时原样发布而不丢帧，状态
话题中的 `motion_compensated_clouds`、`motion_compensation_fallbacks` 可用于核对是否生效。
Nav2 速度平滑器使用 `CLOSED_LOOP`，从 `/odom.twist` 读取上述速度反馈；反馈平均窗口
`odom_duration=0.05 s`，平滑输出频率为 `20 Hz`。每周期的加减速限制以测得的速度为
起点，因此轮速反馈的时延和打滑仍会影响闭环表现。
取消 STM32 和桥接端额外斜坡后，按用户要求将平滑器 X/Y 加减速限值从
`6.0 / -8.0 m/s²` 调整为 `3.0 / -3.0 m/s²`。
yaw 仍为 `1.8 / -2.0 rad/s²`；在 20 Hz 下，X/Y 每轴每周期加速和制动允许变化均为
`0.15 m/s`。平滑器限值不代表底盘实际能达到对应加减速度。
这是闭环响应对照测试参数，实际加减速度、完赛时间和过冲改善需由下一次实车数据验证。
STM32 底盘不再叠加速度斜坡：导航的 X/Y `350 RPM/s`、yaw `600 RPM/s` 斜坡，
以及本地床位对准的 X/Y `300 RPM/s`、yaw `600 RPM/s` 斜坡均已删除。收到的速度目标
直接进入十字底盘运动学和电机速度 PID；导航加减速由上述 Nav2 平滑器控制。
导航航向锁定由 STM32 直接使用 HWT101CT yaw/Wz 执行，护士台、1/3 床和 Home
导航状态的目标均为 `0°`。NUC 仍按原链路下发完整 `vx/vy/wz`，不改动
`POSE`、`/odom`或 TF；STM32 只在 FOLLOWING 且收到非零导航命令时覆盖 `wz`。
所有导航速度和方向统一使用一套 PD：`Kp=2.2`、`Kd=0.45`、死区 `1°`、
最大修正角速度 `0.40 rad/s`，不再按前进/倒车速度切换参数。
全零命令、Collision Monitor 急停、定位/轮速掉线和床位对准不会被航向锁定覆盖。
NUC 桥接的 `gate_release_wheel_accel_m_s2=0.0` 禁用额外斜坡，
`goal_handoff_hold_s=0.0`、`navigator_following_hold_s=0.0` 取消固定起步/交接等待。
仍须按下发车按钮、收到匹配当前请求的有效 FOLLOWING，且定位、轮速和通信有效；
碰撞急停、速度上限及复位后的就绪检查继续生效。这些修改需同时部署 NUC 并重新编译、
烧录 STM32，仅更新 NUC 不会取消旧固件中的斜坡。
1/3 床停车位增加 STP23L 绝对距离校正：A=左、B=前、C=右，三者均安装在距车心
`153 mm` 处。停车圆边与床边相切；侧向传感器光路不受床遮挡，测外侧场地边界约
`1147 mm`，前向传感器测紧贴床体的床头柜前缘约 `347 mm`。程序只在匹配床位、
距离和航向均通过门限时更新 OPS XY 偏置。

## 医疗任务链

STM32 的 `MedicalTask` 只负责任务编排，不参与运动计算：

```text
Nav2 到护士台 -> NUC 相机/GM65 确认任务码
              -> Nav2 到第一张床 -> NUC 相机/GM65 确认床头码 -> 对应药箱放药
              -> Nav2 到第二张床 -> NUC 相机/GM65 确认床头码 -> 对应药箱放药
              -> Nav2 回起点
```

护士台任务码沿用原规则：`11/13` 表示先去 1 号床，`31/33` 表示先去
3 号床；个位决定第一张床使用左/右药箱，另一张床使用另一个药箱。每次移动只发送
`home/nurse/bed1/bed3` 目标给 `medical_navigator`，旧分段转向、STM32 位姿 PID 和 OA
不会重新进入控制链。

DECXIN 相机是主扫码源：任务态 2 只识别 QR，任务态 4/7 只识别 CODE128；两帧一致后
经带 ACK 和重发的串口消息交给 STM32。STM32 会再次检查任务状态和赛事码白名单，只有
通过校验才继续任务。原 GM65 保留为备用源，并继续使用四次一致确认。

速度还有独立任务门控：只有任务状态为前往护士台、1 号床、3 号床或起点，且 Nav2
状态为 `FOLLOWING` 时，STM32 才接受速度。NUC bridge 默认也执行同样检查，因此扫码、
放药、完成和错误状态下的迟到速度帧会被强制替换为零。

## 坐标约定

STM32 场地坐标为 `+X 向右、+Y 向前、航向顺时针为正`，ROS 使用标准 REP-103：

```text
ROS x   = OPS y
ROS y   = -OPS x
ROS yaw = -HWT 顺时针航向
```

Nav2 下发速度保持 ROS 符号：`+x 向前、+y 向左、+z 逆时针`。电机安装方向只在
STM32 的 `DJI_Chassis_SetVelocityCommand()` 中转换。

## Mid360 安装参数

`config/robot.urdf` 当前使用以下实测初值：

- 雷达中心位于车体旋转中心正上方，水平偏移为 `0 m`。
- 雷达中心离地 `0.35 m`。
- 十字底盘的 `base_link +X` 指向物理车头和电机 4；雷达前向轴与物理车头一致，使用 `rpy="0 0 0"`。
- 原始点云保持驱动时间戳和 `livox_frame`；自滤节点使用 URDF TF 转到 `base_link`，删除车体
  半径 `0.26 m`、高度 `-0.05–0.45 m` 内的铝型材/车身回波，再发布
  `/livox/lidar_filtered` 给 costmap 和 Collision Monitor。
- costmap 只采用车体坐标高度约 `0.04–0.32 m` 的点。

安装位置统一由 URDF 中 `livox_joint` 的 `xyz="0 0 0.35"` 和 `rpy="0 0 0"` 表示。

## 主要文件

```text
2_ROS2包_obstacle_detector/
├── obstacle_detector/stm32_bridge.py       串口唯一所有者、里程计/TF、速度下发
├── obstacle_detector/medical_navigator.py  STM32 目标请求转 NavigateToPose action
├── obstacle_detector/lidar_transform.py    Mid360 外参变换与车体自回波过滤
├── obstacle_detector/code_scanner.py       DECXIN QR/CODE128 状态门控扫码
├── obstacle_detector/scanner_core.py       扫码白名单和多帧一致确认
├── config/nav2_params.yaml                 Nav2、MPPI、PointCloud2 costmap 参数
├── config/scanner.yaml                     相机、ROI、焦距和确认参数
├── config/robot.urdf                       车体和 Mid360 安装外参
├── config/static_map.yaml/.pgm             静态场地图
├── scripts/make_static_map.py              场地坐标转 ROS 静态地图
└── launch/obstacle.launch.py               完整 Nav2 启动入口

2_ROS2包_clearance_planner/                 净空与障碍密度优先的 Nav2 全局规划插件
3_可视化工具/medical_nav.rviz               可拖动旋转/缩放的 3D RViz
4_启动脚本/deploy_nuc.py                    两个 ROS 包的完整部署脚本
4_启动脚本/navigation-viz.desktop           已禁用的旧版 RViz 自启动项
```

## NUC 启动

无 GUI 的服务由 systemd 开机启动：

```bash
sudo systemctl status mid360.service
sudo systemctl status obstacle-detector.service
journalctl -fu obstacle-detector.service
```

`start_obstacle.sh` 会先等待串口、两路相机和 Mid360 短暂稳定，再启动整套导航。
启动后必须连续两次确认 `map_server`、planner、smoother、controller、BT、
velocity smoother 与 collision monitor 全部处于 `active`。若 75 秒内仍有节点停在
`inactive`，脚本以失败状态退出，systemd 会在 3 秒后自动重试；不再需要人工重启一次
才能解除界面的“等待 planner/smoother/controller/bt/velocity”。该恢复不会绕过任务门控，
生命周期未完全就绪时仍禁止发车。

RViz 必须在图形用户登录后启动，因此使用：

```text
$HOME/start_rviz.sh
```

`start_rviz.sh` 会等待静态地图、Global Costmap 和 Local Costmap 发布器就绪后再启动
RViz，最长等待 45 秒。部署脚本会在 Nav2 服务重启后主动重启 RViz，避免旧 RViz
跨越 DDS 发布器重建后偶发保持橙色、收不到 Costmap；登录自启动仍保持禁用。

所有本车 ROS 2 进程固定使用独立 Domain，并限制为 NUC 本机发现，避免比赛网络中的其他
ROS 设备发布同名 `/map`、`/tf` 或控制话题：

```text
ROS_DOMAIN_ID=77
ROS_LOCALHOST_ONLY=1
```

通过 SSH 手动执行 `ros2` 命令前，先加载同一环境：

```bash
set -a
source ~/.config/medical-navigation.env
set +a
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
```

默认 RViz Fixed Frame 为 `map`，显示静态地图、全局代价地图、RobotModel、TF 和 Nav2
路径，并提供 Nav2 Goal 工具与 Orbit 三维视角。`Local Costmap` 和
`Mid360 Filtered PointCloud` 已保留在 Displays 中但默认不勾选，需要诊断时再手动开启。
车模顶部红色短块及 `Robot Body Axes` 的红色 X 轴表示真实车头方向。由于场地前方被
映射为 ROS `+X`，默认俯视画面中车头显示在屏幕右侧是正常坐标表现，不代表偏航 90°。

## 路径平滑与显示（2026-10-08）

床位/Home 与护士台行为树都使用 Nav2 原生
`ComputePathToPose -> SmoothPath -> FollowPath`。每次规划成功后使用
`nav2_constrained_smoother/ConstrainedSmoother` 平滑，最大计算预算为 0.45 s，
并显式启用足迹碰撞检查。当前使用 `0.75 m` 的几何最小转弯半径和三次 Bézier
重新上采样，以减少 Smac2D 折线转角；同时保留规划器给出的固定 `0°` 起终点航向，
防止平滑器把停车目标改成路径切线方向。
平滑失败时保留本次新规划的原路径。`medical_navigator` 在每个运行目标中显式
指定行为树，启动资格同时检查 `smoother_server` 是否 active。

控制器固定为唯一的 `FollowPathDWB`，不再通过话题切换普通/弯道 DWB。弯前减速与
影子参考速度剖面仍由曲率节点独立完成。Local/Global Costmap 的床位停车豁免半径均为
`0.30 m`（比赛规定的物理停车圆）；与停车圆相交的栅格会清除膨胀与内切代价，
但仍保留致命障碍和未知区域，
且不改变 `0.24 m` 机器人实体半径。

RViz 默认显示三条路径：

| 显示项 | 话题 | 含义 |
| --- | --- | --- |
| 淡橙色 Raw Plan and Prestart Preview | `/medical_nav/plan` | 原始规划，或按钮按下前的规划预览 |
| 青色 Smoothed Candidate | `/plan_smoothed` | 原生平滑器输出的完整候选路径 |
| 绿色 Controller Followed Path | `/transformed_global_plan` | MPPI 实际使用的变换、裁剪后参考路径 |

Humble 的平滑服务器会在最终足迹碰撞校验前发布青色候选，因此青色不保证被执行。
确认车辆使用平滑路径时，以绿色控制路径的形状为准；它只包含控制器当前使用的
局部路径段。按钮按下前会先验证原始规划，再尝试平滑；平滑通过时预览平滑路径，
平滑拒绝或碰撞校验失败时保留本轮已验证的原始路径并允许发车。路径均为参考路径，
不是实测车辆轨迹。

更新时需同步 `2_ROS2包_obstacle_detector` 整包，重新 `colcon build` 并重启导航服务；
还需将 `3_可视化工具/medical_nav.rviz` 更新到 NUC 的 `~/medical_nav.rviz`，重新打开
RViz。只更新 Windows 源码，或只覆盖 YAML，不能更新 NUC 当前运行的行为树和节点。
在已加载本车 ROS 环境、`ROS_DOMAIN_ID=77` 的 NUC 终端核对：

```bash
ros2 lifecycle get /smoother_server
ros2 topic echo /medical_nav/navigator_status --once
ros2 topic hz /plan_smoothed
ros2 topic hz /transformed_global_plan
```

正常导航时平滑话题通常随重规划发布（床位/Home 1 Hz，护士台 2 Hz），控制路径随
控制周期发布；平滑失败、恢复或已停车时频率可能下降。日志中的
`Received a path to smooth` 可证明服务器收到请求，仍需检查绿色路径及实际运行结果。

## 通行时间优先规划

全局规划器现在使用 `nav2_smac_planner/SmacPlanner2D`。它直接读取 Global Costmap，
因此全局路线和 RViz 中显示的膨胀场使用同一套障碍代价，不再叠加自定义规划器内部的
`preferred_clearance`、`density` 或盲区折线路径。

当前 Global/Local Costmap 统一为：`robot_radius=0.24 m`、`inflation_radius=0.60 m`、
`cost_scaling_factor=10.0`。Smac2D 的 `cost_travel_multiplier=2.0` 只调节代价相对
几何距离的权重；它不会改变车体不可碰撞的足迹边界。`allow_unknown=false`、
`downsample_costmap=false` 保证规划使用已知的 5 cm 栅格。

主要参数位于 `config/nav2_params.yaml` 的 `planner_server.GridBased`：

- `tolerance=0.10`：目标附近允许的规划终点误差。
- `max_planning_time=1.5`：单次全局规划时间上限。
- `max_iterations` 与 `max_on_approach_iterations`：A* 搜索迭代上限。
- `cost_travel_multiplier`：代价图与路径长度的相对权重，调大更愿意远离膨胀区，调小更偏向短路。
- `use_final_approach_orientation=false`：保留任务请求的最终航向，适合当前固定航向全向底盘。

Smac2D 生成的原始路径仍按行为树经过 `SmoothPath`，再交给 DWB 跟踪；平滑服务负责
碰撞检查和平滑，不是全局规划器的代价场。盲区若重新启用，建议放在独立的限速/安全层，
不要作为 Smac2D 的致命障碍直接写入全局代价图。

### 验证方法

- 同时记录 `/plan`、`/plan_smoothed` 和 `/transformed_global_plan`，确认 Smac2D 的原始路径、
  平滑路径与 DWB 实际跟随路径没有断链。
- 在 RViz 同时显示 Global Costmap、Local Costmap 和 `/plan`，确认路径绕开的是可视化膨胀场，
  而不是旧的私有净空场。
- 对比规划耗时、路径长度、最小足迹净空和 `/cmd_vel`；不能只用路径点数判断速度。

## 部署

Windows 主机安装 `paramiko` 后运行：

```powershell
$env:MEDICAL_NUC_HOST = "手机热点下 NUC 的 IP，例如 172.20.10.2"
$env:MEDICAL_NUC_PASSWORD = "输入 NUC 密码"
python "NUC_MID360/4_启动脚本/deploy_nuc.py"
Remove-Item Env:MEDICAL_NUC_HOST
Remove-Item Env:MEDICAL_NUC_PASSWORD
```

未设置 `MEDICAL_NUC_HOST` 时仍使用默认地址 `192.168.50.22`。使用手机热点时，电脑与 NUC
必须连接同一个热点，并且电脑能够通过 SSH 访问该 IP；该变量只改变部署的 SSH 目标，不改变
NUC 上的 `ROS_DOMAIN_ID` 或 `ROS_LOCALHOST_ONLY`。

脚本上传 `obstacle_detector` 与 `medical_clearance_planner` 两个 ROS 包和启动文件、执行
`colcon build`、安装并启用两个 systemd 服务，
部署会移除旧的 RViz desktop 自启动项。RViz 仅在需要诊断时手动运行。当前部署脚本会按比赛模式写入：

```text
$HOME/.config/medical-navigation.env
MEDICAL_NAV_DRY_RUN=false
ROS_DOMAIN_ID=77
ROS_LOCALHOST_ONLY=1
MEDICAL_SCANNER_ENABLED=true
MEDICAL_SCAN_CAMERA=/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0
MEDICAL_TELE_SCAN_CAMERA=/dev/v4l/by-id/usb-BLC-240823--A_SDYH-8P0P-video-index0
```

部署脚本会安装 OpenCV、ZBar、v4l-utils 和 ZXing，并把用户加入 `video` 组。相机参数或
设备路径需要修改时，可先在 PowerShell 设置 `MEDICAL_SCAN_CAMERA`、
`MEDICAL_TELE_SCAN_CAMERA`，或直接编辑
`config/scanner.yaml`，再执行同一个 `deploy_nuc.py`。如果 PowerShell 没有显式提供这两个
环境变量，部署会保留 NUC 的 `medical-navigation.env` 中已有设备路径，不再用仓库默认值
覆盖实机已验证的 `/dev/v4l/by-id`。长焦相机最多允许 45 秒完成 USB 枚举和首帧采集，
成功出帧后会保持二维码扫描，直到 STM32 离开预发车任务态；不会再因部署期间界面尚未重启就
耗尽短扫码窗口。广角和长焦的 V4L2 打开、释放由同一交接锁串行化，
部署重启或任务切换时不会同时抢占 USB 摄像头。部署后可用以下命令检查链路：

```bash
ros2 topic echo /medical_nav/scanner_status
ros2 topic echo /medical_nav/scan_transport_status
ros2 topic echo /medical_nav/task_state
```

扫码界面禁用 Qt 自动 DPI 放大并使用响应式相机区域，不再用固定相机高度、按宽度强制
反推高度或手工设置全屏几何。若现场确实需要额外缩放，可在 `medical-navigation.env` 设置
`MEDICAL_DASHBOARD_SCALE`，默认值为 `1`。

两个 systemd 服务都关闭启动频率上限。`mid360.service` 启动后必须在 30 秒内收到
`/livox/lidar` 首帧，否则自动重启驱动；导航服务在启动前再次等待点云，并在启动后要求
map、planner、smoother、controller、BT、velocity smoother、collision monitor 连续两轮均为
active。任一步超时都会退出并由 systemd 重试，避免冷启动后进程存在但界面永久停在“等待”。

脚本随后会重启服务，STM32 满足任务门控时车体可能立即运动。首次验证新规划器时必须架空
车轮或断开电机驱动。部署完成后若要进行 dry-run，应立即把该文件改成
`MEDICAL_NAV_DRY_RUN=true` 并重启服务；执行 `~/check_nuc.sh` 并完成静态检查后，再恢复比赛模式：

```bash
sed -i 's/MEDICAL_NAV_DRY_RUN=true/MEDICAL_NAV_DRY_RUN=false/' \
  ~/.config/medical-navigation.env
sudo systemctl restart obstacle-detector.service
ros2 param get /stm32_bridge dry_run
```

最后一条必须显示 `Boolean value is: False`。需要重新锁车时改回 `true` 并重启服务。
`enforce_task_gate` 默认必须保持 `True`；它与 dry-run 是两道独立保护。

场地尺寸或床位/护士台实测值更新后，先修改 `config/field_map.yaml`，再重新生成地图：

```bash
cd ~/livox_ws/src/obstacle_detector
python3 scripts/make_static_map.py
```

生成器按 `ROS x=场地Y、ROS y=-场地X` 绘制边界和固定设施，并检查所有 Nav2 目标仍在
地图内且未落入占用栅格。

## 首次上车验证顺序

1. 架空轮子，确认 OPS9、HWT101CT 和串口在线。
2. 静止观察 `map -> odom -> base_link -> livox_frame` TF。
3. 手推平移和原地旋转，确认静态障碍在 `map` 中不漂移、不拖影。
4. 核对点云高度，确认地面和高于雷达有效范围的点未进入 costmap。
5. 确认 bridge 的 `dry_run=True` 且 `enforce_task_gate=True`，只观察 Nav2 的安全速度话题。
6. 架空轮子验证前、后、左、右及正负旋转方向。
7. 最后以不超过 `0.15 m/s` 的速度落地，标定有效旋转半径和机器人真实外轮廓。

当前轮径按 `150 mm`、旋转等效半径按 `250 mm`、机器人物理半径按 `0.225 m` 设置；
代价地图仍通过 inflation layer 额外保留导航安全距离。

### 障碍物漂移隔离测试

保持 `MEDICAL_NAV_DRY_RUN=true`，先不要让 Nav2 驱动车轮：

1. 在 RViz 将 `Global Options -> Fixed Frame` 临时改成 `base_link`，在物理车头正前方约
   1 m 放一个高度低于 30 cm 的箱子。点云应位于 `Robot Body Axes` 红色 `+X` 方向；若在
   后方，MID360 航向外参差 180°；若在左/右侧，外参差约 90°。这个测试不受 OPS9 和
   HWT101CT 航向影响。
2. 将 Fixed Frame 改回 `map`，手动把车顺时针转约 90°。红色车头轴和绿色位姿箭头也应
   顺时针转约 90°。只有转动方向相反时才修改 `yaw_sign`；只有始终固定相差 90°且转动
   方向正确时才增加 yaw offset。不要按屏幕的“上/右”直接判断航向。
3. 在 NUC 执行 `~/check_nuc.sh`。新固件的 `/odom` 应接近 50 Hz，`/livox/lidar` 通常
   接近 10 Hz；雷达 delay 应较小且稳定。静止时观察 `/medical_nav/bridge_status` 中的
   `yaw_cdeg`，若持续抖动超过约 100--200 cdeg，远处障碍会明显左右摆动。

MID360 中心位于旋转中心正上方、离地约 0.35 m。若点云只显示障碍物下部，应先结合
实际视场确认是否属于正常现象。确认 TF、航向和时间戳都正确后，
若 costmap 仍因 Livox 稀疏帧闪烁，再把 `observation_persistence` 从 `0.0` 小幅调到
`0.2--0.3 s`；过大则会留下移动拖影。

## 物理刹车影子模型（2026-10-11）

`braking_safety_shadow` 以 `/cmd_vel_home_limited`、`/livox/lidar_safety` 和
`/odom` 计算圆形足迹沿当前运动方向首次接触障碍物的距离，再按反应时间与恒定减速度
反推出连续安全速度。默认采用 `2.5 m/s²` 保守减速度、`0.20 s` 反应时间、
`0.15 m` 额外余量和 `0.245 m` 足迹半径。

该节点当前严格处于影子模式：只发布 `/medical_nav/braking_shadow_cmd_vel` 和
`/medical_nav/braking_shadow_status`，不会向 `/cmd_vel_safe` 或底盘发布控制命令。
原有 Collision Monitor 的 `ApproachPolygon` 与 `StopPolygon` 仍是实际安全链路。
点云、里程计或命令超时时，影子输出按未来失效安全策略归零，但同样不影响车辆。
