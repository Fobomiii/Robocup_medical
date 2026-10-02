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
1/3 床停车位增加 STP23L 绝对距离校正：A=右、B=前、C=左，三者均安装在距车心
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
- 雷达中心离地 `0.33 m`。
- 雷达正装，前向轴与物理车头一致，使用 `rpy="0 0 0"`。
- 原始点云保持驱动时间戳和 `livox_frame`；自滤节点使用 URDF TF 转到 `base_link`，删除车体
  半径 `0.26 m`、高度 `-0.05–0.45 m` 内的铝型材/车身回波，再发布
  `/livox/lidar_filtered` 给 costmap 和 Collision Monitor。
- costmap 只采用车体坐标高度约 `0.04–0.32 m` 的点。

安装位置统一由 URDF 中 `livox_joint` 的 `xyz="0 0 0.33"` 表示。

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

## 通行时间优先规划

全局规划器使用 `medical_clearance_planner/ClearancePlanner`。它读取实时 Global
Costmap，并在规划器内部另外计算障碍距离场，因此不会扩大 RViz 中的红色/青色代价区，
也不会改变 `robot_radius=0.23 m` 和 `inflation_radius=0.25 m`。

边的权重就是**通过该格的真实秒数** `resolution / speed`，所以 A\* 最小化的是时间，
而不是一个没有量纲的惩罚值。这是本规划器与旧版本最大的区别：过去净空、密度、原始
代价三项是**相加**的，上限可达 `1 + 3.0 + 8.0 + 5.0 = 17` 倍，于是只要绕路不超过
17 倍长度，A\* 就一定会绕开窄缝；而实际车速比只有 `2.00 / 0.35 ≈ 5.7` 倍，多出来的
部分全部变成了无意义的绕行。现在最慢的格也只贵 5.7 倍，绕行自然消失。

速度用与靠站同一套包络（同 `home_approach_core.approach_speed_limit`）：

```
gap   = max(0, 障碍中心距 - body_clearance)
speed = clamp(sqrt(crawl_speed² + 2 · soft_decel · gap), crawl_speed, max_speed)
```

`crawl_speed` 是**爬行地板**，不是可选项：没有它，零净空格的通行时间就是无穷大，
任何能过但要贴边的门都会被当成墙，规划器又会退回原来的绕行行为。地板保证门永远
可通行，只是“贵一些”。

主要参数位于 `config/nav2_params.yaml` 的 `planner_server.GridBased`：

- `max_speed`：速度上限，必须与 MPPI 的 `vx_max` 和 `safety_velocity_smoother` 的
  `max_velocity[0]` 一致；它同时决定启发式，取值过大将使启发式不再可采纳。
- `crawl_speed`：爬行地板，当前 `0.35 m/s`。`ApproachPolygon` 相对 0.23 m 车体
  只留 50 mm 余量，而碰撞监测按 2 s 前瞻，所以地板不宜再抬高。
- `soft_decel`：速度包络的减速度余量，与靠站限速保持一致。
- `body_clearance`：障碍中心距等于它就表示车体已经贴上，等于 `robot_radius`。
- `costmap_weight`：原始代价作为**时间倍率**保留，它在两个方向上对称，因此只影响
  路径贴哪一侧走，不会再制造绕行。设为 `0` 即为纯时间模型。
- `simplification_cost_tolerance`：路径简化时允许的秒数比例上限。

这些参数由插件在每次规划时读取，可以在 SSH 中动态试验，无需清空 Costmap：

```bash
ros2 param set /planner_server GridBased.crawl_speed 0.25
ros2 param set /planner_server GridBased.costmap_weight 1.0
```

参数修改后必须重新发送目标，才会生成新路径。先在 `dry_run=true` 下确认所有任务目标
均能生成路径，再把最终数值写回 YAML。

### 验证方法

- 看 `/plan` 的点数：改前改后同样起终点，点数应明显下降（长绕行消失）。
- 对比 `/cmd_vel` 与 `/cmd_vel_safe` 的速度分布：正常赛道中不应长期停留在 `crawl_speed`，
  若长期在爬行说明 `body_clearance` 偏大或地图比实际窄。
- 若窄缝仍被绕开，先确认该处栅格未被 `isBlocked` 判为不可通行，再考虑下调
  `crawl_speed`；不要用提高 `costmap_weight` 的方式“压”路径，那会重新引入绕行。

## 部署

Windows 主机安装 `paramiko` 后运行：

```powershell
$env:MEDICAL_NUC_PASSWORD = "输入 NUC 密码"
python "NUC_MID360/4_启动脚本/deploy_nuc.py"
Remove-Item Env:MEDICAL_NUC_PASSWORD
```

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
`config/scanner.yaml`，再执行同一个 `deploy_nuc.py`。部署后可用以下命令检查链路：

```bash
ros2 topic echo /medical_nav/scanner_status
ros2 topic echo /medical_nav/scan_transport_status
ros2 topic echo /medical_nav/task_state
```

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

MID360 中心位于旋转中心正上方、离地约 0.33 m。若点云只显示障碍物下部，应先结合
实际视场确认是否属于正常现象。确认 TF、航向和时间戳都正确后，
若 costmap 仍因 Livox 稀疏帧闪烁，再把 `observation_persistence` 从 `0.0` 小幅调到
`0.2--0.3 s`；过大则会留下移动拖影。
