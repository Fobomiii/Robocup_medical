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
        └─ /cmd_vel_safe -> STM32 wheel velocity loop

Mid360 /livox/lidar (livox_frame, raw diagnostics)
        -> robot_state_publisher TF + lidar_self_filter
        -> /livox/lidar_filtered (base_link, chassis returns removed)
Nav2 global/local costmap -> NavFn + MPPI(Omni) -> Collision Monitor
```

定位主体使用 OPS9 的 X/Y 和 HWT101CT 航向，不使用 Mid360 里程计或旧算法估计位置。
1/3 床停车位增加 STP23L 绝对距离校正：A=右、B=前、C=左，三者均安装在距车心
`155 mm` 处。停车圆边与床边相切；侧向传感器光路不受床遮挡，测外侧场地边界约
`1145 mm`，前向传感器测紧贴床体的床头柜前缘约 `345 mm`。程序只在匹配床位、
距离和航向均通过门限时更新 OPS XY 偏置。

## 医疗任务链

STM32 的 `MedicalTask` 只负责任务编排，不参与运动计算：

```text
Nav2 到护士台 -> GM65 连续确认任务码
              -> Nav2 到第一张床 -> 连续确认床头码 -> 对应药箱放药
              -> Nav2 到第二张床 -> 连续确认床头码 -> 对应药箱放药
              -> Nav2 回起点
```

护士台任务码沿用原规则：`11/13` 表示先去 1 号床，`31/33` 表示先去
3 号床；个位决定第一张床使用左/右药箱，另一张床使用另一个药箱。每次移动只发送
`home/nurse/bed1/bed3` 目标给 `medical_navigator`，旧分段转向、STM32 位姿 PID 和 OA
不会重新进入控制链。

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

- 雷达中心在车体旋转中心前方 `0.10 m`。
- 雷达中心离地 `0.30 m`。
- 上下颠倒且朝向反转，按绕 Y 轴 `180°` 表示。
- 原始点云保持驱动时间戳和 `livox_frame`；自滤节点使用 URDF TF 转到 `base_link`，删除车体
  半径 `0.26 m`、高度 `-0.05–0.45 m` 内的铝型材/车身回波，再发布
  `/livox/lidar_filtered` 给 costmap 和 Collision Monitor。
- costmap 只采用车体坐标高度约 `0.04–0.32 m` 的点。

若“前方 10 cm”的实际安装方向相反，只修改 URDF 中 `livox_joint` 的 X 符号。

## 主要文件

```text
2_ROS2包_obstacle_detector/
├── obstacle_detector/stm32_bridge.py       串口唯一所有者、里程计/TF、速度下发
├── obstacle_detector/medical_navigator.py  STM32 目标请求转 NavigateToPose action
├── obstacle_detector/lidar_transform.py    Mid360 外参变换与车体自回波过滤
├── config/nav2_params.yaml                 Nav2、MPPI、PointCloud2 costmap 参数
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
/home/gp-pcie/start_rviz.sh
```

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

## 净空优先规划

全局规划器使用 `medical_clearance_planner/ClearancePlanner`。它读取实时 Global
Costmap，但在规划器内部另外计算障碍距离场和局部障碍密度，因此不会扩大 RViz 中的
红色/青色代价区，也不会改变 `robot_radius=0.225 m` 和 `inflation_radius=0.25 m`。
起点和目标附近只渐进取消净空偏好，致命障碍与车体碰撞检查始终有效。

主要参数位于 `config/nav2_params.yaml` 的 `planner_server.GridBased`：

- `preferred_clearance`：希望路径保持的障碍净空范围；增大后影响距离更远。
- `clearance_weight`：为了更大净空可以接受多少绕路；增大后更愿意绕远。
- `density_radius`：统计周围障碍密度的邻域半径。
- `density_weight`：避开多障碍区域和双障碍夹缝的强度。
- `goal_exemption_radius`：精确靠站区；增大后更早允许接近目标旁固定设施。
- `start_exemption_radius`：从床位、护士台旁离开时的渐进恢复范围。
- `costmap_weight`：保留对 Nav2 原始膨胀代价的权重。

这些参数由插件在每次规划时读取，可以在 SSH 中动态试验，无需清空 Costmap：

```bash
ros2 param set /planner_server GridBased.preferred_clearance 0.70
ros2 param set /planner_server GridBased.clearance_weight 14.0
ros2 param set /planner_server GridBased.density_radius 0.80
ros2 param set /planner_server GridBased.density_weight 10.0
```

参数修改后必须重新发送目标，才会生成新路径。先在 `dry_run=true` 下确认所有任务目标
均能生成路径，再把最终数值写回 YAML。若目标最后一段过早贴近设施，应减小
`goal_exemption_radius`；若目标无法平顺靠近，应适当增大该值，而不是扩大 Costmap。

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
/home/gp-pcie/.config/medical-navigation.env
MEDICAL_NAV_DRY_RUN=false
ROS_DOMAIN_ID=77
ROS_LOCALHOST_ONLY=1
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

MID360 中心离旋转中心 0.10 m、离地约 0.30 m，且倒装后主要看到 0.30 m 以下物体，
所以点云只显示障碍物下部是硬件视场造成的正常现象。确认 TF、航向和时间戳都正确后，
若 costmap 仍因 Livox 稀疏帧闪烁，再把 `observation_persistence` 从 `0.0` 小幅调到
`0.2--0.3 s`；过大则会留下移动拖影。
