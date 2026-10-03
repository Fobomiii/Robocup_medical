# 比赛全过程运动审核

日期：2026-10-04。对象：20 kg、45 度 X-drive 全向底盘及当前 NUC/STM32 工作区。

本轮是审核，没有修改运行代码或参数。审核结合当前工作树、`78ff6bb`、`a187cec`、`c7d5b3a`、`bed13_full_20261004_005623` 录包，以及 20260904 版规则。已有未提交改动保持原样。

## 1. 优先处理的发现

### 1.1 严重：以原始路径的安全性解除另一条执行路径的盲区限速

来源：[corner_speed_limiter.py:292][raw_override]、[行为树:16][bt_path]。

当前行为是：平滑路径命中盲区，原始路径未命中，就用原始路径计算限速。行为树却仍将平滑结果送给 `FollowPath`。这里只换了限速依据，没有换实际执行路径。

这是相对于 HEAD 的未提交改动，是近期引入的明确回归。第一段出发窗口出现 8 次 `raw_final_validation`，原始路径重合为 0，平滑/控制路径重合约 0.62-0.71 m，已经超过 0.60 m 阈值；发布的限速为 1.179-2.959 m/s，而不是 0.70 m/s 盲区回退限速。不能表述成所有限速都被解除。

修复要求：不安全的平滑结果必须被拒绝，并真正选择经过验证的原始路径交给控制器。若两者都不能通过，则保留保护、重新规划或安全停止。`AlwaysSuccess` 只能处理行为树节点返回失败，不能自动识别成功返回但盲区不安全的平滑结果。

### 1.2 严重：Collision Monitor 不是最终运动权限层

来源：[stm32_bridge.py:1179][bridge_input]、[stm32_bridge.py:1323][heading_override]、[main.c:1125][mcu_authority]。

`/cmd_vel_safe` 的零命令被标成 `source="nav2"`，因此独立航向保持仍被允许覆盖 `wz`。最小复现：Home 导航状态、航向误差 10 度、安全输入三轴为零，航向控制仍返回约 -0.262 rad/s。录包中也有 SAFE 为零但发送非零角速度的样本。不过仅凭 SAFE=0 不能认定每个样本都是 Collision Monitor 急停，合法的驻车航向纠正也可能产生零平移输入。

航向保持本身合理，问题是它位于最终碰撞检查之后，且没有明确携带急停原因/权限。这一整合冲突在 `78ff6bb` 中存在，`a187cec` 尚未包含同样的独立航向覆盖。

STM32 将收到的普通零速度交给正常速度斜坡；只有 OPS/HWT/轮反馈失效等分支使用 immediate stop。Collision Monitor 的普通零命令没有独立急停语义，因此上位机要求停，并不等于底盘立即采用相应安全停车策略。

修复要求：航向合成后的完整命令接受最终碰撞检查，或传递明确的停止状态并在所有后级保持最高优先级。正常到点制动与碰撞急停必须区分；不能把所有零平移都当急停，也不能允许明确急停被 PID 重新激活。

### 1.3 严重：Home 制动起点与可执行减速度不匹配，场地边界缺少直接制动保护

来源：[home_approach_core.py:247][home_radial]、[obstacle.launch.py:570][home_launch]、[nav2_params.yaml:517][smoother]。

Home 已有基于径向距离的限速，但会同比缩放两个平移分量，没有轮速反应距离，也不是分轴制动。当前 3 m/s 在 1.00 m 才开始压速，0.10 m 处希望降至 0.10 m/s，等效要求约：

```text
a = (3.0^2 - 0.1^2) / (2 * (1.0 - 0.1)) = 4.994 m/s^2
```

显式 `decel_start_distance_m` 会决定这条包络的形状，单改 `soft_decel_m_s2` 不一定得到预期效果。实际正常平滑器 X/Y 减速度是 2.0/2.6 m/s²；STM32 350 RPM/s 车体斜坡按 152 mm 轮径折算约 2.786 m/s²，且机械抓地和电流限制还可能进一步降低实际能力。

旧版本已存在径向方案和缺少完整场地边界包络的问题；近期 3 m/s 巡航、缩短 Home 制动起点等设置放大了这个缺陷。不是单纯近期平滑改动才造成。

最新录包第一次回 Home：

| 时间 | ROS 位置 x/y (m) | MPPI 原始 vx | 平滑器 vx |
| --- | --- | --- | --- |
| 00:57:08.200 | 0.044 / -0.714 | +0.29 | -0.80 |
| 00:57:08.400 | -0.133 / -0.586 | +0.95 | -0.40 |

上游已经回拉，平滑器仍在向外运动。最终位置约 (-0.359, -0.333)；按当前配置 0.23 m 底盘半径估计，后缘约 -0.589 m，超出 ROS x=-0.5 m 的场地边界。随后起点碰撞、规划失败、看门狗停车，是越界后的后果，不应单独用地图清除解释。

修复要求：Home 终点包络与全场四边的硬保护分开；全场保护覆盖所有运动状态，包括 STM32 对接。按足迹投影、定位余量、实际朝边界轮速和反应时间计算各轴允许速度，保留沿边界分量以及离开边界的修正。不得用“车中心还在地图内”替代整个底盘不越界。

### 1.4 高：后级斜坡可能重新带回已被限制的速度分量

来源：[bridge_safety.py:578][gate_ramp]。

现有 NUC 轮空间斜坡先比较目标/当前轮速峰值；目标峰值增大时，从上一条速度向新速度插值。

最小复现，dt=0.05 s、斜坡 2.8 m/s²：上一命令 `(0.2, 0, 0)`，新命令 `(0, 1, 0)`，输出变成 `(0.1670, 0.1650, 0)`。假如 X 已被床边/边界保护禁止，旧 X 分量就被重新带回。

反例：上一命令 `(1,0,0)` 到新命令 `(-1,0,0)`，因轮速峰值相等而直接通过，没有真正限制这次轮速反转。STM32 后级仍有斜坡，但这表明 NUC 的加速度限制并非对所有轮速变化都有效。

该算法不是最新盲区改动引入的。新增加的分轴保护若只做上游修改，会与它产生冲突。

修复要求：正常斜坡满足逐轮变化约束；最终安全投影不能被旧状态插值解除。下位机残余运动必须纳入提前制动预算，不能把“目标分量置零”等同于“实车分量立即为零”。

### 1.5 高：盲区验证未覆盖最终路线复用、平滑结果及实际运动方向

来源：[clearance_planner.cpp:802][dogleg_check]、[clearance_planner.cpp:1272][travel_yaw]、[clearance_planner.cpp:1332][final_check]、[clearance_planner.cpp:858][route_metrics]。

当前已经有统一的角度/重合长度参数、短狗腿验证、最终几何检查、选侧锁定。但最后的几何检查只打警告，不阻止不合格路径；之后还可能用旧路线替换新路线，而复用评估没有同样的盲区验收。

规划侧用目标 yaw 判断盲区，运行限速侧用实车 yaw；MPPI 绕障/纠偏/残余惯性又可能使实际速度方向偏离路径切线。独立航向控制在下游改变运动，因此仅验证三点狗腿不是完整验收。

该缺口有原有规划不一致，也有近期“最终复检只有警告”的未完成实现，不能全部归因于狗腿选侧锁定。

修复要求：所有候选、复用路线、平滑后的最终选中路线通过同一标准；控制路径和真实速度方向分别检查。路径带目标/版本标识，限速依据与 `FollowPath` 使用同一条路线。规划没有合格快速路径时允许低速回退，但不能宣称盲区已消除。

### 1.6 高：安全点云自车过滤范围与 StopPolygon 大面积重叠

来源：[obstacle.launch.py:110][self_filter]、[nav2_params.yaml:577][stop_polygon]、[lidar_transform.py:47][filter_logic]。

安全点云剔除半径 0.26 m、高度 -0.05 至 0.65 m 的圆柱；安全输出高度是 0.12-0.60 m。StopPolygon 的外接半径约 0.265 m，内切半径约 0.245 m，其大部分区域位于被剔除圆柱内部。

例如 `(0,0.255,0.30)`、`(0.18,0.18,0.30)` 位于停止多边形内，却会被当作自身点过滤。停止检测并非完全失效，但有效区域很薄。自车过滤半径和停止区尺寸在 `c7d5b3a` 已存在；不过旧 Collision Monitor 使用 `/livox/lidar_nav`，其中包含雪糕筒补偿。`a187cec` 到 `78ff6bb` 新增了安全点云短链路，改成直接读 `/livox/lidar_safety`，不再经过这项补偿。因此属于旧几何冲突加近期输入链路改变，不能把当前的实际检测效果简单等同于旧版本。

修复应校准真实自身轮廓和高度分区，不应直接把全部过滤关掉，也不应仅粗暴扩大停止区而导致床位无法接近。需在静态、自身抖动、机械臂展开/收回、雪糕筒和裁判测试下验证。现有证据还不能证明这是本次碰撞的唯一原因。

X 形遮挡本身无法通过软件生成真实观测。避盲区路线可以降低持续不可见风险，但不能保证一个从未被任何传感器观测的障碍必然被避开。需要保留已观察障碍，区分盲区与确认空闲区域，必要时改善安装或补传感器。

### 1.7 高：新鲜的转发时间可能掩盖旧运动命令

来源：[home_approach_limiter.py:189][cmd_cache]、[corner_speed_limiter.py:318][no_path]。

床位/Home 限速器在位姿和任务回调时重复发布缓存命令，却没有记录该原始命令的有效期。上游停止发布后，新位姿仍可让旧运动命令看上去持续新鲜，下游接收看门狗及速度帧序号不能识别原始命令已经过期。这是旧缺陷。

另外，转角/盲区限速器没有新鲜路径或位姿时会发布解除限速。在 Nav2 SpeedLimit 中 `speed_limit=0` 表示取消限制，不表示停车。对于仍在执行缓存路径的高速车，这是保护丢失。

修复要求：明确原始命令有效期、源序号与任务代次，转发不刷新原始有效期；失去必要路径/位姿时进入确定的降级/停止状态，而非默认全速。

## 2. 对用户四项建议的结论

| 建议 | 当前是否已有类似实现 | 原有/新增判断 |
| --- | --- | --- |
| 取消原始路径解除执行路径盲区限速，真正回退执行路径 | 有原始/平滑路径选择限速，但没有真正换控制路径 | 解除限速分支是近期未提交回归；平滑器未做盲区验收是原有不足 |
| 覆盖实际控制路径及运动方向 | 有狗腿、重合长度、yaw 旋转判定，但端到端不足 | 原有不足；近期共享参数和选侧锁定有帮助，但最终复检未闭环 |
| Home/边界分轴制动，床位对称 | 床位已有测量轮速+反应时间+单向分轴包络；Home 没有同等级实现 | Home/全场缺口是原有问题，提速放大；床位实现应保留并修复后级冲突 |
| 预制动匹配真实减速能力 | 转角和床位已有距离制动模型，Home 模型与实际减速度不匹配 | 框架不是没有；近期末端激进参数使不匹配更明显 |

床位现有参数为前后制动 2.0 m/s²、床侧制动 2.2 m/s²、反应时间 0.22 s、余量 0.03 m。实现对 Bed1/Bed3 对称，不需要再写两套特例。

注意坐标：[home_approach_core.py:157][bed_axes] 使用 ROS map-X/map-Y，配置 OPS 转 ROS 为 `(field_y/1000, -field_x/1000)`。Bed1=(5.4,+2.2)，Bed3=(5.4,-2.2)。因此朝床侧的 ±2.20 m 是 ROS Y，不是 ROS X；不能将场地坐标中的 x=2.20 直接照搬成 ROS x=2.20。

## 3. 比赛全过程及额外风险

正常流程：发车/准备完成 -> 护士台扫码 -> 第一床导航 -> STM32 最终对接 -> 床头条码/送药/播报 -> 第二床导航和任务 -> Home -> 全部机构停止。

当前导航命令链：

```text
规划/路线复用 -> 路径平滑 -> FollowPath/MPPI -> /cmd_vel_raw
  -> 20 Hz OPEN_LOOP 速度平滑器 -> /cmd_vel
  -> 床位/Home 限速器 -> /cmd_vel_home_limited
  -> Collision Monitor -> /cmd_vel_safe
  -> STM32 桥接：独立航向、轮速归一化、释放斜坡
  -> 速度协议 -> STM32 车体斜坡 -> 四轮速度环 -> 实车
```

STM32 最终对接是另一条底盘控制来源，不能只验证以上 Nav2 链。

| 阶段 | 已有正确设计 | 仍需要覆盖的问题 |
| --- | --- | --- |
| 复位/发车 | STM32 ready 闭锁，静止恢复，任务/目标关联门控 | 旧缓存不能跨任务复活；正式模式不能意外使用测试捷径 |
| 护士台 | `FollowPathNurse` 允许主动朝二维码旋转，扫码交接有停车守卫 | 当前允许在起点获取订单后直接去床位，不符合正式流程“出发后自主到护士台扫码”要求 |
| Home/护士台到床位 | `FollowPathHeadingHold` + HWT 航向，长直线 3 m/s 上限 | 路径/盲区不一致、朝床分量受后级影响、预测航向与实际航向差异 |
| Bed1/Bed3 最终对接 | 350 mm 接管范围、实测低于 0.45 m/s 稳定 150 ms、yaw 约束、安全走廊、5 稳定样本 | MCU 直接控制底盘，未统一接受 NUC 碰撞/边界保护；超时或达到修正次数可能转入扫码而未确认成功 |
| 送药/播报 | 任务状态限制 NUC 导航，保留条码结果 | 进入床位 SCAN 时先开箱，不以本次正确条码和实车稳定停车为统一前提；语音错误/超时也能继续后续流程 |
| 床到床 | `FollowPathLateralHold` 两方向对称，横向减速度 2.6 | 长横向制动受惯性/抓地/负载影响，必须检验目标轮速/反馈/地面速度；机械臂回收与行驶重叠 |
| 两床回 Home | `FollowPathHeadingHold`，反向 PD 保持至稳定 | Home 径向包络晚制动、削弱纠偏、无直接全场边界守卫；路径切换不应绕过安全验收 |
| 终点 | COMPLETE 清速度、关闭箱体、回收臂 | 未明确以整车/机械臂静止超过 5 s 且所有投影在白框内作为完成资格 |

额外代码依据及处理：

- [MedicalTask.c:1194][start_shortcut]：起点订单捷径是旧行为；`MEDICAL_TEST_AUTO_SKIP_SCAN=0` 并不会关闭这条正常编译分支。正式/测试模式需要明确隔离。
- [MedicalTask.c:545][dock_timeout]：对接超时调用 finish，转入 SCAN；[ChassisCtrl.c:116][reach] 主要以位置/yaw 判断到点，没有独立实测静止判据。应区分成功、超时、失败，不用超时冒充成功。
- [MedicalTask.c:1058][box]：进入 SCAN 就开箱；[MedicalTask.c:1354][tts]：语音错误/超时可以离开等待。符合规则的床位任务门槛应包含本床正确条码、全车在圈内、稳定停车和实际播报完成。
- [MedicalTask.c:1024][arm_overlap]：回收臂与导航重叠，而地图仍用静态 0.23 m 足迹。只有已验证的展开/收回包络处于安全走廊内才允许重叠，否则受控等待或用相应足迹，不必全程一律低速。
- [DJIMotorCtrlSTM32.cpp:108][timer]：先将 DWT 周期计数除成微秒，再做无符号时间差，无法正确跨周期计数器回绕。400 MHz 时约每 10.74 s 回绕，可能导致轮反馈瞬时误判过期；PID 的 dt 有异常回退但不修复在线判断。这是旧缺陷，未证明是最新事故原因。
- [DJIMotorCtrlSTM32.cpp:212][can_init]：CAN 初始化失败仍可能标 ready；发送失败虽已计数，尚缺少连续失败的闭锁、诊断和有界恢复策略。不能把已有计数当作故障已处理。

规则第 10-13 页要求：自主到护士台取订单；床头条码后开正确箱；底盘全部投影在半径 0.30 m 圈内，开箱至播报结束不能移动；碰撞每次扣分；回 Home 所有部分进入白框并停止超过 5 s。排序是先比得分，再比时间。应按有效完成全场的耗时优化，而不只看峰值速度。

## 4. 保留高速的统一制动方式

对每个朝边界或目标的速度分量，用实测速度留出反应距离，再依据可达减速度限制目标速度：

```text
d_required = v_measured_toward * reaction_time
             + v_measured_toward^2 / (2 * achievable_decel)
             + margin
```

以 a=2.0 m/s²、反应 0.22 s、余量 0.03 m 为示例，2.6/2.8/3.0 m/s 停至零分别需要约 2.29/2.61/2.94 m。不是建议对所有路段设置固定 3 m 限速区，更不是实测保证；只针对正在朝约束运动的分量，用当前速度计算。离开边界、沿边界的分量不应一起被压低。

`bed_side_decel_m_s2`、转角 `braking_decel_m_s2` 等包络参数表示“模型假定能达到的减速度”，不是直接驱动电机刹车。把它调大反而会允许更晚制动。只有实车已证明在不同电量/负载/地板上都能达到更大的减速度，才能同步提高模型值。

统一要求：

- 长直线保留 3 m/s 上限和现有高速探索，不全局降速。
- 用最弱实际制动环节确定包络；正常停车、转角、床侧、Home 共享一致的能力定义，但可按前后/横向轴区分，Bed1/Bed3 不区分。
- 加入制动前的延迟，包括消息、处理、下位机响应；不能用“发布频率高”替代实测响应。
- 轮速估计可能受滑移影响，用 OPS 位姿速度核对，定位跳变/旧 TF 必须明确降级，不能把轮速当无误差地面速度。
- 20 Hz 改 50 Hz 能缩小采样时间粒度，但不修复 1 m 制动距离与约 2 m/s² 的矛盾，也不修复安全层排序。
- MPPI 预测与独立航向控制的可执行角速度尽量一致；目前 hold profile 很小的预测旋转范围与下游 0.30/0.40 rad/s 修正有差异，应测量其影响。

## 5. 区分随机误差、系统缺陷及证据边界

已确认的系统缺陷：路径/限速依据不一致；急停权限被航向层覆盖的可能性；斜坡恢复禁止分量；Home 模型超出正常可执行减速度；StopPolygon 与自身过滤冲突；旧命令重新转发；DWT 回绕；任务成功判定不完整。

这些缺陷可以表现得“随机”：只有在特定路径侧别、平滑形状、接近速度、消息相位和障碍位置下才触发，但不是因此就属于纯测量随机误差。

尚未量化的随机/工况误差：轮胎抓地、全向滚子跳动、四轮压载不均、负载/电量、电机差异、OPS 更新抖动、HWT 瞬时偏差、裁判动作、雪糕筒首次可见时机。这些因素需要多轮对称试验评估分布，不能被软件修复承诺为零。

未证明的事故唯一根因：具体碰撞点的点云可见性、自车过滤/盲区各自的贡献；独立 yaw 改动是否是某次偏航的唯一原因；TF 回退是否直接造成本次越界。最新录包已证明执行路径盲区保护被替代，以及第一次 Home 制动滞后/越界，但不应将所有事故归于其中一个参数。

## 6. 建议实施顺序及验收

1. 恢复执行路径一致性：拒绝危险平滑结果，真正回退；对最后选中的复用/平滑路线验收，绑定路线版本。
2. 明确安全停止最高权限；航向和后级斜坡不得解除保护。保留正常驻车航向控制，另行识别明确碰撞急停。
3. 复用已有床位分轴函数实现 Home 和全场边界保护，足迹/余量/实测轮速齐全；检查 MCU 对接直控路径。
4. 按实测减速能力统一提前制动，不先提高峰值/加速度；正常制动与紧急停车分别校验电流/地面滑移。
5. 校准安全自车过滤，补原始命令有效期和 MCU 时钟/CAN 故障处理。
6. 覆盖正式任务成功门槛，之后再提高长直线加速度或 MPPI 探索。

必须增加的回归场景：

- 原始路径安全/平滑后危险，控制器实际执行原始回退路径；失败时不解除盲区保护。
- 旧路线复用、左右狗腿锁定、实际 yaw 误差、正向/反向速度方向、短段拼接累计重合。
- Collision Monitor stop 后误差 yaw 非零，发送命令与 MCU 停止语义都保持停车；合法静态转向仍可通过检查。
- 分轴护栏将一轴置零、另一轴加速，后级不得恢复受禁分量；等轮速峰值反转也检查逐轮加速度。
- Home 两侧斜入、床1/床3 镜像、四个场地边界、离开边界纠偏、全速长直线不被无故压速。
- 上游断流而位姿继续更新；路径/轮反馈/TF 超时；STM32 人工复位；CAN 连续失败；跨 DWT 回绕。
- 护士台二维码必须现场获取、对接失败不得当成功、扫码/静止后开箱、播报期不移动、Home 全部机构静止超过 5 s。

已有正确改动应保留：STM32 10 ms 主机循环、OPS/HWT 新鲜度守卫、全部轮反馈有效性、三轴原子邮箱、速度帧序号/时间戳校验、非阻塞 UART 遥测、公共轮速归一化、任务专用 MPPI 配置和对称床位接管条件。不能为了恢复旧版速度把它们整体撤销。

## 7. 本轮验证

- 重读当前未提交 diff 和 Git 版本关系，并用离线 SQLite/日志分析器复核最新录包关键窗口。
- 用当前纯 Python 控制函数复现禁止分量恢复、同峰值反转、零输入后的航向覆盖。
- 重跑 obstacle_detector 的 unittest discovery：180 项中 178 项通过，2 项因本机缺少 `sensor_msgs` / `rclpy` 导入失败。现有不少测试为配置/源码检查，不能证明跨节点安全链正确。
- 未在本机运行 ROS/Nav2 集成、Keil 构建或实车验证。未修改生产代码。

[raw_override]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/corner_speed_limiter.py:292>
[bt_path]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/config/navigate_to_pose_1hz.xml:16>
[bridge_input]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/stm32_bridge.py:1179>
[heading_override]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/stm32_bridge.py:1323>
[mcu_authority]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/main.c:1125>
[home_radial]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/home_approach_core.py:247>
[home_launch]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/launch/obstacle.launch.py:570>
[smoother]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/config/nav2_params.yaml:517>
[gate_ramp]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/bridge_safety.py:578>
[dogleg_check]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_clearance_planner/src/clearance_planner.cpp:802>
[travel_yaw]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_clearance_planner/src/clearance_planner.cpp:1272>
[final_check]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_clearance_planner/src/clearance_planner.cpp:1332>
[route_metrics]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_clearance_planner/src/clearance_planner.cpp:858>
[self_filter]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/launch/obstacle.launch.py:110>
[stop_polygon]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/config/nav2_params.yaml:577>
[filter_logic]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/lidar_transform.py:47>
[cmd_cache]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/home_approach_limiter.py:189>
[no_path]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/corner_speed_limiter.py:318>
[bed_axes]: <C:/Users/ASUS/Desktop/Robocup医疗/NUC_MID360/2_ROS2包_obstacle_detector/obstacle_detector/home_approach_core.py:157>
[start_shortcut]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/MedicalTask.c:1194>
[dock_timeout]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/MedicalTask.c:545>
[reach]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/ChassisCtrl.c:116>
[box]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/MedicalTask.c:1058>
[tts]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/MedicalTask.c:1354>
[arm_overlap]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/MedicalTask.c:1024>
[timer]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/DJIMotorCtrlSTM32.cpp:108>
[can_init]: <C:/Users/ASUS/Desktop/Robocup医疗/Program/Core/Src/DJIMotorCtrlSTM32.cpp:212>
