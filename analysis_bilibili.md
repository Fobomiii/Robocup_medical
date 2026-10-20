# Bilibili MPPI 评论区分析

更新时间：2026-10-09

参考视频：[mppi虽然不好调，但调出来效果绝对猛](https://www.bilibili.com/video/BV1eojh6pEVw)

## 作者回复中的关键建议

1. MPPI 出现轻微 S 形路线属于正常现象。若确实需要走得更直，可以扩大膨胀区、增加 `batch_size`、降低 `temperature`，但更大的采样量会增加计算负载并降低实时性。
2. 对窄通道，可以保留较大的 `inflation_radius`，同时提高 `cost_scaling_factor`，让远离障碍物后的代价更快衰减。作者在窄道案例中建议过约 `15` 的衰减系数。
3. 作者采用的是全局规划侧形成宽代价场、局部控制侧使用更激进参数保证通过性的方案。这是一种有意的不一致，并不是 Nav2 的强制要求。
4. 作者明确建议避免使用 `CostCritic`，将避障交给 `ObstaclesCritic`，并通过路径 critic 与障碍 critic 的权重平衡路径跟随和临时绕障。
5. 作者多次强调 TF/LIO 延迟。路径方向不稳定不一定只是 MPPI 参数问题，过期的位姿也会让控制器不断修正。
6. 作者提到的 `PathAngleCritic` 主要适合车头需要朝运动方向的车型。当前机器人采用固定航向全向移动，直接启用会惩罚正常横移，因此不应照搬。

## 当前工作区存在的不一致

- 机器人有效内切半径约为 `0.23 m`，原 local/global `inflation_radius` 都是 `0.25 m`，只有约 `0.02 m` 的有效渐变带，接近硬边界，不能形成贯穿通道的连续代价场。
- 原 local `cost_scaling_factor` 为 `3.0`，global 为 `2.5`，全局规划和局部控制读取的软代价公式不同。
- 自定义全局规划器另外生成 `preferred_clearance: 0.65` 和 `density_radius: 0.60` 的私有净空场。全局路径会被这套场推开，但 MPPI 的 local costmap 看不到同一套偏好。
- 三套 MPPI 原先都使用 `CostCritic`。扩大膨胀半径后，它会直接累计大范围原始 costmap 代价，容易与路径进度和路径跟随竞争。
- 配置声明了完整 16 边形足迹，但原 `CostCritic.consider_footprint` 为 `false`，注释和实际碰撞检查方式不一致。
- `FollowPathHeadingHold.PathFollowCritic` 权重为 `16`，而 `FollowPathLateralHold` 仅为 `10`。这会影响 Bed1 到 Bed3 等横移路段对全局路径的跟随强度，但本轮不同时调整该变量，以免混淆代价图试验结果。

## 为什么选择 0.60 / 8.0

Nav2 膨胀层在机器人内切半径之外可近似表示为：

```text
cost(d) = 252 * exp[-cost_scaling_factor * (d - inscribed_radius)]
```

按 `inscribed_radius ~= 0.23 m`、`inflation_radius = 0.60 m` 估算，膨胀边缘代价约为：

| `cost_scaling_factor` | `d = 0.60 m` 的代价 |
| ---: | ---: |
| 3 | 83 |
| 6 | 27 |
| 8 | 13 |
| 15 | 1 |

这解释了过去仅将半径放大到 `0.60 m`、仍保留 `2.5~3.0` 衰减系数时车辆变慢的现象：通道内很大范围仍维持较高代价。`0.60/8.0` 保留了连续的场，同时让远端代价降到较低水平，适合作为第一组基线。

`inflation_radius` 是规划偏好范围，不是绝对不可碰距离。硬碰撞边界仍由足迹、致命障碍和 Collision Monitor 负责。

## 本轮实施方案

1. local/global 统一使用 `inflation_radius: 0.60`、`cost_scaling_factor: 8.0`。
2. 保留私有场的距离参数以便回退，但临时将 `clearance_weight` 和 `density_weight` 置为 `0.0`，让全局路径主要读取可在 RViz 中观察的全局 costmap 场。
3. 三套 MPPI 将 `CostCritic` 替换为 `ObstaclesCritic`。
4. Humble 的 `ObstaclesCritic` 需要显式填写 `inflation_radius` 和 `cost_scaling_factor`；它们必须与 local inflation layer 完全一致，否则从代价值反算障碍距离会出错。
5. 使用完整足迹碰撞检查，保留绝对碰撞高惩罚，并以 `repulsion_weight: 1.5`、`critical_weight: 20.0`、`collision_margin_distance: 0.08` 作为基线。
6. 本轮不改变 `batch_size`、`temperature`、路径 critic 权重、速度和加速度。先隔离验证代价场与障碍 critic 的影响，再决定是否增强路径跟随。

## 验证重点

- RViz 中 local/global 膨胀颜色应呈连续梯度，而不是只在障碍物旁出现一圈窄边。
- 记录全局路径、平滑路径、MPPI 最优轨迹和最终 `cmd_vel`，比较 MPPI 是否仍持续偏离全局路径。
- 检查窄道、护士台和床位交接目标是否仍存在可通行路径。软膨胀代价可以穿越，足迹落入致命或内切碰撞区域才应被禁止。
- 检查 controller server 是否能稳定达到 `20 Hz`；本轮采样规模不变，理论上不会因为批量增加而提高计算负载。
- 若路径稳定但避障距离过远，优先依次测试 `cost_scaling_factor: 10`、`12`，而不是立即缩小 `inflation_radius`。
