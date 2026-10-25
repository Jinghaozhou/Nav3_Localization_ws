# 导航定位接口

`navigation_localizer` 用 FastLIO 持续发布的局部地图与静态全局 PCD
配准。局部地图的每个点必须在 `odom` 坐标系中，消息的
`header.frame_id` 也必须为 `odom`；全局 PCD 的点坐标必须在 `map`
坐标系中。配准直接得到 `T_map_odom`。FastLIO 继续提供
`odom -> base_link`，导航系统使用 `map -> odom -> base_link`。

尚未接入 FastLIO 时，可先用独立的
[`pcd_cloud_publisher`](../pcd_cloud_publisher/README.md) 包发布
`/local_map`，检查输入话题和定位服务。该调试包不属于定位包的
构建依赖。
可用独立的
[`localization_visualizer`](../localization_visualizer/README.md) 包在
RViz 中叠加全局地图与定位后的局部地图；它也不属于定位包的构建依赖。

新节点与原有 `inter_frame_alignment` PCD 演示节点是两个入口。
导航时只启动 `navigation_localizer`，避免出现两个 TF 发布者。

## 运行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml \
  local_map_topic:=/local_map
```

默认地图为安装在功能包中的 `maps/3_part.pcd`（众创空间五号楼整体地图）。
也可通过 `map_pcd:=/absolute/path/to/global_map.pcd` 指定其他地图。
若使用 Fast-LIO 持续累积的 `/Laser_map`，需传入
`local_map_topic:=/Laser_map local_map_reliability:=reliable`；
`scans.pcd` 可用工作空间根目录下的
`map_pcd:="$PWD/src/localization/maps/scans.pcd"` 直接指定，无需安装该地图。
地图文件作为部署数据，需在目标机器上单独复制；详见
[`maps/README.md`](maps/README.md)。

全局地图启动时读取并体素化一次；局部地图订阅使用 sensor-data QoS，
持续接收并只保留最新的 `PointCloud2`。启动时获得有效局部地图后
自动运行无先验的全局定位（KISS 粗配准 + VGICP 精配准）；成功应用后按
`periodic_interval_sec` 周期运行带 TF 初值的局部定位（仅 VGICP）。
局部配准未收敛、重叠率不足、修正过大或无法取得 TF 初值时，会在同一次
尝试中用同一帧点云回退到全局定位。回退结果仍须通过质量与自动应用检查。
将该参数设为 `0.0` 可关闭周期定位，保留启动定位和服务调用。
首次定位失败，或已定位后一次自动配准失败时，按 `retry_interval_sec`
重试。配准成功但因位姿跳变成为待确认候选时，不用短间隔重试覆盖候选。

局部输入点云本身在 `odom` 下，所以 VGICP 所需初值就是 TF 中现成的
`map ← odom`，无需经过雷达坐标系换算。节点查询最新的 `map ← odom`，
并检查其时效不超过 `max_prior_tf_age_sec`。首次定位仍强制使用全局模式；
局部服务在没有
已应用位姿时返回 `no_local_prior`。后续 TF 中的
`map → odom` 来自本节点上次应用的结果，因此局部初值是里程计预测，
不是独立的全局观测；出现大幅漂移时依靠全局回退重新定位。

## 服务和状态

```bash
ros2 service call /localization/relocalize \
  kiss_matcher_localization/srv/Relocalize \
  "{mode: 1, apply_if_safe: true}"

# 已有定位后，仅用 TF 初值运行 VGICP；不自动回退全局
ros2 service call /localization/relocalize \
  kiss_matcher_localization/srv/Relocalize \
  "{mode: 2, apply_if_safe: true}"

ros2 topic echo /localization/status
```

`/localization/relocalize` 使用最新局部地图快照立即运行一次配准。
`mode=1` 强制全局，`mode=2` 强制局部且失败不回退；`mode=0` 按自动规则
选择局部或全局，原先只传 `apply_if_safe` 的调用也会得到默认值 `0`。
无效模式返回 `invalid_mode`。`apply_if_safe=false` 始终只返回候选，
不更新 TF。响应为 `LocalizationResult`：

| 字段 | 含义 |
| --- | --- |
| `matched` | 本次选定的配准模式通过质量检查 |
| `applied` | 候选 `map -> odom` 已应用 |
| `requested_mode`、`actual_mode` | 请求模式和最终执行的配准模式（`global` / `local`）；尚未运行配准时后者为空 |
| `fallback_used`、`fallback_reason` | 自动模式是否从局部回退全局及回退原因 |
| `reason` | 应用、预览、超出跳变阈值、输入问题或配准失败原因 |
| `candidate_map_to_odom` | 本次候选变换；仅 `matched=true` 时有效 |
| `has_previous` | 本次尝试前是否已有已应用的定位 |
| `delta_valid` | 是否成功利用 `odom -> base_link` 比较新旧机器人全局位姿 |
| `translation_delta_m` | 同一机器人里程计位姿下，新旧 `map -> base_link` 的平移差（m）；仅 `delta_valid=true` 时有效 |
| `rotation_delta_deg` | 同一机器人里程计位姿下，新旧 `map -> base_link` 的三维旋转差（°）；仅 `delta_valid=true` 时有效 |
| `inliers`、`overlap_percent` | KISS 粗配准内点数和 VGICP 重叠率（%）；局部模式的 `inliers=0` |
| `registration_ms` | 本次配准耗时（ms）；自动回退时包含局部和全局两次尝试，不含 ROS/PCL 转换与体素化 |
| `local_map_stamp` | 本次局部地图的采集时间 |
| `last_success_stamp` | 最近一次应用定位结果的时间；此前从未成功时为零 |
| `candidate_id` | 成功但未应用时的候选编号 |

首次定位若 `apply_if_safe=true`，通过质量检查即可应用。
已有定位时，只有 `delta_valid=true`，且机器人位姿差同时不超过
`max_auto_translation_m` 与 `max_auto_rotation_deg` 才自动应用。
通过质量检查还要求 VGICP 实际收敛，变换有效；局部模式还要求修正量不超过
`local_max_correction_translation_m` 和 `local_max_correction_rotation_deg`。

成功但未应用的结果可在 `candidate_timeout_sec` 内确认：

```bash
ros2 service call /localization/accept_candidate \
  kiss_matcher_localization/srv/AcceptCandidate \
  "{candidate_id: '返回的编号'}"
```

确认服务应用已有候选，不再重新计算配准。新定位尝试会取代旧候选。
服务、启动和周期定位使用同一个互斥执行组；局部地图接收与 TF
广播有各自的执行组。每次尝试及候选确认都会在
`/localization/status` 发布同一结构的结果；状态话题使用
reliable + transient-local QoS。

## TF 和失效处理

节点在第一次应用定位结果后以 `10 Hz` 发布 `map -> odom`。
失败或大幅变化暂缓应用时保留上一次变换，状态里的 `applied=false`
和 `last_success_stamp` 告诉导航系统结果的新鲜程度。发布 TF 时
使用当前 ROS 时间；状态保留局部地图的原始时间戳。导航系统应
依据状态自行决定多久以前的全局定位仍可用。
使用局部模式时，雷达外参 TF 必须与建图配置一致。若 `/Laser_map` 随运行
不断累积到全局地图之外，重叠率可能下降，即使初值正确也会被拒绝；
需要结合实际场景调整输入范围或重叠率判定。

默认参数见 [`config/navigation_localization.yaml`](config/navigation_localization.yaml)。
`1.0 m` 与 `15.0°` 是初始自动应用阈值，需结合实测漂移与导航
允许的位姿跳变调整。局部地图过期、`frame_id` 不符、无点或配准
质量不足都会拒绝本次结果。
