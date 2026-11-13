# 导航定位接口

全局 PCD 的点坐标为 `map`；实时点云的点坐标及 `header.frame_id`
必须为 `odom`。Fast-LIO 提供 `odom → base_link`，本包在成功应用后
提供 `map → odom`。区域候选初始化要在点云采集时间读取
`odom → base_link`，不能用最新 TF 代替。

## 首次定位与跟踪

`navigation_localizer` 不构造 KISS 粗配准器。首次定位需要区域候选位姿
或 `/localization/keyframe_candidates`。每个候选先预测局部地图在全局图的位置，
裁剪目标点云，再运行 VGICP。候选还需满足重叠率、精配内点、残差、
修正量及歧义检查，且至少两个不同时间的输入窗口给出一致结果后才能应用。
`space_2` 的实验候选配置见 `config/region_initialization_space.yaml`。

成功应用后，开启 `use_prior_tracking` 时用上次应用的 `map ← odom`
作为先验，对裁剪后的地图运行 VGICP；先验失败时进入候选恢复流程，
不执行 KISS 回退。`global_local_map_topic` 可提供较大的初始化／恢复窗口。
已应用位姿的自动更新仍受 `max_auto_translation_m`、
`max_auto_rotation_deg` 限制；超限结果保留为待确认候选。

## 话题

| 名称 | 类型 | 发布／订阅条件 |
| --- | --- | --- |
| `/localization/keyframe_candidates` | `kiss_matcher_localization/msg/KeyframeCandidates` | 可选订阅；外部检索器提供短时点云、候选 `map ← odom`、ID、评分、区域、哈希和时间窗口 |
| `/localization/selected_cloud` | `sensor_msgs/msg/PointCloud2` | 启用关键帧功能后，发布实际选用的原始点云 |
| `/localization/input_snapshot` | `kiss_matcher_localization/msg/LocalizationInput` | 开启 `record_inputs` 且有订阅者时，记录尝试编号、阶段、原话题、选择时间及原始点云 |
| `/localization/status` | `kiss_matcher_localization/msg/LocalizationResult` | 每次尝试／确认后发布；reliable + transient-local |

`KeyframeCandidates` 必须和当前地图、关键帧库、配置的 SHA-256 匹配；
候选个数、ID、时间窗口和位姿也会检查。检索器位于外部
`wyn/导航工程/mid360_lidar/scripts/keyframe_retriever.py`，订阅
`/fast_lio/local_cloud`、`/fast_lio/odom`、`/fast_lio/gravity`。
wyn 当前库为 40 帧；启动时应传入实际数量，不使用启动文件中的历史默认值 28。

## 服务

```bash
ros2 service call /localization/set_region \
  kiss_matcher_localization/srv/SetRegion "{region: 'space_2'}"
ros2 service call /localization/relocalize \
  kiss_matcher_localization/srv/Relocalize "{apply_if_safe: true}"
ros2 topic echo /localization/status
```

`SetRegion` 接受区域 ID、CSV 中的名称或当前所在地描述；空值、`all`、
`全图` 会清除区域提示。响应包含 `success`、`region_id`、
`region_name`、`target_points`、`message`。区域提示在成功应用后消耗。
`Relocalize` 请求只有 `apply_if_safe`，响应为 `LocalizationResult`；
`false` 只计算候选，不应用 TF。当前服务没有全局／局部 `mode` 字段。

质量通过但未自动应用的结果可在 `candidate_timeout_sec` 内确认：

```bash
ros2 service call /localization/accept_candidate \
  kiss_matcher_localization/srv/AcceptCandidate "{candidate_id: '返回的编号'}"
```

`LocalizationResult` 的 `matched` 表示本次候选通过质量检查，
`applied` 表示已更新 TF；`reason` 解释拒绝或等待原因。
`candidate_map_to_odom` 在 `matched=true` 时有效，
`delta_valid` 为真时平移／旋转差才有意义。
`inliers` 是历史 KISS 粗配准内点字段，连续定位中为 0；
使用 `overlap_percent`、`registration_ms`、时间戳和 `candidate_id`
监控当前行为。

## 地图与配置

默认地图 `building_split_space.pcd` 及区域名称 CSV 在 `maps/`。
地图包含 `region_id`，区域由点标签筛出；无标签地图无法接受区域提示。
`config/navigation_localization.yaml` 是裸节点默认配置，
`config/continuous_localization.yaml` 是 wyn 连续定位配置，启用先验跟踪、
设置 1 秒周期及 60% 最小重叠率。区域候选、关键帧与分层筛选均为显式开关。
完整命令见[工作空间 README](../../README.md#3-接入-fast-lio-连续定位)。
