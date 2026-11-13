# 点云导航定位包

`kiss_matcher_localization` 将 `odom` 坐标系的局部地图与 `map` 坐标系的
全局 PCD 对齐，成功应用后发布 `map → odom` TF。连续定位使用 wyn 项目的
区域／关键帧候选加 VGICP 初始化、先验 VGICP 跟踪与候选恢复流程。
KISS 粗配准在 `navigation_localizer` 中关闭；历史
`inter_frame_alignment` 演示及其 KISS 源码仍保留。

## 输入和输出

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `local_map_topic`，默认 `/local_map` | `sensor_msgs/msg/PointCloud2` | 点坐标和 `header.frame_id` 均为 `odom`；wyn 组合启动使用 `/fast_lio/local_map` |
| `global_local_map_topic`，默认空 | `sensor_msgs/msg/PointCloud2` | 可选的大窗口初始化／恢复输入；wyn 配置为 `/fast_lio/local_map_global` |
| `/localization/keyframe_candidates` | `KeyframeCandidates` | 可选，外部检索器发布的关键帧候选和配套短时点云 |
| `/localization/status` | `LocalizationResult` | 每次尝试和候选确认的结果，reliable + transient-local |
| `/localization/input_snapshot` | `LocalizationInput` | 开启 `record_inputs` 且有订阅者时发布本次实际选用的原始点云 |
| `/localization/selected_cloud` | `sensor_msgs/msg/PointCloud2` | 关键帧功能启用时发布选中的点云 |
| `/localization/relocalize` | `Relocalize` | 对最新输入立即尝试一次定位 |
| `/localization/set_region` | `SetRegion` | 设置当前所在区域提示 |
| `/localization/accept_candidate` | `AcceptCandidate` | 确认未自动应用的候选结果 |
| `map → odom` | TF | 首次成功应用后约 10 Hz 发布 |

完整字段、坐标和质量门槛见[导航接口说明](NAVIGATION_INTERFACE.md)。

## 地图和候选位姿

默认地图为 `maps/building_split_space.pcd`，包含 `region_id` 标签；
`maps/region_labels.csv` 定义 `space_1`～`space_13` 的名称。
地图按标签拆分并体素化。`config/region_initialization_space.yaml`
提供 `space_2` 的 15 个实验候选位姿，并记录其地图 SHA-256；
使用前应确认地图与雷达安装姿态一致。其他区域尚无候选位姿配置。
大地图未纳入 Git，部署时需单独复制。详见[地图说明](maps/README.md)。

## 构建与启动

在工作空间根目录执行：

```bash
bash src/localization/deployment/build_from_source.sh
```

脚本构建本地算法库和四个 ROS 功能包，安装包中为大地图建立本机软链接。
完整启动命令见[工作空间 README](../../README.md#3-接入-fast-lio-连续定位)。
关键帧检索器及库位于相邻的 `wyn/导航工程/mid360_lidar` 项目；
启用 `keyframe_initialization_enabled` 时必须同时发布候选消息，
并传入地图、库和描述配置的真实哈希及候选数量。

`navigation_localization.launch.yaml` 的所有初始化开关默认关闭。
直接运行裸启动文件而不提供候选配置时，节点会返回
`initialization_candidates_unavailable`，不会启用 KISS 全局匹配。

## 历史演示入口

`inter_frame_alignment` 和 `pcd_inter_frame_demo.launch.yaml` 保留原来的
KISS 粗配准演示能力。它们与持续导航定位是不同入口，不要同时发布导航 TF。
