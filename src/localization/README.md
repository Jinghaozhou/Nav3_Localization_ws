# KISS Matcher 点云定位 ROS 2 包

`kiss_matcher_localization` 将 `odom` 坐标系中的局部点云与 `map` 坐标系中的
全局 PCD 地图配准，计算并发布 `map → odom`。配准先使用 KISS Matcher
寻找初始位姿，再使用 VGICP 精配准，并按内点数和重叠率检查结果。

包内有两个运行入口：`navigation_localizer` 用于持续定位和重定位服务；
`inter_frame_alignment` 用于两路点云之间的配准演示。导航定位时启动前者。

## 输入与输出

| 项目 | 默认值或行为 |
| --- | --- |
| 全局地图 | 启动文件默认使用 `maps/3_part.pcd`；点坐标位于 `map` 坐标系，启动时加载并体素化 |
| 局部地图 | 启动文件默认订阅 `/local_map`，消息类型为 `sensor_msgs/msg/PointCloud2`；点坐标和 `header.frame_id` 均为 `odom` |
| 定位变换 | 首次成功应用后发布 `map → odom` TF |
| 定位状态 | `/localization/status`，消息类型为 `kiss_matcher_localization/msg/LocalizationResult` |
| 手动重定位 | `/localization/relocalize` 服务 |
| 候选确认 | `/localization/accept_candidate` 服务 |

默认地图文件放在 [`maps/`](maps/README.md)。PCD 数据不随 Git 仓库分发；
也可以在启动时用 `map_pcd` 指定其他地图。参数默认值见
[`config/navigation_localization.yaml`](config/navigation_localization.yaml)，
服务、状态字段和 TF 更新规则见[导航接口说明](NAVIGATION_INTERFACE.md)。

## 构建

需要 ROS 2 Humble、CMake ≥ 3.22、colcon、PCL、Eigen3、TBB、FLANN、LZ4，
以及 `robin`、`kiss_matcher`、`small_gicp` 算法库。当前工作空间包含算法
源码，在工作空间根目录运行：

```bash
bash src/localization/deployment/build_from_source.sh
```

脚本依次构建算法库和本包、`pcd_cloud_publisher`、
`localization_visualizer`、`localization_path_recorder` 四个 ROS 包。算法库安装在 `external/`，
ROS 包安装在 `install/`。详细构建依赖和检查方法见
[从源码构建说明](deployment/README.md)。

## 运行导航定位

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml \
  map_pcd:="$PWD/src/localization/maps/scans.pcd" \
  local_map_topic:=/Laser_map \
  local_map_reliability:=reliable
```

上面的命令使用 `scans.pcd` 作为全局地图，并订阅 Fast-LIO 的
历史累积点云 `/Laser_map`。该话题发布可靠 QoS，点云较大，需设置
`local_map_reliability:=reliable`；改回滑动窗口输入时可使用
`local_map_topic:=/fast_lio/local_map` 并省略可靠性参数。节点收到有效局部
地图后自动尝试首次定位，成功后按配置周期重新定位。用 PCD 做离线测试时，可启动
[`pcd_cloud_publisher`](../pcd_cloud_publisher/README.md) 发布测试点云，并用
[`localization_visualizer`](../localization_visualizer/README.md) 查看对齐结果。
PCD 离线调试的三包启动命令见[工作空间 README](../../README.md#3-使用-pcd-调试输入)。
实时定位时，可视化包还能在一个 RViz 窗口并排显示全局图和
尚未对齐的局部图；启动命令见[可视化包说明](../localization_visualizer/README.md)。
定位成功后，可启动[轨迹记录包](../localization_path_recorder/README.md)，
从 `map ← base_link` TF 发布 `/localization/map_path` 并叠加到可视化窗口的全局图上。

启动日志会显示全局地图加载前后的点数；首次收到局部地图时记录话题、
frame 和点数。每次配准会记录局部地图解码与降采样后的点数、两张地图
提取的 KISS 特征点数、初始与筛选后的对应点数、旋转与平移内点数，
以及 VGICP 的重叠点数。终端不再逐项打印计算耗时；
`/localization/status` 中仍保留 `registration_ms` 供需要时查看。

### 启动参数与配置文件

`navigation_localization.launch.yaml` 支持以下 launch 参数；可用
`ros2 launch kiss_matcher_localization navigation_localization.launch.yaml --show-args`
查看当前安装版本的参数列表。

| 参数 | 启动文件默认值 | 用途 |
| --- | --- | --- |
| `map_pcd` | 包内 `maps/3_part.pcd` | `map` 坐标系下的全局 PCD；文件必须存在且非空 |
| `local_map_topic` | `/local_map` | 局部地图话题，消息中的点坐标及 `header.frame_id` 必须为 `odom_frame` |
| `local_map_reliability` | `best_effort` | 点云订阅可靠性：`best_effort` 或 `reliable`；大体量的 `/Laser_map` 使用后者 |
| `map_frame` | `map` | 全局地图坐标系，也是输出 TF 的父坐标系 |
| `odom_frame` | `odom` | 局部地图坐标系，也是输出 TF 的子坐标系 |
| `base_frame` | `base_link` | 比较重定位前后机器人位姿时使用的机器人坐标系 |
| `config_path` | 包内 `config/navigation_localization.yaml` | 定位节点的 ROS 2 参数文件 |

在自定义 YAML 中沿用仓库参数文件的 `/**: ros__parameters:` 结构，修改下表
参数后用 `config_path` 加载。配置文件路径可指向工作空间以外的文件。

| YAML 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `voxel_resolution` | `0.1` m | 全局图和局部图的体素大小；必须大于 0 |
| `max_local_map_age_sec` | `10.0` s | 局部图时间戳允许的最大年龄；设为 `0.0` 关闭此项检查 |
| `retry_interval_sec` | `5.0` s | 首次定位成功前的自动重试间隔；必须大于 0 |
| `periodic_interval_sec` | `60.0` s | 已定位后的自动重定位间隔；设为 `0.0` 关闭周期重定位 |
| `max_auto_translation_m` | `1.0` m | 已定位时自动应用新结果允许的最大平移差 |
| `max_auto_rotation_deg` | `15.0`° | 已定位时自动应用新结果允许的最大旋转差 |
| `candidate_timeout_sec` | `60.0` s | 未自动应用的候选结果可确认的时长；必须大于 0 |
| `verbose` | `true` | 输出配准过程日志 |
| `min_inliers` | `15` | KISS Matcher 粗配准的最少内点数；必须大于 0 |
| `vgicp_threads` | `4` | VGICP 使用的线程数；必须大于 0 |
| `vgicp_neighbors` | `20` | VGICP 邻域点数；必须大于 0 |
| `vgicp_max_correspondence_scale` | `3.0` | VGICP 最大对应距离与 `voxel_resolution` 的倍率；必须大于 0 |
| `min_overlap_percent` | `70.0`% | VGICP 重叠率阈值；取值范围为 0–100，实际接受条件为重叠率高于阈值 |

`map_pcd`、`local_map_topic` 和三个坐标系名称也可写在 YAML 中，但启动文件
会用同名 launch 参数覆盖它们。因此只修改 YAML 里的
`local_map_topic: /fast_lio/local_map` 不会改变默认启动时的订阅话题，需同时
传入 `local_map_topic:=/fast_lio/local_map`。修改包内源码目录的配置文件后，
若安装目录不是符号链接安装，需要重新构建本包，或直接用 `config_path`
指向修改后的文件。参数在节点启动时读取；更改 YAML 后需重启节点。

调参时可先核对实际生效值：

```bash
ros2 param get /localization/navigation_localizer local_map_topic
ros2 param get /localization/navigation_localizer min_overlap_percent
```

局部地图来自真实设备时，需同时保证 `odom_frame → base_frame` TF 可用，
以便在已有定位后比较新旧机器人位姿；详细应用规则见
[导航接口说明](NAVIGATION_INTERFACE.md)。

检查定位结果：

```bash
ros2 topic echo /localization/status
ros2 run tf2_ros tf2_echo map odom
```

手动请求一次重定位：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 service call /localization/relocalize \
  kiss_matcher_localization/srv/Relocalize \
  "{apply_if_safe: true}"
```

首次匹配通过质量检查后可以直接应用。已有定位时，若缺少
`odom → base_link`，节点无法比较新旧机器人位姿，会保留匹配结果作为候选，
而不会自动更新 `map → odom`；候选可通过 `accept_candidate` 服务确认。

## 两路点云配准演示

`pcd_inter_frame_demo.launch.yaml` 使用包内的 PCD 发布器读取两份文件，
启动 `inter_frame_alignment`，默认打开 RViz：

```bash
ros2 launch kiss_matcher_localization pcd_inter_frame_demo.launch.yaml \
  source_pcd:=/absolute/path/to/local.pcd \
  target_pcd:=/absolute/path/to/map.pcd
```

已有点云话题可使用 `inter_frame_alignment.launch.yaml`，设置
`source_topic`、`target_topic`、`source_frame` 和 `target_frame`。
无图形界面时添加 `start_rviz:=false`。演示入口还支持
`config_path`（默认 [`config/alignment_config.yaml`](config/alignment_config.yaml)）、
`frame_update_hz`、`namespace` 和 `rviz_path`；配准参数在该 YAML 中使用
`local_reg.*`、`global_reg.*` 等键，不能直接套用上面的导航定位参数表。
`pcd_inter_frame_demo.launch.yaml` 额外提供 `source_pcd`、`target_pcd`、
`publish_rate` 等 PCD 输入参数。该演示入口与导航定位入口的 TF
用途不同；导航定位使用 `navigation_localization.launch.yaml`。
