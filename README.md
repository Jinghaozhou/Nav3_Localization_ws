# Nav3 Localization 工作空间

> 本项目是基于 [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)
> 开发的独立 ROS 2 定位扩展，不属于 MIT-SPARK 官方项目。

本工作空间 `Nav3_Localization_ws` 用于从本地算法源码构建定位模块并验证部署。历史 KISS 演示流程
已在 ROS 2 Humble 完成 PCD 离线验证；当前持续导航定位采用 wyn 的
候选位姿加 VGICP 流程，已完成构建、地图加载和接口检查，仍需现场匹配验证。

定位实现参考 [MIT-SPARK/KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)。
项目自有代码采用 [MIT 许可](LICENSE)；随附算法及第三方源码的来源、
版权和许可见[第三方声明](THIRD_PARTY_NOTICES.md)。

## 目录

- `src/localization/`：定位节点、服务、全局地图和部署文档。
- `src/pcd_cloud_publisher/`：调试用 `/local_map` 发布节点。
- `src/localization_visualizer/`：独立可视化节点和 RViz 配置。
- `src/localization_path_recorder/`：从 TF 记录全局地图中的定位轨迹。
- `vendor/`：KISS Matcher、ROBIN、small_gicp、PMC、xenium 源码。
- `external/`：构建脚本将在此安装算法头文件和库。

## 1. 从本地源码构建

确认本机已安装 ROS 2 Humble、colcon、CMake ≥ 3.22、C++ 编译器、PCL、
Eigen3、TBB、FLANN、LZ4 和 OpenMP。ROBIN 的本地源码补丁已经应用，
无需再次运行 `patch`。

```bash
# 在工作空间根目录执行
bash src/localization/deployment/build_from_source.sh
```

脚本加载 `/opt/ros/humble`，依次构建三个算法库和四个 ROS 包；
构建结果分别位于 `build_algorithms/`、`external/`、`build/` 和
`install/`。工作空间改名或移动后，需删除旧的 `build/`、
`build_algorithms/`、`install/` 和 `external/` 生成目录，再运行脚本重新构建；
CMake 缓存与 ROS 安装脚本包含旧绝对路径。
不要在构建前 source 旧 KISS-Matcher 工作空间。详细源码版本、
补丁用途和构建后检查命令见
[从源码构建说明](src/localization/deployment/README.md)。

## 2. 检查独立性

构建成功后，在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix kiss_matcher_localization
ros2 pkg prefix pcd_cloud_publisher
ros2 pkg prefix localization_visualizer
ros2 pkg prefix localization_path_recorder
grep -E '^(robin|kiss_matcher|small_gicp)_DIR:' \
  build/kiss_matcher_localization/CMakeCache.txt
```

前四项应都指向本工作空间的 `install/`，最后三项应指向
`external/lib/cmake/`。检查动态库的命令也见源码构建说明。

## 3. 接入 Fast-LIO 连续定位

当前导航定位使用 wyn 实测分区地图和 VGICP：首次定位从区域候选位姿开始，
成功后使用已有位姿跟踪；连续定位节点不运行 KISS 粗配准。
`inter_frame_alignment` 历史演示节点仍保留 KISS 代码。

Fast-LIO 需要发布 `odom → base_link` TF、`odom` 坐标系下的
`/fast_lio/local_map`；wyn 配置还可订阅较大的 `/fast_lio/local_map_global`
作为初始化或恢复输入。定位终端和 Fast-LIO 使用同一个 `ROS_DOMAIN_ID`。
本机已放入 `building_split_space.pcd`、区域名称表及 `space_2` 的
15 个候选位姿。候选位姿来自特定安装姿态的实验标定，现场使用前应核对。

在工作空间根目录启动 `space_2` 区域候选定位：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml \
  map_pcd:="$PWD/src/localization/maps/building_split_space.pcd" \
  region_labels_csv:="$PWD/src/localization/maps/region_labels.csv" \
  config_path:="$PWD/src/localization/config/continuous_localization.yaml" \
  initial_region:=space_2 \
  region_multistart_enabled:=true \
  region_seed_file:="$PWD/src/localization/config/region_initialization_space.yaml" \
  local_map_topic:=/fast_lio/local_map \
  local_map_reliability:=reliable
```

候选初始化要求输入点云和相同时间的 `odom → base_link` TF。
其他区域已有标签，但当前没有对应的候选位姿配置；不能仅切换 `initial_region`
就期望首次定位成功。关键帧候选模式是可选实验流程，其
`/localization/keyframe_candidates` 由 `wyn/导航工程/mid360_lidar`
中的检索器发布，还需要对应关键帧库、地图及配置哈希。详见
[导航接口说明](src/localization/NAVIGATION_INTERFACE.md)。

成功应用后定位包发布 `map → odom` 和 `/localization/status`。
手动设置当前区域：

```bash
ros2 service call /localization/set_region \
  kiss_matcher_localization/srv/SetRegion "{region: 'space_2'}"
ros2 service call /localization/relocalize \
  kiss_matcher_localization/srv/Relocalize "{apply_if_safe: true}"
```

## 4. 可视化和轨迹

在另一个终端启动并排比较视图：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch localization_visualizer localization_visualizer.launch.yaml \
  map_pcd:="$PWD/src/localization/maps/building_split_space.pcd" \
  local_map_topic:=/fast_lio/local_map \
  local_map_reliability:=reliable \
  map_voxel_resolution:=0.3 \
  local_voxel_resolution:=0.3
```

轨迹记录节点从 `map ← base_link` TF 发布 `/localization/map_path`，
可视化窗口以黄色轨迹和紫红色箭头显示：

```bash
ros2 launch localization_path_recorder localization_path_recorder.launch.yaml
```

## 5. 用 PCD 模拟点云输入

`pcd_cloud_publisher` 可以发布 `/local_map`，用于检查订阅和显示：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pcd_cloud_publisher local_map_from_pcd.launch.yaml
```

候选定位还需要相同时间的 `odom → base_link` TF、适用该点云的候选位姿
和对应全局地图；单独发布 PCD 不能完成 wyn 的首次定位流程。
地图 PCD 是本机部署数据，已被 Git 忽略，克隆仓库后须单独提供。

## 2026-11-23 experimental dual-source synchronization

The `wyn/race-auto-library-20261123` integration branch merges the local BBS/keyframe race, asynchronous query waiting, and exact voxel-map caches while retaining the shared navigation-permission freshness/failure gate. Only the authority publishes `map → odom` and `/localization/navigation_allowed`; private validators publish neither. `RaceInitialization.srv` validates epoch, source/map/profile versions, input age, timestamp TF and correction bounds before applying the winning independently confirmed proposal. KISS coarse fallback remains disabled.

The matching Sensor branch supplies `tools/mid360_lidar/scripts/start_scan.sh --mode race --region space_2`, data manifests and versioned automatic-library admission. Standard Nav3 bringup defaults are unchanged; do not launch both sensor entries together. Transfer matching non-Git map/library/BBS data separately. This branch has compile/component and isolated recorded-geometry checks; it does not claim Go2 extrinsic calibration or field navigation acceptance.


## space_10 candidate consumers

The matching Sensor experimental bundle adds independent space_10 BBS and virtual-template modes. Dynamic-only region profiles may explicitly omit fixed seeds; legacy empty-seed profiles are still rejected. The `virtual` source requires hierarchical validation and a bounded virtual shortlist with matching map/library/profile/region/frame and timestamp metadata. Initialization ambiguity, two-window confirmation, freshness and jump protection remain in force. Successful initialization switches to full-map prior VGICP. Existing navigation-permission checks and the single-authority race contract are retained; no KISS fallback is restored. Build this package again before using the matching Sensor branch. Map/model payloads remain separate from Git.
