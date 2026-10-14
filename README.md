# KISS Matcher 独立定位工作空间

> 本项目是基于 [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)
> 开发的独立 ROS 2 定位扩展，不属于 MIT-SPARK 官方项目。

本目录用于从本地算法源码构建定位模块并验证部署。当前工作空间已在 ROS 2 Humble
完成源码构建和 PCD 离线定位验证，并验证了 Fast-LIO 局部地图话题接入；
实时配准效果与导航系统仍待联调。

定位实现参考 [MIT-SPARK/KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)。
项目自有代码采用 [MIT 许可](LICENSE)；随附算法及第三方源码的来源、
版权和许可见[第三方声明](THIRD_PARTY_NOTICES.md)。

## 目录

- `src/localization/`：定位节点、服务、全局地图和部署文档。
- `src/pcd_cloud_publisher/`：调试用 `/local_map` 发布节点。
- `src/localization_visualizer/`：独立可视化节点和 RViz 配置。
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

脚本加载 `/opt/ros/humble`，依次构建三个算法库和三个 ROS 包；
构建结果分别位于 `build_algorithms/`、`external/`、`build/` 和
`install/`。迁移工作空间后，旧 CMake 构建缓存中的绝对路径需要重新生成。
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
grep -E '^(robin|kiss_matcher|small_gicp)_DIR:' \
  build/kiss_matcher_localization/CMakeCache.txt
```

前三项应都指向本工作空间的 `install/`，最后三项应指向
`external/lib/cmake/`。检查动态库的命令也见源码构建说明。

## 3. 使用 PCD 调试输入

在三个终端分别切换到工作空间根目录，再执行对应命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch pcd_cloud_publisher local_map_from_pcd.launch.yaml
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch localization_visualizer localization_visualizer.launch.yaml \
  map_pcd:="$PWD/src/localization/maps/3_part.pcd"
```

当前工作空间已包含全局地图 `src/localization/maps/3_part.pcd` 和调试点云
`src/pcd_cloud_publisher/clouds/part_1_hard.pcd`。完整导航接口见
[导航接口文档](src/localization/NAVIGATION_INTERFACE.md)。

上述两份 PCD 是本机验证数据，已被 Git 忽略；从 GitHub 克隆后需要自行
提供数据，或通过 launch 参数指定其他 PCD 路径。

2026-10-12 的 Humble 离线测试中，首次定位成功应用：43 个 KISS 内点、
99.69% VGICP 重叠率，`map → odom` 和对齐点云均已发布。调试 PCD 不提供
`odom → base_link`，因此后续重定位虽匹配成功，但候选结果不会自动应用。

## 4. 接入 Fast-LIO 局部地图

Fast-LIO 发布 `/fast_lio/local_map`（点坐标与 `header.frame_id` 均为
`odom`）及 `odom → base_link` TF 后，在工作空间根目录启动定位：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/external/lib:${LD_LIBRARY_PATH:-}"
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml \
  map_pcd:="$PWD/src/localization/maps/scans.pcd" \
  local_map_topic:=/Laser_map \
  local_map_reliability:=reliable
```

此处的 `scans.pcd` 是当前本机的全局地图文件，不随 Git 分发。运行时无需
`pcd_cloud_publisher`；定位结果见 `/localization/status`，成功应用后发布
`map → odom`。`/Laser_map` 是 Fast-LIO 持续累积的地图，体量会随运行增加；
使用滑动窗口点云时改为 `local_map_topic:=/fast_lio/local_map` 并省略
`local_map_reliability`。是否匹配成功取决于地图与实时点云的实际重叠情况。

需要在匹配时观察两张点云，可构建 `localization_visualizer`，并在另一终端运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch localization_visualizer localization_visualizer.launch.yaml \
  map_pcd:="$PWD/src/localization/maps/scans.pcd" \
  local_map_topic:=/Laser_map \
  local_map_reliability:=reliable \
  map_voxel_resolution:=0.3 \
  local_voxel_resolution:=0.3
```

一个 RViz 窗口并排显示左侧全局图和右侧未对齐的原始局部图；
匹配成功并发布 `map → odom` 后，左侧还会叠加对齐后的局部图。
右侧为便于比较做了显示平移，不代表定位变换。
详见[可视化包说明](src/localization_visualizer/README.md)。
