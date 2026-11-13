> **目录说明（2026-10-24）**：实际工程已移至 `/home/pc/人形/kiss_localization_ws1.1`。`/home/pc/kiss_localization_ws1.1` 保留为兼容符号链接，构建缓存仍依赖该英文路径；请勿删除。两者是同一份文件。日常加载环境可用新目录，构建时遵循项目操作手册。

> **本机集成入口（2026-10-24）**：使用 `/home/pc/人形/项目文档/01_操作手册/扫描与连续定位.md` 的两个终端命令。项目组合启动会自动启动本工作空间的定位、可视化、TF轨迹节点，以及项目的成功标记/CSV节点，不需要重复启动轨迹包。默认计算结束后等待1s调度、120s/30s候选窗口、先验VGICP与全局回退。下文独立启动示例是原模块用法，不替代组合入口。
> 本机雷达配置当前 `det_range=25m`（见项目 `mid360_robot.yaml`），不是来件实验的10m；回放必须使用与数据话题匹配的输入，窗口联调优先遵循项目手册。

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
ros2 pkg prefix localization_path_recorder
grep -E '^(robin|kiss_matcher|small_gicp)_DIR:' \
  build/kiss_matcher_localization/CMakeCache.txt
```

前四项应都指向本工作空间的 `install/`，最后三项应指向
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

需要显示定位成功后的运动轨迹时，另开终端启动路径记录包：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch localization_path_recorder localization_path_recorder.launch.yaml
```

节点在 `map → base_link` TF 可用后发布 `map` 坐标系的
`/localization/map_path`；上面的 RViz 窗口会在左侧全局地图上用醒目的
黄色轨迹和紫红色当前位置箭头显示它。定位尚未成功、TF 链不完整时不会
生成轨迹。详见[路径记录包说明](src/localization_path_recorder/README.md)。

### 录制本次测试

在本工作空间根目录、与雷达和 Fast-LIO 相同 `ROS_DOMAIN_ID` 的终端，
**先启动录包，再开始扫描**：

```bash
source /opt/ros/humble/setup.bash
source ~/lio_ws/install/setup.bash
mkdir -p bags
ros2 bag record -o "bags/stair_10m_$(date +%Y%m%d_%H%M%S)" \
  /lidar_points /imu /Laser_map /fast_lio/odom /tf /tf_static \
  /localization/map_path /localization/map_pose
```

测试结束后在录包终端按 **Ctrl-C**，等待写盘完成，再用
`ros2 bag info bags/实际生成的目录名` 核对各话题消息数，尤其是
`/lidar_points`、`/imu` 和 `/Laser_map`。`bags/` 已被 Git 忽略。
`/Laser_map` 是累计地图，每条消息会随扫描时间增长；录制期间留意磁盘空间。
靖昊来件实验将 FAST-LIO 的最远处理距离调为 10 米（本机集成不据此修改量程），不影响 `/lidar_points` 的
原始数据，因此以后还可以用同一个包重跑不同的 FAST-LIO 距离参数。

回去只调定位参数时，可启动定位节点，将其 `use_sim_time` 设为 `true`，
然后只回放累计地图；这足以重复 KISS 与 VGICP 的匹配计算：

```bash
ros2 param set /localization/navigation_localizer use_sim_time true
ros2 bag play bags/实际生成的目录名 --clock 100 --topics /Laser_map
```

先设置 `use_sim_time`，再启动回放，否则历史点云可能被
`max_local_map_age_sec` 判为过期。只回放 `/Laser_map` 不提供
`odom → base_link`，候选位姿可能无法自动应用；要连同建图一起重跑，
按 Fast-LIO 工作空间的离线回放说明启动 `driver:=false use_sim_time:=true`，
仅回放 `/lidar_points`、`/imu`，同时让播放器发布 `/clock`。
