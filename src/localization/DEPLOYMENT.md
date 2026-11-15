# 独立工作空间部署

独立工作空间的目录结构：

```text
Nav3_Localization_ws/
├── src/
│   ├── localization/             # kiss_matcher_localization
│   ├── pcd_cloud_publisher/      # PCD 模拟输入，调试时使用
│   ├── localization_visualizer/  # 地图叠加与 RViz
│   └── localization_path_recorder/ # TF 轨迹记录
├── vendor/                     # 五份算法源码
├── external/                   # 本机从源码生成的算法安装前缀
└── build_algorithms/           # 构建时生成
```

ROS 包之间没有编译依赖。定位包使用 `external/` 中由本地源码编译的
`kiss_matcher`、`robin` 和 `small_gicp` C++ 库。
不要把原工作空间的 `install/setup.bash` 当作目标工作空间的长期依赖。
目标机器还需 ROS 2 Humble、CMake ≥ 3.22、PCL、Eigen3、TBB、FLANN、LZ4 等
依赖；具体名称见各包的 `package.xml`。

跨电脑部署请携带并在目标电脑编译 `vendor/` 中的算法源码，而不是
复制另一台电脑编译好的 `.a` 或 `.so`。源码清单、离线构建脚本及
当前验证状态见 [从源码构建说明](deployment/README.md)。

## 准备数据

- 将带 `region_id` 的全局地图复制到
  `src/localization/maps/building_split_space.pcd`，或启动定位
  时用 `map_pcd` 参数给出绝对路径。
- 将同一地图版本的 `region_labels.csv` 和区域候选配置放入对应目录；
  `space_2` 的候选位姿配置需要相同的雷达安装姿态和地图坐标。
- 若使用模拟输入，将局部点云复制到
  `src/pcd_cloud_publisher/clouds/part_1_hard.pcd`，或启动发布器时
  用 `pcd_path` 参数给出绝对路径。
- 两个 PCD 都被各自目录的 `.gitignore` 排除；仅复制源码仓库不会
  自动带上点云文件。

## 从源码构建

当前工作空间的 ROBIN 本地源码补丁已应用，直接运行：

```bash
# 在工作空间根目录执行
bash src/localization/deployment/build_from_source.sh
```

脚本会依次构建 ROBIN、KISS Matcher、small_gicp 和四个 ROS 2 包；
不需要先 source 旧工作空间。构建后的检查命令和源码版本见
[从源码构建说明](deployment/README.md)。

## 分别启动

调试 PCD 输入：

```bash
ros2 launch pcd_cloud_publisher local_map_from_pcd.launch.yaml
```

模拟发布器不提供同一时间的 `odom → base_link` TF；在当前候选初始化流程中，
单独启动它只能检查点云订阅和可视化，不能验证首次定位成功。

定位：

```bash
ros2 launch kiss_matcher_localization navigation_localization.launch.yaml \
  map_pcd:="$PWD/src/localization/maps/building_split_space.pcd" \
  region_labels_csv:="$PWD/src/localization/maps/region_labels.csv" \
  config_path:="$PWD/src/localization/config/continuous_localization.yaml" \
  initial_region:=space_2 region_multistart_enabled:=true \
  region_seed_file:="$PWD/src/localization/config/region_initialization_space.yaml" \
  local_map_topic:=/fast_lio/local_map local_map_reliability:=reliable
```

可视化：

```bash
ros2 launch localization_visualizer localization_visualizer.launch.yaml \
  map_pcd:="$PWD/src/localization/maps/building_split_space.pcd"
```

三个命令在各自终端运行，且每个终端都需先 source
`/opt/ros/humble/setup.bash` 与新工作空间的 `install/setup.bash`，
并设置 `LD_LIBRARY_PATH` 指向新工作空间的 `external/lib`；完整命令见
[运行终端](deployment/README.md#运行终端)。接入真实 FastLIO 后关闭模拟 PCD 发布器，
由 FastLIO 提供 `/local_map` 和 `odom -> base_link`。定位节点独占
`map -> odom` 的发布权。

## 验证状态

当前机器的 Humble 构建和 PCD 离线定位已通过：首次定位应用成功，
`map → odom` 与可视化对齐点云均已发布。调试数据没有 `odom → base_link`，
因此后续重定位的候选结果不会自动应用；真实 FastLIO 输入和导航系统
联调尚未验证。
