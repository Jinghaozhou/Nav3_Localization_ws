# 独立定位可视化包

`localization_visualizer` 与定位算法、服务和自定义消息没有编译依赖。
它读取可选的全局 PCD、订阅局部点云，并发布真实坐标系下的三路可视化点云：

| 输出话题 | 坐标系 | 内容 |
| --- | --- | --- |
| `/localization_visualization/global_map` | `map` | 启动时加载并降采样的全局地图 |
| `/localization_visualization/local_map_odom` | `odom` | 降采样后的原始局部地图；无需定位成功即可显示 |
| `/localization_visualization/aligned_local_map` | `map` | 按最新 `map -> odom` 对齐的局部地图 |

默认只启动一个 RViz 窗口，在同一 3D 视区左右摆放两张点云。左侧灰色是
全局地图，右侧橙色是 `odom` 中未经配准的局部地图；定位成功后，左侧
额外显示青色的对齐结果。窗口内的文字标签说明每张图的来源。
为实现并排显示，节点还发布以下仅供观察的话题：

| 输出话题 | 坐标系 | 内容 |
| --- | --- | --- |
| `/localization_visualization/comparison/global_map` | `localization_comparison` | 左侧全局地图 |
| `/localization_visualization/comparison/local_map` | `localization_comparison` | 右侧原始局部图，平移到全局图右侧 |
| `/localization_visualization/comparison/aligned_local_map` | `localization_comparison` | 定位成功后叠加在左侧的配准结果 |
| `/localization_visualization/comparison/labels` | `localization_comparison` | 两侧文字标签 |

`localization_comparison` 是专用显示画布，不是机器人实际坐标系；右侧平移
不代表定位结果，也不要用这个话题计算位姿。右侧点云会随累计范围变化
重新平移，以保持它与左侧至少相隔 `comparison_gap_m`（默认 10 m）。
两侧点云共用一个相机视角，不能单独旋转或缩放其中一侧。

输入局部点云的 `header.frame_id` 必须是 `odom`。节点只读取 TF，
不会发布 TF，也不会影响定位结果。所有可视化输出使用 reliable +
transient-local QoS。独立查看真实坐标系时，可在启动命令中传入
`rviz_path:=$(ros2 pkg prefix localization_visualizer)/share/localization_visualizer/rviz/localization.rviz`
查看 `map` 下的叠加图；原始 `odom` 点云仍可通过
`/localization_visualization/local_map_odom` 单独订阅。
地图和局部地图的可视化体素大小默认均为
`0.2 m`，不改变定位包自己的 `0.1 m` 处理参数。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select localization_visualizer --event-handlers console_direct+
source install/setup.bash
```

## 运行

定位节点与局部地图发布节点已经运行后，在工作空间根目录的另一终端
启动单个 RViz 窗口，查看 `scans.pcd` 和 Fast-LIO 的 `/Laser_map`：

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

`/Laser_map` 是可靠发布的大点云，故此处指定可靠订阅，并用 `0.3 m`
体素降低 RViz 负载。启动后左侧灰色地图和右侧橙色点云并排显示；
若视野没有完整覆盖两侧，可在 RViz 中调整缩放。部署到另一个工作空间时，可以把该包与
`kiss_matcher_localization`、`pcd_cloud_publisher` 并列放在 `src/`。
可视化包不引用另两个包的源码或安装路径；只需要在启动时指定地图路径。
不指定 `map_pcd` 时，仍可显示对齐后的局部地图，但不会发布全局地图。

离线 PCD 测试时，可用
`map_pcd:="$PWD/src/localization/maps/3_part.pcd"` 指定对应的全局地图。

全局地图启动时只读取、体素化一次；随后发布一次，由
transient-local QoS 保留最后一条消息供稍后启动的 RViz 订阅。
局部地图只保留最新一份，并默认每 `1 s` 降采样、发布一次。
只有收到 `map -> odom` 后才会额外发布对齐后的局部地图。
