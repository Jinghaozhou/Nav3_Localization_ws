# 定位轨迹记录

`localization_path_recorder` 从 TF 查询 `map ← base_link`，将定位成功后的
位姿持续发布为 `map` 坐标系的 `/localization/map_pose`
（`geometry_msgs/msg/PoseStamped`）和累计轨迹 `/localization/map_path`
（`nav_msgs/msg/Path`）。它只读取 TF，不发布或修改 TF，也不参与配准。
定位模块应用结果、发布 `map → odom`，且 Fast-LIO 发布
`odom → base_link` 后，节点才开始记录。若只得到未应用的候选位姿，
轨迹不会出现。

## 构建与启动

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select localization_path_recorder localization_visualizer
source install/setup.bash
ros2 launch localization_path_recorder localization_path_recorder.launch.yaml
```

已有定位与可视化节点可照常运行。`localization_visualizer` 的默认 RViz
配置会把黄色轨迹和紫红色当前位置箭头叠加在左侧全局地图上。
轨迹的原始话题始终使用 `map` frame；可视化包另发布专用画布副本，
不会用显示平移冒充定位结果。

## 参数与行为

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map_frame` | `map` | 全局地图 frame |
| `tracked_frame` | `base_link` | 被记录的 frame；需要雷达原点轨迹时可设为 `lidar` |
| `sample_hz` | `5.0` | 查询 TF 的频率 |
| `min_distance_m` | `0.05` | 位移达到此值才向 Path 增加一点 |
| `max_tf_age_sec` | `1.0` | 超过此时长的 TF 不记录 |
| `max_poses` | `10000` | 内存中保留的最多轨迹点数 |
| `use_sim_time` | `false` | 回放带 `/clock` 的 bag 时设为 `true` |

位置和朝向都从 TF 直接读取。`/localization/map_pose` 随新 TF 更新时间戳，
即使位移未达到阈值也会发布；`/localization/map_path` 只在新增轨迹点时发布。
两个输出均为 reliable + transient-local，晚启动的 RViz 可取得当前轨迹。
TF 暂时缺失或过期时会跳过采样，不重复旧点；回放时间回退超过 1 秒时
清空旧路径并重新开始。轨迹只保存在节点内存，重启节点后重新记录。
