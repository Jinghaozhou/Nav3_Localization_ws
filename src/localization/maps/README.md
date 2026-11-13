# 全局地图

> **2026-11-04 当前地图更新**：默认参考图已切换为 `kiss_localization_ws1.1/src/localization/maps/building_split_space.pcd`，由楚昕 `building_split(space).bin` 转换，区域统一为 `space_1`～`space_13`（region_id=1～13）。保持新图原生坐标；旧入口楼梯对应新图 **space_2**。一键启动默认 space_2，使用 `mid360_lidar/config/region_initialization_space.yaml` 的15个迁移候选；其他区域尚无候选库，使用区域KISS及全图回退。旧3_part和旧候选图保留用于历史复现，不再默认调用。详见 [space分区地图操作手册](../../../..//home/pc/人形/项目文档/01_操作手册/地图与区域初始化.md)；新地图接入已完成离线和隔离节点测试，尚待新一轮实扫验证。

此目录存放导航定位使用的全局点云地图。当前本机放有
`3_part.pcd`（当前WYN带region_id版本，24,807,030个点，约568MiB）及region_labels.csv。ID1为入口楼梯；ID0仍含旧完整图，不是楼梯补集。
地图中的点坐标必须处于导航所用的 `map` 坐标系。

`*.pcd`、`*.ply`、`*.bin` 默认不纳入 Git。部署到另一台机器时，
请将所需地图复制到本目录后构建功能包；CMake 会把 PCD 文件安装到
`share/kiss_matcher_localization/maps/`，但跳过体量较大的 `scans.pcd`。
也可以在启动时通过
`map_pcd:=/absolute/path/to/map.pcd` 使用包外地图。
例如，在工作空间根目录可用 `map_pcd:="$PWD/src/localization/maps/scans.pcd"`。
导航 launch 默认使用安装后的 `maps/3_part.pcd`。若该文件不存在，
请先放入地图并重新构建，或显式指定 `map_pcd`。


2026-11-02离线核验发现当前3_part在入口楼梯上部缺少部分结构；scans.pcd有可补充的几何，但坐标原点不同且无region_id。经坐标变换的小范围参考可将后期同初值VGICP从44.1%提升至94.4%，仍不代表首次粗配准成功。当前未切换地图；不要直接将scans改名覆盖。矩阵、候选和证据见[核验报告](/home/pc/人形/联调记录/20261102_162807_覆盖与楼层核验/核验报告.md)。

## 2026-11-02 完整候选（未替换默认图）

`3_part_complete_candidate.pcd`由scans.pcd通过T_current_from_scans刚体对齐生成；34,513,591点，ID1=3,080,186、ID0=31,433,405（此候选中0为补集）。字段x/y/z/rgb/region_id，每点20字节；RGB保留，法向量和强度未输出。旧3_part及其安装链接保留。候选通过源码绝对路径显式加载，CMake不复制到install。构建与验证见 `/home/pc/人形/联调记录/20261102_完整地图与区域初始化实现/map_build/README.md`。入口标签按旧ID1表面转写，不会自动将全部新楼层标成入口。
