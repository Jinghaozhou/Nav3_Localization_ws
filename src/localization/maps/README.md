# 全局地图

当前使用 wyn 的 `building_split_space.pcd` 作为导航参考地图，坐标属于
`map`。它包含 `region_id` 标签；`region_labels.csv` 将 1～13 映射为
`space_1`～`space_13`。旧的 `3_part.pcd` 保留作历史复现；两份同名
`3_part.pcd` 在迁移前后内容不同，不能混用坐标或区域配置。

`*.pcd`、`*.ply`、`*.bin` 不纳入 Git，换机部署须另行复制地图。
`building_split_space.pcd` 体积约 880 MB，CMake 不重复安装它；
`build_from_source.sh` 在安装目录创建指向本机地图的软链接。
也可在启动时显式传入
`map_pcd:="$PWD/src/localization/maps/building_split_space.pcd"`。
地图和 `config/region_initialization_space.yaml` 中的 `map_sha256`
应一致。没有该地图时不能使用默认启动路径。
