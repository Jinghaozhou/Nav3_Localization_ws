# 点云地图格式转换

本目录保存 FAST-LIO 生成的 PCD 地图。当前 Ubuntu 安装的 CloudCompare 不支持直接读取 PCD，可先使用 PCL 工具将其转换为 PLY。

## 首次安装转换工具

只需安装一次：

```bash
sudo apt update
sudo apt install pcl-tools
```

检查工具：

```bash
command -v pcl_pcd2ply
command -v pcl_ply2pcd
```

## 将 PCD 转换为 PLY

在工作空间根目录执行：

```bash
cd /home/jinghao/Fast_lio_ws
pcl_pcd2ply maps/test.pcd maps/test.ply
```

通用格式：

```bash
pcl_pcd2ply <输入文件.pcd> <输出文件.ply>
```

转换后检查：

```bash
ls -lh maps/test.pcd maps/test.ply
```

## 使用 CloudCompare 打开

```bash
CloudCompare /home/jinghao/Fast_lio_ws/maps/test.ply
```

也可以在 CloudCompare 中通过 `File -> Open` 选择 PLY 文件。

## 将 PLY 转换为 PCD

在本目录将 `scans.ply` 转成新的 PCD 文件：

```bash
cd /home/jinghao/kiss_localization_ws/src/localization/maps
pcl_ply2pcd scans.ply scans_from_ply.pcd
```

通用格式：

```bash
pcl_ply2pcd <输入文件.ply> <输出文件.pcd>
```

转换后检查：

```bash
ls -lh scans_from_ply.pcd
```

## 注意事项

- 建议保留原始 PCD，不要直接覆盖或删除。
- 编辑后的地图另存为新文件，例如 `test_clean.ply`。
- FAST-LIO 地图通常包含坐标和 `intensity` 强度字段；CloudCompare 可将强度作为 Scalar Field 着色显示。
- PLY 转 PCD 时使用新的输出文件名，避免覆盖原有的 `scans.pcd`。
- 如果转换命令提示 `command not found`，请先安装 `pcl-tools`。
