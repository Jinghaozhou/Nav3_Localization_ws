# 点云地图格式转换

本目录保存 FAST-LIO 生成的 PCD 地图。已安装带 PCD I/O 插件的 CloudCompare，可直接打开和另存为 PCD。下面的 PLY 转换方法保留作备用。

## 直接编辑 PCD

启动带插件的版本：

```bash
cloudcompare-pcd
```

在 `File -> Open` 中选择 `.pcd`；编辑后在 `File -> Save` 中选择 **PCD cloud** 格式，并使用新的文件名，避免覆盖原图。

命令行也可直接打开：

```bash
cloudcompare-pcd "$PWD/src/localization/maps/building_split_space.pcd"
```

应用菜单中选择 **CloudCompare (PCD)**。上面的相对路径命令应在工作空间根目录执行。

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
pcl_pcd2ply src/localization/maps/scans.pcd /tmp/scans.ply
```

通用格式：

```bash
pcl_pcd2ply <输入文件.pcd> <输出文件.ply>
```

转换后检查：

```bash
ls -lh src/localization/maps/scans.pcd /tmp/scans.ply
```

## 使用 CloudCompare 打开

```bash
cloudcompare-pcd /tmp/scans.ply
```

也可以在 CloudCompare 中通过 `File -> Open` 选择 PLY 文件。

## 将 PLY 转换为 PCD

在本目录将 `scans.ply` 转成新的 PCD 文件：

```bash
cd src/localization/maps
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
