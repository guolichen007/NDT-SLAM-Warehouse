# ndt_slam ROS Package

这是 NDT-SLAM-Warehouse 的核心 ROS 包，包含双雷达合并、NDT_OMP 建图、动态物体过滤、长期在线建图和定位运行节点。

完整项目说明见仓库根目录 [README.md](../../README.md)。

详细文档见 [doc/](doc/) 目录。

## 常用入口

```bash
# 主建图
roslaunch ndt_slam mapping.launch

# 长期在线建图
roslaunch ndt_slam warehouse_live_longterm_mapping.launch

# 定位运行
roslaunch ndt_slam warehouse_runtime.launch
```
