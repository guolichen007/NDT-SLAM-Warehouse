# 地图生命周期

## 工作层与正式层

运行线程维护 `global/display/ground/objects/objects_clean` 工作指针。MapCommit 先更新 raw 层并推进 objects version，然后请求后台 clean 重建。

Clean Worker 在一个锁内深拷贝 raw bundle N，锁外构建 clean N，完成后组成：

```text
MapLayerBundle {
  generation, objects_version, lifecycle_epoch, source_stamp,
  registration, display, ground, objects, objects_clean
}
```

五个指针在 bundle 完成后均为只读。ROS 发布与正式多层保存只读取同一个 completed bundle，所以同一 `header.seq` 对应同一内容代次，而不是同一复制时刻的混合层。

## 并发与活性

若工作地图在 build N 期间前进到 N+1：

- 完整 N 仍发布，避免持续提交造成 clean 饥饿；
- clean N 不安装到当前工作地图；
- 调度 N+1 clean 重建。

reset/load 会增加 lifecycle epoch。旧 epoch 的后台结果直接丢弃，不能在新地图之后重新发布。

## 写入门控

`STATIONARY_HOLD`、`MOVING_CONFIRM`、`CATCH_UP`、prediction-only 和严重退化均禁止 local map 更新与持久 MapCommit。两类权限分别统计，不能用一个布尔量隐式代替。

## 保存

正式五层 PCD 来自 latest completed bundle。诊断层可使用当前工作快照，并在文件名与文档中保持其非正式性质。
