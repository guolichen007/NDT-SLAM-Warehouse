# Map Frame Convention（冻结）

> 这是数学坐标约定，非现场物理事实。现场只需测量「轨道正方向是哪一侧」与
> `rail_yaw_in_map_rad` 的真实值，不需要重新发明坐标系约定。

```text
map_frame_convention_id =
ndt-map-rh-zup-yaw-ccw-rail-angle-from-map-x-v1
```

## 约定内容

- 坐标系：右手系（right-handed）
- +Z：向上（up）
- Yaw 正方向：绕 +Z 逆时针（counter-clockwise, CCW）
- `rail_yaw_in_map`：从 `map +X` 到「规定的轨道正方向」的有符号角
- 范围：规范化到 `(-pi, pi]`
- `map` frame 原点与轴向由 persistent map 冻结时定义，由
  `map_frame_uuid` 唯一标识（path-independent semantic identity）。

## 与现场的关系

- `map_frame_convention_id` 已冻结，现场不需要再定义。
- 现场只确定：轨道正方向是 +X 还是 +Y 侧（即 `rail_yaw_in_map_rad` 的真实值）。
- 候选值来自 Bag 只读轴向分析：约 0.031°（≈0.00054 rad），`verified=NO`。
