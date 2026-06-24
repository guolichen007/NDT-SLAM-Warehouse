# 动态物体过滤

## 过滤链路

```
merged_points
→ BasePayloadChannelFilter（吊货通道过滤）
→ PayloadTrackManager（吊货轨迹跟踪）
→ HumanObjectDynamicFilter（人体动态过滤）
→ DynamicEventManager（统一事件管理）
```

## 吊货通道过滤

在 base_link 坐标系下，从中间通道筛选吊货候选：

```yaml
base_payload_channel:
  enabled: true
  lateral_center: 0.0
  lateral_half_width: 3.0
  longitudinal_min: -10.0
  longitudinal_max: 10.0
```

## 吊货轨迹跟踪

双坐标系跟踪，确认 DYNAMIC_PAYLOAD：

```yaml
payload_tracker:
  enabled: true
  base_stability_std_thresh: 0.35
```

## 人体动态过滤

BEV 聚类 + 人体特征判断：

```yaml
human_object_filter:
  enabled: true
  min_hag: 0.35
  max_hag: 2.30
  min_cluster_height: 0.60
  max_cluster_height: 2.20
```

## 动态事件管理

PayloadSession 状态机：
```
CANDIDATE → CARRIED_MOVING → CARRIED_STOPPING → PLACED_STATIC → CLOSED
```
