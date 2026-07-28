# 真实仓库地图审计（2026-07-21）

数据源为 `maps.zip`，大小 128,203,277 字节，SHA-256：

`F4D9A7115B601E17B50259223D49239DA4EDEE576FFA0DC4356EA292940C819C`

审计对象是 `warehouse_baseline_20260721_095645/session_1784599040.792685`。完整机器可读结果见 `real_map_audit_20260721.json`，可用以下命令复现：

```powershell
python tools/analyze_map_session.py C:\path\to\maps.zip --output docs\real_map_audit_20260721.json
```

## 正式层统计

| 层 | 点数 | 0.25 m XY 格 | 0.25 m XYZ 体素 |
|---|---:|---:|---:|
| registration | 1,015,062 | 21,084 | 66,209 |
| display | 2,094,010 | 21,384 | 70,614 |
| ground | 902,795 | 20,992 | 52,001 |
| objects_raw | 333,357 | 9,135 | 31,800 |
| objects_clean | 287,927 | 6,735 | 26,729 |
| objects_filtered | 151,050 | 6,989 | 21,651 |
| display_full | 151,050 | 6,989 | 21,651 |

所有上述 PCD 的 XYZ 都是有限值。`objects_clean` 的全部 287,927 个唯一点都在 `objects_raw` 中，占 raw 唯一点的 86.37%。`objects_filtered` 与 clean 只有 127,771 个唯一点相交，因此 filtered 不是 clean 的简单同代子集。

按 V1 高度场参数（0.25 m XY、层间最大合并间隔 0.18 m、每层至少 6 点、每格最多 3 层）离线投影后，共有 5,440 个有效高度格和 6,379 个高度层；其中单层格 4,733 个、双层格 475 个、三层格 232 个，多层格合计 707 个。该数字是 clean PCD 的几何投影，不等于正式 mature 统计；由于旧会话缺少静态证据文件，其正式 mature cell 仍为 0。

## 关键问题

1. 会话没有 `manifest.yaml`，无法证明五层地图属于同一 generation，也没有 map UUID 或正式文件哈希契约。
2. 会话没有 `static_evidence.csv`。clean PCD 只能证明某次清理结果，不能证明时间成熟度或操作员批准；其正式静态权威必须视为未验证。
3. `map_display_full.pcd` 与 `map_objects_filtered.pcd` SHA-256 完全相同，均为 151,050 点。这说明旧 `display_full` 实际不是完整显示图，不能作为完整静态地图或安全基线。
4. 80 个关键帧和 `poses_raw.txt` 存在，但旧格式没有把它们纳入文件级哈希和完整性事务。

因此本数据可用于算法回放和离线批准流程，但在补齐 manifest、哈希、UUID 和静态权威之前，不能单独授权正式 14/17/18。
