# Diagnostic Behavior Proof — 证明本 forensic 不改产品行为

## 判定

```text
ALGORITHM_CHANGED = NO
PRODUCT_BEHAVIOR_CHANGED = NO
```

## 证据

### 1. 环境变量门控，默认关闭

```cpp
// ndt_slam.cpp 构造函数
const char* d5_env = std::getenv("NDT_D5_FRAGMENT_FORENSIC");
d5_fragment_forensic_enabled_ =
    d5_env != nullptr &&
    (std::string(d5_env) == "1" || std::string(d5_env) == "true");
```

默认（未设置环境变量）时 `d5_fragment_forensic_enabled_ = false`，所有 forensic 代码路径短路。

### 2. 接线位置：clustering 之后、只读

```cpp
// detectCargoAroundOdomAnchor，EuclideanClusterExtraction 之后
if (d5_fragment_forensic_enabled_) {
    D5FragmentForensicConfig d5_config;
    d5_config.cluster_tolerance_m = odom_anchor_config_.tight_box.component_cluster_tolerance_m;  // 只读
    d5_config.min_cluster_size = odom_anchor_config_.weak_min_points;                              // 只读
    d5_config.voxel_leaf_size_m = 0.05F;
    const D5FragmentForensicResult d5_result = analyzeD5FragmentForensic(   // 纯函数
        *voxel_cloud, cluster_indices, stamp.toSec(), d5_config);
    dumpD5FragmentForensic(d5_result, stamp.toSec());                        // 只写 CSV
}
```

- `analyzeD5FragmentForensic` 是纯函数：输入 `const&`，返回新对象，不修改 `voxel_cloud` / `cluster_indices` / 任何产品状态。
- `dumpD5FragmentForensic` 只写 `/tmp/cargo_forensic/d5_*.csv`，不读回任何产品决策。

### 3. 产品数据流零改动

diff 检查（DIAGNOSTIC_DIFF.patch，5 文件）：

| 文件 | 改动性质 |
|---|---|
| `cargo_d5_fragment_forensic.hpp`（新增） | 纯结构定义 + 纯函数声明 |
| `cargo_d5_fragment_forensic.cpp`（新增） | 纯函数实现 + CSV 写入 |
| `ndt_slam.hpp` | +1 include、+成员（ofstream×2、bool×1）、+方法声明 |
| `ndt_slam.cpp` | +2 构造函数 env 读取、+1 接线块、+1 dump 方法实现、+1 `<cstdlib>` include |
| `CMakeLists.txt` | +1 源文件 |

**没有任何产品逻辑（detect/identity/geometry/bottom/safety/obstacle/map）的语义被改变**。所有新增代码都受 `d5_fragment_forensic_enabled_` 门控，且只读产品数据、只写诊断 CSV。

### 4. 编译验证

```text
catkin_make --pkg ndt_slam → exit 0
forensic 符号已进 libndt_slam_lib.so（analyzeD5FragmentForensic / dumpD5FragmentForensic）
```

### 5. runtime 验证（大件 replay）

```text
Bag Validation: PASS（23 passes, 0 failures）
产品链正常启动，forensic CSV 附加产出（d5_voxel_lineage.csv 273879 行、d5_weak_fragments.csv 4529 行）
```

## 结论

本 forensic 是 append-only 的观测器，产品行为在关闭（默认）和开启（`NDT_D5_FRAGMENT_FORENSIC=1`）两种情况下**完全一致**，唯一差异是是否多写两个诊断 CSV。
