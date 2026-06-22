#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include <mutex>
#include <string>

namespace ndt_slam {

// 动态事件类型
enum class DynamicEventType {
    PAYLOAD_SESSION,    // 吊货会话
    HUMAN_TRACK         // 人体轨迹
};

// 吊货会话状态
enum class PayloadSessionState {
    CANDIDATE,          // 候选阶段：channel 内出现 candidate，但还没确认移动
    CARRIED_MOVING,     // 吊运移动中：map 下移动明显，确认吊货
    CARRIED_STOPPING,   // 减速/短暂停顿：可能正在放下
    PLACED_STATIC,      // 已停放：连续稳定若干秒，认为已经放下
    CLOSED              // 事件结束
};

// 3D 包围盒
struct Box3D {
    Eigen::Vector3d min_pt;
    Eigen::Vector3d max_pt;

    bool contains(const pcl::PointXYZ& p) const {
        return p.x >= min_pt.x() && p.x <= max_pt.x() &&
               p.y >= min_pt.y() && p.y <= max_pt.y() &&
               p.z >= min_pt.z() && p.z <= max_pt.z();
    }

    void expand(double xy, double z) {
        min_pt.x() -= xy; min_pt.y() -= xy; min_pt.z() -= z;
        max_pt.x() += xy; max_pt.y() += xy; max_pt.z() += z;
    }

    double iou(const Box3D& other) const {
        double inter_x = std::max(0.0, std::min(max_pt.x(), other.max_pt.x()) - std::max(min_pt.x(), other.min_pt.x()));
        double inter_y = std::max(0.0, std::min(max_pt.y(), other.max_pt.y()) - std::max(min_pt.y(), other.min_pt.y()));
        double inter_z = std::max(0.0, std::min(max_pt.z(), other.max_pt.z()) - std::max(min_pt.z(), other.min_pt.z()));
        double inter_vol = inter_x * inter_y * inter_z;
        double vol1 = (max_pt.x()-min_pt.x()) * (max_pt.y()-min_pt.y()) * (max_pt.z()-min_pt.z());
        double vol2 = (other.max_pt.x()-other.min_pt.x()) * (other.max_pt.y()-other.min_pt.y()) * (other.max_pt.z()-other.min_pt.z());
        double union_vol = vol1 + vol2 - inter_vol;
        return union_vol > 0 ? inter_vol / union_vol : 0;
    }
};

// 胶囊体（用于人体轨迹）
struct Capsule3D {
    std::deque<Eigen::Vector3d> centerline;
    double radius = 0.65;
    double z_min = 0.0;
    double z_max = 2.5;

    bool contains(const pcl::PointXYZ& p) const {
        if (p.z < z_min || p.z > z_max) return false;
        for (const auto& center : centerline) {
            double dx = p.x - center.x();
            double dy = p.y - center.y();
            if (dx*dx + dy*dy <= radius*radius) return true;
        }
        return false;
    }
};

// Swept Volume（吊货扫掠体积）
struct SweptVolume {
    std::deque<Eigen::Vector3d> centroid_history;
    std::deque<Box3D> bbox_history;
    double expand_xy = 0.5;
    double expand_z = 0.4;

    bool contains(const pcl::PointXYZ& p) const {
        for (const auto& bbox : bbox_history) {
            Box3D expanded = bbox;
            expanded.expand(expand_xy, expand_z);
            if (expanded.contains(p)) return true;
        }
        return false;
    }
};

// 吊货会话
struct PayloadSession {
    int id;
    PayloadSessionState state = PayloadSessionState::CANDIDATE;
    int track_id = -1;

    double first_candidate_time = 0;
    double moving_start_time = 0;
    double placed_time = 0;
    double end_time = 0;

    // 当前帧信息
    Eigen::Vector3d centroid_map = Eigen::Vector3d::Zero();
    Box3D current_bbox;
    double velocity = 0;
    double map_displacement = 0;

    // 历史
    std::deque<Eigen::Vector3d> centroid_history;
    std::deque<double> timestamp_history;
    std::deque<Box3D> bbox_history;

    // 稳定性检测
    int stable_frames = 0;
    double last_moving_time = 0;

    // Swept volume（只包含移动阶段的 bbox）
    SweptVolume swept_volume;

    // 停放保护 bbox
    Box3D placed_bbox;
    bool placed_protected = false;

    bool active = true;
    bool confirmed = false;
};

// 人体轨迹事件
struct HumanEvent {
    int id;
    double start_time;
    double end_time;
    bool active = true;
    Capsule3D capsule;
};

// DynamicEventManager 配置
struct DynamicEventConfig {
    bool enabled = true;

    // 吊货会话
    int payload_min_candidate_frames = 3;
    double payload_pre_guard_sec = 3.0;          // 从 8.0 降低到 3.0
    double moving_post_guard_sec = 2.0;          // 移动阶段的 post_guard
    double unknown_post_guard_sec = 5.0;         // 未检测到停放时的 post_guard

    // 会话合并
    bool merge_same_track = true;
    double merge_time_gap_sec = 3.0;
    double merge_iou_thresh = 0.2;
    int max_active_sessions = 3;

    // 停放检测
    bool placement_detection_enabled = true;
    double stable_window_sec = 4.0;
    int stable_frames_thresh = 5;
    double stable_map_disp_thresh_m = 0.15;
    double stable_velocity_thresh_mps = 0.04;
    double placed_bbox_expand_xy = 0.3;
    double placed_bbox_expand_z = 0.3;

    // 人体事件
    double human_pre_guard_sec = 2.0;
    double human_post_guard_sec = 3.0;
    double human_capsule_radius = 0.65;
    bool human_use_track_height = true;
    double human_z_margin = 0.3;

    // CleanMap deny gate
    bool clean_deny_enabled = true;
    double max_dynamic_ratio = 0.2;

    // 分层地图策略
    bool placed_to_objects_clean = true;
    bool placed_to_display_map = true;
    bool placed_to_registration_map = false;      // 默认不进定位图
};

class DynamicEventManager {
public:
    DynamicEventManager() = default;

    void configure(const DynamicEventConfig& config);
    const DynamicEventConfig& getConfig() const { return config_; }

    // ========== 吊货会话管理 ==========

    // 查找或创建吊货会话（自动去重合并）
    int findOrCreatePayloadSession(int track_id, double current_time,
                                    const Eigen::Vector3d& centroid,
                                    const Box3D& bbox, double velocity);

    // 更新吊货会话
    void updatePayloadSession(int event_id, double current_time,
                              const Eigen::Vector3d& centroid,
                              const Box3D& bbox, double velocity,
                              double map_displacement);

    // 确认吊货会话为动态
    void confirmPayloadSession(int event_id, double current_time);

    // 结束吊货会话
    void endPayloadSession(int event_id, double current_time);

    // ========== 人体事件管理 ==========

    int createHumanEvent(double start_time, double current_time,
                         const std::deque<Eigen::Vector3d>& centroid_history,
                         double z_min, double z_max);

    void endHumanEvent(int event_id, double current_time);

    // ========== 查询接口 ==========

    const std::vector<PayloadSession>& getPayloadSessions() const { return payload_sessions_; }
    const std::vector<HumanEvent>& getHumanEvents() const { return human_events_; }

    std::vector<const PayloadSession*> getActivePayloadSessions() const;
    std::vector<const PayloadSession*> getPlacedSessions() const;

    // ========== Mask 生成 ==========

    // 获取 dynamic deny cells（只包含移动阶段）
    std::set<std::pair<int,int>> getDynamicDenyCells(double bev_resolution, double timestamp) const;

    // 获取 static protect cells（停放位置）
    std::set<std::pair<int,int>> getStaticProtectCells(double bev_resolution, double timestamp) const;

    // 检查点是否在停放保护区域内
    bool isPointInPlacedProtect(const pcl::PointXYZ& p, double timestamp) const;

    // ========== 生命周期 ==========

    void finalizeActiveEvents(double current_time);
    void cleanupExpiredEvents(double current_time, double max_age_sec = 300.0);

    int getActiveCount() const;
    int getPlacedCount() const;
    int getEventCount() const { return payload_sessions_.size() + human_events_.size(); }

private:
    // 查找匹配的 active session
    PayloadSession* findMatchingSession(int track_id, double current_time,
                                         const Box3D& bbox);

    // 更新停放检测
    void updatePlacementDetection(PayloadSession& session, double current_time);

    DynamicEventConfig config_;
    std::vector<PayloadSession> payload_sessions_;
    std::vector<HumanEvent> human_events_;
    int next_session_id_ = 0;
    int next_human_id_ = 0;
    mutable std::mutex mutex_;
};

} // namespace ndt_slam
