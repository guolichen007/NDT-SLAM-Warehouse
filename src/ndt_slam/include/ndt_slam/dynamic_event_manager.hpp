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

// 动态事件
struct DynamicEvent {
    int id;
    DynamicEventType type;
    double start_time;
    double end_time;
    bool active = true;
    bool confirmed = false;
    bool applied_to_keyframes = false;

    // 吊货相关
    SweptVolume payload_swept_volume;

    // 人体相关
    Capsule3D human_capsule;

    // 受影响的关键帧 ID
    std::vector<int> affected_keyframe_ids;
};

// DynamicEventManager 配置
struct DynamicEventConfig {
    bool enabled = true;

    // 吊货会话
    int payload_min_candidate_frames = 3;
    double payload_pre_guard_sec = 8.0;
    double payload_post_guard_sec = 15.0;

    // 人体事件
    double human_pre_guard_sec = 2.0;
    double human_post_guard_sec = 3.0;
    double human_capsule_radius = 0.65;
    bool human_use_track_height = true;
    double human_z_margin = 0.3;

    // CleanMap deny gate
    bool clean_deny_enabled = true;
    double max_dynamic_ratio = 0.2;
};

class DynamicEventManager {
public:
    DynamicEventManager() = default;

    void configure(const DynamicEventConfig& config);
    const DynamicEventConfig& getConfig() const { return config_; }

    // 创建吊货会话事件
    int createPayloadSession(double first_candidate_time, double current_time);

    // 更新吊货会话（有新的 candidate 或 dynamic 确认）
    void updatePayloadSession(int event_id, double current_time,
                              const Eigen::Vector3d& centroid,
                              const Box3D& bbox);

    // 确认吊货会话为动态
    void confirmPayloadSession(int event_id, double current_time);

    // 结束吊货会话
    void endPayloadSession(int event_id, double current_time);

    // 创建人体轨迹事件
    int createHumanEvent(double start_time, double current_time,
                         const std::deque<Eigen::Vector3d>& centroid_history,
                         double z_min, double z_max);

    // 结束人体事件
    void endHumanEvent(int event_id, double current_time);

    // 获取所有事件
    const std::vector<DynamicEvent>& getEvents() const { return events_; }

    // 获取活跃事件
    std::vector<const DynamicEvent*> getActiveEvents() const;

    // 获取已确认的事件
    std::vector<const DynamicEvent*> getConfirmedEvents() const;

    // 结束所有活跃事件
    void finalizeActiveEvents(double current_time);

    // 检查点是否在任何动态事件的 deny 区域内
    bool isPointDenied(const pcl::PointXYZ& p, double timestamp) const;

    // 获取 BEV deny cells
    std::set<std::pair<int,int>> getDenyCells(double bev_resolution, double timestamp) const;

    // 清理过期事件
    void cleanupExpiredEvents(double current_time, double max_age_sec = 300.0);

    // 获取事件数量
    int getEventCount() const { return events_.size(); }
    int getActiveCount() const;
    int getConfirmedCount() const;

private:
    DynamicEventConfig config_;
    std::vector<DynamicEvent> events_;
    int next_event_id_ = 0;
    mutable std::mutex mutex_;
};

} // namespace ndt_slam
