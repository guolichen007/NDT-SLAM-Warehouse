#include "ndt_slam/dynamic_event_manager.hpp"
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

namespace ndt_slam {

void DynamicEventManager::configure(const DynamicEventConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

// ========== 吊货会话管理 ==========

PayloadSession* DynamicEventManager::findMatchingSession(int track_id, double current_time,
                                                          const Box3D& bbox) {
    for (auto& session : payload_sessions_) {
        if (!session.active) continue;
        if (session.state == PayloadSessionState::CLOSED ||
            session.state == PayloadSessionState::PLACED_STATIC) continue;

        // 同一 track_id
        if (config_.merge_same_track && session.track_id == track_id) {
            return &session;
        }

        // 时间间隔检查
        double time_gap = current_time - session.end_time;
        if (time_gap > config_.merge_time_gap_sec) continue;

        // bbox IoU 检查
        if (session.current_bbox.iou(bbox) > config_.merge_iou_thresh) {
            return &session;
        }
    }
    return nullptr;
}

int DynamicEventManager::findOrCreatePayloadSession(int track_id, double current_time,
                                                     const Eigen::Vector3d& centroid,
                                                     const Box3D& bbox, double velocity) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 查找匹配的 session
    PayloadSession* existing = findMatchingSession(track_id, current_time, bbox);

    if (existing) {
        // 更新已有 session
        existing->centroid_map = centroid;
        existing->current_bbox = bbox;
        existing->velocity = velocity;
        existing->end_time = current_time;
        existing->centroid_history.push_back(centroid);
        existing->timestamp_history.push_back(current_time);
        existing->bbox_history.push_back(bbox);

        // 限制历史长度
        size_t max_history = 100;
        while (existing->centroid_history.size() > max_history) {
            existing->centroid_history.pop_front();
            existing->timestamp_history.pop_front();
            existing->bbox_history.pop_front();
        }

        ROS_DEBUG("[PayloadSession] update id=%d, track=%d, state=%d",
                  existing->id, track_id, (int)existing->state);
        return existing->id;
    }

    // 创建新 session
    if ((int)getActiveCount() >= config_.max_active_sessions) {
        return -1;  // 超过最大活跃数
    }

    PayloadSession session;
    session.id = next_session_id_++;
    session.track_id = track_id;
    session.state = PayloadSessionState::CANDIDATE;
    session.first_candidate_time = current_time;
    session.end_time = current_time;
    session.centroid_map = centroid;
    session.current_bbox = bbox;
    session.velocity = velocity;
    session.centroid_history.push_back(centroid);
    session.timestamp_history.push_back(current_time);
    session.bbox_history.push_back(bbox);
    session.active = true;

    payload_sessions_.push_back(session);
    ROS_INFO("[PayloadSession] create id=%d, track=%d", session.id, track_id);
    return session.id;
}

void DynamicEventManager::updatePayloadSession(int event_id, double current_time,
                                                const Eigen::Vector3d& centroid,
                                                const Box3D& bbox, double velocity,
                                                double map_displacement) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& session : payload_sessions_) {
        if (session.id == event_id) {
            session.centroid_map = centroid;
            session.current_bbox = bbox;
            session.velocity = velocity;
            session.map_displacement = map_displacement;
            session.end_time = current_time;
            session.centroid_history.push_back(centroid);
            session.timestamp_history.push_back(current_time);
            session.bbox_history.push_back(bbox);

            // 状态转换
            if (session.state == PayloadSessionState::CANDIDATE) {
                if (map_displacement > 0.20 || velocity > 0.10) {
                    session.state = PayloadSessionState::CARRIED_MOVING;
                    session.moving_start_time = current_time;
                    session.confirmed = true;
                    // 记录到 swept volume
                    session.swept_volume.centroid_history.push_back(centroid);
                    session.swept_volume.bbox_history.push_back(bbox);
                    ROS_INFO("[PayloadSession] id=%d -> CARRIED_MOVING", session.id);
                }
            }
            else if (session.state == PayloadSessionState::CARRIED_MOVING) {
                // 持续更新 swept volume
                session.swept_volume.centroid_history.push_back(centroid);
                session.swept_volume.bbox_history.push_back(bbox);
                session.last_moving_time = current_time;

                // 检查是否开始减速
                if (velocity < 0.08 && map_displacement < 0.10) {
                    session.stable_frames++;
                    if (session.stable_frames >= 3) {
                        session.state = PayloadSessionState::CARRIED_STOPPING;
                        ROS_INFO("[PayloadSession] id=%d -> CARRIED_STOPPING", session.id);
                    }
                } else {
                    session.stable_frames = 0;
                }
            }
            else if (session.state == PayloadSessionState::CARRIED_STOPPING) {
                // 持续更新 swept volume（保守）
                session.swept_volume.centroid_history.push_back(centroid);
                session.swept_volume.bbox_history.push_back(bbox);

                // 检查停放
                updatePlacementDetection(session, current_time);
            }

            // 限制历史长度
            size_t max_history = 100;
            while (session.centroid_history.size() > max_history) {
                session.centroid_history.pop_front();
                session.timestamp_history.pop_front();
                session.bbox_history.pop_front();
            }
            while (session.swept_volume.centroid_history.size() > max_history) {
                session.swept_volume.centroid_history.pop_front();
                session.swept_volume.bbox_history.pop_front();
            }

            break;
        }
    }
}

void DynamicEventManager::confirmPayloadSession(int event_id, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& session : payload_sessions_) {
        if (session.id == event_id) {
            session.confirmed = true;
            if (session.state == PayloadSessionState::CANDIDATE) {
                session.state = PayloadSessionState::CARRIED_MOVING;
                session.moving_start_time = current_time;
                ROS_INFO("[PayloadSession] id=%d -> CARRIED_MOVING (confirmed)", session.id);
            }
            break;
        }
    }
}

void DynamicEventManager::endPayloadSession(int event_id, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& session : payload_sessions_) {
        if (session.id == event_id) {
            session.active = false;
            if (session.state != PayloadSessionState::PLACED_STATIC) {
                session.end_time = current_time + config_.moving_post_guard_sec;
            }
            session.state = PayloadSessionState::CLOSED;
            ROS_INFO("[PayloadSession] id=%d -> CLOSED", session.id);
            break;
        }
    }
}

void DynamicEventManager::updatePlacementDetection(PayloadSession& session, double current_time) {
    if (!config_.placement_detection_enabled) return;

    // 检查稳定性
    if (session.velocity < config_.stable_velocity_thresh_mps &&
        session.map_displacement < config_.stable_map_disp_thresh_m) {
        session.stable_frames++;
    } else {
        session.stable_frames = 0;
    }

    // 判定停放
    if (session.stable_frames >= config_.stable_frames_thresh) {
        session.state = PayloadSessionState::PLACED_STATIC;
        session.placed_time = current_time;
        session.active = false;
        session.placed_protected = true;

        // 生成停放保护 bbox
        session.placed_bbox = session.current_bbox;
        session.placed_bbox.expand(config_.placed_bbox_expand_xy, config_.placed_bbox_expand_z);

        // 截断 end_time 到停放时间（不再用 post_guard 覆盖）
        session.end_time = session.placed_time;

        ROS_INFO("[PayloadSession] id=%d -> PLACED_STATIC at t=%.2f, bbox=(%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f)",
                 session.id, session.placed_time,
                 session.placed_bbox.min_pt.x(), session.placed_bbox.min_pt.y(), session.placed_bbox.min_pt.z(),
                 session.placed_bbox.max_pt.x(), session.placed_bbox.max_pt.y(), session.placed_bbox.max_pt.z());
    }
}

// ========== 人体事件管理 ==========

int DynamicEventManager::createHumanEvent(double start_time, double current_time,
                                           const std::deque<Eigen::Vector3d>& centroid_history,
                                           double z_min, double z_max) {
    std::lock_guard<std::mutex> lock(mutex_);

    HumanEvent event;
    event.id = next_human_id_++;
    event.start_time = start_time - config_.human_pre_guard_sec;
    event.end_time = current_time + config_.human_post_guard_sec;
    event.active = true;

    event.capsule.centerline = centroid_history;
    event.capsule.radius = config_.human_capsule_radius;
    if (config_.human_use_track_height) {
        event.capsule.z_min = z_min - config_.human_z_margin;
        event.capsule.z_max = z_max + config_.human_z_margin;
    } else {
        event.capsule.z_min = 0.0;
        event.capsule.z_max = 2.5;
    }

    human_events_.push_back(event);
    return event.id;
}

void DynamicEventManager::endHumanEvent(int event_id, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& event : human_events_) {
        if (event.id == event_id) {
            event.active = false;
            event.end_time = std::max(event.end_time, current_time + config_.human_post_guard_sec);
            break;
        }
    }
}

// ========== 查询接口 ==========

std::vector<const PayloadSession*> DynamicEventManager::getActivePayloadSessions() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<const PayloadSession*> result;
    for (const auto& s : payload_sessions_) {
        if (s.active) result.push_back(&s);
    }
    return result;
}

std::vector<const PayloadSession*> DynamicEventManager::getPlacedSessions() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<const PayloadSession*> result;
    for (const auto& s : payload_sessions_) {
        if (s.state == PayloadSessionState::PLACED_STATIC && s.placed_protected) {
            result.push_back(&s);
        }
    }
    return result;
}

// ========== Mask 生成 ==========

std::set<std::pair<int,int>> DynamicEventManager::getDynamicDenyCells(double bev_resolution, double timestamp) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::set<std::pair<int,int>> deny_cells;

    for (const auto& session : payload_sessions_) {
        if (!session.confirmed) continue;
        if (session.state == PayloadSessionState::PLACED_STATIC) continue;  // 停放不 deny
        if (session.state == PayloadSessionState::CLOSED) continue;

        // deny 范围：moving_start - pre_guard 到 min(end_time, placed_time)
        double deny_start = session.moving_start_time - config_.payload_pre_guard_sec;
        double deny_end = session.end_time;
        if (session.placed_time > 0) {
            deny_end = std::min(deny_end, session.placed_time);
        }

        if (timestamp < deny_start || timestamp > deny_end + config_.moving_post_guard_sec) continue;

        // 使用 swept volume 的 bbox
        for (const auto& bbox : session.swept_volume.bbox_history) {
            Box3D expanded = bbox;
            expanded.expand(session.swept_volume.expand_xy, session.swept_volume.expand_z);
            int x_min = std::floor(expanded.min_pt.x() / bev_resolution);
            int x_max = std::floor(expanded.max_pt.x() / bev_resolution);
            int y_min = std::floor(expanded.min_pt.y() / bev_resolution);
            int y_max = std::floor(expanded.max_pt.y() / bev_resolution);
            for (int x = x_min; x <= x_max; x++) {
                for (int y = y_min; y <= y_max; y++) {
                    deny_cells.insert({x, y});
                }
            }
        }
    }

    // 人体事件
    for (const auto& event : human_events_) {
        if (timestamp < event.start_time || timestamp > event.end_time) continue;
        for (const auto& center : event.capsule.centerline) {
            int cx = std::floor(center.x() / bev_resolution);
            int cy = std::floor(center.y() / bev_resolution);
            int r_cells = std::ceil(event.capsule.radius / bev_resolution);
            for (int dx = -r_cells; dx <= r_cells; dx++) {
                for (int dy = -r_cells; dy <= r_cells; dy++) {
                    if (dx*dx + dy*dy <= r_cells*r_cells) {
                        deny_cells.insert({cx+dx, cy+dy});
                    }
                }
            }
        }
    }

    return deny_cells;
}

std::set<std::pair<int,int>> DynamicEventManager::getStaticProtectCells(double bev_resolution, double timestamp) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::set<std::pair<int,int>> protect_cells;

    for (const auto& session : payload_sessions_) {
        if (session.state != PayloadSessionState::PLACED_STATIC) continue;
        if (!session.placed_protected) continue;

        // 保护区域在停放后持续一段时间
        double protect_end = session.placed_time + 60.0;  // 保护 60 秒
        if (timestamp > protect_end) continue;

        const auto& bbox = session.placed_bbox;
        int x_min = std::floor(bbox.min_pt.x() / bev_resolution);
        int x_max = std::floor(bbox.max_pt.x() / bev_resolution);
        int y_min = std::floor(bbox.min_pt.y() / bev_resolution);
        int y_max = std::floor(bbox.max_pt.y() / bev_resolution);
        for (int x = x_min; x <= x_max; x++) {
            for (int y = y_min; y <= y_max; y++) {
                protect_cells.insert({x, y});
            }
        }
    }

    return protect_cells;
}

bool DynamicEventManager::isPointInPlacedProtect(const pcl::PointXYZ& p, double timestamp) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& session : payload_sessions_) {
        if (session.state != PayloadSessionState::PLACED_STATIC) continue;
        if (!session.placed_protected) continue;
        if (timestamp > session.placed_time + 60.0) continue;
        if (session.placed_bbox.contains(p)) return true;
    }
    return false;
}

// ========== 生命周期 ==========

void DynamicEventManager::finalizeActiveEvents(double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& session : payload_sessions_) {
        if (session.active && session.state != PayloadSessionState::PLACED_STATIC) {
            session.active = false;
            session.state = PayloadSessionState::CLOSED;
            session.end_time = current_time + config_.moving_post_guard_sec;
        }
    }
    for (auto& event : human_events_) {
        if (event.active) {
            event.active = false;
            event.end_time = std::max(event.end_time, current_time + config_.human_post_guard_sec);
        }
    }
}

void DynamicEventManager::cleanupExpiredEvents(double current_time, double max_age_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    payload_sessions_.erase(
        std::remove_if(payload_sessions_.begin(), payload_sessions_.end(),
                       [&](const PayloadSession& s) {
                           return !s.active && (current_time - s.end_time) > max_age_sec;
                       }),
        payload_sessions_.end());
    human_events_.erase(
        std::remove_if(human_events_.begin(), human_events_.end(),
                       [&](const HumanEvent& e) {
                           return !e.active && (current_time - e.end_time) > max_age_sec;
                       }),
        human_events_.end());
}

int DynamicEventManager::getActiveCount() const {
    int count = 0;
    for (const auto& s : payload_sessions_) if (s.active) count++;
    return count;
}

int DynamicEventManager::getPlacedCount() const {
    int count = 0;
    for (const auto& s : payload_sessions_) {
        if (s.state == PayloadSessionState::PLACED_STATIC && s.placed_protected) count++;
    }
    return count;
}

} // namespace ndt_slam
