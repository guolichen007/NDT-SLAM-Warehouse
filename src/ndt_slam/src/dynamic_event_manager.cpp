#include "ndt_slam/dynamic_event_manager.hpp"
#include <algorithm>
#include <cmath>

namespace ndt_slam {

void DynamicEventManager::configure(const DynamicEventConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

int DynamicEventManager::createPayloadSession(double first_candidate_time, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);

    DynamicEvent event;
    event.id = next_event_id_++;
    event.type = DynamicEventType::PAYLOAD_SESSION;
    event.start_time = first_candidate_time - config_.payload_pre_guard_sec;
    event.end_time = current_time + config_.payload_post_guard_sec;
    event.active = true;
    event.confirmed = false;

    events_.push_back(event);
    return event.id;
}

void DynamicEventManager::updatePayloadSession(int event_id, double current_time,
                                                const Eigen::Vector3d& centroid,
                                                const Box3D& bbox) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& event : events_) {
        if (event.id == event_id && event.type == DynamicEventType::PAYLOAD_SESSION) {
            event.payload_swept_volume.centroid_history.push_back(centroid);
            event.payload_swept_volume.bbox_history.push_back(bbox);
            event.end_time = std::max(event.end_time,
                                      current_time + config_.payload_post_guard_sec);
            break;
        }
    }
}

void DynamicEventManager::confirmPayloadSession(int event_id, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& event : events_) {
        if (event.id == event_id && event.type == DynamicEventType::PAYLOAD_SESSION) {
            event.confirmed = true;
            event.end_time = std::max(event.end_time,
                                      current_time + config_.payload_post_guard_sec);
            break;
        }
    }
}

void DynamicEventManager::endPayloadSession(int event_id, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& event : events_) {
        if (event.id == event_id && event.type == DynamicEventType::PAYLOAD_SESSION) {
            event.active = false;
            event.end_time = std::max(event.end_time,
                                      current_time + config_.payload_post_guard_sec);
            break;
        }
    }
}

int DynamicEventManager::createHumanEvent(double start_time, double current_time,
                                           const std::deque<Eigen::Vector3d>& centroid_history,
                                           double z_min, double z_max) {
    std::lock_guard<std::mutex> lock(mutex_);

    DynamicEvent event;
    event.id = next_event_id_++;
    event.type = DynamicEventType::HUMAN_TRACK;
    event.start_time = start_time - config_.human_pre_guard_sec;
    event.end_time = current_time + config_.human_post_guard_sec;
    event.active = true;
    event.confirmed = true;  // 人体事件创建时就确认

    event.human_capsule.centerline = centroid_history;
    event.human_capsule.radius = config_.human_capsule_radius;
    if (config_.human_use_track_height) {
        event.human_capsule.z_min = z_min - config_.human_z_margin;
        event.human_capsule.z_max = z_max + config_.human_z_margin;
    } else {
        event.human_capsule.z_min = 0.0;
        event.human_capsule.z_max = 2.5;
    }

    events_.push_back(event);
    return event.id;
}

void DynamicEventManager::endHumanEvent(int event_id, double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& event : events_) {
        if (event.id == event_id && event.type == DynamicEventType::HUMAN_TRACK) {
            event.active = false;
            event.end_time = std::max(event.end_time,
                                      current_time + config_.human_post_guard_sec);
            break;
        }
    }
}

std::vector<const DynamicEvent*> DynamicEventManager::getActiveEvents() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<const DynamicEvent*> result;
    for (const auto& event : events_) {
        if (event.active) result.push_back(&event);
    }
    return result;
}

std::vector<const DynamicEvent*> DynamicEventManager::getConfirmedEvents() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<const DynamicEvent*> result;
    for (const auto& event : events_) {
        if (event.confirmed) result.push_back(&event);
    }
    return result;
}

void DynamicEventManager::finalizeActiveEvents(double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& event : events_) {
        if (event.active) {
            event.active = false;
            event.end_time = std::max(event.end_time,
                                      current_time + config_.payload_post_guard_sec);
        }
    }
}

bool DynamicEventManager::isPointDenied(const pcl::PointXYZ& p, double timestamp) const {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& event : events_) {
        if (!event.confirmed) continue;
        if (timestamp < event.start_time || timestamp > event.end_time) continue;

        if (event.type == DynamicEventType::PAYLOAD_SESSION &&
            event.payload_swept_volume.contains(p)) {
            return true;
        }

        if (event.type == DynamicEventType::HUMAN_TRACK &&
            event.human_capsule.contains(p)) {
            return true;
        }
    }
    return false;
}

std::set<std::pair<int,int>> DynamicEventManager::getDenyCells(
    double bev_resolution, double timestamp) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::set<std::pair<int,int>> deny_cells;

    for (const auto& event : events_) {
        if (!event.confirmed) continue;
        if (timestamp < event.start_time || timestamp > event.end_time) continue;

        if (event.type == DynamicEventType::PAYLOAD_SESSION) {
            for (const auto& bbox : event.payload_swept_volume.bbox_history) {
                Box3D expanded = bbox;
                expanded.expand(event.payload_swept_volume.expand_xy,
                               event.payload_swept_volume.expand_z);
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

        if (event.type == DynamicEventType::HUMAN_TRACK) {
            for (const auto& center : event.human_capsule.centerline) {
                int cx = std::floor(center.x() / bev_resolution);
                int cy = std::floor(center.y() / bev_resolution);
                int r_cells = std::ceil(event.human_capsule.radius / bev_resolution);
                for (int dx = -r_cells; dx <= r_cells; dx++) {
                    for (int dy = -r_cells; dy <= r_cells; dy++) {
                        if (dx*dx + dy*dy <= r_cells*r_cells) {
                            deny_cells.insert({cx+dx, cy+dy});
                        }
                    }
                }
            }
        }
    }
    return deny_cells;
}

void DynamicEventManager::cleanupExpiredEvents(double current_time, double max_age_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    events_.erase(
        std::remove_if(events_.begin(), events_.end(),
                       [&](const DynamicEvent& e) {
                           return !e.active && (current_time - e.end_time) > max_age_sec;
                       }),
        events_.end());
}

int DynamicEventManager::getActiveCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    int count = 0;
    for (const auto& e : events_) if (e.active) count++;
    return count;
}

int DynamicEventManager::getConfirmedCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    int count = 0;
    for (const auto& e : events_) if (e.confirmed) count++;
    return count;
}

} // namespace ndt_slam
