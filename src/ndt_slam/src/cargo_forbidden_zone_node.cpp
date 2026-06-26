/**
 * cargo_forbidden_zone_node.cpp
 *
 * 吊装货物识别与 bbox 输出节点
 * - 从当前帧点云实时识别吊起货物
 * - 区分吊货、地面货物、工人、固定结构
 * - 输出 raw bbox 和 stable bbox
 * - 吊货悬停时保持 SUSPENDED_STATIC
 */

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include <Eigen/Core>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>

namespace ndt_slam {

// 货物语义状态
enum class PayloadSemanticState {
    UNKNOWN = 0,
    GROUND_CARGO = 1,
    SUSPENDED_CANDIDATE = 2,
    SUSPENDED_MOVING = 3,
    SUSPENDED_STATIC = 4,
    HUMAN_DYNAMIC = 5,
    STATIC_STRUCTURE = 6,
    LOST = 7
};

// 风险等级（保留）
enum class RiskLevel {
    IDLE = 0,
    NORMAL = 1,
    WARNING = 2,
    SLOW_DOWN = 3,
    STOP = 4,
    UNKNOWN = 5
};

// 聚类特征
struct ClusterFeatures {
    Eigen::Vector3f centroid_base = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroid_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f size = Eigen::Vector3f::Zero();
    int point_count = 0;
    float z_min = std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    float bottom_hag = 0.0f;
    float support_ratio = 0.0f;
    float base_stability = 0.0f;
    float map_displacement = 0.0f;
    float map_velocity = 0.0f;
    bool in_hook_roi = false;
    bool size_valid = false;
    PayloadSemanticState state = PayloadSemanticState::UNKNOWN;
    float score = 0.0f;
};

// 吊货信息
struct CargoInfo {
    bool valid = false;
    int track_id = -1;
    PayloadSemanticState state = PayloadSemanticState::UNKNOWN;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();
    Eigen::Vector3f stable_centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f stable_size = Eigen::Vector3f::Zero();
    int point_count = 0;
    float score = 0.0f;
    float bottom_hag = 0.0f;
    float support_ratio = 0.0f;
};

class CargoForbiddenZoneNode {
public:
    CargoForbiddenZoneNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh) {
        loadConfig();
        setupPublishers();
        setupSubscribers();

        ROS_INFO("[CargoForbiddenZone] Node initialized");
        ROS_INFO("[CargoForbiddenZone] ROI: x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]",
                 roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
    }

    void spin() {
        ros::Rate rate(10);  // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            update();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // 配置
    std::string map_frame_ = "map";

    // ROI 参数
    double roi_x_min_ = -5.0;
    double roi_x_max_ = 5.0;
    double roi_y_min_ = -4.0;
    double roi_y_max_ = 4.0;
    double roi_z_min_ = -2.0;
    double roi_z_max_ = 6.0;

    // 聚类参数
    double cluster_tolerance_ = 0.35;
    int min_cluster_points_ = 60;
    int max_cluster_points_ = 20000;

    // HAG 判断
    double min_floating_gap_ = 0.30;
    double strong_floating_gap_ = 0.60;
    double max_support_ratio_ = 0.20;

    // 评分权重
    double min_score_to_lock_ = 0.70;
    double w_base_stability_ = 0.30;
    double w_map_motion_ = 0.20;
    double w_floating_gap_ = 0.20;
    double w_support_ = 0.15;
    double w_roi_ = 0.10;
    double w_size_ = 0.05;

    // 货物定位
    int max_lost_frames_ = 8;
    int min_confirm_frames_ = 2;
    double keep_suspended_timeout_sec_ = 5.0;
    int moving_confirm_frames_ = 2;
    int min_bbox_point_count_ = 50;
    double min_map_displacement_ = 0.25;
    double min_direction_consistency_ = 0.65;

    // bbox 滤波
    bool use_quantile_bbox_ = true;
    double quantile_xy_low_ = 0.05;
    double quantile_xy_high_ = 0.95;
    double quantile_z_low_ = 0.10;
    double quantile_z_high_ = 0.90;
    double centroid_filter_alpha_ = 0.60;
    double size_filter_alpha_ = 0.25;
    double max_position_jump_ = 0.80;
    double max_size_change_per_frame_ = 0.40;
    bool use_velocity_compensation_ = true;
    double max_compensation_dt_ = 0.30;

    // 默认尺寸
    double default_length_x_ = 3.0;
    double default_width_y_ = 1.0;
    double default_height_z_ = 1.0;

    // 安全边距
    double safety_margin_x_ = 0.25;
    double safety_margin_y_ = 0.25;

    // 可视化配置
    bool publish_raw_bbox_ = true;
    bool publish_stable_bbox_ = true;
    bool publish_status_text_ = true;
    bool publish_candidate_cloud_ = true;
    bool publish_suspended_cloud_ = true;

    // 货物状态
    PayloadSemanticState cargo_state_ = PayloadSemanticState::UNKNOWN;
    int current_track_id_ = -1;
    int lost_count_ = 0;
    int confirm_count_ = 0;
    ros::Time last_suspended_time_;

    // 当前货物信息
    CargoInfo cargo_;

    // 稳定值
    Eigen::Vector3f stable_centroid_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f stable_velocity_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f stable_size_ = Eigen::Vector3f(3.0, 1.0, 1.0);
    Eigen::Vector3f stable_bbox_min_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f stable_bbox_max_ = Eigen::Vector3f::Zero();
    float stable_cargo_z_min_ = 0.0f;
    ros::Time last_track_time_;

    // 风险等级
    RiskLevel risk_level_ = RiskLevel::UNKNOWN;

    // 发布者
    ros::Publisher raw_bbox_marker_pub_;
    ros::Publisher stable_bbox_marker_pub_;
    ros::Publisher status_text_marker_pub_;
    ros::Publisher candidate_cloud_pub_;
    ros::Publisher suspended_cloud_pub_;
    ros::Publisher cargo_markers_pub_;

    // 订阅者
    ros::Subscriber payload_track_sub_;

    void loadConfig() {
        std::string config_file;
        pnh_.param<std::string>("config_file", config_file, "");
        if (config_file.empty()) {
            ROS_WARN("[CargoForbiddenZone] No config file specified, using defaults");
            return;
        }

        try {
            YAML::Node cfg = YAML::LoadFile(config_file);
            YAML::Node root = cfg["cargo_forbidden_zone"] ? cfg["cargo_forbidden_zone"] : cfg;

            map_frame_ = root["map_frame"].as<std::string>("map");

            // ROI 参数
            if (root["suspended_payload"]) {
                auto sp = root["suspended_payload"];
                roi_x_min_ = sp["roi_x_min"].as<double>(-5.0);
                roi_x_max_ = sp["roi_x_max"].as<double>(5.0);
                roi_y_min_ = sp["roi_y_min"].as<double>(-4.0);
                roi_y_max_ = sp["roi_y_max"].as<double>(4.0);
                roi_z_min_ = sp["roi_z_min"].as<double>(-2.0);
                roi_z_max_ = sp["roi_z_max"].as<double>(6.0);
                cluster_tolerance_ = sp["cluster_tolerance"].as<double>(0.35);
                min_cluster_points_ = sp["min_cluster_points"].as<int>(60);
                max_cluster_points_ = sp["max_cluster_points"].as<int>(20000);
                min_floating_gap_ = sp["min_floating_gap"].as<double>(0.30);
                strong_floating_gap_ = sp["strong_floating_gap"].as<double>(0.60);
                max_support_ratio_ = sp["max_support_ratio"].as<double>(0.20);
                min_score_to_lock_ = sp["min_score_to_lock"].as<double>(0.70);
                w_base_stability_ = sp["w_base_stability"].as<double>(0.30);
                w_map_motion_ = sp["w_map_motion"].as<double>(0.20);
                w_floating_gap_ = sp["w_floating_gap"].as<double>(0.20);
                w_support_ = sp["w_support"].as<double>(0.15);
                w_roi_ = sp["w_roi"].as<double>(0.10);
                w_size_ = sp["w_size"].as<double>(0.05);
            }

            // 货物定位
            if (root["cargo_localization"]) {
                auto cl = root["cargo_localization"];
                max_lost_frames_ = cl["max_lost_frames"].as<int>(8);
                min_confirm_frames_ = cl["min_confirm_frames"].as<int>(2);
                keep_suspended_timeout_sec_ = cl["keep_suspended_timeout_sec"].as<double>(5.0);
                moving_confirm_frames_ = cl["moving_confirm_frames"].as<int>(2);
                min_bbox_point_count_ = cl["min_bbox_point_count"].as<int>(50);
                min_map_displacement_ = cl["min_map_displacement"].as<double>(0.25);
                min_direction_consistency_ = cl["min_direction_consistency"].as<double>(0.65);
            }

            // bbox 滤波
            if (root["bbox_filter"]) {
                auto bf = root["bbox_filter"];
                use_quantile_bbox_ = bf["use_quantile_bbox"].as<bool>(true);
                quantile_xy_low_ = bf["quantile_xy_low"].as<double>(0.05);
                quantile_xy_high_ = bf["quantile_xy_high"].as<double>(0.95);
                quantile_z_low_ = bf["quantile_z_low"].as<double>(0.10);
                quantile_z_high_ = bf["quantile_z_high"].as<double>(0.90);
                centroid_filter_alpha_ = bf["centroid_filter_alpha"].as<double>(0.60);
                size_filter_alpha_ = bf["size_filter_alpha"].as<double>(0.25);
                max_position_jump_ = bf["max_position_jump"].as<double>(0.80);
                max_size_change_per_frame_ = bf["max_size_change_per_frame"].as<double>(0.40);
                use_velocity_compensation_ = bf["use_velocity_compensation"].as<bool>(true);
                max_compensation_dt_ = bf["max_compensation_dt"].as<double>(0.30);
            }

            // 默认尺寸
            if (root["cargo_size"]) {
                auto cs = root["cargo_size"];
                default_length_x_ = cs["default_length_x"].as<double>(3.0);
                default_width_y_ = cs["default_width_y"].as<double>(1.0);
                default_height_z_ = cs["default_height_z"].as<double>(1.0);
            }

            // 安全边距
            if (root["inflation"]) {
                auto inf = root["inflation"];
                safety_margin_x_ = inf["safety_margin_x"].as<double>(0.25);
                safety_margin_y_ = inf["safety_margin_y"].as<double>(0.25);
            }

            // 可视化配置
            if (root["visualization"]) {
                auto vis = root["visualization"];
                publish_raw_bbox_ = vis["publish_raw_bbox"].as<bool>(true);
                publish_stable_bbox_ = vis["publish_stable_bbox"].as<bool>(true);
                publish_status_text_ = vis["publish_status_text"].as<bool>(true);
                publish_candidate_cloud_ = vis["publish_candidate_cloud"].as<bool>(true);
                publish_suspended_cloud_ = vis["publish_suspended_cloud"].as<bool>(true);
            }

            ROS_INFO("[CargoForbiddenZone] Config loaded from %s", config_file.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("[CargoForbiddenZone] Config error: %s", e.what());
        }
    }

    void setupPublishers() {
        raw_bbox_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_raw_bbox_marker", 10);
        stable_bbox_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_stable_bbox_marker", 10);
        status_text_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_track_status_marker", 10);
        candidate_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/suspended_payload_candidate_cloud", 10);
        suspended_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/suspended_payload_cloud", 10);
        cargo_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_forbidden_markers", 10);
    }

    void setupSubscribers() {
        payload_track_sub_ = nh_.subscribe("/payload_track_info", 10,
                                           &CargoForbiddenZoneNode::payloadTrackCallback, this);
    }

    void payloadTrackCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 19) return;

        cargo_.valid = (msg->data[1] >= 0);
        if (!cargo_.valid) return;

        cargo_.track_id = static_cast<int>(msg->data[1]);
        cargo_.state = static_cast<PayloadSemanticState>(static_cast<int>(msg->data[2]));
        cargo_.centroid = Eigen::Vector3f(msg->data[3], msg->data[4], msg->data[5]);
        cargo_.velocity = Eigen::Vector3f(msg->data[6], msg->data[7], msg->data[8]);
        cargo_.bbox_min = Eigen::Vector3f(msg->data[9], msg->data[10], msg->data[11]);
        cargo_.bbox_max = Eigen::Vector3f(msg->data[12], msg->data[13], msg->data[14]);
        cargo_.point_count = static_cast<int>(msg->data[15]);
        cargo_.score = msg->data[16];
        cargo_.bottom_hag = msg->data[17];
        cargo_.support_ratio = msg->data[18];

        // 更新状态机
        updateCargoStateMachine();
    }

    void updateCargoStateMachine() {
        if (!cargo_.valid) {
            if (cargo_state_ == PayloadSemanticState::SUSPENDED_MOVING ||
                cargo_state_ == PayloadSemanticState::SUSPENDED_STATIC) {
                lost_count_++;
                if (lost_count_ > max_lost_frames_) {
                    cargo_state_ = PayloadSemanticState::LOST;
                    current_track_id_ = -1;
                    ROS_INFO("[SuspendedCargo] Track lost after %d frames", lost_count_);
                }
            }
            return;
        }

        // 检查是否是新的有效 track
        if (current_track_id_ < 0 || current_track_id_ != cargo_.track_id) {
            // 新 track，需要确认
            if (cargo_.score >= min_score_to_lock_) {
                confirm_count_++;
                if (confirm_count_ >= min_confirm_frames_) {
                    current_track_id_ = cargo_.track_id;
                    cargo_state_ = PayloadSemanticState::SUSPENDED_MOVING;
                    lost_count_ = 0;
                    confirm_count_ = 0;
                    last_suspended_time_ = ros::Time::now();
                    ROS_INFO("[SuspendedCargo] Locked track %d, score=%.2f", current_track_id_, cargo_.score);
                }
            } else {
                confirm_count_ = 0;
            }
            return;
        }

        // 已锁定的 track
        lost_count_ = 0;
        confirm_count_ = 0;

        // 检查是否还在吊货状态
        bool still_suspended = (cargo_.score >= min_score_to_lock_ * 0.8);

        if (still_suspended) {
            // 检查是否移动
            float speed = cargo_.velocity.norm();
            if (speed > 0.1f) {
                cargo_state_ = PayloadSemanticState::SUSPENDED_MOVING;
                last_suspended_time_ = ros::Time::now();
            } else {
                // 检查是否在超时时间内
                double time_since_suspended = (ros::Time::now() - last_suspended_time_).toSec();
                if (time_since_suspended < keep_suspended_timeout_sec_) {
                    cargo_state_ = PayloadSemanticState::SUSPENDED_STATIC;
                } else {
                    // 超时，检查是否还在 ROI
                    if (cargo_.bottom_hag > min_floating_gap_) {
                        cargo_state_ = PayloadSemanticState::SUSPENDED_STATIC;
                    } else {
                        cargo_state_ = PayloadSemanticState::GROUND_CARGO;
                    }
                }
            }
        } else {
            cargo_state_ = PayloadSemanticState::LOST;
            current_track_id_ = -1;
        }
    }

    void update() {
        updateStableBbox();
        updateRiskLevel();
        publishResults();
    }

    void updateStableBbox() {
        if (!cargo_.valid || cargo_state_ == PayloadSemanticState::UNKNOWN ||
            cargo_state_ == PayloadSemanticState::LOST) {
            return;
        }

        // 速度补偿
        Eigen::Vector3f predicted_centroid = cargo_.centroid;
        if (use_velocity_compensation_ && stable_centroid_.norm() > 0.01f) {
            ros::Time now = ros::Time::now();
            double dt = (now - last_track_time_).toSec();
            dt = std::min(dt, max_compensation_dt_);
            predicted_centroid = stable_centroid_ + stable_velocity_ * dt;

            // 检查跳变
            float jump = (cargo_.centroid - stable_centroid_).norm();
            if (jump > max_position_jump_) {
                predicted_centroid = cargo_.centroid;
                ROS_WARN_THROTTLE(2.0, "[SuspendedCargo] Large jump: %.2f > %.2f", jump, max_position_jump_);
            }
        }
        last_track_time_ = ros::Time::now();

        // 低通滤波 centroid
        stable_centroid_ = centroid_filter_alpha_ * predicted_centroid +
                           (1.0f - centroid_filter_alpha_) * stable_centroid_;

        // 更新速度估计
        if (stable_centroid_.norm() > 0.01f) {
            Eigen::Vector3f vel = (stable_centroid_ - predicted_centroid) / 0.1f;
            stable_velocity_ = 0.7f * stable_velocity_ + 0.3f * vel;
        }

        // 计算 size
        Eigen::Vector3f raw_size = cargo_.bbox_max - cargo_.bbox_min;
        if (raw_size.x() < 0.5f || raw_size.x() > 8.0f ||
            raw_size.y() < 0.3f || raw_size.y() > 3.0f) {
            raw_size = Eigen::Vector3f(default_length_x_, default_width_y_, default_height_z_);
        }

        // 低通滤波 size
        stable_size_ = size_filter_alpha_ * raw_size +
                       (1.0f - size_filter_alpha_) * stable_size_;

        // 限制单帧变化
        Eigen::Vector3f size_change = stable_size_ - raw_size;
        for (int i = 0; i < 3; i++) {
            if (std::abs(size_change[i]) > max_size_change_per_frame_) {
                stable_size_[i] = raw_size[i] + std::copysign(max_size_change_per_frame_, size_change[i]);
            }
        }

        // 更新 bbox
        float half_l = stable_size_.x() / 2.0f + safety_margin_x_;
        float half_w = stable_size_.y() / 2.0f + safety_margin_y_;
        stable_bbox_min_ = stable_centroid_ - Eigen::Vector3f(half_l, half_w, 0);
        stable_bbox_max_ = stable_centroid_ + Eigen::Vector3f(half_l, half_w, stable_size_.z());
        stable_cargo_z_min_ = stable_centroid_.z() - stable_size_.z() / 2.0f;
    }

    void updateRiskLevel() {
        if (cargo_state_ == PayloadSemanticState::UNKNOWN ||
            cargo_state_ == PayloadSemanticState::LOST) {
            risk_level_ = RiskLevel::UNKNOWN;
            return;
        }

        if (cargo_state_ == PayloadSemanticState::SUSPENDED_STATIC) {
            risk_level_ = RiskLevel::IDLE;
        } else {
            risk_level_ = RiskLevel::NORMAL;
        }
    }

    void publishResults() {
        ros::Time now = ros::Time::now();

        // 发布 raw bbox
        if (publish_raw_bbox_ && cargo_.valid && cargo_.point_count >= min_bbox_point_count_) {
            publishRawBbox(now);
        } else if (publish_raw_bbox_) {
            // 无有效 cargo，发布 DELETE 清理旧 marker
            visualization_msgs::MarkerArray markers;
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time(0);
            marker.header.frame_id = map_frame_;
            marker.ns = "cargo_raw_bbox";
            marker.id = 0;
            marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(marker);
            raw_bbox_marker_pub_.publish(markers);
        }

        // 发布 stable bbox
        if (publish_stable_bbox_ && cargo_.valid &&
            (cargo_state_ == PayloadSemanticState::SUSPENDED_MOVING ||
             cargo_state_ == PayloadSemanticState::SUSPENDED_STATIC)) {
            publishStableBbox(now);
        } else if (publish_stable_bbox_) {
            // 无有效 cargo，发布 DELETE 清理旧 marker
            visualization_msgs::MarkerArray markers;
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time(0);
            marker.header.frame_id = map_frame_;
            marker.ns = "cargo_stable_bbox";
            marker.id = 0;
            marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(marker);
            stable_bbox_marker_pub_.publish(markers);
        }

        // 发布状态文字
        if (publish_status_text_) {
            publishStatusText(now);
        }

        // 发布候选点云
        if (publish_candidate_cloud_) {
            publishCandidateCloud(now);
        }

        // 发布吊货点云
        if (publish_suspended_cloud_ && cargo_.valid) {
            publishSuspendedCloud(now);
        }
    }

    void publishRawBbox(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time(0);  // 调试阶段避免 TF 时间问题
        marker.header.frame_id = map_frame_;
        marker.ns = "cargo_raw_bbox";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // 使用原始 centroid 作为中心
        marker.pose.position.x = (cargo_.bbox_min.x() + cargo_.bbox_max.x()) / 2.0;
        marker.pose.position.y = (cargo_.bbox_min.y() + cargo_.bbox_max.y()) / 2.0;
        marker.pose.position.z = (cargo_.bbox_min.z() + cargo_.bbox_max.z()) / 2.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = cargo_.bbox_max.x() - cargo_.bbox_min.x();
        marker.scale.y = cargo_.bbox_max.y() - cargo_.bbox_min.y();
        marker.scale.z = cargo_.bbox_max.z() - cargo_.bbox_min.z();

        // 最小尺寸保护：不小于 0.30m
        marker.scale.x = std::max(marker.scale.x, 0.30);
        marker.scale.y = std::max(marker.scale.y, 0.30);
        marker.scale.z = std::max(marker.scale.z, 0.30);

        // 紫色线框，alpha 改成 0.75 以上
        marker.color.r = 0.6;
        marker.color.g = 0.2;
        marker.color.b = 0.8;
        marker.color.a = 0.75;
        marker.lifetime = ros::Duration(0.5);

        markers.markers.push_back(marker);
        raw_bbox_marker_pub_.publish(markers);
    }

    void publishStableBbox(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time(0);  // 调试阶段避免 TF 时间问题
        marker.header.frame_id = map_frame_;
        marker.ns = "cargo_stable_bbox";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // 使用 stable centroid 作为中心
        marker.pose.position.x = stable_centroid_.x();
        marker.pose.position.y = stable_centroid_.y();
        marker.pose.position.z = stable_centroid_.z();
        marker.pose.orientation.w = 1.0;

        marker.scale.x = stable_size_.x();
        marker.scale.y = stable_size_.y();
        marker.scale.z = stable_size_.z();

        // 最小尺寸保护：不小于 0.30m
        marker.scale.x = std::max(marker.scale.x, 0.30);
        marker.scale.y = std::max(marker.scale.y, 0.30);
        marker.scale.z = std::max(marker.scale.z, 0.30);

        // 绿色半透明，alpha 改成 0.45 以上
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.45;
        marker.lifetime = ros::Duration(0.5);

        markers.markers.push_back(marker);
        stable_bbox_marker_pub_.publish(markers);
    }

    void publishStatusText(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time(0);  // 调试阶段避免 TF 时间问题
        marker.header.frame_id = map_frame_;
        marker.ns = "cargo_status_text";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        if (cargo_.valid) {
            marker.pose.position.x = stable_centroid_.x();
            marker.pose.position.y = stable_centroid_.y();
            marker.pose.position.z = stable_centroid_.z() + stable_size_.z() / 2.0 + 0.5;
        } else {
            marker.pose.position.z = 1.0;
        }
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.4;

        std::string state_str;
        switch (cargo_state_) {
            case PayloadSemanticState::UNKNOWN: state_str = "UNKNOWN"; break;
            case PayloadSemanticState::GROUND_CARGO: state_str = "GROUND"; break;
            case PayloadSemanticState::SUSPENDED_CANDIDATE: state_str = "CANDIDATE"; break;
            case PayloadSemanticState::SUSPENDED_MOVING: state_str = "MOVING"; break;
            case PayloadSemanticState::SUSPENDED_STATIC: state_str = "STATIC"; break;
            case PayloadSemanticState::HUMAN_DYNAMIC: state_str = "HUMAN"; break;
            case PayloadSemanticState::STATIC_STRUCTURE: state_str = "STRUCTURE"; break;
            case PayloadSemanticState::LOST: state_str = "LOST"; break;
        }

        std::stringstream ss;
        ss << "track=" << current_track_id_ << "\n"
           << "state=" << state_str << "\n"
           << "score=" << std::fixed << std::setprecision(2) << cargo_.score << "\n"
           << "hag=" << std::fixed << std::setprecision(2) << cargo_.bottom_hag << "\n"
           << "support=" << std::fixed << std::setprecision(2) << cargo_.support_ratio << "\n"
           << "size=(" << std::fixed << std::setprecision(1)
           << stable_size_.x() << "," << stable_size_.y() << "," << stable_size_.z() << ")";

        marker.text = ss.str();
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.5);

        markers.markers.push_back(marker);
        status_text_marker_pub_.publish(markers);
    }

    void publishCandidateCloud(const ros::Time& stamp) {
        // TODO: 从当前帧点云中提取候选 cluster 并发布
    }

    void publishSuspendedCloud(const ros::Time& stamp) {
        // TODO: 从当前帧点云中提取吊货点云并发布
    }
};

}  // namespace ndt_slam

int main(int argc, char** argv) {
    ros::init(argc, argv, "cargo_forbidden_zone_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ndt_slam::CargoForbiddenZoneNode node(nh, pnh);
    node.spin();

    return 0;
}
