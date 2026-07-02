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
#include <nav_msgs/Odometry.h>

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

        // P0.5 新增：timer 高频发布 marker
        marker_timer_ = nh_.createTimer(ros::Duration(1.0 / marker_publish_rate_),
                                        &CargoForbiddenZoneNode::markerTimerCallback, this);

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
    double max_position_jump_ = 1.50;  // P0: 增大到 1.5m（soft gate）
    double residual_hard_gate_ = 3.00;  // P0: hard gate 阈值
    int switch_confirm_frames_ = 3;     // P0: 连续 N 帧异常才切换
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
    int observed_frames_ = 0;  // v8: 用于 marker gate
    int moving_frame_count_ = 0;
    int stationary_frame_count_ = 0;
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

    // P0.5 新增：三层 marker 发布
    ros::Publisher core_bbox_marker_pub_;      // /cargo_core_bbox_marker
    ros::Publisher remove_bbox_marker_pub_;    // /cargo_remove_bbox_marker
    ros::Publisher forbidden_zone_marker_pub_; // /cargo_forbidden_zone_marker

    // 订阅者
    ros::Subscriber payload_track_sub_;
    ros::Subscriber odom_sub_;  // P0.5: 订阅 odom 用于预测

    // P0.5 新增：odom 状态
    Eigen::Vector3f last_odom_position_ = Eigen::Vector3f::Zero();
    Eigen::Quaternionf last_odom_orientation_ = Eigen::Quaternionf::Identity();
    bool has_odom_ = false;
    int suspect_jump_count_ = 0;

    // P0.5 新增：timer 高频发布
    ros::Timer marker_timer_;
    double marker_publish_rate_ = 15.0;  // 15Hz

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
                max_position_jump_ = bf["max_position_jump"].as<double>(1.50);
                residual_hard_gate_ = bf["residual_hard_gate"].as<double>(3.00);
                switch_confirm_frames_ = bf["switch_confirm_frames"].as<int>(3);
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

        // P0.5 新增：三层 marker 发布
        core_bbox_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_core_bbox_marker", 1);  // v8-stable-r3-hotfix-minimal: queue_size=1
        remove_bbox_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_remove_bbox_marker", 10);
        forbidden_zone_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_forbidden_zone_marker", 10);
    }

    void setupSubscribers() {
        payload_track_sub_ = nh_.subscribe("/payload_track_info", 10,
                                           &CargoForbiddenZoneNode::payloadTrackCallback, this);
        // P0.5: 订阅 odom 用于预测
        odom_sub_ = nh_.subscribe("/odom", 10,
                                  &CargoForbiddenZoneNode::odomCallback, this);
    }

    // P0.5 新增：odom 回调
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        Eigen::Vector3f new_pos(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
        Eigen::Quaternionf new_ori(msg->pose.pose.orientation.w,
                                   msg->pose.pose.orientation.x,
                                   msg->pose.pose.orientation.y,
                                   msg->pose.pose.orientation.z);

        if (has_odom_) {
            // 计算 odom delta
            Eigen::Vector3f odom_delta = new_pos - last_odom_position_;
            // 更新 stable_centroid_ 的预测
            stable_centroid_ += odom_delta;
        }

        last_odom_position_ = new_pos;
        last_odom_orientation_ = new_ori;
        has_odom_ = true;
    }

    // P0.5 新增：timer 高频发布 marker
    void markerTimerCallback(const ros::TimerEvent& event) {
        // 高频发布三层 marker，使用当前 odom 位置
        // 这样即使检测低频更新，marker 也会跟随 odom 平滑移动
        publishThreeLayerMarkers(ros::Time::now());
    }

    void payloadTrackCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 19) {
            ROS_WARN("[PayloadTrackInfoSub] Invalid message size: %zu (expected >= 19)", msg->data.size());
            return;
        }

        // 定义统一索引常量，与发布端一致
        constexpr int IDX_VALID = 0;
        constexpr int IDX_TRACK_ID = 1;
        constexpr int IDX_STATE = 2;
        constexpr int IDX_CENTER_X = 3;
        constexpr int IDX_CENTER_Y = 4;
        constexpr int IDX_CENTER_Z = 5;
        constexpr int IDX_VEL_X = 6;
        constexpr int IDX_VEL_Y = 7;
        constexpr int IDX_VEL_Z = 8;
        constexpr int IDX_BBOX_MIN_X = 9;
        constexpr int IDX_BBOX_MIN_Y = 10;
        constexpr int IDX_BBOX_MIN_Z = 11;
        constexpr int IDX_BBOX_MAX_X = 12;
        constexpr int IDX_BBOX_MAX_Y = 13;
        constexpr int IDX_BBOX_MAX_Z = 14;
        constexpr int IDX_POINT_COUNT = 15;
        constexpr int IDX_SCORE = 16;
        constexpr int IDX_BOTTOM_HAG = 17;
        constexpr int IDX_SUPPORT_RATIO = 18;

        cargo_.valid = (msg->data[IDX_VALID] >= 0);
        if (!cargo_.valid) return;

        cargo_.track_id = static_cast<int>(msg->data[IDX_TRACK_ID]);
        cargo_.state = static_cast<PayloadSemanticState>(static_cast<int>(msg->data[IDX_STATE]));

        // v8: 更新 observed_frames_
        if (cargo_.track_id == current_track_id_) {
            observed_frames_++;
        } else {
            observed_frames_ = 1;
            current_track_id_ = cargo_.track_id;
        }
        cargo_.centroid = Eigen::Vector3f(msg->data[IDX_CENTER_X], msg->data[IDX_CENTER_Y], msg->data[IDX_CENTER_Z]);
        cargo_.velocity = Eigen::Vector3f(msg->data[IDX_VEL_X], msg->data[IDX_VEL_Y], msg->data[IDX_VEL_Z]);
        cargo_.bbox_min = Eigen::Vector3f(msg->data[IDX_BBOX_MIN_X], msg->data[IDX_BBOX_MIN_Y], msg->data[IDX_BBOX_MIN_Z]);
        cargo_.bbox_max = Eigen::Vector3f(msg->data[IDX_BBOX_MAX_X], msg->data[IDX_BBOX_MAX_Y], msg->data[IDX_BBOX_MAX_Z]);
        cargo_.point_count = static_cast<int>(msg->data[IDX_POINT_COUNT]);
        cargo_.score = msg->data[IDX_SCORE];
        cargo_.bottom_hag = msg->data[IDX_BOTTOM_HAG];
        cargo_.support_ratio = msg->data[IDX_SUPPORT_RATIO];

        // 接收端日志
        Eigen::Vector3f size = cargo_.bbox_max - cargo_.bbox_min;
        ROS_DEBUG("[PayloadTrackInfoSub] id=%d state=%d center=(%.2f,%.2f,%.2f) "
                  "bbox_min=(%.2f,%.2f,%.2f) bbox_max=(%.2f,%.2f,%.2f) "
                  "size=(%.2f,%.2f,%.2f) pts=%d score=%.2f hag=%.2f support=%.2f",
                  cargo_.track_id, (int)cargo_.state,
                  cargo_.centroid.x(), cargo_.centroid.y(), cargo_.centroid.z(),
                  cargo_.bbox_min.x(), cargo_.bbox_min.y(), cargo_.bbox_min.z(),
                  cargo_.bbox_max.x(), cargo_.bbox_max.y(), cargo_.bbox_max.z(),
                  size.x(), size.y(), size.z(),
                  cargo_.point_count, cargo_.score, cargo_.bottom_hag, cargo_.support_ratio);

        // 检查字段合理性
        if (cargo_.support_ratio < 0 || cargo_.support_ratio > 1) {
            ROS_WARN("[PayloadTrackInfoSub] Invalid support_ratio=%.2f (expected 0~1)", cargo_.support_ratio);
        }
        if (cargo_.bottom_hag < 0) {
            ROS_WARN("[PayloadTrackInfoSub] Invalid bottom_hag=%.2f (expected > 0)", cargo_.bottom_hag);
        }
        if (size.x() <= 0 || size.y() <= 0 || size.z() <= 0) {
            ROS_WARN("[PayloadTrackInfoSub] Invalid bbox size=(%.2f,%.2f,%.2f)", size.x(), size.y(), size.z());
        }

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

        // P0-4: 直接使用从 payload_track_info 接收的 bbox，不做低通滤波
        // 这样可以确保 cargo_forbidden_zone_node 显示的框与 CargoBoxV2 计算的框一致

        // 直接使用测量值
        stable_centroid_ = cargo_.centroid;

        // 计算 size
        Eigen::Vector3f raw_size = cargo_.bbox_max - cargo_.bbox_min;
        if (raw_size.x() < 0.5f || raw_size.x() > 8.0f ||
            raw_size.y() < 0.3f || raw_size.y() > 3.0f) {
            raw_size = Eigen::Vector3f(default_length_x_, default_width_y_, default_height_z_);
        }

        // 直接使用 raw_size，不做低通滤波
        stable_size_ = raw_size;

        // 更新 bbox
        float half_l = stable_size_.x() / 2.0f + safety_margin_x_;
        float half_w = stable_size_.y() / 2.0f + safety_margin_y_;
        stable_bbox_min_ = stable_centroid_ - Eigen::Vector3f(half_l, half_w, 0);
        stable_bbox_max_ = stable_centroid_ + Eigen::Vector3f(half_l, half_w, stable_size_.z());
        stable_cargo_z_min_ = stable_centroid_.z() - stable_size_.z() / 2.0f;

        last_track_time_ = ros::Time::now();
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

        // P0.5 新增：发布三层 marker
        publishThreeLayerMarkers(now);

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
        // P5: stable_bbox 改为和 core_box 一致的线框显示
        // 使用和 publishThreeLayerMarkers 相同的 LINE_LIST 方式
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time(0);
        marker.header.frame_id = map_frame_;
        marker.ns = "cargo_stable_bbox";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        // 线宽
        marker.scale.x = 0.05;

        // 黄色线框（区别于 core_box 的绿色）
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.5);

        // 计算 box 的 8 个角点
        float cx = stable_centroid_.x();
        float cy = stable_centroid_.y();
        float cz = stable_centroid_.z();
        float hx = stable_size_.x() / 2.0f;
        float hy = stable_size_.y() / 2.0f;
        float hz = stable_size_.z() / 2.0f;

        // 最小尺寸保护
        hx = std::max(hx, 0.15f);
        hy = std::max(hy, 0.15f);
        hz = std::max(hz, 0.15f);

        // 8 个角点
        geometry_msgs::Point p[8];
        p[0].x = cx - hx; p[0].y = cy - hy; p[0].z = cz - hz;
        p[1].x = cx + hx; p[1].y = cy - hy; p[1].z = cz - hz;
        p[2].x = cx + hx; p[2].y = cy + hy; p[2].z = cz - hz;
        p[3].x = cx - hx; p[3].y = cy + hy; p[3].z = cz - hz;
        p[4].x = cx - hx; p[4].y = cy - hy; p[4].z = cz + hz;
        p[5].x = cx + hx; p[5].y = cy - hy; p[5].z = cz + hz;
        p[6].x = cx + hx; p[6].y = cy + hy; p[6].z = cz + hz;
        p[7].x = cx - hx; p[7].y = cy + hy; p[7].z = cz + hz;

        // 12 条边
        marker.points.push_back(p[0]); marker.points.push_back(p[1]);
        marker.points.push_back(p[1]); marker.points.push_back(p[2]);
        marker.points.push_back(p[2]); marker.points.push_back(p[3]);
        marker.points.push_back(p[3]); marker.points.push_back(p[0]);
        marker.points.push_back(p[4]); marker.points.push_back(p[5]);
        marker.points.push_back(p[5]); marker.points.push_back(p[6]);
        marker.points.push_back(p[6]); marker.points.push_back(p[7]);
        marker.points.push_back(p[7]); marker.points.push_back(p[4]);
        marker.points.push_back(p[0]); marker.points.push_back(p[4]);
        marker.points.push_back(p[1]); marker.points.push_back(p[5]);
        marker.points.push_back(p[2]); marker.points.push_back(p[6]);
        marker.points.push_back(p[3]); marker.points.push_back(p[7]);

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

    // base_link → map 坐标转换（Z 直接透传，天车 z 锁死）
    Eigen::Vector3f transformToMap(const Eigen::Vector3f& pt_base) {
        Eigen::Vector2f rotated_xy = (last_odom_orientation_ * pt_base).head<2>();
        Eigen::Vector2f map_xy = last_odom_position_.head<2>() + rotated_xy;
        return Eigen::Vector3f(map_xy.x(), map_xy.y(), pt_base.z());
    }

    void publishThreeLayerMarkers(const ros::Time& stamp) {
        float speed = cargo_.velocity.norm();
        bool is_moving = speed > 0.1f;

        // 防闪烁：连续帧计数
        if (is_moving) {
            moving_frame_count_++;
            stationary_frame_count_ = 0;
        } else {
            stationary_frame_count_++;
            moving_frame_count_ = 0;
        }

        bool should_show = cargo_.valid &&
                           (cargo_state_ == PayloadSemanticState::SUSPENDED_MOVING ||
                            cargo_state_ == PayloadSemanticState::GROUND_CARGO) &&
                           observed_frames_ >= 3 &&
                           moving_frame_count_ >= 3 &&
                           has_odom_;

        if (!should_show) {
            if (stationary_frame_count_ >= 3) {
                publishDeleteAllCoreBox();
            }
            return;
        }

        // base_link → map 转换
        Eigen::Vector3f center_map = transformToMap(stable_centroid_);

        float hx = std::max(stable_size_.x() / 2.0f, 0.15f);
        float hy = std::max(stable_size_.y() / 2.0f, 0.15f);
        float hz = std::max(stable_size_.z() / 2.0f, 0.04f);

        Eigen::Vector3f corners_base[8] = {
            {-hx, -hy, -hz}, { hx, -hy, -hz}, { hx,  hy, -hz}, {-hx,  hy, -hz},
            {-hx, -hy,  hz}, { hx, -hy,  hz}, { hx,  hy,  hz}, {-hx,  hy,  hz}
        };

        geometry_msgs::Point corners_map[8];
        for (int i = 0; i < 8; i++) {
            Eigen::Vector2f rot_xy = (last_odom_orientation_ * corners_base[i]).head<2>();
            Eigen::Vector2f map_xy = last_odom_position_.head<2>() + rot_xy;
            corners_map[i].x = map_xy.x();
            corners_map[i].y = map_xy.y();
            corners_map[i].z = center_map.z() + corners_base[i].z();
        }

        // CargoMarkerGuard：禁止黄色 CUBE
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "cargo_core";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.frame_locked = false;
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.85;
        marker.lifetime = ros::Duration(0.25);

        // 硬保护：拦截黄色 CUBE
        if (marker.type == visualization_msgs::Marker::CUBE &&
            marker.color.r > 0.9 && marker.color.g > 0.9 && marker.color.b < 0.1) {
            ROS_WARN_THROTTLE(1.0, "[CargoMarkerGuard] blocked yellow CUBE");
            return;
        }

        auto addEdge = [&](int a, int b) {
            marker.points.push_back(corners_map[a]);
            marker.points.push_back(corners_map[b]);
        };
        addEdge(0,1); addEdge(1,2); addEdge(2,3); addEdge(3,0);
        addEdge(4,5); addEdge(5,6); addEdge(6,7); addEdge(7,4);
        addEdge(0,4); addEdge(1,5); addEdge(2,6); addEdge(3,7);

        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(marker);
        core_bbox_marker_pub_.publish(arr);
    }

    void publishDeleteAllCoreBox() {
        visualization_msgs::MarkerArray arr;
        visualization_msgs::Marker del;
        del.header.frame_id = "map";
        del.header.stamp = ros::Time::now();
        del.ns = "cargo_core";
        del.id = 0;
        del.action = visualization_msgs::Marker::DELETEALL;

        arr.markers.push_back(del);
        core_bbox_marker_pub_.publish(arr);
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
