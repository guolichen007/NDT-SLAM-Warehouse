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

// P0-5: CargoBoxSource 枚举
enum CargoBoxSource : int {
    BOX_SOURCE_NONE = 0,
    BOX_SOURCE_V2_CORE = 1,
    BOX_SOURCE_LAST_GOOD = 2,
    BOX_SOURCE_OLD_BBOX = 3,
    BOX_SOURCE_CENTER_ONLY = 4
};

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
    int box_source = 0;  // P0-5: 框来源（0=NONE, 1=V2_CORE, 2=LAST_GOOD, 3=OLD_BBOX, 4=CENTER_ONLY）
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

    // P0-5: precise geometry 状态
    bool has_precise_bbox_ = false;
    Eigen::Vector3f precise_bbox_center_base_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f precise_bbox_size_ = Eigen::Vector3f::Zero();
    float precise_z_min_base_ = 0.0f;
    float precise_z_max_base_ = 0.0f;
    float stable_z_min_base_ = 0.0f;
    float stable_z_max_base_ = 0.0f;

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

    // Commit C: precise box info 状态
    bool precise_box_active_ = false;
    ros::Time precise_box_stamp_;
    std::vector<geometry_msgs::Point> precise_corners_map_;
    int precise_track_id_ = -1;
    int precise_source_ = BOX_SOURCE_NONE;

    // P0.5 新增：odom 状态
    Eigen::Vector3f last_odom_position_ = Eigen::Vector3f::Zero();
    Eigen::Quaternionf last_odom_orientation_ = Eigen::Quaternionf::Identity();
    bool has_odom_ = false;
    int suspect_jump_count_ = 0;

    // P0.5 新增：timer 高频发布
    ros::Timer marker_timer_;
    double marker_publish_rate_ = 15.0;  // 15Hz

    // P0-4: CargoDisplayState 独立状态结构
    struct CargoDisplayState {
        bool valid = false;
        int locked_track_id = -1;

        Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
        Eigen::Vector3f size = Eigen::Vector3f::Zero();

        Eigen::Vector3f last_good_center = Eigen::Vector3f::Zero();
        Eigen::Vector3f last_good_size = Eigen::Vector3f::Zero();

        ros::Time last_update_stamp;
        int lost_count = 0;

        int candidate_track_id = -1;
        int candidate_confirm_count = 0;

        bool deleteall_sent_after_invalid = false;
        int box_source = BOX_SOURCE_NONE;  // P0-5: 框来源

        // P0-5: 真实几何信息
        float z_min_base = 0.0f;
        float z_max_base = 0.0f;
        float yaw_base = 0.0f;
    };

    CargoDisplayState display_state_;

    // P0-6: marker 模式参数
    bool use_precise_map_corners_marker_ = false;
    bool use_base_link_marker_ = true;

    // P0-4: 常量定义
    static constexpr double DISPLAY_HOLD_TIME_SEC = 1.2;
    static constexpr double JUMP_REJECT_GATE_M = 1.20;
    static constexpr int SWITCH_CONFIRM_FRAMES = 3;
    static constexpr double SIZE_ALPHA_GROW = 0.10;
    static constexpr double SIZE_ALPHA_SHRINK = 0.02;
    static constexpr double SIZE_MAX_STEP_M = 0.08;
    static constexpr double CENTER_MAX_STEP_M = 0.30;
    static constexpr double SIZE_MIN_RATIO = 0.55;
    static constexpr double SIZE_MAX_RATIO = 1.80;
    static constexpr double DELETEALL_THROTTLE_SEC = 5.0;

    void loadConfig() {
        std::string config_file;
        pnh_.param<std::string>("config_file", config_file, "");

        // P0-6: 读取 marker 模式参数
        pnh_.param<bool>("use_precise_map_corners_marker", use_precise_map_corners_marker_, false);
        pnh_.param<bool>("use_base_link_marker", use_base_link_marker_, true);

        ROS_INFO("[CargoForbiddenZone] use_precise_map_corners_marker=%d use_base_link_marker=%d",
                 use_precise_map_corners_marker_ ? 1 : 0, use_base_link_marker_ ? 1 : 0);

        // P0-3: 配置加载保护
        if (config_file.empty()) {
            ROS_ERROR("[CargoForbiddenZone] config_file param is EMPTY");
            return;
        }

        // 检查文件是否存在
        std::ifstream f(config_file);
        if (!f.good()) {
            ROS_ERROR("[CargoForbiddenZone] config file does not exist: %s", config_file.c_str());
            return;
        }
        f.close();

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

            // P0-5: stable_centroid_ / display_state_.center_base are in base_link coordinates.
            // They must NOT be shifted by odom/map delta.
            // publishThreeLayerMarkers transforms base_link corners to map at publish time.
            // Odom callback must not modify cargo box geometry.
            // stable_centroid_ += odom_delta;  // REMOVED: This was polluting cargo geometry
        }

        last_odom_position_ = new_pos;
        last_odom_orientation_ = new_ori;
        has_odom_ = true;
    }

    // Commit C: payload_precise_box_info 回调

    // P0.5 新增：timer 高频发布 marker
    void markerTimerCallback(const ros::TimerEvent& event) {
        // 高频发布三层 marker，使用当前 odom 位置
        // 这样即使检测低频更新，marker 也会跟随 odom 平滑移动
        publishThreeLayerMarkers(ros::Time::now());
    }

    void payloadTrackCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        // P0-5: 检查消息大小，必须至少 20 个 float
        if (msg->data.size() < 20) {
            ROS_WARN("[PayloadTrackInfoSub] Invalid message size: %zu (expected >= 20)", msg->data.size());
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
        constexpr int IDX_BOX_SOURCE = 19;

        cargo_.valid = (msg->data[IDX_VALID] >= 0);
        if (!cargo_.valid) return;

        cargo_.track_id = static_cast<int>(msg->data[IDX_TRACK_ID]);
        cargo_.state = static_cast<PayloadSemanticState>(static_cast<int>(msg->data[IDX_STATE]));

        // P0-5: 读取 box_source
        cargo_.box_source = static_cast<int>(msg->data[IDX_BOX_SOURCE]);

        // P0-6: 无效 source 必须清框
        if (!isPreciseSource(cargo_.box_source)) {
            display_state_.valid = false;
            has_precise_bbox_ = false;
            publishDeleteAllCoreBoxThrottled();

            ROS_INFO_THROTTLE(
                1.0,
                "[CargoDisplayClear] source=%s action=DELETEALL",
                sourceToString(cargo_.box_source));
            return;
        }

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

        // P0-5: 从 payload 更新精确几何
        updatePreciseGeometryFromPayload(ros::Time::now());

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
        updateStableBboxStateMachine();
        updateRiskLevel();
        publishResults();
    }

    // P0-5: 判断是否是精确来源
    bool isPreciseSource(int source) const {
        return source == BOX_SOURCE_V2_CORE || source == BOX_SOURCE_LAST_GOOD;
    }

    // P0-5: 判断是否是默认来源
    bool isDefaultSource(int source) const {
        return source == BOX_SOURCE_OLD_BBOX || source == BOX_SOURCE_CENTER_ONLY || source == BOX_SOURCE_NONE;
    }

    // P0-5: source 转字符串
    const char* sourceToString(int source) const {
        switch (source) {
            case BOX_SOURCE_NONE: return "NONE";
            case BOX_SOURCE_V2_CORE: return "V2_CORE";
            case BOX_SOURCE_LAST_GOOD: return "LAST_GOOD";
            case BOX_SOURCE_OLD_BBOX: return "OLD_BBOX";
            case BOX_SOURCE_CENTER_ONLY: return "CENTER_ONLY";
            default: return "UNKNOWN";
        }
    }

    // P0-5: 判断是否是有效的精确 bbox
    bool isValidPreciseBbox(const CargoInfo& c) const {
        if (!isPreciseSource(c.box_source)) return false;

        const Eigen::Vector3f size = c.bbox_max - c.bbox_min;

        if (!std::isfinite(size.x()) ||
            !std::isfinite(size.y()) ||
            !std::isfinite(size.z())) {
            return false;
        }

        if (size.x() < 0.05f || size.y() < 0.05f || size.z() < 0.05f) {
            return false;
        }

        if (size.x() > 8.0f || size.y() > 8.0f || size.z() > 5.0f) {
            return false;
        }

        if (c.bbox_max.z() <= c.bbox_min.z()) {
            return false;
        }

        return true;
    }

    // P0-5: 从 payload 更新精确几何
    void updatePreciseGeometryFromPayload(const ros::Time& stamp) {
        if (!isValidPreciseBbox(cargo_)) {
            has_precise_bbox_ = false;

            ROS_WARN_THROTTLE(
                1.0,
                "[CargoGeometryFix] reject track=%d source=%s invalid_precise_bbox "
                "bbox_min=(%.2f,%.2f,%.2f) bbox_max=(%.2f,%.2f,%.2f)",
                cargo_.track_id,
                sourceToString(cargo_.box_source),
                cargo_.bbox_min.x(), cargo_.bbox_min.y(), cargo_.bbox_min.z(),
                cargo_.bbox_max.x(), cargo_.bbox_max.y(), cargo_.bbox_max.z());
            return;
        }

        precise_bbox_center_base_ = 0.5f * (cargo_.bbox_min + cargo_.bbox_max);
        precise_bbox_size_ = cargo_.bbox_max - cargo_.bbox_min;
        precise_z_min_base_ = cargo_.bbox_min.z();
        precise_z_max_base_ = cargo_.bbox_max.z();

        stable_centroid_ = precise_bbox_center_base_;
        stable_size_ = precise_bbox_size_;
        stable_z_min_base_ = precise_z_min_base_;
        stable_z_max_base_ = precise_z_max_base_;
        has_precise_bbox_ = true;

        ROS_DEBUG_THROTTLE(
            2.0,
            "[CargoGeometryFix] track=%d source=%s "
            "bbox_min=(%.2f,%.2f,%.2f) bbox_max=(%.2f,%.2f,%.2f) "
            "center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f]",
            cargo_.track_id,
            sourceToString(cargo_.box_source),
            cargo_.bbox_min.x(), cargo_.bbox_min.y(), cargo_.bbox_min.z(),
            cargo_.bbox_max.x(), cargo_.bbox_max.y(), cargo_.bbox_max.z(),
            stable_centroid_.x(), stable_centroid_.y(), stable_centroid_.z(),
            stable_size_.x(), stable_size_.y(), stable_size_.z(),
            stable_z_min_base_,
            stable_z_max_base_);
    }

    // P0-5: 判断 display_state 是否被污染
    bool displayStateLooksPolluted() const {
        if (!display_state_.valid) return false;

        const float center_err = (display_state_.center_base - stable_centroid_).norm();

        const bool center_z_bad =
            std::abs(display_state_.center_base.z()) < 0.05f &&
            stable_z_min_base_ > 0.3f &&
            stable_z_max_base_ > stable_z_min_base_;

        const bool default_size =
            std::abs(display_state_.size.x() - 3.0f) < 0.05f &&
            std::abs(display_state_.size.y() - 1.0f) < 0.05f &&
            std::abs(display_state_.size.z() - 1.0f) < 0.05f;

        return center_err > 0.50f || center_z_bad || default_size;
    }

    // P0-5: 从精确 bbox 重置 display_state
    void resetDisplayStateFromPreciseBbox(const ros::Time& stamp, const char* reason) {
        display_state_.valid = true;
        display_state_.locked_track_id = cargo_.track_id;
        display_state_.center_base = stable_centroid_;
        display_state_.size = stable_size_;
        display_state_.z_min_base = stable_z_min_base_;
        display_state_.z_max_base = stable_z_max_base_;
        display_state_.yaw_base = 0.0f;
        display_state_.box_source = cargo_.box_source;

        display_state_.last_good_center = stable_centroid_;
        display_state_.last_good_size = stable_size_;
        display_state_.last_update_stamp = stamp;
        display_state_.lost_count = 0;
        display_state_.candidate_track_id = -1;
        display_state_.candidate_confirm_count = 0;
        display_state_.deleteall_sent_after_invalid = false;

        ROS_WARN(
            "[CargoGeometryReset] reason=%s track=%d source=%s "
            "center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f]",
            reason,
            cargo_.track_id,
            sourceToString(cargo_.box_source),
            stable_centroid_.x(), stable_centroid_.y(), stable_centroid_.z(),
            stable_size_.x(), stable_size_.y(), stable_size_.z(),
            stable_z_min_base_,
            stable_z_max_base_);
    }

    // P0-4: 重写 CargoDisplayStateMachine
    void updateStableBboxStateMachine() {
        ros::Time now = ros::Time::now();

        // P0-5: 污染状态强制 reset
        if (has_precise_bbox_) {
            if (!display_state_.valid || displayStateLooksPolluted()) {
                resetDisplayStateFromPreciseBbox(now, "precise_bbox_or_polluted_state");
                return;
            }
        }

        // P0-5: source-aware 判断
        const bool precise_source = isPreciseSource(cargo_.box_source);

        if (cargo_.valid && !precise_source) {
            ROS_INFO_THROTTLE(
                1.0,
                "[CargoBoxSource] track=%d source=%s action=NO_GREEN_BOX keep_valid=%d",
                cargo_.track_id,
                sourceToString(cargo_.box_source),
                display_state_.valid ? 1 : 0);

            // 如果已有 precise display_state，保持 hold
            // 如果没有，不初始化绿色框
            if (!display_state_.valid) {
                publishDeleteAllCoreBoxThrottled();
                return;
            }

            // 不改变 locked_track_id，不改变 size，不改变 last_good
            // 只让原来的 hold 逻辑继续运行
            return;
        }

        // 没有有效 cargo 时，处理 hold/expire 逻辑
        if (!cargo_.valid || cargo_state_ == PayloadSemanticState::UNKNOWN ||
            cargo_state_ == PayloadSemanticState::LOST) {
            if (display_state_.valid) {
                display_state_.lost_count++;
                double hold_age = (now - display_state_.last_update_stamp).toSec();

                if (hold_age > DISPLAY_HOLD_TIME_SEC && display_state_.lost_count > 8) {
                    // 只打印一次 expire
                    if (!display_state_.deleteall_sent_after_invalid) {
                        ROS_INFO("[CargoDisplayState] expired track=%d hold_age=%.2f lost=%d",
                                 display_state_.locked_track_id, hold_age, display_state_.lost_count);
                        display_state_.valid = false;
                        display_state_.deleteall_sent_after_invalid = true;
                    }
                }
            }
            return;
        }

        // 有有效 cargo
        if (!display_state_.valid) {
            // 首次锁定
            display_state_.valid = true;
            display_state_.locked_track_id = cargo_.track_id;
            display_state_.center_base = stable_centroid_;
            display_state_.size = stable_size_;
            display_state_.last_good_center = stable_centroid_;
            display_state_.last_good_size = stable_size_;
            display_state_.last_update_stamp = now;
            display_state_.lost_count = 0;
            display_state_.candidate_track_id = -1;
            display_state_.candidate_confirm_count = 0;
            display_state_.deleteall_sent_after_invalid = false;
            display_state_.box_source = cargo_.box_source;

            // P0-5: 更新真实几何信息
            display_state_.z_min_base = stable_z_min_base_;
            display_state_.z_max_base = stable_z_max_base_;
            display_state_.yaw_base = 0.0f;  // 暂时没有 yaw，后续再做 PCA OBB
        } else if (display_state_.locked_track_id == cargo_.track_id) {
            // 同 track 更新
            display_state_.center_base = limitCenterStep(display_state_.center_base, stable_centroid_);
            display_state_.size = safeSizeUpdate(display_state_.size, stable_size_);
            display_state_.last_good_center = display_state_.center_base;
            display_state_.last_good_size = display_state_.size;
            display_state_.last_update_stamp = now;
            display_state_.lost_count = 0;
            display_state_.candidate_track_id = -1;
            display_state_.candidate_confirm_count = 0;
            display_state_.deleteall_sent_after_invalid = false;
            display_state_.box_source = cargo_.box_source;

            // P0-5: 更新真实几何信息
            display_state_.z_min_base = stable_z_min_base_;
            display_state_.z_max_base = stable_z_max_base_;
            display_state_.yaw_base = 0.0f;  // 暂时没有 yaw，后续再做 PCA OBB

            // P0-5: 防默认尺寸锁死
            const bool size_still_default =
                std::abs(display_state_.size.x() - 3.0f) < 0.05f &&
                std::abs(display_state_.size.y() - 1.0f) < 0.05f &&
                std::abs(display_state_.size.z() - 1.0f) < 0.05f;

            const bool stable_not_default =
                (std::abs(stable_size_.x() - 3.0f) > 0.10f ||
                 std::abs(stable_size_.y() - 1.0f) > 0.10f ||
                 std::abs(stable_size_.z() - 1.0f) > 0.10f);

            if (size_still_default && stable_not_default) {
                display_state_.size = stable_size_;
            }
        } else {
            // 不同 track，检查 jump
            double jump = (stable_centroid_ - display_state_.center_base).norm();

            if (jump > JUMP_REJECT_GATE_M) {
                // jump 太大，拒绝
                ROS_WARN_THROTTLE(1.0, "[CargoDisplayState] reject jump=%.2f old=%d new=%d",
                                  jump, display_state_.locked_track_id, cargo_.track_id);
                display_state_.candidate_track_id = -1;
                display_state_.candidate_confirm_count = 0;
                return;
            }

            // 小跳变，连续确认后切换
            if (display_state_.candidate_track_id != cargo_.track_id) {
                display_state_.candidate_track_id = cargo_.track_id;
                display_state_.candidate_confirm_count = 1;
            } else {
                display_state_.candidate_confirm_count++;
            }

            if (display_state_.candidate_confirm_count >= SWITCH_CONFIRM_FRAMES) {
                // 切换到新 track
                ROS_WARN("[CargoDisplayState] switch old=%d new=%d confirm=%d",
                         display_state_.locked_track_id, cargo_.track_id, display_state_.candidate_confirm_count);
                display_state_.locked_track_id = cargo_.track_id;
                display_state_.center_base = stable_centroid_;
                display_state_.size = stable_size_;
                display_state_.last_good_center = stable_centroid_;
                display_state_.last_good_size = stable_size_;
                display_state_.last_update_stamp = now;
                display_state_.lost_count = 0;
                display_state_.candidate_track_id = -1;
                display_state_.candidate_confirm_count = 0;
                display_state_.valid = true;
                display_state_.deleteall_sent_after_invalid = false;
                display_state_.box_source = cargo_.box_source;

                // P0-5: 更新真实几何信息
                display_state_.z_min_base = stable_z_min_base_;
                display_state_.z_max_base = stable_z_max_base_;
                display_state_.yaw_base = 0.0f;  // 暂时没有 yaw，后续再做 PCA OBB
            }
        }
    }

    // P0-4: 安全尺寸更新函数
    Eigen::Vector3f safeSizeUpdate(const Eigen::Vector3f& old_size, const Eigen::Vector3f& new_size) {
        Eigen::Vector3f result = old_size;

        for (int i = 0; i < 3; ++i) {
            const double old_val = old_size[i];
            const double new_val = new_size[i];

            if (old_val < 0.01) {
                result[i] = new_val;
                continue;
            }

            const double ratio = new_val / old_val;
            if (ratio < SIZE_MIN_RATIO || ratio > SIZE_MAX_RATIO) {
                continue;
            }

            const double diff = new_val - old_val;
            const double limited = std::max(-SIZE_MAX_STEP_M, std::min(SIZE_MAX_STEP_M, diff));
            const double alpha = diff > 0.0 ? SIZE_ALPHA_GROW : SIZE_ALPHA_SHRINK;
            result[i] = old_val + alpha * limited;
        }

        return result;
    }

    // P0-4: 限制中心步长
    Eigen::Vector3f limitCenterStep(const Eigen::Vector3f& old_center, const Eigen::Vector3f& new_center) {
        Eigen::Vector3f diff = new_center - old_center;
        float dist = diff.norm();

        if (dist > CENTER_MAX_STEP_M) {
            diff *= CENTER_MAX_STEP_M / dist;
        }

        return old_center + diff;
    }

    // P0-4: 节流 DELETEALL
    void publishDeleteAllCoreBoxThrottled() {
        static ros::Time last_deleteall_time;
        ros::Time now = ros::Time::now();

        if ((now - last_deleteall_time).toSec() < DELETEALL_THROTTLE_SEC) {
            return;
        }

        last_deleteall_time = now;
        publishDeleteAllCoreBox();
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

    // base_link → map 坐标转换 helper
    Eigen::Vector3f transformToMap(const Eigen::Vector3f& pt_base) {
        Eigen::Vector3f rotated = last_odom_orientation_ * pt_base;
        return last_odom_position_ + rotated;
    }

    // v8-stable-r3: LINE_LIST 线框 + base_link 坐标系 + frame_locked
    void publishThreeLayerMarkers(const ros::Time& stamp) {
        // P0-6: 禁用 map corners 主显示，改用 base_link marker
        if (use_precise_map_corners_marker_ &&
            precise_box_active_ &&
            (ros::Time::now() - precise_box_stamp_).toSec() < 0.5) {
            // 只允许发 debug marker，不能发 /cargo_core_bbox_marker
            ROS_INFO_THROTTLE(1.0, "[CargoMarkerMapCorners] debug_only=1 not_core_marker");
            // 不 return，继续往下走 base_link marker
        }

        // 检查 display_state 是否有效
        if (!display_state_.valid) {
            publishDeleteAllCoreBoxThrottled();
            return;
        }

        // source 检查，禁止默认框发布到绿色 core marker
        if (!isPreciseSource(display_state_.box_source)) {
            publishDeleteAllCoreBoxThrottled();
            ROS_DEBUG("[CargoBoxSource] track=%d source=%s action=NO_GREEN_BOX",
                      display_state_.locked_track_id,
                      sourceToString(display_state_.box_source));
            return;
        }

        // 检查是否启用 base_link marker
        if (!use_base_link_marker_) {
            publishDeleteAllCoreBoxThrottled();
            return;
        }

        ROS_INFO_THROTTLE(
            2.0,
            "[CargoMarkerMode] mode=base_link use_map_corners=%d",
            use_precise_map_corners_marker_ ? 1 : 0);

        // 构造 base_link marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time(0);
        marker.ns = "cargo_core";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.frame_locked = true;
        marker.scale.x = 0.05;
        marker.lifetime = ros::Duration(0.3);

        // 根据 source 设置颜色
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = (display_state_.box_source == BOX_SOURCE_V2_CORE) ? 0.9f : 0.45f;

        // 角点生成（base_link 坐标系）
        const float hx = std::max(display_state_.size.x() * 0.5f, 0.05f);
        const float hy = std::max(display_state_.size.y() * 0.5f, 0.05f);
        const float z_min = display_state_.z_min_base;
        const float z_max = display_state_.z_max_base;

        // z 检查
        if (z_max <= z_min || (z_max - z_min) < 0.05f) {
            ROS_WARN_THROTTLE(1.0, "[CargoMarkerBaseLink] invalid_z z=[%.2f,%.2f]", z_min, z_max);
            publishDeleteAllCoreBoxThrottled();
            return;
        }

        const float yaw = display_state_.yaw_base;
        const float cy = std::cos(yaw);
        const float sy = std::sin(yaw);

        std::vector<Eigen::Vector2f> local_xy = {
            {-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}
        };

        std::vector<Eigen::Vector3f> corners;
        for (float z : {z_min, z_max}) {
            for (const auto& xy : local_xy) {
                Eigen::Vector3f p;
                p.x() = display_state_.center_base.x() + cy * xy.x() - sy * xy.y();
                p.y() = display_state_.center_base.y() + sy * xy.x() + cy * xy.y();
                p.z() = z;
                corners.push_back(p);
            }
        }

        const int edges[12][2] = {
            {0,1},{1,2},{2,3},{3,0},
            {4,5},{5,6},{6,7},{7,4},
            {0,4},{1,5},{2,6},{3,7}
        };

        for (const auto& e : edges) {
            geometry_msgs::Point p;
            p.x = corners[e[0]].x();
            p.y = corners[e[0]].y();
            p.z = corners[e[0]].z();
            marker.points.push_back(p);

            p.x = corners[e[1]].x();
            p.y = corners[e[1]].y();
            p.z = corners[e[1]].z();
            marker.points.push_back(p);
        }

        // 发布 marker
        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(marker);
        core_bbox_marker_pub_.publish(arr);

        ROS_DEBUG_THROTTLE(
            2.0,
            "[CargoMarkerBaseLink] track=%d source=%s frame=base_link frame_locked=1 "
            "center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f]",
            display_state_.locked_track_id,
            sourceToString(display_state_.box_source),
            display_state_.center_base.x(), display_state_.center_base.y(), display_state_.center_base.z(),
            display_state_.size.x(), display_state_.size.y(), display_state_.size.z(),
            display_state_.z_min_base, display_state_.z_max_base);
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
