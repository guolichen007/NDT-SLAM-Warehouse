/**
 * cargo_forbidden_zone_node.cpp
 *
 * 天车吊货 2.5D 禁行区节点
 * - 2.5D 高度禁行判断（obstacle_z_max + z_clearance >= cargo_z_min）
 * - CargoLocalization 锁定 track_id，避免跳变
 * - bbox 低通滤波 + 跳变限制
 * - 红色 overlay 显示在当前吊货高度层
 * - 预测/STOP 可配置禁用
 */

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace ndt_slam {

// 货物状态
enum class CargoState {
    NO_CARGO = 0,
    CANDIDATE = 1,
    TRACKING = 2,
    LOST = 3
};

// 风险等级（保留，但 decision.enabled=false 时只输出 IDLE/UNKNOWN）
enum class RiskLevel {
    IDLE = 0,
    NORMAL = 1,
    WARNING = 2,
    SLOW_DOWN = 3,
    STOP = 4,
    UNKNOWN = 5
};

// 2.5D 栅格单元
struct Cell2_5D {
    bool occupied = false;
    float z_min = std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    int point_count = 0;
};

// 吊货信息（原始）
struct CargoRawInfo {
    bool valid = false;
    int track_id = -1;
    int state = 0;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();
    int point_count = 0;
    float track_duration = 0.0f;
    float direction_consistency = 0.0f;
    float map_displacement = 0.0f;
};

// 稳定吊货信息（滤波后）
struct StableCargoInfo {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f size = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();
    float cargo_z_min = 0.0f;
    bool valid = false;
};

class CargoForbiddenZoneNode {
public:
    CargoForbiddenZoneNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh) {
        loadConfig();
        loadTilesObjects();
        buildBaseObstacleGrid();
        setupPublishers();
        setupSubscribers();

        ROS_INFO("[CargoForbiddenZone] Node initialized");
        ROS_INFO("[CargoForbiddenZone] Grid: %d x %d, resolution=%.2f",
                 grid_width_, grid_height_, resolution_);
        ROS_INFO("[CargoForbiddenZone] Occupied cells: %d", occupied_cell_count_);
        ROS_INFO("[CargoForbiddenZone] z_clearance: %.2f", z_clearance_);
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
    std::string objects_tiles_dir_;
    double resolution_ = 0.10;
    int min_points_per_cell_ = 2;
    double min_obstacle_height_ = 0.15;
    double z_clearance_ = 0.30;

    // 货物定位
    bool cargo_localization_enabled_ = true;
    int max_lost_frames_ = 8;
    int moving_confirm_frames_ = 2;
    int min_bbox_point_count_ = 50;
    double min_map_displacement_ = 0.25;
    double min_direction_consistency_ = 0.65;

    // bbox 滤波
    double centroid_filter_alpha_ = 0.60;
    double size_filter_alpha_ = 0.25;
    double max_size_change_per_frame_ = 0.50;
    double max_valid_length_x_ = 8.0;
    double max_valid_width_y_ = 3.0;
    double max_valid_height_z_ = 3.0;
    double min_valid_length_x_ = 0.5;
    double min_valid_width_y_ = 0.3;
    double min_valid_height_z_ = 0.2;

    // 默认尺寸
    double default_length_x_ = 3.0;
    double default_width_y_ = 1.0;
    double default_height_z_ = 1.0;

    // 安全边距
    double safety_margin_x_ = 0.25;
    double safety_margin_y_ = 0.25;

    // bbox 滤波参数
    double max_position_jump_ = 0.80;
    bool use_velocity_compensation_ = true;
    double max_compensation_dt_ = 0.30;
    Eigen::Vector3f stable_velocity_ = Eigen::Vector3f::Zero();
    ros::Time last_track_time_;

    // 功能开关
    bool prediction_enabled_ = false;
    bool collision_warning_enabled_ = false;
    bool decision_enabled_ = false;

    // 可视化配置
    bool publish_forbidden_overlay_markers_ = true;
    std::string forbidden_overlay_mode_ = "height_binned_volume";
    int overlay_stride_ = 1;
    double height_bin_size_ = 0.25;
    int max_overlay_cells_ = 30000;
    double forbidden_overlay_alpha_ = 0.35;
    bool publish_height_slice_debug_ = false;
    bool show_raw_bbox_ = true;
    bool show_stable_bbox_ = true;
    bool show_status_text_ = true;

    // 发布者（新增）
    ros::Publisher height_slice_marker_pub_;

    // 2.5D 栅格
    int grid_width_ = 0;
    int grid_height_ = 0;
    float origin_x_ = 0.0f;
    float origin_y_ = 0.0f;
    std::vector<Cell2_5D> obstacle_grid_;
    int occupied_cell_count_ = 0;

    // 货物状态机
    CargoState cargo_state_ = CargoState::NO_CARGO;
    int current_track_id_ = -1;
    int lost_count_ = 0;

    // 原始吊货信息
    CargoRawInfo cargo_raw_;

    // 稳定吊货信息（滤波后）
    StableCargoInfo stable_cargo_;

    // 风险等级
    RiskLevel risk_level_ = RiskLevel::UNKNOWN;

    // 发布者
    ros::Publisher forbidden_grid_pub_;
    ros::Publisher risk_level_pub_;
    ros::Publisher forbidden_overlay_marker_pub_;
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
            // 兼容根节点
            YAML::Node root = cfg["cargo_forbidden_zone"] ? cfg["cargo_forbidden_zone"] : cfg;

            map_frame_ = root["map_frame"].as<std::string>("map");

            auto input = root["input"];
            objects_tiles_dir_ = input["objects_tiles_dir"].as<std::string>("");

            auto grid = root["grid"];
            resolution_ = grid["resolution"].as<double>(0.10);
            min_points_per_cell_ = grid["min_points_per_cell"].as<int>(2);
            min_obstacle_height_ = grid["min_obstacle_height"].as<double>(0.15);
            z_clearance_ = grid["z_clearance"].as<double>(0.30);

            if (root["cargo_localization"]) {
                auto cl = root["cargo_localization"];
                cargo_localization_enabled_ = cl["enabled"].as<bool>(true);
                max_lost_frames_ = cl["max_lost_frames"].as<int>(8);
                moving_confirm_frames_ = cl["moving_confirm_frames"].as<int>(2);
                min_bbox_point_count_ = cl["min_bbox_point_count"].as<int>(50);
                min_map_displacement_ = cl["min_map_displacement"].as<double>(0.25);
                min_direction_consistency_ = cl["min_direction_consistency"].as<double>(0.65);
                centroid_filter_alpha_ = cl["centroid_filter_alpha"].as<double>(0.60);
                size_filter_alpha_ = cl["size_filter_alpha"].as<double>(0.25);
            }

            if (root["bbox_filter"]) {
                auto bf = root["bbox_filter"];
                max_size_change_per_frame_ = bf["max_size_change_per_frame"].as<double>(0.40);
                max_valid_length_x_ = bf["max_valid_length_x"].as<double>(8.0);
                max_valid_width_y_ = bf["max_valid_width_y"].as<double>(3.0);
                max_valid_height_z_ = bf["max_valid_height_z"].as<double>(3.0);
                min_valid_length_x_ = bf["min_valid_length_x"].as<double>(0.5);
                min_valid_width_y_ = bf["min_valid_width_y"].as<double>(0.3);
                min_valid_height_z_ = bf["min_valid_height_z"].as<double>(0.2);
                max_position_jump_ = bf["max_position_jump"].as<double>(0.80);
                use_velocity_compensation_ = bf["use_velocity_compensation"].as<bool>(true);
                max_compensation_dt_ = bf["max_compensation_dt"].as<double>(0.30);
            }

            if (root["cargo_size"]) {
                auto cs = root["cargo_size"];
                default_length_x_ = cs["default_length_x"].as<double>(3.0);
                default_width_y_ = cs["default_width_y"].as<double>(1.0);
                default_height_z_ = cs["default_height_z"].as<double>(1.0);
            }

            if (root["inflation"]) {
                auto inf = root["inflation"];
                safety_margin_x_ = inf["safety_margin_x"].as<double>(0.25);
                safety_margin_y_ = inf["safety_margin_y"].as<double>(0.25);
            }

            if (root["prediction"]) {
                prediction_enabled_ = root["prediction"]["enabled"].as<bool>(false);
            }
            if (root["collision_warning"]) {
                collision_warning_enabled_ = root["collision_warning"]["enabled"].as<bool>(false);
            }
            if (root["decision"]) {
                decision_enabled_ = root["decision"]["enabled"].as<bool>(false);
            }

            if (root["visualization"]) {
                auto vis = root["visualization"];
                publish_forbidden_overlay_markers_ = vis["publish_forbidden_overlay_markers"].as<bool>(true);
                forbidden_overlay_mode_ = vis["forbidden_overlay_mode"].as<std::string>("height_binned_volume");
                overlay_stride_ = vis["overlay_stride"].as<int>(1);
                height_bin_size_ = vis["height_bin_size"].as<double>(0.25);
                max_overlay_cells_ = vis["max_overlay_cells"].as<int>(30000);
                forbidden_overlay_alpha_ = vis["forbidden_overlay_alpha"].as<double>(0.35);
                publish_height_slice_debug_ = vis["publish_height_slice_debug"].as<bool>(false);
                show_raw_bbox_ = vis["show_raw_bbox"].as<bool>(true);
                show_stable_bbox_ = vis["show_stable_bbox"].as<bool>(true);
                show_status_text_ = vis["show_status_text"].as<bool>(true);
            }

            ROS_INFO("[CargoForbiddenZone] Config loaded from %s", config_file.c_str());
            ROS_INFO("[CargoForbiddenZone] z_clearance=%.2f, min_obstacle_height=%.2f",
                     z_clearance_, min_obstacle_height_);
            ROS_INFO("[CargoForbiddenZone] prediction=%s, collision_warning=%s, decision=%s",
                     prediction_enabled_ ? "true" : "false",
                     collision_warning_enabled_ ? "true" : "false",
                     decision_enabled_ ? "true" : "false");
        } catch (const std::exception& e) {
            ROS_ERROR("[CargoForbiddenZone] Config error: %s", e.what());
        }
    }

    void loadTilesObjects() {
        if (objects_tiles_dir_.empty()) {
            ROS_WARN("[CargoForbiddenZone] No tiles directory specified");
            return;
        }

        ROS_INFO("[CargoForbiddenZone] Loading tiles from %s", objects_tiles_dir_.c_str());

        pcl::PointCloud<pcl::PointXYZ>::Ptr all_points(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& entry : std::filesystem::directory_iterator(objects_tiles_dir_)) {
            if (entry.path().extension() == ".pcd") {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPCDFile(entry.path().string(), *cloud) == 0) {
                    *all_points += *cloud;
                    ROS_INFO("[CargoForbiddenZone] Loaded %s: %zu points",
                             entry.path().filename().c_str(), cloud->size());
                }
            }
        }

        ROS_INFO("[CargoForbiddenZone] Total points loaded: %zu", all_points->size());

        if (all_points->empty()) {
            ROS_ERROR("[CargoForbiddenZone] No points loaded!");
            return;
        }

        // 计算栅格范围
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();

        for (const auto& p : all_points->points) {
            min_x = std::min(min_x, p.x);
            min_y = std::min(min_y, p.y);
            max_x = std::max(max_x, p.x);
            max_y = std::max(max_y, p.y);
        }

        origin_x_ = min_x;
        origin_y_ = min_y;
        grid_width_ = static_cast<int>((max_x - min_x) / resolution_) + 1;
        grid_height_ = static_cast<int>((max_y - min_y) / resolution_) + 1;

        ROS_INFO("[CargoForbiddenZone] Grid range: (%.1f,%.1f) to (%.1f,%.1f), size: %d x %d",
                 min_x, min_y, max_x, max_y, grid_width_, grid_height_);

        // 构建 2.5D 栅格
        obstacle_grid_.resize(grid_width_ * grid_height_);

        for (const auto& p : all_points->points) {
            int ix = static_cast<int>((p.x - origin_x_) / resolution_);
            int iy = static_cast<int>((p.y - origin_y_) / resolution_);

            if (ix < 0 || ix >= grid_width_ || iy < 0 || iy >= grid_height_) continue;

            int idx = iy * grid_width_ + ix;
            auto& cell = obstacle_grid_[idx];
            cell.occupied = true;
            cell.z_min = std::min(cell.z_min, p.z);
            cell.z_max = std::max(cell.z_max, p.z);
            cell.point_count++;
        }

        // 过滤：点数不足 或 高度不足
        occupied_cell_count_ = 0;
        for (auto& cell : obstacle_grid_) {
            if (cell.point_count < min_points_per_cell_) {
                cell.occupied = false;
            }
            if (cell.occupied && (cell.z_max - cell.z_min) < min_obstacle_height_) {
                cell.occupied = false;
            }
            if (cell.occupied) occupied_cell_count_++;
        }

        ROS_INFO("[CargoForbiddenZone] Occupied cells: %d (after height filter)", occupied_cell_count_);
    }

    void buildBaseObstacleGrid() {
        // 已在 loadTilesObjects 中完成
    }

    void setupPublishers() {
        forbidden_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/cargo_forbidden_grid", 1, true);
        risk_level_pub_ = nh_.advertise<std_msgs::Int32>("/cargo_collision_warning", 10);
        forbidden_overlay_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_forbidden_overlay_markers", 10, true);
        height_slice_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_forbidden_height_slice_markers", 10, true);
        cargo_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_forbidden_markers", 10);
    }

    void setupSubscribers() {
        payload_track_sub_ = nh_.subscribe("/payload_track_info", 10,
                                           &CargoForbiddenZoneNode::payloadTrackCallback, this);
    }

    void payloadTrackCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 19) return;

        cargo_raw_.valid = (msg->data[1] >= 0);
        if (!cargo_raw_.valid) return;

        cargo_raw_.track_id = static_cast<int>(msg->data[1]);
        cargo_raw_.state = static_cast<int>(msg->data[2]);
        cargo_raw_.centroid = Eigen::Vector3f(msg->data[3], msg->data[4], msg->data[5]);
        cargo_raw_.velocity = Eigen::Vector3f(msg->data[6], msg->data[7], msg->data[8]);
        cargo_raw_.bbox_min = Eigen::Vector3f(msg->data[9], msg->data[10], msg->data[11]);
        cargo_raw_.bbox_max = Eigen::Vector3f(msg->data[12], msg->data[13], msg->data[14]);
        cargo_raw_.point_count = static_cast<int>(msg->data[15]);
        cargo_raw_.track_duration = msg->data[16];
        cargo_raw_.direction_consistency = msg->data[17];
        cargo_raw_.map_displacement = msg->data[18];
    }

    void update() {
        updateCargoLocalization();
        updateRiskLevel();
        publishResults();
    }

    void updateCargoLocalization() {
        if (!cargo_localization_enabled_) {
            // 简单模式：直接使用原始数据
            if (cargo_raw_.valid) {
                stable_cargo_.centroid = cargo_raw_.centroid;
                stable_cargo_.size = Eigen::Vector3f(default_length_x_, default_width_y_, default_height_z_);
                stable_cargo_.valid = true;
                updateStableBbox();
            } else {
                stable_cargo_.valid = false;
            }
            return;
        }

        // 状态机模式
        if (cargo_state_ == CargoState::TRACKING) {
            // 已锁定 track，检查是否还有效
            if (cargo_raw_.valid && cargo_raw_.track_id == current_track_id_ &&
                isTrackStillValid(cargo_raw_)) {
                // 更新稳定信息
                updateStableCargoFromRaw(cargo_raw_);
                lost_count_ = 0;
                return;
            }

            // track 丢失
            lost_count_++;
            if (lost_count_ <= max_lost_frames_) {
                // 保持上次稳定值
                return;
            }

            // 超过最大丢失帧数，切换到 LOST
            ROS_INFO("[CargoLocalization] Track %d lost after %d frames", current_track_id_, lost_count_);
            cargo_state_ = CargoState::LOST;
            current_track_id_ = -1;
            stable_cargo_.valid = false;
        }

        // NO_CARGO 或 LOST 状态：寻找新的有效 track
        if (cargo_raw_.valid && isValidNewTrack(cargo_raw_)) {
            // 锁定新 track
            current_track_id_ = cargo_raw_.track_id;
            cargo_state_ = CargoState::TRACKING;
            lost_count_ = 0;

            // 初始化稳定值
            stable_cargo_.centroid = cargo_raw_.centroid;
            stable_cargo_.size = Eigen::Vector3f(default_length_x_, default_width_y_, default_height_z_);
            stable_cargo_.valid = true;

            ROS_INFO("[CargoLocalization] Locked track %d, centroid=(%.1f,%.1f,%.1f)",
                     current_track_id_,
                     stable_cargo_.centroid.x(),
                     stable_cargo_.centroid.y(),
                     stable_cargo_.centroid.z());
        } else {
            cargo_state_ = CargoState::NO_CARGO;
            stable_cargo_.valid = false;
        }
    }

    bool isTrackStillValid(const CargoRawInfo& track) {
        return track.state == 2 &&  // DYNAMIC
               track.point_count >= min_bbox_point_count_ / 2;  // 放宽条件
    }

    bool isValidNewTrack(const CargoRawInfo& track) {
        return track.state == 2 &&  // DYNAMIC
               track.point_count >= min_bbox_point_count_ &&
               track.map_displacement >= min_map_displacement_ &&
               track.direction_consistency >= min_direction_consistency_;
    }

    void updateStableCargoFromRaw(const CargoRawInfo& raw) {
        // 速度补偿：预测 centroid 位置
        Eigen::Vector3f predicted_centroid = raw.centroid;
        if (use_velocity_compensation_ && stable_cargo_.valid) {
            ros::Time now = ros::Time::now();
            double dt = (now - last_track_time_).toSec();
            dt = std::min(dt, max_compensation_dt_);

            // 使用稳定速度补偿
            predicted_centroid = stable_cargo_.centroid + stable_velocity_ * dt;

            // 检查跳变
            float position_jump = (raw.centroid - stable_cargo_.centroid).norm();
            if (position_jump > max_position_jump_) {
                // 跳变过大，使用原始值
                predicted_centroid = raw.centroid;
                ROS_WARN_THROTTLE(2.0, "[CargoLocalization] Large position jump: %.2f > %.2f, using raw",
                                  position_jump, max_position_jump_);
            }
        }
        last_track_time_ = ros::Time::now();

        // 低通滤波 centroid
        stable_cargo_.centroid = centroid_filter_alpha_ * predicted_centroid +
                                 (1.0f - centroid_filter_alpha_) * stable_cargo_.centroid;

        // 更新速度估计
        if (stable_cargo_.valid) {
            Eigen::Vector3f velocity = (stable_cargo_.centroid - predicted_centroid) / 0.1f;  // 假设 10Hz
            stable_velocity_ = 0.7f * stable_velocity_ + 0.3f * velocity;
        }

        // 计算 raw size
        Eigen::Vector3f raw_size = raw.bbox_max - raw.bbox_min;

        // 检查 size 合理性（使用配置的 min/max）
        bool size_valid = (raw_size.x() >= min_valid_length_x_ && raw_size.x() <= max_valid_length_x_ &&
                          raw_size.y() >= min_valid_width_y_ && raw_size.y() <= max_valid_width_y_ &&
                          raw_size.z() >= min_valid_height_z_ && raw_size.z() <= max_valid_height_z_);

        if (!size_valid) {
            // size 不合理，使用默认值
            raw_size = Eigen::Vector3f(default_length_x_, default_width_y_, default_height_z_);
        }

        // 低通滤波 size
        stable_cargo_.size = size_filter_alpha_ * raw_size +
                             (1.0f - size_filter_alpha_) * stable_cargo_.size;

        // 限制单帧变化
        Eigen::Vector3f size_change = stable_cargo_.size - raw_size;
        for (int i = 0; i < 3; i++) {
            if (std::abs(size_change[i]) > max_size_change_per_frame_) {
                stable_cargo_.size[i] = raw_size[i] + std::copysign(max_size_change_per_frame_, size_change[i]);
            }
        }

        // 更新 bbox
        updateStableBbox();
    }

    void updateStableBbox() {
        float half_l = stable_cargo_.size.x() / 2.0f + safety_margin_x_;
        float half_w = stable_cargo_.size.y() / 2.0f + safety_margin_y_;

        stable_cargo_.bbox_min = stable_cargo_.centroid - Eigen::Vector3f(half_l, half_w, 0);
        stable_cargo_.bbox_max = stable_cargo_.centroid + Eigen::Vector3f(half_l, half_w, stable_cargo_.size.z());

        // 计算 cargo_z_min（吊货底部高度）
        stable_cargo_.cargo_z_min = stable_cargo_.centroid.z() - stable_cargo_.size.z() / 2.0f;
    }

    void updateRiskLevel() {
        if (!stable_cargo_.valid) {
            risk_level_ = RiskLevel::UNKNOWN;
            return;
        }

        // decision.enabled = false 时，只输出 IDLE
        if (!decision_enabled_) {
            risk_level_ = RiskLevel::IDLE;
            return;
        }

        // 以下逻辑保留，但 decision.enabled = false 时不执行
        // 后续启用时需要调用 isForbiddenForCargo 判断
        risk_level_ = RiskLevel::NORMAL;
    }

    // 2.5D 高度禁行判断
    bool isForbiddenForCargo(const Cell2_5D& cell, float cargo_z_min) {
        if (!cell.occupied) {
            return false;
        }
        return cell.z_max + z_clearance_ >= cargo_z_min;
    }

    void publishResults() {
        ros::Time now = ros::Time::now();

        // 发布风险等级
        std_msgs::Int32 risk_msg;
        risk_msg.data = static_cast<int>(risk_level_);
        risk_level_pub_.publish(risk_msg);

        // 发布 OccupancyGrid（debug 用）
        publishOccupancyGrid(now);

        // 发布红色 overlay（当前高度层禁行区）
        if (publish_forbidden_overlay_markers_) {
            publishForbiddenOverlayMarkers(now);
        }

        // 发布 cargo markers（bbox + 状态文字）
        publishCargoMarkers(now);
    }

    void publishOccupancyGrid(const ros::Time& stamp) {
        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.stamp = stamp;
        grid_msg.header.frame_id = map_frame_;

        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_width_;
        grid_msg.info.height = grid_height_;
        grid_msg.info.origin.position.x = origin_x_;
        grid_msg.info.origin.position.y = origin_y_;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_width_ * grid_height_, 0);

        for (int iy = 0; iy < grid_height_; ++iy) {
            for (int ix = 0; ix < grid_width_; ++ix) {
                int idx = iy * grid_width_ + ix;
                if (obstacle_grid_[idx].occupied) {
                    grid_msg.data[idx] = 100;
                }
            }
        }

        forbidden_grid_pub_.publish(grid_msg);
    }

    void publishForbiddenOverlayMarkers(const ros::Time& stamp) {
        if (!publish_forbidden_overlay_markers_) return;

        // 确定 cargo_z_min
        float cargo_z_min = stable_cargo_.valid ? stable_cargo_.cargo_z_min : 0.0f;

        // 按模式分发
        if (forbidden_overlay_mode_ == "height_binned_volume") {
            publishHeightBinnedVolumeMarkers(stamp, cargo_z_min);
        } else if (forbidden_overlay_mode_ == "height_slice") {
            publishHeightSliceMarkers(stamp, cargo_z_min);
        } else {
            // 默认使用 height_binned_volume
            publishHeightBinnedVolumeMarkers(stamp, cargo_z_min);
        }

        // 发布 debug 薄片（如果启用）
        if (publish_height_slice_debug_) {
            publishHeightSliceMarkers(stamp, cargo_z_min);
        }
    }

    void publishHeightBinnedVolumeMarkers(const ros::Time& stamp, float cargo_z_min) {
        visualization_msgs::MarkerArray markers;

        // 按高度分 bin
        struct HeightBin {
            float z_bottom;
            float z_top;
            float height;
            std::vector<geometry_msgs::Point> points;
        };
        std::map<int, HeightBin> bins;

        int occupied_count = 0;
        int forbidden_count = 0;
        int passable_count = 0;
        float obs_z_min = std::numeric_limits<float>::max();
        float obs_z_max = -std::numeric_limits<float>::max();

        // 遍历所有 cell
        for (int iy = 0; iy < grid_height_; iy += overlay_stride_) {
            for (int ix = 0; ix < grid_width_; ix += overlay_stride_) {
                int idx = iy * grid_width_ + ix;
                const auto& cell = obstacle_grid_[idx];

                if (!cell.occupied) continue;
                occupied_count++;

                // 记录 z 范围
                obs_z_min = std::min(obs_z_min, cell.z_min);
                obs_z_max = std::max(obs_z_max, cell.z_max);

                // 高度过滤
                if (!isForbiddenForCargo(cell, cargo_z_min)) {
                    passable_count++;
                    continue;
                }
                forbidden_count++;

                // 计算高度
                float z_bottom = cell.z_min;
                float z_top = cell.z_max + z_clearance_;
                float height = z_top - z_bottom;

                // 按高度分 bin
                int bin_id = static_cast<int>(std::ceil(height / height_bin_size_));
                if (bins.find(bin_id) == bins.end()) {
                    bins[bin_id] = {z_bottom, z_top, height, {}};
                }

                // 添加点
                geometry_msgs::Point p;
                p.x = origin_x_ + (ix + 0.5) * resolution_;
                p.y = origin_y_ + (iy + 0.5) * resolution_;
                p.z = z_bottom + height / 2.0f;
                bins[bin_id].points.push_back(p);
            }
        }

        // 每个 bin 发一个 CUBE_LIST
        int marker_id = 0;
        for (const auto& [bin_id, bin] : bins) {
            if (bin.points.empty()) continue;

            visualization_msgs::Marker cube_list;
            cube_list.header.stamp = stamp;
            cube_list.header.frame_id = map_frame_;
            cube_list.ns = "forbidden_volume";
            cube_list.id = marker_id++;
            cube_list.type = visualization_msgs::Marker::CUBE_LIST;
            cube_list.action = visualization_msgs::Marker::ADD;
            cube_list.pose.orientation.w = 1.0;
            cube_list.scale.x = resolution_ * overlay_stride_;
            cube_list.scale.y = resolution_ * overlay_stride_;
            cube_list.scale.z = std::max(0.05f, bin.height);
            cube_list.color.r = 1.0;
            cube_list.color.g = 0.0;
            cube_list.color.b = 0.0;
            cube_list.color.a = forbidden_overlay_alpha_;
            cube_list.points = bin.points;
            markers.markers.push_back(cube_list);
        }

        // 清除多余的 marker
        visualization_msgs::Marker clear_marker;
        clear_marker.header.stamp = stamp;
        clear_marker.header.frame_id = map_frame_;
        clear_marker.ns = "forbidden_volume";
        clear_marker.id = marker_id;
        clear_marker.action = visualization_msgs::Marker::DELETE;
        markers.markers.push_back(clear_marker);

        forbidden_overlay_marker_pub_.publish(markers);

        // 统计日志
        ROS_INFO_THROTTLE(2.0, "[ForbiddenOverlay] mode=height_binned_volume cargo_z_min=%.2f "
                          "occupied=%d forbidden=%d passable=%d bins=%zu "
                          "obs_z_range=(%.1f, %.1f)",
                          cargo_z_min, occupied_count, forbidden_count, passable_count,
                          bins.size(), obs_z_min, obs_z_max);
    }

    void publishHeightSliceMarkers(const ros::Time& stamp, float cargo_z_min) {
        visualization_msgs::MarkerArray markers;

        // 使用 CUBE_LIST 批量发布禁行区（薄片模式）
        visualization_msgs::Marker cube_list;
        cube_list.header.stamp = stamp;
        cube_list.header.frame_id = map_frame_;
        cube_list.ns = "height_slice";
        cube_list.id = 0;
        cube_list.type = visualization_msgs::Marker::CUBE_LIST;
        cube_list.action = visualization_msgs::Marker::ADD;
        cube_list.pose.orientation.w = 1.0;
        cube_list.scale.x = resolution_;
        cube_list.scale.y = resolution_;
        cube_list.scale.z = 0.05;
        cube_list.color.r = 1.0;
        cube_list.color.g = 0.0;
        cube_list.color.b = 0.0;
        cube_list.color.a = forbidden_overlay_alpha_;

        for (int iy = 0; iy < grid_height_; ++iy) {
            for (int ix = 0; ix < grid_width_; ++ix) {
                int idx = iy * grid_width_ + ix;
                const auto& cell = obstacle_grid_[idx];

                if (isForbiddenForCargo(cell, cargo_z_min)) {
                    geometry_msgs::Point p;
                    p.x = origin_x_ + (ix + 0.5) * resolution_;
                    p.y = origin_y_ + (iy + 0.5) * resolution_;
                    p.z = cargo_z_min;
                    cube_list.points.push_back(p);
                }
            }
        }

        if (!cube_list.points.empty()) {
            markers.markers.push_back(cube_list);
        }

        height_slice_marker_pub_.publish(markers);
    }

    void publishCargoMarkers(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        // 如果没有有效 track，清除所有 marker
        if (!stable_cargo_.valid || cargo_state_ != CargoState::TRACKING) {
            // 删除 raw bbox
            visualization_msgs::Marker delete_raw;
            delete_raw.header.stamp = stamp;
            delete_raw.header.frame_id = map_frame_;
            delete_raw.ns = "cargo_raw_bbox";
            delete_raw.id = 0;
            delete_raw.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(delete_raw);

            // 删除 stable bbox
            visualization_msgs::Marker delete_stable;
            delete_stable.header.stamp = stamp;
            delete_stable.header.frame_id = map_frame_;
            delete_stable.ns = "cargo_stable_bbox";
            delete_stable.id = 0;
            delete_stable.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(delete_stable);

            // 状态文字
            if (show_status_text_) {
                visualization_msgs::Marker text_marker;
                text_marker.header.stamp = stamp;
                text_marker.header.frame_id = map_frame_;
                text_marker.ns = "cargo_status_text";
                text_marker.id = 0;
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::Marker::ADD;
                text_marker.pose.position.z = 1.0;
                text_marker.pose.orientation.w = 1.0;
                text_marker.scale.z = 0.5;
                text_marker.text = "NO CARGO\nstate=" + std::to_string(static_cast<int>(cargo_state_));
                text_marker.color.r = 0.5;
                text_marker.color.g = 0.5;
                text_marker.color.b = 0.5;
                text_marker.color.a = 1.0;
                text_marker.lifetime = ros::Duration(0.5);
                markers.markers.push_back(text_marker);
            }

            cargo_markers_pub_.publish(markers);
            return;
        }

        // 有有效 track
        Eigen::Vector3f center = stable_cargo_.centroid;

        // 1. Raw bbox（紫色线框）
        if (show_raw_bbox_ && cargo_raw_.valid) {
            visualization_msgs::Marker raw_marker;
            raw_marker.header.stamp = stamp;
            raw_marker.header.frame_id = map_frame_;
            raw_marker.ns = "cargo_raw_bbox";
            raw_marker.id = 0;
            raw_marker.type = visualization_msgs::Marker::CUBE;
            raw_marker.action = visualization_msgs::Marker::ADD;
            raw_marker.pose.position.x = (cargo_raw_.bbox_min.x() + cargo_raw_.bbox_max.x()) / 2.0;
            raw_marker.pose.position.y = (cargo_raw_.bbox_min.y() + cargo_raw_.bbox_max.y()) / 2.0;
            raw_marker.pose.position.z = (cargo_raw_.bbox_min.z() + cargo_raw_.bbox_max.z()) / 2.0;
            raw_marker.pose.orientation.w = 1.0;
            raw_marker.scale.x = cargo_raw_.bbox_max.x() - cargo_raw_.bbox_min.x();
            raw_marker.scale.y = cargo_raw_.bbox_max.y() - cargo_raw_.bbox_min.y();
            raw_marker.scale.z = cargo_raw_.bbox_max.z() - cargo_raw_.bbox_min.z();
            raw_marker.color.r = 0.6;
            raw_marker.color.g = 0.2;
            raw_marker.color.b = 0.8;
            raw_marker.color.a = 0.3;
            raw_marker.lifetime = ros::Duration(0.5);
            markers.markers.push_back(raw_marker);
        }

        // 2. Stable bbox（绿色线框，跟随 centroid）
        if (show_stable_bbox_) {
            visualization_msgs::Marker stable_marker;
            stable_marker.header.stamp = stamp;
            stable_marker.header.frame_id = map_frame_;
            stable_marker.ns = "cargo_stable_bbox";
            stable_marker.id = 0;
            stable_marker.type = visualization_msgs::Marker::CUBE;
            stable_marker.action = visualization_msgs::Marker::ADD;
            stable_marker.pose.position.x = center.x();
            stable_marker.pose.position.y = center.y();
            stable_marker.pose.position.z = center.z();
            stable_marker.pose.orientation.w = 1.0;
            stable_marker.scale.x = stable_cargo_.size.x();
            stable_marker.scale.y = stable_cargo_.size.y();
            stable_marker.scale.z = stable_cargo_.size.z();
            stable_marker.color.r = 0.0;
            stable_marker.color.g = 1.0;
            stable_marker.color.b = 0.0;
            stable_marker.color.a = 0.5;
            stable_marker.lifetime = ros::Duration(0.5);
            markers.markers.push_back(stable_marker);
        }

        // 3. 状态文字
        if (show_status_text_) {
            visualization_msgs::Marker text_marker;
            text_marker.header.stamp = stamp;
            text_marker.header.frame_id = map_frame_;
            text_marker.ns = "cargo_status_text";
            text_marker.id = 0;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = center.x();
            text_marker.pose.position.y = center.y();
            text_marker.pose.position.z = center.z() + stable_cargo_.size.z() / 2.0 + 0.5;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.4;

            std::string state_str;
            switch (cargo_state_) {
                case CargoState::NO_CARGO: state_str = "NO_CARGO"; break;
                case CargoState::CANDIDATE: state_str = "CANDIDATE"; break;
                case CargoState::TRACKING: state_str = "TRACKING"; break;
                case CargoState::LOST: state_str = "LOST"; break;
            }

            std::stringstream ss;
            ss << "track=" << current_track_id_ << "\n"
               << "state=" << state_str << "\n"
               << "size=(" << std::fixed << std::setprecision(1)
               << stable_cargo_.size.x() << "," << stable_cargo_.size.y() << "," << stable_cargo_.size.z() << ")\n"
               << "z_min=" << std::fixed << std::setprecision(2) << stable_cargo_.cargo_z_min;

            text_marker.text = ss.str();
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.lifetime = ros::Duration(0.5);
            markers.markers.push_back(text_marker);
        }

        // 日志输出（每秒一次）
        ROS_INFO_THROTTLE(1.0, "[CargoForbiddenZone] track=%d state=%d pts=%d centroid=(%.1f,%.1f,%.1f) size=(%.1f,%.1f,%.1f) z_min=%.2f",
                          current_track_id_, static_cast<int>(cargo_state_),
                          cargo_raw_.point_count,
                          center.x(), center.y(), center.z(),
                          stable_cargo_.size.x(), stable_cargo_.size.y(), stable_cargo_.size.z(),
                          stable_cargo_.cargo_z_min);

        cargo_markers_pub_.publish(markers);
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
