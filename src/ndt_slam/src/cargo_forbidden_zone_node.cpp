/**
 * cargo_forbidden_zone_node.cpp
 *
 * 吊货禁行区节点
 * - 加载 tiles_objects 生成 2.5D 障碍物栅格
 * - 订阅 /payload_track_info 获取吊货位置和尺寸
 * - 三档尺寸策略 + 高度过滤 + 矩形膨胀
 * - 轨迹预测和风险评估
 * - 发布禁行区和风险等级
 */

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

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

// 风险等级
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

// 吊货信息
struct CargoInfo {
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

    // 货物尺寸
    bool use_detected_bbox_ = true;
    double default_length_x_ = 4.0;
    double default_width_y_ = 1.5;
    double default_height_z_ = 1.0;
    double min_valid_length_x_ = 1.0;
    double min_valid_width_y_ = 0.5;
    double max_valid_length_x_ = 12.0;
    double max_valid_width_y_ = 4.0;
    double min_safety_length_x_ = 1.5;
    double min_safety_width_y_ = 0.8;
    double size_change_threshold_ = 0.30;
    int shrink_confirm_frames_ = 10;

    // 高度过滤
    bool height_filter_enabled_ = true;
    double z_clearance_ = 0.30;

    // 膨胀
    double safety_margin_x_ = 0.50;
    double safety_margin_y_ = 0.50;
    double buffer_margin_extra_ = 0.50;

    // 运动门控
    bool only_warn_when_xy_moving_ = true;
    double moving_speed_threshold_ = 0.03;
    int idle_confirm_frames_ = 3;
    int moving_confirm_frames_ = 2;

    // 速度滤波
    double velocity_filter_alpha_ = 0.3;
    double min_predict_speed_ = 0.3;
    double max_predict_speed_ = 1.5;

    // 预测
    double horizon_sec_ = 3.0;
    double dt_ = 0.2;

    // 风险阈值
    double stop_ttc_ = 1.0;
    double slow_ttc_ = 2.0;
    double warn_ttc_ = 3.0;

    // 2.5D 栅格
    int grid_width_ = 0;
    int grid_height_ = 0;
    float origin_x_ = 0.0f;
    float origin_y_ = 0.0f;
    std::vector<Cell2_5D> base_obstacle_grid_;
    std::vector<bool> forbidden_grid_;
    int occupied_cell_count_ = 0;

    // 吊货信息
    CargoInfo cargo_;
    double used_length_x_ = 0.0;
    double used_width_y_ = 0.0;
    double used_cargo_z_min_ = 0.0;
    int shrink_counter_ = 0;
    bool need_rebuild_forbidden_grid_ = false;

    // 速度滤波
    double filtered_vx_ = 0.0;
    double filtered_vy_ = 0.0;
    int idle_frame_count_ = 0;
    int moving_frame_count_ = 0;
    bool is_idle_ = true;

    // 风险等级
    RiskLevel risk_level_ = RiskLevel::UNKNOWN;

    // 发布者
    ros::Publisher forbidden_grid_pub_;
    ros::Publisher risk_level_pub_;
    ros::Publisher predicted_path_pub_;
    ros::Publisher markers_pub_;

    // 订阅者
    ros::Subscriber payload_track_sub_;

    // 时间
    ros::Time last_update_time_;

    void loadConfig() {
        std::string config_file;
        pnh_.param<std::string>("config_file", config_file, "");
        if (config_file.empty()) {
            ROS_WARN("[CargoForbiddenZone] No config file specified, using defaults");
            return;
        }

        try {
            YAML::Node config = YAML::LoadFile(config_file);
            auto cfz = config["cargo_forbidden_zone"];

            map_frame_ = cfz["map_frame"].as<std::string>("map");

            auto input = cfz["input"];
            objects_tiles_dir_ = input["objects_tiles_dir"].as<std::string>("");

            auto grid = cfz["grid"];
            resolution_ = grid["resolution"].as<double>(0.10);
            min_points_per_cell_ = grid["min_points_per_cell"].as<int>(2);

            auto cs = cfz["cargo_size"];
            use_detected_bbox_ = cs["use_detected_bbox"].as<bool>(true);
            default_length_x_ = cs["default_length_x"].as<double>(4.0);
            default_width_y_ = cs["default_width_y"].as<double>(1.5);
            default_height_z_ = cs["default_height_z"].as<double>(1.0);
            min_valid_length_x_ = cs["min_valid_length_x"].as<double>(1.0);
            min_valid_width_y_ = cs["min_valid_width_y"].as<double>(0.5);
            max_valid_length_x_ = cs["max_valid_length_x"].as<double>(12.0);
            max_valid_width_y_ = cs["max_valid_width_y"].as<double>(4.0);
            min_safety_length_x_ = cs["min_safety_length_x"].as<double>(1.5);
            min_safety_width_y_ = cs["min_safety_width_y"].as<double>(0.8);
            size_change_threshold_ = cs["size_change_threshold"].as<double>(0.30);
            shrink_confirm_frames_ = cs["shrink_confirm_frames"].as<int>(10);

            auto hf = cfz["height_filter"];
            height_filter_enabled_ = hf["enabled"].as<bool>(true);
            z_clearance_ = hf["z_clearance"].as<double>(0.30);

            auto inf = cfz["inflation"];
            safety_margin_x_ = inf["safety_margin_x"].as<double>(0.50);
            safety_margin_y_ = inf["safety_margin_y"].as<double>(0.50);
            buffer_margin_extra_ = inf["buffer_margin_extra"].as<double>(0.50);

            auto mg = cfz["motion_gate"];
            only_warn_when_xy_moving_ = mg["only_warn_when_xy_moving"].as<bool>(true);
            moving_speed_threshold_ = mg["moving_speed_threshold"].as<double>(0.03);
            idle_confirm_frames_ = mg["idle_confirm_frames"].as<int>(3);
            moving_confirm_frames_ = mg["moving_confirm_frames"].as<int>(2);

            auto vel = cfz["velocity"];
            velocity_filter_alpha_ = vel["velocity_filter_alpha"].as<double>(0.3);
            min_predict_speed_ = vel["min_predict_speed"].as<double>(0.3);
            max_predict_speed_ = vel["max_predict_speed"].as<double>(1.5);

            auto pred = cfz["prediction"];
            horizon_sec_ = pred["horizon_sec"].as<double>(3.0);
            dt_ = pred["dt"].as<double>(0.2);

            auto risk = cfz["risk"];
            stop_ttc_ = risk["stop_ttc"].as<double>(1.0);
            slow_ttc_ = risk["slow_ttc"].as<double>(2.0);
            warn_ttc_ = risk["warn_ttc"].as<double>(3.0);

            ROS_INFO("[CargoForbiddenZone] Config loaded from %s", config_file.c_str());
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

        // 计算栅格范围
        if (all_points->empty()) {
            ROS_ERROR("[CargoForbiddenZone] No points loaded!");
            return;
        }

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
        base_obstacle_grid_.resize(grid_width_ * grid_height_);

        for (const auto& p : all_points->points) {
            int ix = static_cast<int>((p.x - origin_x_) / resolution_);
            int iy = static_cast<int>((p.y - origin_y_) / resolution_);

            if (ix < 0 || ix >= grid_width_ || iy < 0 || iy >= grid_height_) continue;

            int idx = iy * grid_width_ + ix;
            auto& cell = base_obstacle_grid_[idx];
            cell.occupied = true;
            cell.z_min = std::min(cell.z_min, p.z);
            cell.z_max = std::max(cell.z_max, p.z);
            cell.point_count++;
        }

        // 过滤点数不足的 cell
        occupied_cell_count_ = 0;
        for (auto& cell : base_obstacle_grid_) {
            if (cell.point_count < min_points_per_cell_) {
                cell.occupied = false;
            }
            if (cell.occupied) occupied_cell_count_++;
        }

        ROS_INFO("[CargoForbiddenZone] Occupied cells: %d", occupied_cell_count_);
    }

    void buildBaseObstacleGrid() {
        forbidden_grid_.resize(grid_width_ * grid_height_, false);
        need_rebuild_forbidden_grid_ = true;
    }

    void setupPublishers() {
        forbidden_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/cargo_forbidden_grid", 1, true);
        risk_level_pub_ = nh_.advertise<std_msgs::Int32>("/cargo_collision_warning", 10);
        predicted_path_pub_ = nh_.advertise<nav_msgs::Path>("/cargo_predicted_path", 10);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_forbidden_markers", 10);
    }

    void setupSubscribers() {
        payload_track_sub_ = nh_.subscribe("/payload_track_info", 10,
                                           &CargoForbiddenZoneNode::payloadTrackCallback, this);
    }

    void payloadTrackCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 19) return;

        cargo_.valid = (msg->data[1] >= 0);  // track_id >= 0
        if (!cargo_.valid) return;

        cargo_.track_id = static_cast<int>(msg->data[1]);
        cargo_.state = static_cast<int>(msg->data[2]);
        cargo_.centroid = Eigen::Vector3f(msg->data[3], msg->data[4], msg->data[5]);
        cargo_.velocity = Eigen::Vector3f(msg->data[6], msg->data[7], msg->data[8]);
        cargo_.bbox_min = Eigen::Vector3f(msg->data[9], msg->data[10], msg->data[11]);
        cargo_.bbox_max = Eigen::Vector3f(msg->data[12], msg->data[13], msg->data[14]);
        cargo_.point_count = static_cast<int>(msg->data[15]);
        cargo_.track_duration = msg->data[16];
        cargo_.direction_consistency = msg->data[17];
        cargo_.map_displacement = msg->data[18];
    }

    void update() {
        updateCargoSize();
        updateMotionState();
        updateRiskLevel();
        publishResults();
    }

    void updateCargoSize() {
        double new_length = default_length_x_;
        double new_width = default_width_y_;

        if (cargo_.valid && use_detected_bbox_) {
            double bbox_length = cargo_.bbox_max.x() - cargo_.bbox_min.x();
            double bbox_width = cargo_.bbox_max.y() - cargo_.bbox_min.y();

            bool bbox_valid =
                cargo_.state == 2 &&  // DYNAMIC
                cargo_.point_count >= 80 &&
                cargo_.track_duration >= 0.5 &&
                bbox_length >= min_valid_length_x_ &&
                bbox_width >= min_valid_width_y_ &&
                bbox_length <= max_valid_length_x_ &&
                bbox_width <= max_valid_width_y_;

            if (bbox_valid) {
                new_length = std::max(bbox_length, min_safety_length_x_);
                new_width = std::max(bbox_width, min_safety_width_y_);
            }
        }

        // 尺寸变化稳定策略
        bool bigger = (new_length > used_length_x_ + size_change_threshold_) ||
                      (new_width > used_width_y_ + size_change_threshold_);
        bool smaller = (new_length < used_length_x_ - size_change_threshold_) ||
                       (new_width < used_width_y_ - size_change_threshold_);

        if (bigger) {
            used_length_x_ = std::max(new_length, used_length_x_);
            used_width_y_ = std::max(new_width, used_width_y_);
            shrink_counter_ = 0;
            need_rebuild_forbidden_grid_ = true;
        } else if (smaller) {
            shrink_counter_++;
            if (shrink_counter_ >= shrink_confirm_frames_) {
                used_length_x_ = new_length;
                used_width_y_ = new_width;
                shrink_counter_ = 0;
                need_rebuild_forbidden_grid_ = true;
            }
        }

        // 更新 cargo_z_min
        if (cargo_.valid) {
            used_cargo_z_min_ = cargo_.bbox_min.z();
        } else {
            used_cargo_z_min_ = cargo_.centroid.z() - default_height_z_ / 2.0;
        }
    }

    void updateMotionState() {
        if (!cargo_.valid) {
            is_idle_ = true;
            idle_frame_count_ = 0;
            moving_frame_count_ = 0;
            return;
        }

        double speed_xy = std::sqrt(cargo_.velocity.x() * cargo_.velocity.x() +
                                     cargo_.velocity.y() * cargo_.velocity.y());

        // 速度滤波
        filtered_vx_ = velocity_filter_alpha_ * cargo_.velocity.x() +
                       (1.0 - velocity_filter_alpha_) * filtered_vx_;
        filtered_vy_ = velocity_filter_alpha_ * cargo_.velocity.y() +
                       (1.0 - velocity_filter_alpha_) * filtered_vy_;

        double filtered_speed = std::sqrt(filtered_vx_ * filtered_vx_ + filtered_vy_ * filtered_vy_);

        if (filtered_speed < moving_speed_threshold_) {
            idle_frame_count_++;
            moving_frame_count_ = 0;
            if (idle_frame_count_ >= idle_confirm_frames_) {
                is_idle_ = true;
            }
        } else {
            moving_frame_count_++;
            idle_frame_count_ = 0;
            if (moving_frame_count_ >= moving_confirm_frames_) {
                is_idle_ = false;
            }
        }
    }

    void updateRiskLevel() {
        if (!cargo_.valid) {
            risk_level_ = RiskLevel::UNKNOWN;
            return;
        }

        if (is_idle_) {
            risk_level_ = RiskLevel::IDLE;
            return;
        }

        // 重建禁行区
        if (need_rebuild_forbidden_grid_) {
            rebuildForbiddenGrid();
            need_rebuild_forbidden_grid_ = false;
        }

        // 预测轨迹
        double predict_speed = std::sqrt(filtered_vx_ * filtered_vx_ + filtered_vy_ * filtered_vy_);
        predict_speed = std::max(predict_speed, min_predict_speed_);
        predict_speed = std::min(predict_speed, max_predict_speed_);

        double dir_x = filtered_vx_ / (predict_speed + 1e-6);
        double dir_y = filtered_vy_ / (predict_speed + 1e-6);

        double min_ttc = std::numeric_limits<double>::max();

        for (double t = 0.0; t <= horizon_sec_; t += dt_) {
            double pred_x = cargo_.centroid.x() + dir_x * predict_speed * t;
            double pred_y = cargo_.centroid.y() + dir_y * predict_speed * t;

            if (isForbidden(pred_x, pred_y)) {
                min_ttc = t;
                break;
            }
        }

        if (min_ttc < stop_ttc_) {
            risk_level_ = RiskLevel::STOP;
        } else if (min_ttc < slow_ttc_) {
            risk_level_ = RiskLevel::SLOW_DOWN;
        } else if (min_ttc < warn_ttc_) {
            risk_level_ = RiskLevel::WARNING;
        } else {
            risk_level_ = RiskLevel::NORMAL;
        }
    }

    void rebuildForbiddenGrid() {
        forbidden_grid_.assign(grid_width_ * grid_height_, false);

        double inflate_x = used_length_x_ / 2.0 + safety_margin_x_;
        double inflate_y = used_width_y_ / 2.0 + safety_margin_y_;
        int rx = static_cast<int>(std::ceil(inflate_x / resolution_));
        int ry = static_cast<int>(std::ceil(inflate_y / resolution_));

        for (int iy = 0; iy < grid_height_; ++iy) {
            for (int ix = 0; ix < grid_width_; ++ix) {
                int idx = iy * grid_width_ + ix;
                const auto& cell = base_obstacle_grid_[idx];

                if (!cell.occupied) continue;

                // 高度过滤
                if (height_filter_enabled_ && cargo_.valid) {
                    if (cell.z_max + z_clearance_ < used_cargo_z_min_) {
                        continue;  // 吊货能从上方掠过
                    }
                }

                // 矩形膨胀
                int min_ix = std::max(0, ix - rx);
                int max_ix = std::min(grid_width_ - 1, ix + rx);
                int min_iy = std::max(0, iy - ry);
                int max_iy = std::min(grid_height_ - 1, iy + ry);

                for (int fy = min_iy; fy <= max_iy; ++fy) {
                    for (int fx = min_ix; fx <= max_ix; ++fx) {
                        forbidden_grid_[fy * grid_width_ + fx] = true;
                    }
                }
            }
        }

        ROS_DEBUG("[CargoForbiddenZone] Forbidden grid rebuilt: length=%.1f, width=%.1f",
                  used_length_x_, used_width_y_);
    }

    bool isForbidden(double x, double y) const {
        int ix = static_cast<int>((x - origin_x_) / resolution_);
        int iy = static_cast<int>((y - origin_y_) / resolution_);

        if (ix < 0 || ix >= grid_width_ || iy < 0 || iy >= grid_height_) {
            return false;
        }

        return forbidden_grid_[iy * grid_width_ + ix];
    }

    void publishResults() {
        ros::Time now = ros::Time::now();

        // 发布风险等级
        std_msgs::Int32 risk_msg;
        risk_msg.data = static_cast<int>(risk_level_);
        risk_level_pub_.publish(risk_msg);

        // 发布 OccupancyGrid（禁行区栅格）
        publishOccupancyGrid(now);

        // 发布预测轨迹
        if (cargo_.valid && !is_idle_) {
            nav_msgs::Path path_msg;
            path_msg.header.stamp = now;
            path_msg.header.frame_id = map_frame_;

            double predict_speed = std::sqrt(filtered_vx_ * filtered_vx_ + filtered_vy_ * filtered_vy_);
            predict_speed = std::max(predict_speed, min_predict_speed_);
            predict_speed = std::min(predict_speed, max_predict_speed_);

            double dir_x = filtered_vx_ / (predict_speed + 1e-6);
            double dir_y = filtered_vy_ / (predict_speed + 1e-6);

            for (double t = 0.0; t <= horizon_sec_; t += dt_) {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = now;
                pose.header.frame_id = map_frame_;
                pose.pose.position.x = cargo_.centroid.x() + dir_x * predict_speed * t;
                pose.pose.position.y = cargo_.centroid.y() + dir_y * predict_speed * t;
                pose.pose.position.z = cargo_.centroid.z();
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }

            predicted_path_pub_.publish(path_msg);
        }

        // 发布可视化标记
        publishMarkers(now);
    }

    void publishMarkers(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        // 吊货 BBox
        if (cargo_.valid) {
            visualization_msgs::Marker bbox_marker;
            bbox_marker.header.stamp = stamp;
            bbox_marker.header.frame_id = map_frame_;
            bbox_marker.ns = "cargo_bbox";
            bbox_marker.id = 0;
            bbox_marker.type = visualization_msgs::Marker::CUBE;
            bbox_marker.action = visualization_msgs::Marker::ADD;
            bbox_marker.pose.position.x = (cargo_.bbox_min.x() + cargo_.bbox_max.x()) / 2.0;
            bbox_marker.pose.position.y = (cargo_.bbox_min.y() + cargo_.bbox_max.y()) / 2.0;
            bbox_marker.pose.position.z = (cargo_.bbox_min.z() + cargo_.bbox_max.z()) / 2.0;
            bbox_marker.pose.orientation.w = 1.0;
            bbox_marker.scale.x = cargo_.bbox_max.x() - cargo_.bbox_min.x();
            bbox_marker.scale.y = cargo_.bbox_max.y() - cargo_.bbox_min.y();
            bbox_marker.scale.z = cargo_.bbox_max.z() - cargo_.bbox_min.z();
            bbox_marker.color.r = 0.6;
            bbox_marker.color.g = 0.2;
            bbox_marker.color.b = 0.8;
            bbox_marker.color.a = 0.5;
            markers.markers.push_back(bbox_marker);
        }

        // 风险等级文字
        visualization_msgs::Marker text_marker;
        text_marker.header.stamp = stamp;
        text_marker.header.frame_id = map_frame_;
        text_marker.ns = "risk_text";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = cargo_.valid ? cargo_.centroid.x() : 0.0;
        text_marker.pose.position.y = cargo_.valid ? cargo_.centroid.y() : 0.0;
        text_marker.pose.position.z = cargo_.valid ? cargo_.centroid.z() + 1.0 : 1.0;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.5;

        switch (risk_level_) {
            case RiskLevel::IDLE: text_marker.text = "IDLE"; text_marker.color.g = 1.0; break;
            case RiskLevel::NORMAL: text_marker.text = "NORMAL"; text_marker.color.g = 1.0; break;
            case RiskLevel::WARNING: text_marker.text = "WARNING"; text_marker.color.r = 1.0; text_marker.color.g = 1.0; break;
            case RiskLevel::SLOW_DOWN: text_marker.text = "SLOW_DOWN"; text_marker.color.r = 1.0; text_marker.color.g = 0.5; break;
            case RiskLevel::STOP: text_marker.text = "STOP"; text_marker.color.r = 1.0; break;
            case RiskLevel::UNKNOWN: text_marker.text = "UNKNOWN"; text_marker.color.r = 0.5; text_marker.color.g = 0.5; text_marker.color.b = 0.5; break;
        }
        text_marker.color.a = 1.0;
        markers.markers.push_back(text_marker);

        markers_pub_.publish(markers);
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
                if (forbidden_grid_[idx]) {
                    grid_msg.data[idx] = 100;  // 禁行区
                } else if (base_obstacle_grid_[idx].occupied) {
                    grid_msg.data[idx] = 50;   // 障碍物但不在禁行区
                } else {
                    grid_msg.data[idx] = 0;    // 空闲
                }
            }
        }

        forbidden_grid_pub_.publish(grid_msg);
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
