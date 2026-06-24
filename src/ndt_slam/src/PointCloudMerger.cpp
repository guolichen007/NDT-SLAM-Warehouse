#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <vector>
#include <XmlRpcValue.h>

class PointCloudMerger {
public:
    PointCloudMerger() : nh_("~") {
        nh_.param<std::string>("output_topic", output_topic_, "/merged_points");
        nh_.param<std::string>("output_frame", output_frame_, "rslidar");
        nh_.param<bool>("transform_to_base", transform_to_base_, false);
        nh_.param<bool>("use_voxel_filter", use_voxel_filter_, true);
        nh_.param<double>("voxel_size", voxel_size_, 0.15);

        // 读取雷达配置
        XmlRpc::XmlRpcValue lidars;
        nh_.getParam("lidars", lidars);

        ROS_ASSERT(lidars.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_INFO("========================================");
        ROS_INFO("PointCloudMerger config:");
        ROS_INFO("  Output: %s (frame: %s)", output_topic_.c_str(), output_frame_.c_str());
        ROS_INFO("  Transform to base: %s", transform_to_base_ ? "true" : "false");
        ROS_INFO("  Voxel filter: %s (size: %.3f m)", use_voxel_filter_ ? "ON" : "OFF", voxel_size_);
        ROS_INFO("  Lidars: %d", lidars.size());
        ROS_INFO("========================================");

        for (int i = 0; i < lidars.size(); i++) {
            XmlRpc::XmlRpcValue& lidar_cfg = lidars[i];
            ROS_ASSERT(lidar_cfg.getType() == XmlRpc::XmlRpcValue::TypeStruct);

            std::string name = static_cast<std::string>(lidar_cfg["name"]);
            std::string topic = static_cast<std::string>(lidar_cfg["topic"]);

            LidarInfo lidar;
            lidar.name = name;
            lidar.topic = topic;
            lidar.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            lidar.received = false;

            // 读取外参
            if (lidar_cfg.hasMember("Lidar2BaseExtrinsic")) {
                XmlRpc::XmlRpcValue extrinsic = lidar_cfg["Lidar2BaseExtrinsic"];
                if (extrinsic.getType() == XmlRpc::XmlRpcValue::TypeArray && extrinsic.size() == 16) {
                    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                    for (int r = 0; r < 4; r++) {
                        for (int c = 0; c < 4; c++) {
                            XmlRpc::XmlRpcValue& val = extrinsic[r * 4 + c];
                            if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                                T(r, c) = static_cast<double>(val);
                            } else if (val.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                                T(r, c) = static_cast<int>(val);
                            }
                        }
                    }
                    lidar.T_lidar2base = T;
                    ROS_INFO("  [%s] topic: %s (extrinsic loaded)", name.c_str(), topic.c_str());
                }
            } else {
                ROS_INFO("  [%s] topic: %s", name.c_str(), topic.c_str());
            }

            // 用 lambda 订阅
            std::string captured_name = name;
            lidar.sub = nh_.subscribe<sensor_msgs::PointCloud2>(
                topic, 10,
                [this, captured_name](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                    this->pointCloudCallback(msg, captured_name);
                });

            lidars_[name] = lidar;
        }

        pub_merged_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        timer_ = nh_.createTimer(ros::Duration(0.1), &PointCloudMerger::mergeAndPublish, this);

        ROS_INFO("PointCloudMerger ready");
    }

private:
    struct LidarInfo {
        std::string name;
        std::string topic;
        Eigen::Matrix4d T_lidar2base = Eigen::Matrix4d::Identity();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        ros::Time stamp;
        bool received;
        ros::Subscriber sub;
    };

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg,
                            const std::string& name) {
        auto it = lidars_.find(name);
        if (it == lidars_.end()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (transform_to_base_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *transformed, it->second.T_lidar2base);
            cloud = transformed;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        it->second.cloud = cloud;
        it->second.stamp = msg->header.stamp;
        it->second.received = true;
    }

    void mergeAndPublish(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
        ros::Time latest_stamp;
        bool has_data = false;
        int lidar_count = 0;
        std::string lidar_detail;

        for (auto& pair : lidars_) {
            auto& lidar = pair.second;
            if (!lidar.received || !lidar.cloud || lidar.cloud->empty()) {
                ROS_WARN_THROTTLE(5.0, "[%s] no data received yet", pair.first.c_str());
                continue;
            }
            lidar_count++;

            if (!lidar_detail.empty()) lidar_detail += ", ";
            lidar_detail += pair.first + ":" + std::to_string(lidar.cloud->size());

            *merged += *lidar.cloud;

            if (!has_data || lidar.stamp > latest_stamp) {
                latest_stamp = lidar.stamp;
                has_data = true;
            }
        }

        if (!has_data || merged->empty()) return;

        merge_count_++;
        bool should_log = (merge_count_ % 10 == 1);  // 每10帧输出一次

        // 体素滤波去重（关键！解决双雷达重叠区域重复点问题）
        if (use_voxel_filter_ && merged->size() > 100) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(merged);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.filter(*filtered);
            if (should_log) {
                ROS_DEBUG("[Merger] lidars=%d {%s}, raw=%lu -> %lu (dup=%lu)",
                         lidar_count, lidar_detail.c_str(),
                         merged->size(), filtered->size(),
                         merged->size() - filtered->size());
            }
            merged = filtered;
        } else if (should_log) {
            ROS_DEBUG("[Merger] lidars=%d {%s}, points=%lu",
                     lidar_count, lidar_detail.c_str(), merged->size());
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*merged, output);
        output.header.stamp = latest_stamp;
        output.header.frame_id = output_frame_;
        pub_merged_.publish(output);

        ROS_DEBUG("Published merged cloud: %lu points, frame=%s, stamp=%.3f",
                  merged->size(), output_frame_.c_str(), latest_stamp.toSec());
    }

    ros::NodeHandle nh_;

    std::map<std::string, LidarInfo> lidars_;
    ros::Publisher pub_merged_;
    ros::Timer timer_;
    std::mutex mutex_;

    std::string output_topic_;
    std::string output_frame_;
    bool transform_to_base_;
    bool use_voxel_filter_;
    double voxel_size_;
    int merge_count_ = 0;  // 合并计数器，用于控制日志频率
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_merger");
    PointCloudMerger merger;
    ros::spin();
    return 0;
}
