#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <XmlRpcValue.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// A bounded, consume-once synchronizer for the warehouse's one- or two-lidar
// configuration.  Point-cloud conversion, transforms, merging and filtering
// deliberately run outside queue_mutex_; the critical section only enqueues or
// selects small shared_ptr records.
class PointCloudMerger {
public:
    PointCloudMerger() : nh_("~") {
        nh_.param<std::string>("output_topic", output_topic_, "/merged_points");
        nh_.param<std::string>("output_frame", output_frame_, "base_link");
        nh_.param<std::string>("diagnostics_topic", diagnostics_topic_,
                               "/pointcloud_merger/diagnostics");
        nh_.param<bool>("transform_to_base", transform_to_base_, false);
        nh_.param<bool>("use_voxel_filter", use_voxel_filter_, true);
        nh_.param<double>("voxel_size", voxel_size_, 0.15);
        nh_.param<double>("sync/max_pair_dt_sec", max_pair_dt_sec_, 0.060);
        nh_.param<double>("sync/stale_timeout_sec", stale_timeout_sec_, 0.120);
        nh_.param<double>("sync/timer_period_sec", timer_period_sec_, 0.010);
        nh_.param<int>("sync/max_queue_size", max_queue_size_, 8);
        nh_.param<int>("sync/subscriber_queue_size", subscriber_queue_size_, 10);
        nh_.param<double>("sync/diagnostic_log_period_sec", diagnostic_log_period_sec_, 2.0);

        max_pair_dt_sec_ = std::max(0.0, max_pair_dt_sec_);
        stale_timeout_sec_ = std::max(0.001, stale_timeout_sec_);
        timer_period_sec_ = std::max(0.001, timer_period_sec_);
        max_queue_size_ = std::max(1, max_queue_size_);
        subscriber_queue_size_ = std::max(1, subscriber_queue_size_);
        diagnostic_log_period_sec_ = std::max(0.1, diagnostic_log_period_sec_);

        XmlRpc::XmlRpcValue lidars;
        if (!nh_.getParam("lidars", lidars) ||
            lidars.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            lidars.size() < 1 || lidars.size() > 2) {
            throw std::runtime_error(
                "PointCloudMerger requires a 'lidars' array containing one or two entries");
        }

        ROS_INFO("========================================");
        ROS_INFO("PointCloudMerger consume-once synchronizer:");
        ROS_INFO("  output=%s frame=%s diagnostics=%s",
                 output_topic_.c_str(), output_frame_.c_str(), diagnostics_topic_.c_str());
        ROS_INFO("  transform_to_base=%s voxel=%s leaf=%.3fm",
                 transform_to_base_ ? "true" : "false",
                 use_voxel_filter_ ? "ON" : "OFF", voxel_size_);
        ROS_INFO("  max_pair_dt=%.1fms stale_timeout=%.1fms timer=%.1fms queue=%d",
                 max_pair_dt_sec_ * 1000.0, stale_timeout_sec_ * 1000.0,
                 timer_period_sec_ * 1000.0, max_queue_size_);

        for (int i = 0; i < lidars.size(); ++i) {
            XmlRpc::XmlRpcValue& lidar_cfg = lidars[i];
            if (lidar_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                !lidar_cfg.hasMember("name") || !lidar_cfg.hasMember("topic")) {
                throw std::runtime_error("Each lidar entry must contain string 'name' and 'topic'");
            }

            const std::string name = static_cast<std::string>(lidar_cfg["name"]);
            const std::string topic = static_cast<std::string>(lidar_cfg["topic"]);
            if (lidars_.count(name) != 0U) {
                throw std::runtime_error("Duplicate lidar name: " + name);
            }

            LidarInfo lidar;
            lidar.name = name;
            lidar.topic = topic;

            if (lidar_cfg.hasMember("Lidar2BaseExtrinsic")) {
                const XmlRpc::XmlRpcValue extrinsic = lidar_cfg["Lidar2BaseExtrinsic"];
                if (extrinsic.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                    extrinsic.size() != 16) {
                    throw std::runtime_error(name + ": Lidar2BaseExtrinsic must contain 16 values");
                }
                for (int row = 0; row < 4; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        const XmlRpc::XmlRpcValue& value = extrinsic[row * 4 + col];
                        if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                            lidar.T_lidar2base(row, col) = static_cast<double>(value);
                        } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                            lidar.T_lidar2base(row, col) = static_cast<int>(value);
                        } else {
                            throw std::runtime_error(name + ": extrinsic entries must be numeric");
                        }
                    }
                }
            } else if (transform_to_base_) {
                ROS_WARN("[%s] transform_to_base=true but no extrinsic was supplied; using identity",
                         name.c_str());
            }

            lidar_names_.push_back(name);
            lidars_.emplace(name, std::move(lidar));
            ROS_INFO("  [%s] topic=%s extrinsic=%s", name.c_str(), topic.c_str(),
                     lidar_cfg.hasMember("Lidar2BaseExtrinsic") ? "loaded" : "identity");
        }
        ROS_INFO("========================================");

        // Construct subscriptions only after all map entries are stable.  The
        // extrinsics are immutable after construction and may be read lock-free.
        for (const std::string& name : lidar_names_) {
            LidarInfo& lidar = lidars_.at(name);
            lidar.sub = nh_.subscribe<sensor_msgs::PointCloud2>(
                lidar.topic, subscriber_queue_size_,
                [this, name](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                    pointCloudCallback(msg, name);
                });
        }

        pub_merged_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        pub_diagnostics_ = nh_.advertise<std_msgs::String>(diagnostics_topic_, 10);
        timer_ = nh_.createTimer(ros::Duration(timer_period_sec_),
                                 &PointCloudMerger::mergeAndPublish, this);
        ROS_INFO("PointCloudMerger ready (%zu lidar%s)", lidar_names_.size(),
                 lidar_names_.size() == 1U ? "" : "s");
    }

private:
    using SteadyClock = std::chrono::steady_clock;

    struct CloudFrame {
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
        ros::Time stamp;
        SteadyClock::time_point received_at;
        std::uint64_t sequence = 0;  // Per-lidar, monotonic, process-local ID.
        std::uint32_t ros_sequence = 0;
    };

    struct LidarInfo {
        std::string name;
        std::string topic;
        Eigen::Matrix4d T_lidar2base = Eigen::Matrix4d::Identity();
        std::deque<CloudFrame> queue;
        std::uint64_t next_sequence = 0;
        ros::Subscriber sub;
    };

    struct SelectedFrame {
        std::string lidar_name;
        CloudFrame frame;
    };

    struct PublishWork {
        std::vector<SelectedFrame> frames;
        std::string mode;
        double pair_dt_sec = -1.0;
        double oldest_age_sec = 0.0;
    };

    static double secondsSince(const SteadyClock::time_point& from,
                               const SteadyClock::time_point& to) {
        return std::chrono::duration_cast<std::chrono::duration<double>>(to - from).count();
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg,
                            const std::string& name) {
        const auto lidar_it = lidars_.find(name);
        if (lidar_it == lidars_.end()) {
            return;
        }

        // Conversion and the rigid transform are intentionally outside the
        // synchronization lock.  The corresponding extrinsic is immutable.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) {
            ++empty_drop_count_;
            ROS_WARN_THROTTLE(2.0, "[MergerSync] dropped empty cloud from %s", name.c_str());
            return;
        }

        if (transform_to_base_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(
                new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *transformed, lidar_it->second.T_lidar2base);
            cloud = transformed;
        }

        CloudFrame frame;
        frame.cloud = cloud;
        frame.stamp = msg->header.stamp;
        frame.ros_sequence = msg->header.seq;
        frame.received_at = SteadyClock::now();

        std::lock_guard<std::mutex> lock(queue_mutex_);
        LidarInfo& lidar = lidars_.at(name);
        frame.sequence = ++lidar.next_sequence;

        if (static_cast<int>(lidar.queue.size()) >= max_queue_size_) {
            const CloudFrame& dropped = lidar.queue.front();
            ROS_WARN_THROTTLE(1.0,
                              "[MergerSync] queue overflow lidar=%s drop_seq=%llu",
                              name.c_str(),
                              static_cast<unsigned long long>(dropped.sequence));
            lidar.queue.pop_front();
            ++overflow_drop_count_;
        }

        // Normal ROS sensor streams are ordered.  Preserve correctness for bag
        // seeks or reordered transport by keeping this very small queue sorted.
        const auto insert_at = std::upper_bound(
            lidar.queue.begin(), lidar.queue.end(), frame.stamp,
            [](const ros::Time& stamp, const CloudFrame& queued) {
                return stamp < queued.stamp;
            });
        lidar.queue.insert(insert_at, std::move(frame));
    }

    bool selectPublishWork(PublishWork& work) {
        const SteadyClock::time_point now = SteadyClock::now();
        std::lock_guard<std::mutex> lock(queue_mutex_);

        if (lidar_names_.size() == 1U) {
            LidarInfo& lidar = lidars_.at(lidar_names_.front());
            if (lidar.queue.empty()) {
                return false;
            }
            work.mode = "single_config";
            work.oldest_age_sec = secondsSince(lidar.queue.front().received_at, now);
            work.frames.push_back(SelectedFrame{lidar.name, lidar.queue.front()});
            lidar.queue.pop_front();
            return true;
        }

        LidarInfo& lidar_a = lidars_.at(lidar_names_[0]);
        LidarInfo& lidar_b = lidars_.at(lidar_names_[1]);

        if (!lidar_a.queue.empty() && !lidar_b.queue.empty()) {
            double best_dt = std::numeric_limits<double>::infinity();
            std::size_t best_a = 0U;
            std::size_t best_b = 0U;

            // Bounded O(queue^2) search (default 8x8) chooses the actual closest
            // timestamp pair instead of whichever callbacks happened to be latest.
            for (std::size_t i = 0; i < lidar_a.queue.size(); ++i) {
                for (std::size_t j = 0; j < lidar_b.queue.size(); ++j) {
                    const double dt = std::abs(
                        (lidar_a.queue[i].stamp - lidar_b.queue[j].stamp).toSec());
                    if (dt < best_dt) {
                        best_dt = dt;
                        best_a = i;
                        best_b = j;
                    }
                }
            }

            if (best_dt <= max_pair_dt_sec_) {
                work.mode = "paired";
                work.pair_dt_sec = best_dt;
                work.oldest_age_sec = std::max(
                    secondsSince(lidar_a.queue[best_a].received_at, now),
                    secondsSince(lidar_b.queue[best_b].received_at, now));
                work.frames.push_back(SelectedFrame{lidar_a.name, lidar_a.queue[best_a]});
                work.frames.push_back(SelectedFrame{lidar_b.name, lidar_b.queue[best_b]});

                // Earlier unselected frames can no longer be published in stamp
                // order.  Drop them, then consume both selected frames exactly once.
                superseded_drop_count_ += best_a + best_b;
                lidar_a.queue.erase(lidar_a.queue.begin(),
                                    lidar_a.queue.begin() + static_cast<std::ptrdiff_t>(best_a + 1U));
                lidar_b.queue.erase(lidar_b.queue.begin(),
                                    lidar_b.queue.begin() + static_cast<std::ptrdiff_t>(best_b + 1U));
                return true;
            }
        }

        // No valid pair exists yet.  A frame may leave as a true one-lidar
        // fallback only after waiting for its counterpart for stale_timeout.
        LidarInfo* fallback_lidar = nullptr;
        if (!lidar_a.queue.empty() && !lidar_b.queue.empty()) {
            fallback_lidar = lidar_a.queue.front().received_at <= lidar_b.queue.front().received_at
                                 ? &lidar_a
                                 : &lidar_b;
        } else if (!lidar_a.queue.empty()) {
            fallback_lidar = &lidar_a;
        } else if (!lidar_b.queue.empty()) {
            fallback_lidar = &lidar_b;
        }

        if (fallback_lidar == nullptr) {
            return false;
        }

        const double age_sec = secondsSince(fallback_lidar->queue.front().received_at, now);
        if (age_sec < stale_timeout_sec_) {
            return false;
        }

        work.mode = "single_timeout";
        work.oldest_age_sec = age_sec;
        work.frames.push_back(
            SelectedFrame{fallback_lidar->name, fallback_lidar->queue.front()});
        fallback_lidar->queue.pop_front();
        ++fallback_count_;
        return true;
    }

    void mergeAndPublish(const ros::TimerEvent&) {
        PublishWork work;
        if (!selectPublishWork(work)) {
            return;
        }

        // Everything below is intentionally lock-free with respect to callbacks.
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
        ros::Time latest_stamp;
        std::size_t input_points[2] = {0U, 0U};
        for (std::size_t i = 0; i < work.frames.size(); ++i) {
            const SelectedFrame& selected = work.frames[i];
            if (!selected.frame.cloud || selected.frame.cloud->empty()) {
                continue;
            }
            input_points[i] = selected.frame.cloud->size();
            *merged += *selected.frame.cloud;
            latest_stamp = std::max(latest_stamp, selected.frame.stamp);
        }

        if (merged->empty()) {
            ROS_WARN_THROTTLE(2.0, "[MergerSync] selected work contained no points");
            return;
        }

        const std::size_t raw_points = merged->size();
        if (use_voxel_filter_ && merged->size() > 100U) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(merged);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
                new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.filter(*filtered);
            merged = filtered;
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*merged, output);
        output.header.stamp = latest_stamp;
        output.header.frame_id = output_frame_;
        pub_merged_.publish(output);

        if (work.mode == "paired") {
            ++paired_count_;
        }
        ++published_count_;

        std::ostringstream diag;
        diag << std::fixed << std::setprecision(3)
             << "mode=" << work.mode
             << " lidar_a=" << work.frames[0].lidar_name
             << " seq_a=" << work.frames[0].frame.sequence
             << " ros_seq_a=" << work.frames[0].frame.ros_sequence;
        if (work.frames.size() == 2U) {
            diag << " lidar_b=" << work.frames[1].lidar_name
                 << " seq_b=" << work.frames[1].frame.sequence
                 << " ros_seq_b=" << work.frames[1].frame.ros_sequence;
        } else {
            diag << " lidar_b=none seq_b=0 ros_seq_b=0";
        }
        diag << " pair_dt_ms=" << (work.pair_dt_sec < 0.0 ? -1.0 : work.pair_dt_sec * 1000.0)
             << " age_ms=" << work.oldest_age_sec * 1000.0
             << " reuse=0"
             << " points_a=" << input_points[0]
             << " points_b=" << input_points[1]
             << " raw_points=" << raw_points
             << " output_points=" << merged->size()
             << " paired_total=" << paired_count_.load()
             << " fallback_total=" << fallback_count_.load()
             << " queue_drop_total=" << overflow_drop_count_.load()
             << " superseded_drop_total=" << superseded_drop_count_.load()
             << " empty_drop_total=" << empty_drop_count_.load();

        std_msgs::String diag_msg;
        diag_msg.data = diag.str();
        pub_diagnostics_.publish(diag_msg);
        ROS_DEBUG_STREAM("[MergerSync] " << diag_msg.data);
        ROS_INFO_STREAM_THROTTLE(diagnostic_log_period_sec_,
                                 "[MergerSync] " << diag_msg.data);
        if (work.mode == "single_timeout") {
            ROS_WARN_STREAM_THROTTLE(
                1.0, "[MergerSync] counterpart timeout; published true single cloud: "
                         << diag_msg.data);
        }
    }

    ros::NodeHandle nh_;
    std::map<std::string, LidarInfo> lidars_;
    std::vector<std::string> lidar_names_;
    ros::Publisher pub_merged_;
    ros::Publisher pub_diagnostics_;
    ros::Timer timer_;
    std::mutex queue_mutex_;

    std::string output_topic_;
    std::string output_frame_;
    std::string diagnostics_topic_;
    bool transform_to_base_ = false;
    bool use_voxel_filter_ = true;
    double voxel_size_ = 0.15;
    double max_pair_dt_sec_ = 0.060;
    double stale_timeout_sec_ = 0.120;
    double timer_period_sec_ = 0.010;
    int max_queue_size_ = 8;
    int subscriber_queue_size_ = 10;
    double diagnostic_log_period_sec_ = 2.0;

    std::atomic<std::uint64_t> published_count_{0};
    std::atomic<std::uint64_t> paired_count_{0};
    std::atomic<std::uint64_t> fallback_count_{0};
    std::atomic<std::uint64_t> overflow_drop_count_{0};
    std::atomic<std::uint64_t> superseded_drop_count_{0};
    std::atomic<std::uint64_t> empty_drop_count_{0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_merger");
    try {
        PointCloudMerger merger;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("PointCloudMerger startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
