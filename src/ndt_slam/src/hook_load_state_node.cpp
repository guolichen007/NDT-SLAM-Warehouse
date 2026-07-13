#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <yaml-cpp/yaml.h>

#include <lidar_slam2_msgs/HookLoadState.h>
#include "ndt_slam/hook_load_state_filter.hpp"

namespace ndt_slam {

class HookLoadStateNode {
public:
    HookLoadStateNode() : nh_(), pnh_("~"), filter_() {
        pnh_.param<std::string>("config_file", config_file_, std::string());
        if (!config_file_.empty()) {
            try {
                hook_config_ = YAML::LoadFile(config_file_)["hook_load_signal"];
            } catch (const YAML::Exception& error) {
                ROS_ERROR("[HookLoadState] config load failed: %s",
                          error.what());
            }
        }
        if (!hook_config_ || !hook_config_.IsMap()) {
            hook_config_ = YAML::Node(YAML::NodeType::Map);
        }
        filter_.setConfig(readConfig());

        input_topic_ = hook_config_["topic"].as<std::string>("/gravity");
        output_topic_ =
            hook_config_["state_topic"].as<std::string>("/hook/load_state");
        pnh_.param<std::string>("input_topic", input_topic_, input_topic_);
        pnh_.param<std::string>("output_topic", output_topic_, output_topic_);
        double publish_hz = hook_config_["publish_hz"].as<double>(10.0);
        pnh_.param("publish_hz", publish_hz, publish_hz);
        if (!std::isfinite(publish_hz) || publish_hz <= 0.0) publish_hz = 10.0;

        state_pub_ = nh_.advertise<lidar_slam2_msgs::HookLoadState>(
            output_topic_, 1, true);
        voltage_sub_ = nh_.subscribe(
            input_topic_, 1, &HookLoadStateNode::voltageCallback, this);
        timer_ = nh_.createWallTimer(
            ros::WallDuration(1.0 / publish_hz),
            &HookLoadStateNode::timerCallback, this);
        publish(filter_.tick(ros::WallTime::now().toSec()), ros::Time::now());
        const HookLoadStateConfig& active = filter_.config();
        ROS_INFO("[GRAVITY] topic=%s type=std_msgs/Float32 "
                 "thresholds=(%.2f,%.2f) hysteresis=%.2f confirm=%u",
                 input_topic_.c_str(), active.low_threshold_v,
                 active.high_threshold_v, active.hysteresis_v,
                 active.confirm_samples);
        ROS_INFO("[HookLoadState] output=%s publish=%.1fHz",
                 output_topic_.c_str(), publish_hz);
    }

private:
    HookLoadStateConfig readConfig() {
        HookLoadStateConfig config;
        config.low_threshold_v =
            hook_config_["low_threshold_v"].as<double>(1.90);
        config.high_threshold_v =
            hook_config_["high_threshold_v"].as<double>(2.10);
        config.hysteresis_v =
            hook_config_["hysteresis_v"].as<double>(0.03);
        int confirm_samples =
            hook_config_["confirm_samples"].as<int>(3);
        config.stale_timeout_sec =
            hook_config_["stale_timeout_sec"].as<double>(0.50);
        config.valid_voltage_min_v =
            hook_config_["valid_voltage_min_v"].as<double>(0.0);
        config.valid_voltage_max_v =
            hook_config_["valid_voltage_max_v"].as<double>(5.0);
        pnh_.param("low_threshold_v", config.low_threshold_v,
                   config.low_threshold_v);
        pnh_.param("high_threshold_v", config.high_threshold_v,
                   config.high_threshold_v);
        pnh_.param("hysteresis_v", config.hysteresis_v,
                   config.hysteresis_v);
        pnh_.param("confirm_samples", confirm_samples, confirm_samples);
        config.confirm_samples = static_cast<std::uint32_t>(
            std::max(1, confirm_samples));
        pnh_.param("stale_timeout_sec", config.stale_timeout_sec,
                   config.stale_timeout_sec);
        pnh_.param("valid_voltage_min_v", config.valid_voltage_min_v,
                   config.valid_voltage_min_v);
        pnh_.param("valid_voltage_max_v", config.valid_voltage_max_v,
                   config.valid_voltage_max_v);
        return config;
    }

    void voltageCallback(const std_msgs::Float32::ConstPtr& message) {
        last_source_stamp_ = ros::Time::now();
        publish(filter_.ingest(message->data, ros::WallTime::now().toSec()),
                last_source_stamp_);
    }

    void timerCallback(const ros::WallTimerEvent& event) {
        publish(filter_.tick(event.current_real.toSec()), last_source_stamp_);
    }

    void publish(const HookLoadStateResult& result, const ros::Time& stamp) {
        lidar_slam2_msgs::HookLoadState message;
        message.header.stamp = stamp;
        message.schema_version = lidar_slam2_msgs::HookLoadState::SCHEMA_VERSION;
        message.valid = result.valid;
        message.fresh = result.fresh;
        message.state = static_cast<std::uint8_t>(result.state);
        message.voltage = result.voltage;
        message.stable_samples = result.stable_samples;
        message.reason = result.reason;
        state_pub_.publish(message);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber voltage_sub_;
    ros::Publisher state_pub_;
    ros::WallTimer timer_;
    HookLoadStateFilter filter_;
    std::string input_topic_;
    std::string output_topic_;
    std::string config_file_;
    YAML::Node hook_config_;
    ros::Time last_source_stamp_;
};

}  // namespace ndt_slam

int main(int argc, char** argv) {
    ros::init(argc, argv, "hook_load_state");
    ndt_slam::HookLoadStateNode node;
    ros::spin();
    return 0;
}
