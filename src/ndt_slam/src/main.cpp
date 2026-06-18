#include <ros/ros.h>
#include "ndt_slam/ndt_slam.hpp"

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    ros::init(argc, argv, "ndt_slam");
    ros::NodeHandle nh;

    std::string config_file_path = "/home/ydkj/NDT-slam-ws/src/ndt_slam/config/dual_lidar_slam_params.yaml";
    if (argc > 1) {
        config_file_path = argv[1];
    }

    ROS_INFO("Starting NDT SLAM System");
    ROS_INFO("Using config file: %s", config_file_path.c_str());

    ndt_slam::NdtSlamNode slam_node(config_file_path, nh);

    ROS_INFO("NDT SLAM System started, spinning...");

    ros::spin();

    return 0;
}
