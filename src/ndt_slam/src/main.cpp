#include <ros/ros.h>
#include "ndt_slam/ndt_slam.hpp"

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    ros::init(argc, argv, "ndt_slam");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");  // 私有命名空间，读取节点参数

    // 从 ROS 参数服务器读取配置文件路径
    std::string config_file_path;
    pnh.param<std::string>("config_file", config_file_path,
                          "/home/ydkj/NDT-slam-ws/src/ndt_slam/config/slam_params.yaml");

    // 命令行参数覆盖
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
