#pragma once

#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

namespace lidar_slam2 {

struct Camera {
    Eigen::Vector3d position = Eigen::Vector3d(0, -10, 5);
    Eigen::Vector3d target = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d up = Eigen::Vector3d(0, 0, 1);
    double fov = 45.0;
    double aspect = 16.0 / 9.0;
    double near_plane = 0.1;
    double far_plane = 1000.0;
    double zoom = 1.0;
    
    Eigen::Matrix4d getViewMatrix() const;
    Eigen::Matrix4d getProjectionMatrix() const;
};

struct VisualizerConfig {
    int window_width = 1280;
    int window_height = 720;
    std::string window_title = "LiDAR SLAM Visualizer";
    float point_size = 2.0f;
    float map_point_size = 1.0f;
    Eigen::Vector3f current_cloud_color = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f map_cloud_color = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f trajectory_color = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    bool show_grid = true;
    bool show_axes = true;
};

class Visualizer {
public:
    Visualizer();
    ~Visualizer();
    
    bool initialize();
    
    void startThread();
    void stop();
    void join();
    
    void updateMap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void updateCurrentCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void updatePose(const Sophus::SE3d& pose);
    void addTrajectoryPoint(const Eigen::Vector3d& position);
    
    bool isRunning() const { return running_; }
    
private:
    void mainLoop();
    
    void render();
    void renderPointCloud(const std::vector<Eigen::Vector3f>& points, 
                          const Eigen::Vector3f& color,
                          float point_size);
    void renderTrajectory();
    void renderGrid();
    void renderAxes();
    void renderGUI();
    
    void processInput();
    void updateCamera();
    
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void glfwErrorCallback(int error, const char* description);
    
    Visualizer* getInstancePtr() { return this; }
    static Visualizer* instance_ptr_;
    
    GLFWwindow* window_ = nullptr;
    VisualizerConfig config_;
    Camera camera_;
    
    std::vector<Eigen::Vector3f> map_points_;
    std::vector<Eigen::Vector3f> current_cloud_points_;
    std::vector<Eigen::Vector3d> trajectory_;
    
    std::mutex map_mutex_;
    std::mutex cloud_mutex_;
    std::mutex trajectory_mutex_;
    
    std::thread thread_;
    std::atomic<bool> running_{false};
    
    bool mouse_pressed_ = false;
    bool right_mouse_pressed_ = false;
    double last_mouse_x_ = 0.0;
    double last_mouse_y_ = 0.0;
    double camera_yaw_ = 0.0;
    double camera_pitch_ = -20.0;
    
    int frame_count_ = 0;
    double fps_ = 0.0;
    double last_fps_time_ = 0.0;
};

} // namespace lidar_slam2
