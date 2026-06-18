#include "lidar_slam2/Visualizer.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lidar_slam2 {

Visualizer* Visualizer::instance_ptr_ = nullptr;

Eigen::Matrix4d Camera::getViewMatrix() const {
    Eigen::Vector3d f = (target - position).normalized();
    Eigen::Vector3d s = f.cross(up).normalized();
    Eigen::Vector3d u = s.cross(f);
    
    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();
    view(0, 0) = s.x(); view(0, 1) = s.y(); view(0, 2) = s.z();
    view(1, 0) = u.x(); view(1, 1) = u.y(); view(1, 2) = u.z();
    view(2, 0) = -f.x(); view(2, 1) = -f.y(); view(2, 2) = -f.z();
    view(0, 3) = -s.dot(position);
    view(1, 3) = -u.dot(position);
    view(2, 3) = f.dot(position);
    
    return view;
}

Eigen::Matrix4d Camera::getProjectionMatrix() const {
    double f = 1.0 / std::tan(fov * M_PI / 360.0);
    double aspect_ratio = aspect * zoom;
    
    Eigen::Matrix4d proj = Eigen::Matrix4d::Zero();
    proj(0, 0) = f / aspect_ratio;
    proj(1, 1) = f;
    proj(2, 2) = (far_plane + near_plane) / (near_plane - far_plane);
    proj(2, 3) = 2.0 * far_plane * near_plane / (near_plane - far_plane);
    proj(3, 2) = -1.0;
    
    return proj;
}

void Visualizer::glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

Visualizer::Visualizer() {
    instance_ptr_ = this;
}

Visualizer::~Visualizer() {
    stop();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (window_) {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

bool Visualizer::initialize() {
    glfwSetErrorCallback(glfwErrorCallback);
    
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // ====================== 最终正确配置 ======================
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_FALSE);

    const char* glsl_version = "#version 330 core";
    // ==========================================================

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    
    window_ = glfwCreateWindow(
        config_.window_width,
        config_.window_height,
        config_.window_title.c_str(),
        nullptr,
        nullptr
    );
    
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    
    glfwSetFramebufferSizeCallback(window_, framebufferSizeCallback);
    glfwSetKeyCallback(window_, keyCallback);
    glfwSetMouseButtonCallback(window_, mouseButtonCallback);
    glfwSetCursorPosCallback(window_, cursorPosCallback);
    glfwSetScrollCallback(window_, scrollCallback);
    
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "GLSL Version: " << glsl_version << std::endl;
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << std::endl;
    
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPointSize(config_.point_size);
    
    return true;
}

void Visualizer::startThread() {
    if (!initialize()) {
        throw std::runtime_error("Visualizer initialization failed");
    }
    running_ = true;
    thread_ = std::thread(&Visualizer::mainLoop, this);
}

void Visualizer::stop() {
    running_ = false;
}

void Visualizer::join() {
    if (thread_.joinable()) thread_.join();
}

void Visualizer::mainLoop() {
    last_fps_time_ = glfwGetTime();
    
    while (running_ && !glfwWindowShouldClose(window_)) {
        double current_time = glfwGetTime();
        processInput();
        updateCamera();
        render();
        
        glfwSwapBuffers(window_);
        glfwPollEvents();
        
        frame_count_++;
        if (current_time - last_fps_time_ >= 1.0) {
            fps_ = frame_count_ / (current_time - last_fps_time_);
            frame_count_ = 0;
            last_fps_time_ = current_time;
        }
    }
    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
}

void Visualizer::render() {
    glfwPollEvents();
    
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    glViewport(0, 0, width, height);
    
    Eigen::Matrix4d view = camera_.getViewMatrix();
    Eigen::Matrix4d projection = camera_.getProjectionMatrix();
    
    if (config_.show_grid) renderGrid();
    if (config_.show_axes) renderAxes();
    
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!map_points_.empty())
            renderPointCloud(map_points_, config_.map_cloud_color, config_.map_point_size);
    }
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (!current_cloud_points_.empty())
            renderPointCloud(current_cloud_points_, config_.current_cloud_color, config_.point_size);
    }
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (!trajectory_.empty()) renderTrajectory();
    }
    
    renderGUI();
    
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Visualizer::renderPointCloud(const std::vector<Eigen::Vector3f>& points,
                                   const Eigen::Vector3f& color, float point_size) {
    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(color.x(), color.y(), color.z());
    for (const auto& point : points) glVertex3f(point.x(), point.y(), point.z());
    glEnd();
}

void Visualizer::renderTrajectory() {
    if (trajectory_.size() < 2) return;
    glBegin(GL_LINE_STRIP);
    glColor3f(config_.trajectory_color.x(), config_.trajectory_color.y(), config_.trajectory_color.z());
    for (const auto& point : trajectory_) glVertex3d(point.x(), point.y(), point.z());
    glEnd();
}

void Visualizer::renderGrid() {
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_LINES);
    double grid_size = 50.0, grid_step = 1.0;
    for (double x = -grid_size; x <= grid_size; x += grid_step) {
        glVertex3d(x, -grid_size, 0.0); glVertex3d(x, grid_size, 0.0);
    }
    for (double y = -grid_size; y <= grid_size; y += grid_step) {
        glVertex3d(-grid_size, y, 0.0); glVertex3d(grid_size, y, 0.0);
    }
    glEnd();
}

void Visualizer::renderAxes() {
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3d(0,0,0); glVertex3d(1,0,0);
    glColor3f(0,1,0); glVertex3d(0,0,0); glVertex3d(0,1,0);
    glColor3f(0,0,1); glVertex3d(0,0,0); glVertex3d(0,0,1);
    glEnd();
    glLineWidth(1.0f);
}

void Visualizer::processInput() {
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) running_ = false;
}

void Visualizer::updateCamera() {
    double radius = 10.0 / camera_.zoom;
    double theta = camera_yaw_ * M_PI / 180.0;
    double phi = (90.0 + camera_pitch_) * M_PI / 180.0;
    camera_.position.x() = camera_.target.x() + radius * sin(phi) * cos(theta);
    camera_.position.y() = camera_.target.y() + radius * sin(phi) * sin(theta);
    camera_.position.z() = camera_.target.z() + radius * cos(phi);
}

void Visualizer::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void Visualizer::keyCallback(GLFWwindow* window, int key, int scanc, int act, int mods) {
    if (!instance_ptr_) return;
    if (key == GLFW_KEY_W && act == GLFW_PRESS) instance_ptr_->camera_.target.y() -= 0.5;
    if (key == GLFW_KEY_S && act == GLFW_PRESS) instance_ptr_->camera_.target.y() += 0.5;
    if (key == GLFW_KEY_A && act == GLFW_PRESS) instance_ptr_->camera_.target.x() -= 0.5;
    if (key == GLFW_KEY_D && act == GLFW_PRESS) instance_ptr_->camera_.target.x() += 0.5;
}

void Visualizer::mouseButtonCallback(GLFWwindow* window, int b, int a, int m) {
    if (!instance_ptr_) return;
    if (b == GLFW_MOUSE_BUTTON_LEFT) instance_ptr_->mouse_pressed_ = (a == GLFW_PRESS);
    if (b == GLFW_MOUSE_BUTTON_RIGHT) instance_ptr_->right_mouse_pressed_ = (a == GLFW_PRESS);
    if (instance_ptr_->mouse_pressed_ || instance_ptr_->right_mouse_pressed_)
        glfwGetCursorPos(window, &instance_ptr_->last_mouse_x_, &instance_ptr_->last_mouse_y_);
}

void Visualizer::cursorPosCallback(GLFWwindow* window, double x, double y) {
    if (!instance_ptr_) return;
    if (instance_ptr_->mouse_pressed_) {
        double dx = x - instance_ptr_->last_mouse_x_, dy = y - instance_ptr_->last_mouse_y_;
        instance_ptr_->camera_yaw_ += dx * 0.5;
        instance_ptr_->camera_pitch_ += dy * 0.5;
        instance_ptr_->camera_pitch_ = std::clamp(instance_ptr_->camera_pitch_, -89.0, 89.0);
        instance_ptr_->last_mouse_x_ = x;
        instance_ptr_->last_mouse_y_ = y;
    } else if (instance_ptr_->right_mouse_pressed_) {
        double dx = x - instance_ptr_->last_mouse_x_, dy = y - instance_ptr_->last_mouse_y_;
        instance_ptr_->camera_.target.x() -= dx * 0.05;
        instance_ptr_->camera_.target.y() += dy * 0.05;
        instance_ptr_->last_mouse_x_ = x;
        instance_ptr_->last_mouse_y_ = y;
    }
}

void Visualizer::scrollCallback(GLFWwindow* window, double x, double y) {
    if (!instance_ptr_) return;
    instance_ptr_->camera_.zoom += y * 0.1;
    instance_ptr_->camera_.zoom = std::clamp(instance_ptr_->camera_.zoom, 0.1, 5.0);
}

void Visualizer::updateMap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_points_.clear();
    if (!cloud) return;
    for (const auto& p : cloud->points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
            map_points_.emplace_back(p.x, p.y, p.z);
}

void Visualizer::updateCurrentCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    current_cloud_points_.clear();
    if (!cloud) return;
    for (const auto& p : cloud->points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
            current_cloud_points_.emplace_back(p.x, p.y, p.z);
}

void Visualizer::updatePose(const Sophus::SE3d& pose) {
    addTrajectoryPoint(pose.translation());
}

void Visualizer::addTrajectoryPoint(const Eigen::Vector3d& pos) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_.push_back(pos);
}

void Visualizer::renderGUI() {
    ImGui::Begin("SLAM Status");
    ImGui::Text("FPS: %.1f", fps_);
    ImGui::Separator();
    { std::lock_guard<std::mutex> _(map_mutex_); ImGui::Text("Map Points: %ld", map_points_.size()); }
    { std::lock_guard<std::mutex> _(cloud_mutex_); ImGui::Text("Current Cloud: %ld", current_cloud_points_.size()); }
    { std::lock_guard<std::mutex> _(trajectory_mutex_); ImGui::Text("Trajectory Points: %ld", trajectory_.size()); }
    ImGui::Separator();
    ImGui::Text("Camera: %.2f %.2f %.2f", camera_.position.x(), camera_.position.y(), camera_.position.z());
    ImGui::Text("Target: %.2f %.2f %.2f", camera_.target.x(), camera_.target.y(), camera_.target.z());
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Display Options")) {
        ImGui::Checkbox("Grid", &config_.show_grid);
        ImGui::Checkbox("Axes", &config_.show_axes);
        ImGui::SliderFloat("Point Size", &config_.point_size, 1,10);
        ImGui::SliderFloat("Map Point Size", &config_.map_point_size, 0.5f,5);
        ImGui::ColorEdit3("Current Color", config_.current_cloud_color.data());
        ImGui::ColorEdit3("Map Color", config_.map_cloud_color.data());
        ImGui::ColorEdit3("Traj Color", config_.trajectory_color.data());
    }
    ImGui::End();
}

} // namespace lidar_slam2