cmake_minimum_required(VERSION 3.8)
project(ndt_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  message_filters
  lidar_slam2_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(g2o REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_slam_lib
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs tf tf2 tf2_ros pcl_ros pcl_conversions message_filters lidar_slam2_msgs
)

include(3rdparty/tbb/tbb.cmake)
include(3rdparty/tsl_robin/tsl_robin.cmake)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(GLFW_LIBRARY "/usr/lib/x86_64-linux-gnu/libglfw.so")
set(GLFW_LIBRARIES ${GLFW_LIBRARY})

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/imgui)
set(IMGUI_BACKENDS_DIR ${IMGUI_DIR}/backends)
set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_BACKENDS_DIR}/imgui_impl_glfw.cpp
  ${IMGUI_BACKENDS_DIR}/imgui_impl_opengl3.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${lidar_slam2_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIR}
  /home/ydkj/.local/include
  ${OpenCV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${IMGUI_DIR}
  ${IMGUI_BACKENDS_DIR}
  ${ndt_omp_INCLUDE_DIRS}
  include/kiss_icp
  include/kiss_icp/core
  include/kiss_icp/metrics
  include/kiss_icp/pipeline
)

# ========== 新的源文件结构（7 cpp + 6 hpp） ==========
set(LIBRARY_SOURCES
  # 主节点
  src/ndt_slam.cpp
  # 里程计
  src/odometry.cpp
  # 建图
  src/keyframe_manager.cpp
  # 闭环检测
  src/loop_closure.cpp
  # 点云处理
  src/point_cloud_processing.cpp
  # 通道吊货候选过滤
  src/base_payload_channel_filter.cpp
  # 吊货跟踪管理
  src/payload_tracker.cpp
  # 人体动态过滤
  src/human_object_filter.cpp
  # 动态事件管理
  src/dynamic_event_manager.cpp
  # 可视化
  src/visualizer.cpp
  # KISS-ICP（保留）
  src/kiss_icp/core/Preprocessing.cpp
  src/kiss_icp/core/Registration.cpp
  src/kiss_icp/core/Threshold.cpp
  src/kiss_icp/core/VoxelHashMap.cpp
  src/kiss_icp/core/VoxelUtils.cpp
  src/kiss_icp/metrics/Metrics.cpp
  src/kiss_icp/pipeline/KissICP.cpp
)

add_library(ndt_slam_lib SHARED
  ${LIBRARY_SOURCES}
  ${IMGUI_SOURCES}
)

target_link_libraries(ndt_slam_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Sophus_LIBRARIES}
  yaml-cpp
  g2o::core g2o::stuff g2o::types_slam3d g2o::solver_eigen
  TBB::tbb
  robin_map
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLFW_LIBRARIES}
  ${ndt_omp_LIBRARIES}
  Boost::filesystem
  pthread
  dl
  usb-1.0
)

# 主节点可执行文件
add_executable(ndt_slam_node src/main.cpp)
target_link_libraries(ndt_slam_node
  ndt_slam_lib
  ${catkin_LIBRARIES}
  g2o::core g2o::stuff g2o::types_slam3d g2o::solver_eigen
  pthread
  dl
)

# 双雷达点云合并（独立）
add_executable(pointcloud_merger src/PointCloudMerger.cpp)
target_link_libraries(pointcloud_merger
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 点云诊断工具（独立）
add_executable(cloud_diagnostics src/CloudDiagnostics.cpp)
target_link_libraries(cloud_diagnostics
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS ndt_slam_lib ndt_slam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
