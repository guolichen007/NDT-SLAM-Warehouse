cmake_minimum_required(VERSION 3.8)
project(ndt_slam)

set(NDT_SLAM_GIT_SHA "unknown")
set(NDT_SLAM_GIT_BRANCH "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE NDT_SLAM_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE NDT_SLAM_GIT_SHA_RESULT
    ERROR_QUIET)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE NDT_SLAM_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE NDT_SLAM_GIT_BRANCH_RESULT
    ERROR_QUIET)
  if(NOT NDT_SLAM_GIT_SHA_RESULT EQUAL 0 OR NDT_SLAM_GIT_SHA STREQUAL "")
    set(NDT_SLAM_GIT_SHA "unknown")
  endif()
  if(NOT NDT_SLAM_GIT_BRANCH_RESULT EQUAL 0 OR NDT_SLAM_GIT_BRANCH STREQUAL "")
    set(NDT_SLAM_GIT_BRANCH "unknown")
  endif()
endif()
configure_file(
  cmake/build_info.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/ndt_slam/build_info.hpp
  @ONLY)

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
find_package(yaml-cpp CONFIG REQUIRED
  PATHS /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp
  NO_DEFAULT_PATH)

# yaml-cpp target naming differs across distributions:
# Ubuntu 20.04 exports `yaml-cpp`, while newer releases may export
# `yaml-cpp::yaml-cpp`.
if(TARGET yaml-cpp::yaml-cpp)
  set(NDT_SLAM_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(NDT_SLAM_YAML_CPP_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "No usable yaml-cpp CMake target was exported")
endif()
find_package(g2o REQUIRED)
find_package(fmt CONFIG REQUIRED)
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

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/generated
  ${catkin_INCLUDE_DIRS}
  ${lidar_slam2_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${ndt_omp_INCLUDE_DIRS}
)

# ========== 源文件 ==========
set(LIBRARY_SOURCES
  # 主节点
  src/ndt_slam.cpp
  # 关键帧管理
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
  # P0.5: 货物框估计器
  src/cargo_box_estimator.cpp
  # v8-stable-r3: CraneMotionEKF
  src/crane_motion_ekf.cpp
  src/ndt_fitness_circuit_breaker.cpp
  src/rail_localization_authority.cpp
  src/registration_target_snapshot.cpp
  src/fixed_yaw_translation_solver.cpp
  src/rail_translation_pose_graph.cpp
  src/stationary_motion_policy.cpp
  src/registration_cloud_builder.cpp
  src/ndt_observability.cpp
  src/crane_pose_constraint.cpp
  src/rigid_transform_conversion.cpp
  # Asynchronous local/global NDT relocalization
  src/ndt_relocalizer.cpp
  src/relocalization_confirmation_policy.cpp
  # Runtime diagnostics (1.0x/1.5x acceptance testing)
  src/runtime_diagnostics.cpp
  # Formal cargo bottom and structured safety-status pipeline
  src/cargo_bottom_fusion.cpp
  src/cargo_vertical_evidence.cpp
  src/clean_map_builder.cpp
  src/cargo_oriented_footprint.cpp
  src/cargo_track_policy.cpp
  src/cargo_component_fusion.cpp
  src/cargo_rigid_geometry.cpp
  src/cargo_marker_lifecycle.cpp
  src/obstacle_perception.cpp
  src/hazard_evaluator.cpp
  src/avoidance_decision.cpp
  src/anomaly_review_episode_tracker.cpp
  src/cargo_safety_evaluator.cpp
  src/cargo_safety_temporal_filter.cpp
  src/cargo_obstacle_tracker.cpp
  src/static_obstacle_evidence_index.cpp
  src/static_height_field.cpp
  src/static_height_component_extractor.cpp
  src/cargo_lift_origin_binder.cpp
  src/cargo_physical_identity_authority.cpp
  src/integrated_cargo_identity_shadow.cpp
  src/revealed_support_observer.cpp
  src/cargo_preload_baseline_tracker.cpp
  src/cargo_geometry_fusion.cpp
  src/cargo_capability.cpp
  src/cargo_subsystem.cpp
  src/cargo_frame_decision.cpp
  src/cargo_avoidance_fusion.cpp
  src/pending_static_hazard_tracker.cpp
  src/cargo_presence_state_machine.cpp
  src/cargo_physical_motion_estimator.cpp
  src/pending_cargo_envelope.cpp
  src/pending_cargo_self_evidence.cpp
  src/cargo_swing_monitor.cpp
  src/map_session_snapshot.cpp
  src/persistent_registration_loader.cpp
  src/cargo_motion_corridor.cpp
  src/cargo_residual_classifier.cpp
  src/hook_load_state_filter.cpp
  src/hook_load_evidence_policy.cpp
  src/pending_origin_binding_policy.cpp
)

add_library(ndt_slam_lib SHARED
  ${LIBRARY_SOURCES}
)
add_dependencies(ndt_slam_lib ${catkin_EXPORTED_TARGETS})

target_link_libraries(ndt_slam_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${NDT_SLAM_YAML_CPP_TARGET}
  g2o::core g2o::stuff g2o::types_slam3d g2o::solver_eigen
  fmt::fmt
  TBB::tbb
  robin_map
  ${OpenCV_LIBRARIES}
  ${ndt_omp_LIBRARIES}
  Boost::filesystem
  pthread
  dl
)

# 主节点可执行文件
add_executable(ndt_slam_node src/main.cpp)
add_dependencies(ndt_slam_node ${catkin_EXPORTED_TARGETS})
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

# 吊货禁行区节点
add_executable(cargo_forbidden_zone_node src/cargo_forbidden_zone_node.cpp)
target_link_libraries(cargo_forbidden_zone_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${NDT_SLAM_YAML_CPP_TARGET}
)

# Independent wall-clock status heartbeat. This is the sole publisher of the
# simple 14/17/18/30-35 status-code interface.
add_executable(cargo_alarm_heartbeat_node src/cargo_alarm_heartbeat_node.cpp)
add_dependencies(cargo_alarm_heartbeat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cargo_alarm_heartbeat_node ${catkin_LIBRARIES})

add_executable(hook_load_state_node src/hook_load_state_node.cpp)
add_dependencies(hook_load_state_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(hook_load_state_node
  ndt_slam_lib
  ${catkin_LIBRARIES}
  ${NDT_SLAM_YAML_CPP_TARGET}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(time_epoch_contract_test
    test/time_epoch_contract_test.cpp)
  if(TARGET time_epoch_contract_test)
    target_link_libraries(time_epoch_contract_test
      ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(keyframe_manager_time_epoch_test
    test/keyframe_manager_time_epoch_test.cpp)
  if(TARGET keyframe_manager_time_epoch_test)
    target_link_libraries(keyframe_manager_time_epoch_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_bottom_fusion_test
    test/cargo_bottom_fusion_test.cpp)
  if(TARGET cargo_bottom_fusion_test)
    target_link_libraries(cargo_bottom_fusion_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_vertical_evidence_test
    test/cargo_vertical_evidence_test.cpp)
  if(TARGET cargo_vertical_evidence_test)
    target_link_libraries(cargo_vertical_evidence_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_marker_lifecycle_test
    test/cargo_marker_lifecycle_test.cpp)
  if(TARGET cargo_marker_lifecycle_test)
    target_link_libraries(cargo_marker_lifecycle_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_oriented_footprint_test
    test/cargo_oriented_footprint_test.cpp)
  if(TARGET cargo_oriented_footprint_test)
    target_link_libraries(cargo_oriented_footprint_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_rigid_geometry_test
    test/cargo_rigid_geometry_test.cpp)
  if(TARGET cargo_rigid_geometry_test)
    target_link_libraries(cargo_rigid_geometry_test
      ndt_slam_lib
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      gtest_main)
  endif()

  catkin_add_gtest(cargo_track_policy_test
    test/cargo_track_policy_test.cpp)
  if(TARGET cargo_track_policy_test)
    target_link_libraries(cargo_track_policy_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_component_fusion_test
    test/cargo_component_fusion_test.cpp)
  if(TARGET cargo_component_fusion_test)
    target_link_libraries(cargo_component_fusion_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(clean_map_builder_test
    test/clean_map_builder_test.cpp)
  if(TARGET clean_map_builder_test)
    target_link_libraries(clean_map_builder_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_safety_evaluator_test
    test/cargo_safety_evaluator_test.cpp)
  if(TARGET cargo_safety_evaluator_test)
    target_link_libraries(cargo_safety_evaluator_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(anomaly_review_episode_tracker_test
    test/anomaly_review_episode_tracker_test.cpp)
  if(TARGET anomaly_review_episode_tracker_test)
    target_link_libraries(anomaly_review_episode_tracker_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(hazard_evaluator_test
    test/hazard_evaluator_test.cpp)
  if(TARGET hazard_evaluator_test)
    target_link_libraries(hazard_evaluator_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_safety_temporal_filter_test
    test/cargo_safety_temporal_filter_test.cpp)
  if(TARGET cargo_safety_temporal_filter_test)
    target_link_libraries(cargo_safety_temporal_filter_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_obstacle_tracker_test
    test/cargo_obstacle_tracker_test.cpp)
  if(TARGET cargo_obstacle_tracker_test)
    target_link_libraries(cargo_obstacle_tracker_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(static_obstacle_evidence_index_test
    test/static_obstacle_evidence_index_test.cpp)
  if(TARGET static_obstacle_evidence_index_test)
    target_link_libraries(static_obstacle_evidence_index_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(static_height_field_test
    test/static_height_field_test.cpp)
  if(TARGET static_height_field_test)
    target_link_libraries(static_height_field_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(static_height_component_extractor_test
    test/static_height_component_extractor_test.cpp)
  if(TARGET static_height_component_extractor_test)
    target_link_libraries(static_height_component_extractor_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_lift_origin_binder_test
    test/cargo_lift_origin_binder_test.cpp)
  if(TARGET cargo_lift_origin_binder_test)
    target_link_libraries(cargo_lift_origin_binder_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_physical_identity_authority_test
    test/cargo_physical_identity_authority_test.cpp)
  if(TARGET cargo_physical_identity_authority_test)
    target_link_libraries(cargo_physical_identity_authority_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(integrated_cargo_identity_shadow_test
    test/integrated_cargo_identity_shadow_test.cpp)
  if(TARGET integrated_cargo_identity_shadow_test)
    target_link_libraries(integrated_cargo_identity_shadow_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_geometry_fusion_test
    test/cargo_geometry_fusion_test.cpp)
  if(TARGET cargo_geometry_fusion_test)
    target_link_libraries(cargo_geometry_fusion_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_capability_test
    test/cargo_capability_test.cpp)
  if(TARGET cargo_capability_test)
    target_link_libraries(cargo_capability_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_frame_decision_test
    test/cargo_frame_decision_test.cpp)
  if(TARGET cargo_frame_decision_test)
    target_link_libraries(cargo_frame_decision_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_preload_baseline_tracker_test
    test/cargo_preload_baseline_tracker_test.cpp)
  if(TARGET cargo_preload_baseline_tracker_test)
    target_link_libraries(cargo_preload_baseline_tracker_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_subsystem_test
    test/cargo_subsystem_test.cpp)
  if(TARGET cargo_subsystem_test)
    target_link_libraries(cargo_subsystem_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_avoidance_fusion_test
    test/cargo_avoidance_fusion_test.cpp)
  if(TARGET cargo_avoidance_fusion_test)
    target_link_libraries(cargo_avoidance_fusion_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(pending_static_hazard_tracker_test
    test/pending_static_hazard_tracker_test.cpp)
  if(TARGET pending_static_hazard_tracker_test)
    target_link_libraries(pending_static_hazard_tracker_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(static_evidence_authorization_test
    test/static_evidence_authorization_test.cpp)
  if(TARGET static_evidence_authorization_test)
    target_link_libraries(static_evidence_authorization_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(pending_cargo_envelope_test
    test/pending_cargo_envelope_test.cpp)
  if(TARGET pending_cargo_envelope_test)
    target_link_libraries(pending_cargo_envelope_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(pending_cargo_self_evidence_test
    test/pending_cargo_self_evidence_test.cpp)
  if(TARGET pending_cargo_self_evidence_test)
    target_link_libraries(pending_cargo_self_evidence_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(revealed_support_observer_test
    test/revealed_support_observer_test.cpp)
  if(TARGET revealed_support_observer_test)
    target_link_libraries(revealed_support_observer_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_swing_monitor_test
    test/cargo_swing_monitor_test.cpp)
  if(TARGET cargo_swing_monitor_test)
    target_link_libraries(cargo_swing_monitor_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_presence_state_machine_test
    test/cargo_presence_state_machine_test.cpp)
  if(TARGET cargo_presence_state_machine_test)
    target_link_libraries(cargo_presence_state_machine_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_physical_motion_estimator_test
    test/cargo_physical_motion_estimator_test.cpp)
  if(TARGET cargo_physical_motion_estimator_test)
    target_link_libraries(cargo_physical_motion_estimator_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(map_session_snapshot_test
    test/map_session_snapshot_test.cpp)
  if(TARGET map_session_snapshot_test)
    target_link_libraries(map_session_snapshot_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(persistent_registration_loader_test
    test/persistent_registration_loader_test.cpp)
  if(TARGET persistent_registration_loader_test)
    target_link_libraries(persistent_registration_loader_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_motion_corridor_test
    test/cargo_motion_corridor_test.cpp)
  if(TARGET cargo_motion_corridor_test)
    target_link_libraries(cargo_motion_corridor_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_residual_classifier_test
    test/cargo_residual_classifier_test.cpp)
  if(TARGET cargo_residual_classifier_test)
    target_link_libraries(cargo_residual_classifier_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_alarm_heartbeat_state_machine_test
    test/cargo_alarm_heartbeat_state_machine_test.cpp)
  if(TARGET cargo_alarm_heartbeat_state_machine_test)
    target_link_libraries(cargo_alarm_heartbeat_state_machine_test
      ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(relocalization_scan_context_test
    test/relocalization_scan_context_test.cpp)
  if(TARGET relocalization_scan_context_test)
    target_link_libraries(relocalization_scan_context_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(relocalization_confirmation_policy_test
    test/relocalization_confirmation_policy_test.cpp)
  if(TARGET relocalization_confirmation_policy_test)
    target_link_libraries(relocalization_confirmation_policy_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(ndt_fitness_circuit_breaker_test
    test/ndt_fitness_circuit_breaker_test.cpp)
  if(TARGET ndt_fitness_circuit_breaker_test)
    target_link_libraries(ndt_fitness_circuit_breaker_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(hook_load_state_filter_test
    test/hook_load_state_filter_test.cpp)
  if(TARGET hook_load_state_filter_test)
    target_link_libraries(hook_load_state_filter_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(cargo_observation_policy_test
    test/cargo_observation_policy_test.cpp)
  if(TARGET cargo_observation_policy_test)
    target_link_libraries(cargo_observation_policy_test
      ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(crane_pose_constraint_test
    test/crane_pose_constraint_test.cpp)
  if(TARGET crane_pose_constraint_test)
    target_link_libraries(crane_pose_constraint_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(rigid_transform_conversion_test
    test/rigid_transform_conversion_test.cpp)
  if(TARGET rigid_transform_conversion_test)
    target_link_libraries(rigid_transform_conversion_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(hook_load_evidence_policy_test
    test/hook_load_evidence_policy_test.cpp)
  if(TARGET hook_load_evidence_policy_test)
    target_link_libraries(hook_load_evidence_policy_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(pending_origin_binding_policy_test
    test/pending_origin_binding_policy_test.cpp)
  if(TARGET pending_origin_binding_policy_test)
    target_link_libraries(pending_origin_binding_policy_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(stationary_motion_policy_test
    test/stationary_motion_policy_test.cpp)
  if(TARGET stationary_motion_policy_test)
    target_link_libraries(stationary_motion_policy_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(registration_cloud_builder_test
    test/registration_cloud_builder_test.cpp)
  if(TARGET registration_cloud_builder_test)
    target_link_libraries(registration_cloud_builder_test
      ndt_slam_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(ndt_observability_test
    test/ndt_observability_test.cpp)
  if(TARGET ndt_observability_test)
    target_link_libraries(ndt_observability_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(runtime_diagnostics_test
    test/runtime_diagnostics_test.cpp)
  if(TARGET runtime_diagnostics_test)
    target_link_libraries(runtime_diagnostics_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()

  catkin_add_gtest(rail_localization_authority_test
    test/rail_localization_authority_test.cpp)
  if(TARGET rail_localization_authority_test)
    target_link_libraries(rail_localization_authority_test
      ndt_slam_lib ${catkin_LIBRARIES} gtest_main)
  endif()
endif()

install(TARGETS ndt_slam_lib ndt_slam_node pointcloud_merger cloud_diagnostics
                cargo_forbidden_zone_node cargo_alarm_heartbeat_node
                hook_load_state_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config rviz doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
)

catkin_install_python(PROGRAMS
  scripts/ops/ndt_recovery_watchdog.py
  scripts/ops/server_runtime_monitor.py
  scripts/ops/summarize_server_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/ops/server_monitorctl.sh
  scripts/ops/server_monitor_bag_validate.sh
  scripts/ops/server_preflight.sh
  scripts/ops/run_server_validation.sh
  scripts/ops/collect_server_artifacts.sh
  scripts/ops/install_server_services.sh
  scripts/ops/server_soak.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/ops/ndt-slam.service.in
  scripts/ops/ndt-slam-monitor.service.in
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/systemd
)
