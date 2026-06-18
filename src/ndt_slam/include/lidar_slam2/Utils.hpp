// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <regex>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>

namespace tf2 {

inline geometry_msgs::Transform sophusToTransform(const Sophus::SE3d &T) {
    geometry_msgs::Transform t;
    t.translation.x = T.translation().x();
    t.translation.y = T.translation().y();
    t.translation.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();

    return t;
}

inline geometry_msgs::Pose sophusToPose(const Sophus::SE3d &T) {
    geometry_msgs::Pose t;
    t.position.x = T.translation().x();
    t.position.y = T.translation().y();
    t.position.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.orientation.x = q.x();
    t.orientation.y = q.y();
    t.orientation.z = q.z();
    t.orientation.w = q.w();

    return t;
}

inline Sophus::SE3d transformToSophus(const geometry_msgs::TransformStamped &transform) {
    const auto &t = transform.transform;
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
        Sophus::SE3d::Point(t.translation.x, t.translation.y, t.translation.z));
}
}  // namespace tf2

namespace lidar_slam2::utils {

using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using Header = std_msgs::Header;

inline std::string FixFrameId(const std::string &frame_id) {
    return std::regex_replace(frame_id, std::regex("^/"), "");
}

inline std::optional<PointField> GetTimestampField(const PointCloud2::ConstPtr msg) {
    PointField timestamp_field;
    for (const auto &field : msg->fields) {
        if ((field.name == "t" || field.name == "timestamp" || field.name == "time" ||
             field.name == "time_stamp")) {
            timestamp_field = field;
        }
    }
    if (timestamp_field.count) return timestamp_field;
    ROS_DEBUG("Field 't', 'timestamp', 'time_stamp', or 'time' does not exist. "
             "Disabling scan deskewing");
    return {};
}

inline std::vector<double> NormalizeTimestamps(const std::vector<double> &timestamps) {
    const auto [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
    const double min_timestamp = *min_it;
    const double max_timestamp = *max_it;

    std::vector<double> timestamps_normalized(timestamps.size());
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
                   [&](const auto &timestamp) {
                       return (timestamp - min_timestamp) / (max_timestamp - min_timestamp);
                   });
    return timestamps_normalized;
}

template <typename T>
inline std::vector<double> ExtractTimestampsFromIterator(const PointCloud2::ConstPtr msg, sensor_msgs::PointCloud2ConstIterator<T> &&it) {
    const size_t n_points = msg->height * msg->width;
    std::vector<double> timestamps;
    timestamps.reserve(n_points);
    for (size_t i = 0; i < n_points; ++i, ++it) {
        timestamps.emplace_back(static_cast<double>(*it));
    }
    return NormalizeTimestamps(timestamps);
}

inline std::vector<double> ExtractTimestampsFromMsg(const PointCloud2::ConstPtr msg,
                                     const PointField &timestamp_field) {
    using sensor_msgs::PointCloud2ConstIterator;
    if (timestamp_field.datatype == PointField::UINT32) {
        return ExtractTimestampsFromIterator(msg, PointCloud2ConstIterator<uint32_t>(*msg, timestamp_field.name));
    } else if (timestamp_field.datatype == PointField::FLOAT32) {
        return ExtractTimestampsFromIterator(msg, PointCloud2ConstIterator<float>(*msg, timestamp_field.name));
    } else if (timestamp_field.datatype == PointField::FLOAT64) {
        return ExtractTimestampsFromIterator(msg, PointCloud2ConstIterator<double>(*msg, timestamp_field.name));
    }

    throw std::runtime_error("timestamp field type not supported");
}

inline std::vector<double> GetTimestamps(const PointCloud2::ConstPtr msg) {
    auto timestamp_field = GetTimestampField(msg);
    if (!timestamp_field.has_value()) return {};

    std::vector<double> timestamps = ExtractTimestampsFromMsg(msg, timestamp_field.value());

    return timestamps;
}

inline std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloud2::ConstPtr msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
    for (size_t i = 0; i < msg->height * msg->width; ++i, ++msg_x, ++msg_y, ++msg_z) {
        points.emplace_back(*msg_x, *msg_y, *msg_z);
    }
    return points;
}

}  // namespace lidar_slam2::utils
