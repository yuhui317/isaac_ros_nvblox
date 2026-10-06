// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVBLOX_ROS__TRANSFORMER_HPP_
#define NVBLOX_ROS__TRANSFORMER_HPP_

#include <nvblox/core/types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// 学习入口：位姿来源、时间容差和坐标系缓存；TF 参数与 topic 参数不能只靠消息名字互换。
namespace nvblox
{

/// Class that binds to either the TF tree or resolves transformations from the
/// ROS parameter server, depending on settings loaded from ROS params.
// 封装 TF 查询与 topic 位姿缓存；输出变换方向以矩阵 T_global_sensor 为准。
class Transformer
{
public:
  // 声明节点参数和 TF 监听资源的构造入口。
  explicit Transformer(rclcpp::Node * node);

  /// @brief Looks up the transform between the frame with the passed name and the global frame
  ///        (which is set by the setters below). We either use tf2 or a stored queue of
  ///         transforms from messages.
  /// @param sensor_frame The frame name.
  /// @param timestamp Time of the transform. Passing rclcpp::Time(0) will return the latest
  ///                  transform in the queue.
  /// @param transform The output transform.
  /// @return true if the lookup was successful.
  // 查询传感器到 global frame 的采集时刻变换，返回 false 表示当前不可处理此观测。
  bool lookupTransformToGlobalFrame(
    const std::string & sensor_frame,
    const rclcpp::Time & timestamp,
    Transform * transform);

  /// Assumes these transforms are from GLOBAL frame to POSE frame. Ignores
  /// frame_id.
  // 声明 topic TransformStamped 位姿输入；本实现使用配置约定而忽略消息 frame 标识。
  void transformCallback(
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_msg);
  // 声明 topic PoseStamped 位姿输入；本实现使用配置约定而忽略消息 frame 标识。
  void poseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr transform_msg);

  /// Set the names of the frames.
  // 保存输出地图/全局参考系名称，供 TF 目标坐标系查询。
  void set_global_frame(const std::string & global_frame)
  {
    global_frame_ = global_frame;
  }

  // 保存 topic 所代表的 pose frame，用它与传感器静态外参组合。
  void set_pose_frame(const std::string & pose_frame)
  {
    pose_frame_ = pose_frame;
  }

private:
  // 声明 target/source 的 TF 查询辅助函数。
  bool lookupTransformTf(
    const std::string & from_frame,
    const std::string & to_frame,
    const rclcpp::Time & timestamp, Transform * transform);

  // 声明 topic 时间队列查询；时间零取最新值，非零没有插值。
  bool lookupTransformQueue(
    const rclcpp::Time & timestamp,
    Transform * transform);

  // 声明可缓存的 pose→sensor 外参查询。
  bool lookupSensorTransform(
    const std::string & sensor_frame,
    Transform * transform);

  // 声明 ROS Transform 到 Eigen 刚体变换的字段转换。
  Transform transformToEigen(const geometry_msgs::msg::Transform & transform) const;
  // 声明 ROS Pose 到 Eigen 刚体变换的字段转换。
  Transform poseToEigen(const geometry_msgs::msg::Pose & pose) const;

  /// ROS State
  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /// Global/map coordinate frame. Will always look up TF transforms to this
  /// frame.
  std::string global_frame_;
  /// Use this as the "pose" frame that's coming in. Needs to be set.
  std::string pose_frame_;

  /// Whether to use TF transforms at all.
  /// If set to false, use_topic_transforms_ must be true
  /// and pose_frame *needs* to be set.
  bool use_tf_transforms_ = true;
  /// Whether to listen to topics for transforms.
  /// If set to true, will try to get `global_frame` to `pose_frame`
  /// transform from the topics. If set to false,
  /// everything will be resolved through TF.
  bool use_topic_transforms_ = false;
  /// Timestamp tolerance to use for transform *topics* only.
  // topic 位姿容差为 1e8 ns=100 ms；TF 路径不使用此成员容差。
  uint64_t timestamp_tolerance_ns_ = 1e8;  // 100 milliseconds

  /// Queues and state
  /// Maps timestamp in ns to transform global -> pose frame.
  // 缓存 T_global_pose；sensor_transforms_ 另缓存 T_pose_sensor，查询结果由两者相乘。
  std::map<uint64_t, Transform> transform_queue_;
  /// Maps sensor frame to transform pose frame -> sensor frame.
  std::unordered_map<std::string, Transform> sensor_transforms_;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__TRANSFORMER_HPP_
