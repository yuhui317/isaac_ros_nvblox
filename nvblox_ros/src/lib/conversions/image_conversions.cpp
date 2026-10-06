// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "nvblox_ros/conversions/image_conversions.hpp"
#include "nvblox_ros/conversions/image_conversions_thrust.hpp"

#include "cuda_buffer/cuda_buffer_api.hpp"

// 学习入口：标准 ROS Image/CameraInfo 与设备图像的转换；release-5.0 的 Buffer 路径与旧裸 data 指针辅助路径并存。
namespace nvblox
{
namespace conversions
{

// Convert camera info message to NVBlox camera object
// 仅从 K 的 fx/fy/cx/cy 与宽高创建针孔 Camera；不读取 D/R/P 做畸变或整流处理。
Camera cameraFromMessage(const sensor_msgs::msg::CameraInfo & camera_info)
{
  Camera camera(camera_info.k[0], camera_info.k[4], camera_info.k[2], camera_info.k[5],
    camera_info.width, camera_info.height);
  return camera;
}

// Convert image to GPU image
// 旧式消息 data 指针路径：仅支持 rgb8/bgra8，经主机到设备转换填入 ColorImage。
bool colorImageFromImageMessageAsync(
  const sensor_msgs::msg::Image & image_msg,
  ColorImage * color_image, Image<Rgb> * rgb_image_tmp,
  Image<Bgra> * bgra_image_tmp, rclcpp::Logger logger,
  const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(color_image);

  const std::string encoding = image_msg.encoding;
  if (encoding == "rgb8") {
    return rgbaFromHostAsync(
      reinterpret_cast<const Rgb *>(&image_msg.data[0]), image_msg.height,
      image_msg.width, color_image, rgb_image_tmp, cuda_stream);
  } else if (encoding == "bgra8") {
    return rgbaFromHostAsync(
      reinterpret_cast<const Bgra *>(&image_msg.data[0]), image_msg.height,
      image_msg.width, color_image, bgra_image_tmp, cuda_stream);
  } else {
    RCLCPP_ERROR_STREAM(logger, "Invalid color image encoding: " << encoding);
    return false;
  }
}

// Convert image to GPU image
// 旧式消息 data 指针路径：仅支持 mono8，按 height×width 异步拷贝掩码。
bool monoImageFromImageMessageAsync(
  const sensor_msgs::msg::Image & image_msg, MonoImage * mono_image,
  const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(mono_image);

  // First check if we actually have a valid image here.
  if (image_msg.encoding != "mono8") {
    return false;
  }

  mono_image->copyFromAsync(image_msg.height, image_msg.width, &image_msg.data[0], cuda_stream);
  return true;
}

// 导出 float 深度为 32FC1，step=width*sizeof(float)；复制后同步 CUDA 流，header.stamp 由调用方处理。
void imageMessageFromDepthImage(
  const DepthImage & depth_image, const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(image_msg);
  size_t image_size = depth_image.width() * depth_image.height() * sizeof(float);
  image_msg->data.resize(image_size);

  image_msg->header.frame_id = frame_id;
  image_msg->width = depth_image.width();
  image_msg->height = depth_image.height();
  image_msg->step = depth_image.width() * sizeof(float);

  image_msg->encoding = "32FC1";

  cudaMemcpyAsync(
    &image_msg->data[0], depth_image.dataConstPtr(), image_size, cudaMemcpyDefault,
    cuda_stream);
  cuda_stream.synchronize();
}

// 导出 RGB 彩色为 rgb8，step=width*3；复制后同步 CUDA 流，header.stamp 由调用方处理。
void imageMessageFromColorImage(
  const ColorImage & color_image, const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(image_msg);
  constexpr int num_channels = 3;
  static_assert(ColorImage::ElementType::size() == num_channels, "Need RGB image");
  size_t image_size = color_image.width() * color_image.height() * sizeof(uint8_t) * num_channels;
  image_msg->data.resize(image_size);

  image_msg->header.frame_id = frame_id;
  image_msg->width = color_image.width();
  image_msg->height = color_image.height();
  image_msg->step = color_image.width() * sizeof(uint8_t) * num_channels;

  image_msg->encoding = "rgb8";

  cudaMemcpyAsync(
    &image_msg->data[0], color_image.dataConstPtr(), image_size, cudaMemcpyDefault,
    cuda_stream);
  cuda_stream.synchronize();
}

// 旧式深度指针路径：32FC1 保留 float，16UC1 经下层转换为米；不支持其它编码。
bool depthImageFromRosMessageAsync(
  const sensor_msgs::msg::Image & image_msg,
  DepthImage * depth_image, Image<int16_t> * image_tmp,
  rclcpp::Logger logger, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(depth_image);

  if (image_msg.encoding == "32FC1") {
    return depthFromFloatHostOrDeviceAsync(
      reinterpret_cast<const float *>(&image_msg.data[0]),
      image_msg.height, image_msg.width, depth_image,
      cuda_stream);
  } else if (image_msg.encoding == "16UC1") {
    return depthFromIntHostAsync(
      reinterpret_cast<const int16_t *>(&image_msg.data[0]),
      image_msg.height, image_msg.width, depth_image, image_tmp,
      cuda_stream);
  } else {
    RCLCPP_ERROR_STREAM(logger, "Invalid depth image encoding: " << image_msg.encoding);
    return false;
  }
}

// 从 Buffer 创建 CUDA 读句柄；32FC1 为米单位 float，16UC1/mono16 由下层除以 1000 转米。
bool depthImageFromImageBufferAsync(
  const sensor_msgs::msg::Image & image, DepthImage * depth_image,
  rclcpp::Logger logger, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(depth_image);

  // Buffer 后端负责取设备可读指针和生产者同步；这里不保证输入本来就在 GPU 或完全零拷贝。
  auto read_handle = cuda_buffer_backend::from_input_buffer(image.data, cuda_stream);
  if (image.encoding == "32FC1") {
    return depthFromFloatHostOrDeviceAsync(
      reinterpret_cast<const float *>(read_handle.get_ptr()),
      image.height, image.width, depth_image,
      cuda_stream);
  } else if (image.encoding == "16UC1" || image.encoding == "mono16") {
    return depthFromIntDeviceAsync(
      reinterpret_cast<const int16_t *>(read_handle.get_ptr()),
      image.height, image.width, depth_image, cuda_stream);
  } else {
    RCLCPP_ERROR_STREAM(logger, "Invalid depth image encoding: " << image.encoding);
    return false;
  }
}

// 从 Buffer 创建 CUDA 读句柄，仅支持 rgb8/bgra8，交给设备转换函数调整通道顺序。
bool colorImageFromImageBufferAsync(
  const sensor_msgs::msg::Image & image, ColorImage * color_image,
  rclcpp::Logger logger, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(color_image);

  // Buffer 后端负责取设备可读指针和生产者同步；这里不保证输入本来就在 GPU 或完全零拷贝。
  auto read_handle = cuda_buffer_backend::from_input_buffer(image.data, cuda_stream);
  // 编码分支明确检查支持范围；转换按 width×height 处理，未在此函数使用 image.step 修正逐行 padding。
  const std::string & encoding = image.encoding;
  if (encoding == "rgb8") {
    return rgbaFromDeviceAsync(
      reinterpret_cast<const Rgb *>(read_handle.get_ptr()), image.height,
      image.width, color_image, cuda_stream);
  } else if (encoding == "bgra8") {
    return rgbaFromDeviceAsync(
      reinterpret_cast<const Bgra *>(read_handle.get_ptr()), image.height,
      image.width, color_image, cuda_stream);
  } else {
    RCLCPP_ERROR_STREAM(logger, "Invalid color image encoding: " << encoding);
    return false;
  }
}

// 从 Buffer 创建 CUDA 读句柄，仅接受 mono8 并按 height×width 转成设备掩码。
bool monoImageFromImageBufferAsync(
  const sensor_msgs::msg::Image & image, MonoImage * mono_image,
  rclcpp::Logger logger, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(mono_image);

  // First check if we actually have a valid image here.
  if (image.encoding != "mono8") {
    RCLCPP_ERROR_STREAM(logger, "Invalid mask image encoding: " << image.encoding);
    return false;
  }

  // Buffer 后端负责取设备可读指针和生产者同步；这里不保证输入本来就在 GPU 或完全零拷贝。
  auto read_handle = cuda_buffer_backend::from_input_buffer(image.data, cuda_stream);
  return monoFromIntDeviceAsync(
    reinterpret_cast<const uint8_t *>(read_handle.get_ptr()),
    image.height, image.width, mono_image, cuda_stream);
}

}  // namespace conversions
}  // namespace nvblox
