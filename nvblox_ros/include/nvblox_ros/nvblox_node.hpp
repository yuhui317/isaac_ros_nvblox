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

#ifndef NVBLOX_ROS__NVBLOX_NODE_HPP_
#define NVBLOX_ROS__NVBLOX_NODE_HPP_

#include <nvblox/nvblox.h>

#include <geometry_msgs/msg/vector3.h>

#include <chrono>
#include <list>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <unordered_map>
#include <variant>
#include <vector>

#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nvblox_msgs/srv/file_path.hpp>
#include <nvblox_msgs/srv/esdf_and_gradients.hpp>

#include "nvblox_ros/layer_publishing.hpp"
#include "nvblox_ros/conversions/image_conversions.hpp"
#include "nvblox_ros/conversions/mesh_conversions.hpp"
#include "nvblox_ros/conversions/pointcloud_conversions.hpp"
#include "nvblox_ros/conversions/esdf_slice_conversions.hpp"
#include "nvblox_ros/conversions/esdf_and_gradients_conversions.hpp"
#include "nvblox_ros/mapper_initialization.hpp"
#include "nvblox_ros/transformer.hpp"
#include "nvblox_ros/camera_cache.hpp"
#include "nvblox_ros/node_params.hpp"
#include "nvblox_ros/service_request_task.hpp"

// 学习入口：建图节点接口和缓冲状态；三维坐标用全局/传感器 frame 标识，图像像素缓冲不是模型张量 batch。
namespace nvblox
{

constexpr int8_t kOccupancyGridUnknownValue = -1;

// ROS 包装节点聚合同步器、队列、TF、CUDA 缓存和静态/动态 mapper；核心几何算法由 nvblox_core 提供。
class NvbloxNode : public rclcpp::Node
{
public:
  // 声明参数可选的组件构造函数，可传 CUDA 流；当前实现按参数创建新的 stream。
  explicit NvbloxNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "nvblox_node", std::shared_ptr<CudaStream> cuda_stream =
    std::make_shared<CudaStreamOwning>());
  // 声明关闭时地图切片输出和统计打印路径。
  virtual ~NvbloxNode();

  // Setup. These are called by the constructor.
  // 创建设备内存上的 MultiMapper 并应用静态/动态积分参数，保存 background/foreground mapper 句柄。
  void initializeMultiMapper();
  // 建立图像+标定同步器和有界处理队列；回调只排队，真正积分由 tick 调用。
  void subscribeToTopics();
  // 声明静态/动态/组合距离切片、占据栅格和诊断可视化发布器；实际输出依赖模式与订阅者。
  void advertiseTopics();
  // 声明地图/PLY/统计保存与 ESDF 查询服务；耗时服务后续转为处理队列任务。
  void advertiseServices();
  // 以 tick_period_ms/1000 的秒周期创建 wall timer，归入互斥处理 callback group。
  void setupTimers();

  // Internal types for passing around images, their matching
  // segmentation masks, as well as the camera intrinsics.
  // 四元组顺序为 image、camera_info、mask、mask_camera_info；普通二元组复用前两项。
  using ImageSegmentationMaskMsgTuple =
    std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr,
      sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr>;

  using ImageMsgTuple =
    std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr>;

  // Expresses the various types of Images that can be queued in the node for processing.
  using ImageTypeVariant = std::variant<ImageMsgTuple, ImageSegmentationMaskMsgTuple>;

  // Internal type of an image message with an *optional* mask.
  using ImageMsgOptionalMaskMsgTuple =
    std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr,
      std::optional<sensor_msgs::msg::Image::ConstSharedPtr>,
      std::optional<sensor_msgs::msg::CameraInfo::ConstSharedPtr>>;

  // Named indices for the MsgTuple members.
  static constexpr size_t kMsgTupleImageIdx = 0;
  static constexpr size_t kMsgTupleCameraInfoIdx = 1;
  static constexpr size_t kMsgTupleMaskIdx = 2;
  static constexpr size_t kMsgTupleMaskCameraInfoIdx = 3;

  // Callback functions. These just stick images in a queue.
  // 将已近似同步的深度/标定/掩码/标定四元组加入深度队列，并统计到达延迟。
  void depthPlusMaskImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_camera_info,
    const sensor_msgs::msg::Image::ConstSharedPtr & seg_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & seg_camera_info);
  // 将已精确同步的深度与 CameraInfo 二元组加入深度队列，不在订阅回调中积分。
  void depthImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_camera_info);
  // 将已精确同步的彩色/标定/掩码/标定四元组加入彩色队列。
  void colorPlusMaskImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & color_camera_info,
    const sensor_msgs::msg::Image::ConstSharedPtr & seg_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & seg_camera_info);
  // 将已精确同步的彩色与 CameraInfo 二元组加入彩色队列。
  void colorImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & color_camera_info);
  // 把 PointCloud2 及其采集时间排入 LiDAR 队列，等待对应时刻的传感器位姿可用。
  void pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud);

  // 将 PLY 输出封装为队列任务并等待完成；完整 .ply 路径与目录路径选择不同输出集合。
  void savePly(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  // 服务排队保存静态 mapper 的 LayerCake，缺少 .nvblx 后缀时补上。
  void saveMap(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  // 服务排队把 .nvblx 地图加载进静态 mapper，返回实际加载结果。
  void loadMap(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  // 直接把速率统计写入 .txt；此服务未走地图处理任务队列。
  void saveRates(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  // 直接把耗时统计写入 .txt；此服务未走地图处理任务队列。
  void saveTimings(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  // 仅允许三维 ESDF；排队执行形状清理、可选 ESDF 更新与 AABB 范围查询，并等待响应。
  void getEsdfAndGradientService(
    const std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> request,
    std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Response> response);

  // Main tick function that process all input data queues in order
  // 串行处理服务与传感器队列，再按独立频率衰减/清图/更新 ESDF/发布层与诊断。
  virtual void tick();

  // Publish data on fixed frequency
  // 查询清图参考系最新位姿，按带宽和可视化范围把有订阅的静态/动态层交给 LayerPublisher。
  void publishLayers();

  // Publish debug visualizations for rviz
  // 发布 TSDF 地面候选点、地面估计、ESDF 高度界与工作空间边界；不修改建图积分。
  void publishDebugVisualizations();

  // Process data
  // 深度核心：限频→按采集时刻查 TF→相机/Buffer 转换→MultiMapper 深度积分→可选调试输出。
  virtual bool processDepthImage(const ImageTypeVariant & depth_mask_msg);
  // 彩色核心：限频与清图时间检查→查询采集时刻 TF→Buffer/内参转换→彩色层积分。
  virtual bool processColorImage(const ImageTypeVariant & color_mask_msg);
  // LiDAR 核心：验证模型、转换点云并按配置读取逐点时间，选用扫描终点 TF 做运动补偿后积分。
  virtual bool processLidarPointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr);

  // Return true if the tf-tree contains a transform for the given frame_id and timestamp
  // 调用 Transformer 试查给定 frame/time 到 global_frame 的变换，只返回可用性。
  bool canTransform(const std::string & frame_id, const rclcpp::Time & timestamp);

  // 保留原有切片平面发布声明；当前所读 nvblox_node.cpp 中未找到实现，不能据此声称此路径可调用。
  void publishSlicePlane(const rclcpp::Time & timestamp, const Transform & T_L_C);

  // Decay the dynamic occupancy and static tsdf grid on fixed frequency
  // 对前景动态 mapper 的全部占据体素执行衰减。
  void decayDynamicOccupancy();
  // 衰减静态 TSDF，排除最近相机视野；原代码注释说明不排除 LiDAR 视野。
  void decayTsdf();

protected:
  // Process functions for individual queues
  // 筛选已有位姿的深度队列项并调用 processDepthImage；processQueue 会先移除就绪项。
  virtual void processDepthQueue();
  // 筛选已有位姿的彩色队列项并调用 processColorImage；回调返回 true 不代表积分一定成功。
  virtual void processColorQueue();
  // 在 LiDAR 时刻位姿可查时移出队列并处理点云；返回值未被模板队列重试机制使用。
  virtual void processPointcloudQueue();
  // 执行两个服务任务队列，并在请求可视化时立即发布层。
  virtual void processServiceRequestTaskQueue();
  // 调用 MultiMapper 更新 ESDF；二维模式按需切片发布静态、动态和组合距离/占据输出。
  virtual void processEsdf();

  // Return true if the time between the two passed timestamps is sufficient to trigger an action
  // under the requested rate.
  // 按秒时间差与目标 Hz 控制处理频率；Hz<=0 禁用，时间差必须严格大于一个周期。
  bool shouldProcess(
    const rclcpp::Time & time_now, const rclcpp::Time & time_last,
    const float desired_frequency_hz);

  // Publish the dynamic outputs
  // 有订阅者时发布最近动态点云与深度遮罩叠图；点云使用 global_frame。
  void publishDynamics(
    const std::string & camera_frame_id);

  // Publish human debug output
  // 将前景深度反投影为点云并转换到全局坐标；按订阅情况发布人物点云与叠图。
  void publishHumanDebugOutput(const std::string & camera_frame_id, const Camera & camera);

  // Publish the back projected depth image for debug purposes
  template<typename SensorType>
  // 对 Camera 或 Lidar 深度按帧抽样反投影，仅为订阅者发布传感器坐标系的调试点云。
  void publishBackProjectedDepth(
    const SensorType & sensor, const DepthImage & depth_image, const std::string & sensor_frame,
    const rclcpp::Time & timestamp);

  // Helper function to update the esdf of a specific mapper
  // 保留原有单 mapper ESDF 更新声明；实际 tick 路径调用 processEsdf/MultiMapper::updateEsdf。
  void updateEsdf(
    const std::string & name,
    const std::shared_ptr<Mapper> & mapper);

  // Helper function to slice and publish the esdf of a specific mapper
  // 仅在相应订阅者存在时提取单层或双层 ESDF 切片，转成点云/DistanceMapSlice/占据栅格。
  void sliceAndPublishEsdf(
    const std::string & name,
    const std::shared_ptr<Mapper> & mapper,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pointcloud_publisher,
    const rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr & slice_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & occupancy_grid_publisher,
    const float unknown_value, const Mapper * mapper_2 = nullptr);

  // Publish an occupancy grid message from a slice image.
  // 将距离切片转换为 width×height 的 OccupancyGrid，设置 global_frame、分辨率与二维原点。
  void publishOccupancyGridMsg(
    const float voxel_size, const int width,
    const int height, const double origin_x_position,
    const double origin_y_position, const nvblox::Image<float> & map_slice_image,
    const float unknown_value,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & occupancy_grid_publisher);

  // Map clearing
  // 当清图半径为正且最新参考位姿可查时，清除静态 mapper 半径外的块。
  void clearMapOutsideOfRadiusOfLastKnownPose();

  /// Used by callbacks (internally) to add input items to queues.
  /// @tparam QueuedType The type of the input item stored by the queue.
  /// @param queue_name Name of the queue, used for logging.
  /// @param item Item to be added to the queue.
  /// @param queue_ptr Queue where to add the item.
  /// @param queue_mutex_ptr Mutex protecting the queue.
  template<typename QueuedType>
  // 模板声明：互斥入队，超过 maximum_input_queue_length 时丢最旧项；实现见 impl/nvblox_node_impl.hpp。
  void pushOntoQueue(
    const std::string & queue_name,
    QueuedType item,
    std::unique_ptr<std::list<QueuedType>> & queue_ptr,
    std::mutex * queue_mutex_ptr);

  // Used internally to unify processing of queues
  // that process an input item based on a ready check.
  template<typename QueuedType>
  using ProcessFunctionType = std::function<bool (const QueuedType &)>;
  template<typename QueuedType>
  using ReadyCheckFunctionType = std::function<bool (const QueuedType &)>;

  /// Returns true if pose(s) for the image message is available.
  /// @param variant_msg The image(s) + cam_info message.
  /// @return True if poses are available.
  // 访问图像变体，检查图像及可选 mask 的采集时刻到 global_frame 的变换是否可用。
  bool isPoseAvailable(const ImageTypeVariant & variant_msg);

  /// Takes an ImageTypeVariant and decomposes it into an image with an optional mask.
  /// @param msg The image(s) + cam_info message.
  /// @return A tuple of an image, camera_info, and optional mask.
  // 把二元/四元图像变体统一拆成 image、CameraInfo 与可选 mask/CameraInfo。
  ImageMsgOptionalMaskMsgTuple decomposeImageTypeVariant(const ImageTypeVariant & msg);

  /// Processes a queue of input items by detecting if they're ready and then
  /// calling a process function.
  /// @tparam QueuedType The type of the input items in the queue.
  /// @param queue_ptr Queue of input items to process.
  /// @param queue_mutex_ptr Mutex protecting the queue.
  /// @param ready_check_function Function called on each item to check if
  /// it's ready to be processed
  /// @param process_function Function to process each ready item.
  template<typename QueuedType>
  // 模板声明：在锁内选出并移除就绪项，解锁后处理；实现没有按处理函数返回值重新入队。
  void processQueue(
    std::unique_ptr<std::list<QueuedType>> & queue_ptr, std::mutex * queue_mutex_ptr,
    ReadyCheckFunctionType<QueuedType> ready_check_function,
    ProcessFunctionType<QueuedType> process_function);

  // Declares a ROS parameter.
  // Calls the underlying method from rclcpp, but also adds the parameter and value to
  // parameter tree for printing.
  template<typename ParamType>
  // 保留参数树辅助模板声明；本轮读取的队列实现头没有它的定义，不假设它参与当前参数初始化。
  ParamType declareParameter(
    const std::string & name, const ParamType & default_value,
    nvblox::parameters::ParameterTreeNode * parameter_tree);

  // ROS publishers and subscribers

  // Transformer to handle... everything, let's be honest.
  Transformer transformer_;

  // Throttling debug messages to reduce spamming the console
  static constexpr float kTimeBetweenDebugMessagesMs = 1000.0;

  /// Number of cameras supported (number of subscribers created).
  // 节点最多建立四路深度/彩色/mask topic；不同 launch 示例可能有更小限制。
  static constexpr size_t kMaxNumCameras = 4;

  /// Depth topics to listen to. At least one of these topics has to transmit images. All topics
  /// are added to the same processing queue (depth or color) and thus gets the same treatment.
  static constexpr std::array<const char *,
    kMaxNumCameras> kDepthTopicBaseNames =
  {
    // Multi-camera topics
    "camera_0/depth",
    "camera_1/depth",
    "camera_2/depth",
    "camera_3/depth",
  };

  /// Color topics to listen to.
  static constexpr std::array<const char *,
    kMaxNumCameras> kColorTopicBaseNames =
  {
    // Multi-camera topics
    "camera_0/color",
    "camera_1/color",
    "camera_2/color",
    "camera_3/color",
  };

  /// Color topics to listen to.
  static constexpr std::array<const char *,
    kMaxNumCameras> kSegTopicBaseNames =
  {
    // Multi-camera topics
    "camera_0/mask",
    "camera_1/mask",
    "camera_2/mask",
    "camera_3/mask",
  };


  /// Image + info subscribers
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::Image>>>
  depth_image_subs_;
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>>
  depth_camera_info_subs_;
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::Image>>>
  color_image_subs_;
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>>
  color_camera_info_subs_;
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::Image>>>
  segmentation_image_subs_;
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>>
  segmentation_camera_info_subs_;

  // Sync Policies
  // 同步策略由实际订阅分支选择：深度+mask 为近似时间，其它图像/标定路径为精确时间。
  using image_mask_approx_policy = ::message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using image_mask_approx_sync = ::message_filters::Synchronizer<image_mask_approx_policy>;

  using image_exact_policy = ::message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using image_exact_sync = ::message_filters::Synchronizer<image_exact_policy>;

  using image_mask_exact_policy = ::message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using image_mask_exact_sync = ::message_filters::Synchronizer<image_mask_exact_policy>;

  std::vector<std::shared_ptr<image_mask_approx_sync>> timesync_depth_mask_;
  std::vector<std::shared_ptr<image_mask_exact_sync>> timesync_color_mask_;
  std::vector<std::shared_ptr<image_exact_sync>> timesync_depth_;
  std::vector<std::shared_ptr<image_exact_sync>> timesync_color_;


  // Pointcloud sub.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_sub_;

  // Optional transform subs.
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
    transform_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publishers
  std::unique_ptr<LayerPublisher> layer_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    static_esdf_pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    static_map_slice_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pessimistic_static_esdf_pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    pessimistic_static_map_slice_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    esdf_slice_bounds_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    workspace_bounds_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    shapes_to_clear_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    color_frame_overlay_publisher_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
  back_projected_depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    dynamic_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    dynamic_depth_frame_overlay_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    dynamic_esdf_pointcloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    combined_esdf_pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    dynamic_map_slice_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    combined_map_slice_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    static_occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    dynamic_occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    combined_occupancy_grid_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    tsdf_zero_crossings_ground_plane_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    tsdf_zero_crossings_pointcloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    lidar_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    esdf_service_pointcloud_publisher_;

  // Services.
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr save_ply_service_;
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr save_map_service_;
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr load_map_service_;
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr save_rates_service_;
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr save_timings_service_;
  rclcpp::Service<nvblox_msgs::srv::EsdfAndGradients>::SharedPtr send_esdf_and_gradient_service_;

  // Callback groups.
  rclcpp::CallbackGroup::SharedPtr group_processing_;

  // Timers.
  rclcpp::TimerBase::SharedPtr queue_processing_timer_;

  // Collection of params for the nvblox node
  NvbloxNodeParams params_;

  // Counter for back projection subsampling
  std::map<std::string, uint32_t> back_projection_idx_;

  /// The last time processing occurred. Used to maintain rates.
  /// NOTE: Because we support multiple cameras, the first two maps map from the frame_id
  /// of the camera to the timestamp of the last image integrated from that camera.
  std::unordered_map<std::string, rclcpp::Time> integrate_depth_last_times_;
  std::unordered_map<std::string, rclcpp::Time> integrate_color_last_times_;
  rclcpp::Time integrate_lidar_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time update_mesh_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time update_esdf_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time publish_layer_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time publish_debug_vis_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time decay_tsdf_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time decay_dynamic_occupancy_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time clear_map_outside_radius_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  /// The time stamp of the last frame contributing to the reconstruction.
  rclcpp::Time newest_integrated_depth_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  /// The time stamp when we last cleared shapes in the reconstruction.
  rclcpp::Time shape_clearing_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // The QoS settings for the image input topics
  const std::string kDefaultInputQos_ = "SYSTEM_DEFAULT";

  // MultiMapper
  // Holding the static and dynamic mapper and
  // handling where input data gets integrated.
  // MultiMapper 管理背景与前景；静态/动态在这里表示建图类型，而非学习式世界模型的预测分支。
  std::shared_ptr<MultiMapper> multi_mapper_;

  // Direct access to Mappers
  // Holding the map layers and their associated integrators
  // - TsdfLayer, FreespaceLayer, ColorLayer, EsdfLayer, MeshLayer
  std::shared_ptr<Mapper> static_mapper_;
  std::shared_ptr<Mapper> dynamic_mapper_;

  // Various converters for ROS message generation.
  EsdfSlicer esdf_slicer_;
  conversions::PointcloudConverter pointcloud_converter_;
  conversions::EsdfSliceConverter esdf_slice_converter_;
  conversions::EsdfAndGradientsConverter esdf_and_gradients_converter_;

  // Caches for GPU images
  // GPU 图像缓存按运行时 width/height 分配，深度积分使用米单位 float；不应假定固定输入分辨率。
  ColorImage color_image_{MemoryType::kDevice};
  DepthImage depth_image_{MemoryType::kDevice};
  MonoImage mask_image_{MemoryType::kDevice};
  Image<conversions::Rgb> rgb_image_tmp_{MemoryType::kDevice};

  // Object for back projecting image to a pointcloud.
  DepthImageBackProjector image_back_projector_;

  // Cache the last known number of subscribers.
  size_t mesh_subscriber_count_ = 0;

  // Whether a service call requested the layers to be visualized.
  bool publish_layers_requested_ = false;

  // Types of items queued in service call queues.
  using EsdfServiceQueuedType = std::shared_ptr<ServiceRequestTask<NvbloxNode,
      nvblox_msgs::srv::EsdfAndGradients>>;
  using FilePathServiceQueuedType = std::shared_ptr<ServiceRequestTask<NvbloxNode,
      nvblox_msgs::srv::FilePath>>;

  // Input queues. Unique pointers are used to enable more flexibility when deallocating.
  std::unique_ptr<std::list<sensor_msgs::msg::PointCloud2::ConstSharedPtr>> pointcloud_queue_;
  std::unique_ptr<std::list<EsdfServiceQueuedType>> esdf_service_queue_;
  std::unique_ptr<std::list<FilePathServiceQueuedType>> file_path_service_queue_;
  std::unique_ptr<std::list<ImageTypeVariant>> depth_image_queue_;
  std::unique_ptr<std::list<ImageTypeVariant>>
  color_image_queue_;

  // Input queue names.
  static constexpr char kDepthQueueName[] = "depth_queue";
  static constexpr char kColorQueueName[] = "color_queue";
  static constexpr char kPointcloudQueueName[] = "pointcloud_queue";
  static constexpr char kFilePathServiceQueueName[] = "file_path_service_queue";
  static constexpr char kEsdfServiceQueueName[] = "esdf_service_queue";

  // Input queue mutexes.
  std::mutex depth_queue_mutex_;
  std::mutex color_queue_mutex_;
  std::mutex depth_mask_queue_mutex_;
  std::mutex color_mask_queue_mutex_;
  std::mutex pointcloud_queue_mutex_;
  std::mutex esdf_service_queue_mutex_;
  std::mutex file_path_service_queue_mutex_;

  // Counts the number of messages dropped from the input queues.
  // Maps the input queue to the number of messages that have been dropped.
  std::unordered_map<std::string, int> number_of_dropped_queued_items_;

  // Device caches
  Pointcloud pointcloud_C_device_;
  Pointcloud human_pointcloud_C_device_;
  Pointcloud human_pointcloud_L_device_;
  Pointcloud tsdf_zero_crossings_device_;

  Transform T_L_C_depth_;

  // The idle timer measures time spent *outside* the main tick function and can thus be used to
  // monitor how much headroom there is until the system gets saturated. A lower number <=1ms/tick
  // means that the system is under high pressure.
  static constexpr bool kStartStopped = true;
  timing::Timer idle_timer_{"ros/idle", kStartStopped};

  // Cuda stream for GPU work
  std::shared_ptr<CudaStream> cuda_stream_ = nullptr;

  // Cached intrinsic calibration for cameras. Separate caches for depth and color in order to
  // support different intrinsics between the two.
  CameraCache depth_camera_cache_;
  CameraCache color_camera_cache_;

  // Parameter tree for parameter printing.
  nvblox::parameters::ParameterTreeNode parameter_tree_{"nvblox_node", {}};
};

}  // namespace nvblox

#include "nvblox_ros/impl/nvblox_node_impl.hpp"

#endif  // NVBLOX_ROS__NVBLOX_NODE_HPP_
