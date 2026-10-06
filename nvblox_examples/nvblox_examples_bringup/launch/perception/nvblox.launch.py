# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# 学习入口：把相机类型/建图模式选择转换成 topic 重映射、YAML 参数层和 NvbloxNode 加载动作。
from typing import List, Tuple

import isaac_ros_launch_utils as lu
from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME
from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxCamera, NvbloxMode


# 把最多前三个模拟相机名称映射为索引深度/彩色接口；按模式额外接入人像掩码与可选 LiDAR。
def get_isaac_sim_remappings(mode: NvbloxMode, num_cameras: int,
                             lidar: bool) -> List[Tuple[str, str]]:
    remappings = []
    # 这里的相机列表最多三项；ROS 节点自身声明最多四路，两者不是同一限制。
    camera_names = ['front_stereo_camera', 'left_stereo_camera',
                    'right_stereo_camera'][:num_cameras]
    for i, name in enumerate(camera_names):
        remappings.append((f'camera_{i}/depth/image', f'{name}/depth/ground_truth'))
        remappings.append((f'camera_{i}/depth/camera_info', f'{name}/left/camera_info'))
        remappings.append((f'camera_{i}/color/image', f'{name}/left/image_raw'))
        remappings.append((f'camera_{i}/color/camera_info', f'{name}/left/camera_info'))
    if mode is NvbloxMode.people_segmentation:
        remappings.append(
            ('camera_0/mask/image', '/semantic_conversion/front_stereo_camera/semantic_mono8'))
        remappings.append(('camera_0/mask/camera_info', '/front_stereo_camera/left/camera_info'))
    if lidar:
        remappings.append(('pointcloud', '/front_3d_lidar/point_cloud'))
    return remappings


# 为 RealSense 建立深度/彩色/掩码映射；仅 camera0 使用 splitter，其它相机直接使用深度 topic。
def get_realsense_remappings(mode: NvbloxMode, num_cameras: int = 1) -> List[Tuple[str, str]]:
    # NOTE(xinjieyao, 04.09.2024): Current in this function we only support:
    # - On/off emitter flashing + realsense_splitter on camera_0 (front camera).
    # - (Optional) people segmentation on all cameras.
    # - (Optional) people detection on all cameras.

    remappings = []
    for i in range(0, num_cameras):
        # camera0 的 splitter 用于示例传感器流程；本函数只连接 topic，不实现红外/深度分流算法。
        if i == 0:
            # Only cam0 (i == 0) runs splitter.
            remappings.append(
                (f'camera_{i}/depth/image', f'/camera{i}/realsense_splitter_node/output/depth'))
            remappings.append((f'camera_{i}/depth/camera_info', f'/camera{i}/depth/camera_info'))
        else:
            remappings.append((f'camera_{i}/depth/image', f'/camera{i}/depth/image_rect_raw'))
            remappings.append((f'camera_{i}/depth/camera_info', f'/camera{i}/depth/camera_info'))

        # 分割模式的彩色与 mask 来自同一缩放后的分割输入，因此对应 CameraInfo 也使用缩放版本。
        if mode is NvbloxMode.people_segmentation:
            # nvblox takes resized images from semseg inputs
            remappings.append(
                (f'camera_{i}/color/image', f'/camera{i}/segmentation/image_resized'))
            remappings.append(
                (f'camera_{i}/color/camera_info', f'/camera{i}/segmentation/camera_info_resized'))
            remappings.append((f'camera_{i}/mask/image', f'/camera{i}/segmentation/people_mask'))
            remappings.append(
                (f'camera_{i}/mask/camera_info', f'/camera{i}/segmentation/camera_info_resized'))

        else:
            remappings.append((f'camera_{i}/color/image', f'/camera{i}/color/image_raw'))
            remappings.append((f'camera_{i}/color/camera_info', f'/camera{i}/color/camera_info'))

            if mode is NvbloxMode.people_detection:
                remappings.append((f'camera_{i}/mask/image', f'/camera{i}/detection/people_mask'))
                remappings.append(
                    (f'camera_{i}/mask/camera_info', f'/camera{i}/color/camera_info'))

    return remappings


# 只允许 ZED 静态模式，接入已注册深度、整流彩色及 pose topic；TF/topic 位姿选择仍由参数决定。
def get_zed_remappings(mode: NvbloxMode) -> List[Tuple[str, str]]:
    assert mode is NvbloxMode.static, 'Nvblox only supports static mode for ZED cameras.'
    remappings = []
    remappings.append(('camera_0/depth/image', '/zed/zed_node/depth/depth_registered'))
    remappings.append(('camera_0/depth/camera_info', '/zed/zed_node/depth/camera_info'))
    remappings.append(('camera_0/color/image', '/zed/zed_node/rgb/color/rect/image'))
    remappings.append(('camera_0/color/camera_info', '/zed/zed_node/rgb/camera_info'))
    remappings.append(('pose', '/zed/zed_node/pose'))
    return remappings


# 只重映射 Ouster OS1 PointCloud2 输入；LiDAR 模型和补偿参数由配置另行提供。
def get_os1_remappings() -> List[Tuple[str, str]]:
    remappings = []
    remappings.append(('pointcloud', '/os1_cloud_node/points'))
    return remappings


# 在 OpaqueFunction 执行时解析参数，选择模式与传感器配置，然后向指定容器加载 nvblox::NvbloxNode。
def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:

    # mode/camera 需对应枚举名称；将相机数量转成整数用于节点参数与 topic 生成。
    mode = NvbloxMode[args.mode]
    camera = NvbloxCamera[args.camera]
    num_cameras = int(args.num_cameras)
    use_lidar = lu.is_true(args.lidar)

    if camera == NvbloxCamera.realsense:
        assert args.num_cameras == 1, 'NvbloxCamera.realsense shall only be set for num_cameras==1'

    # 按基础配置→模式特化→相机特化→显式参数覆盖的顺序组装，后面的同名参数优先。
    base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml')
    segmentation_config = lu.get_path('nvblox_examples_bringup',
                                      'config/nvblox/specializations/nvblox_segmentation.yaml')
    detection_config = lu.get_path('nvblox_examples_bringup',
                                   'config/nvblox/specializations/nvblox_detection.yaml')
    dynamics_config = lu.get_path('nvblox_examples_bringup',
                                  'config/nvblox/specializations/nvblox_dynamics.yaml')
    isaac_sim_config = lu.get_path('nvblox_examples_bringup',
                                   'config/nvblox/specializations/nvblox_sim.yaml')
    realsense_config = lu.get_path('nvblox_examples_bringup',
                                   'config/nvblox/specializations/nvblox_realsense.yaml')
    multi_realsense_config = lu.get_path(
        'nvblox_examples_bringup', 'config/nvblox/specializations/nvblox_multi_realsense.yaml')
    zed_config = lu.get_path('nvblox_examples_bringup',
                             'config/nvblox/specializations/nvblox_zed.yaml')
    os1_config = lu.get_path('nvblox_examples_bringup',
                             'config/nvblox/specializations/nvblox_os1.yaml')

    # 静态模式不额外加入模式 YAML；人像分割/检测禁止此示例中的 LiDAR 输入。
    if mode is NvbloxMode.static:
        mode_config = {}
    elif mode is NvbloxMode.people_segmentation:
        mode_config = segmentation_config
        assert not use_lidar, 'Can not run lidar with people segmentation mode.'
    elif mode is NvbloxMode.people_detection:
        mode_config = detection_config
        assert not use_lidar, 'Can not run lidar with people detection mode.'
    elif mode is NvbloxMode.dynamic:
        mode_config = dynamics_config
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')

    # 各相机示例有独立约束；这里只声明连接和 assert，不能据此认为所有组合已可运行。
    if camera is NvbloxCamera.isaac_sim:
        remappings = get_isaac_sim_remappings(mode, num_cameras, use_lidar)
        camera_config = isaac_sim_config
        assert num_cameras <= 1 or mode is not NvbloxMode.people_segmentation, \
            'Can not run multiple cameras with people segmentation in Isaac Sim.'
    elif camera is NvbloxCamera.realsense:
        remappings = get_realsense_remappings(mode, num_cameras)
        camera_config = realsense_config
        assert not use_lidar, 'Can not run lidar for realsense example.'
    elif camera is NvbloxCamera.multi_realsense:
        remappings = get_realsense_remappings(mode, num_cameras)
        camera_config = multi_realsense_config
        assert not use_lidar, 'Can not run lidar for multi realsense example.'
    elif camera in [NvbloxCamera.zed2, NvbloxCamera.zedx]:
        remappings = get_zed_remappings(mode)
        camera_config = zed_config
        assert num_cameras == 1, 'Zed example can only run with 1 camera.'
        assert not use_lidar, 'Can not run lidar for zed example.'
    elif camera is NvbloxCamera.os1:
        remappings = get_os1_remappings()
        camera_config = os1_config
        assert use_lidar, 'Must run lidar for os1 example.'
    else:
        raise Exception(f'Camera {camera} not implemented for nvblox.')

    # 保留配置覆盖顺序；num_cameras/use_lidar 和可选运动补偿开关在 YAML 之后追加。
    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(camera_config)
    parameters.append({'num_cameras': num_cameras})
    parameters.append({'use_lidar': use_lidar})

    # Add optional parameter overrides (appended last to override YAML configs)
    if args.use_lidar_motion_compensation != '':
        parameters.append(
            {'use_lidar_motion_compensation': lu.is_true(args.use_lidar_motion_compensation)})

    # Add the nvblox node.
    # 实际建图组件来自 nvblox_ros，isaac_ros_nvblox 主要是聚合入口包。
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    # 按 run_standalone 决定是否创建容器，再请求把组件加载到 container_name。
    actions = []
    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info(
            ["Starting nvblox with the '",
             str(camera), "' camera in '",
             str(mode), "' mode."]))
    return actions


# 声明模式、相机、数量、LiDAR 和容器入口；OpaqueFunction 使参数在上下文中解析后再生成组件。
def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode')
    args.add_arg('camera')
    args.add_arg('num_cameras', 1)
    args.add_arg('lidar', 'False')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_arg(
        'use_lidar_motion_compensation',
        '',
        description='Enable lidar motion compensation (empty string means use config default).')

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())
