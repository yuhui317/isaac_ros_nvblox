# isaac_ros_nvblox · 完整代码树

[初步分析](analysis.md) · [源码定位](code_symbols.md) · [范围](annotation_scope.json) · [验证](annotation_validation.json)

固定基线 `edf0484edbf3d6380f739b23a4cf1c7ba6d6e088`，来源 `release-5.0`，学习分支 `codex/study/isaac-ros-code-analysis`。

240 个基线文件、1 个 gitlink；本轮注释 8 个原始文件。树不包含 `.git` 或缓存，固定基线自带的资源仍完整列出。

```text
isaac_ros_nvblox/
├── .clang-format
├── .gitattributes
├── .gitignore
├── .gitmodules  [只读参考；未注释]
├── .style.yapf
├── CONTRIBUTING.md
├── LICENSE
├── README.md  [只读参考；未注释]
├── SECURITY.md
├── isaac_ros_nvblox/
│   ├── CMakeLists.txt
│   └── package.xml
├── nvblox_examples/
│   ├── multi_realsense_emitter_synchronizer/
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── multi_realsense_emitter_synchronizer/
│   │   │       ├── emitter_synchronizer.hpp  [原样保留；未展开]
│   │   │       └── multi_realsense_emitter_synchronizer_node.hpp  [原样保留；未展开]
│   │   ├── package.xml
│   │   ├── src/
│   │   │   ├── emitter_synchronizer.cpp  [原样保留；未展开]
│   │   │   ├── multi_realsense_emitter_synchronizer_main.cpp  [原样保留；未展开]
│   │   │   └── multi_realsense_emitter_synchronizer_node.cpp  [原样保留；未展开]
│   │   └── test/
│   │       └── test_emitter_synchronizer.cpp  [原样保留；未展开]
│   ├── nvblox_examples_bringup/
│   │   ├── CMakeLists.txt
│   │   ├── config/
│   │   │   ├── navigation/
│   │   │   │   └── carter_nav2.yaml
│   │   │   ├── nvblox/
│   │   │   │   ├── fuser.yaml
│   │   │   │   ├── nvblox_base.yaml  [只读参考；未注释]
│   │   │   │   └── specializations/
│   │   │   │       ├── nvblox_detection.yaml
│   │   │   │       ├── nvblox_dynamics.yaml
│   │   │   │       ├── nvblox_multi_realsense.yaml  [只读参考；未注释]
│   │   │   │       ├── nvblox_os1.yaml
│   │   │   │       ├── nvblox_realsense.yaml  [只读参考；未注释]
│   │   │   │       ├── nvblox_segmentation.yaml
│   │   │   │       ├── nvblox_sim.yaml
│   │   │   │       └── nvblox_zed.yaml
│   │   │   ├── sensors/
│   │   │   │   ├── realsense_emitter_flashing.yaml
│   │   │   │   ├── realsense_emitter_on.yaml
│   │   │   │   ├── zed2.yaml
│   │   │   │   ├── zed_common.yaml
│   │   │   │   └── zedx.yaml
│   │   │   ├── urdf/
│   │   │   │   └── 4_realsense_carter_example_calibration.urdf.xacro
│   │   │   └── visualization/
│   │   │       ├── fuser_example.rviz
│   │   │       ├── isaac_sim_dynamics_example.rviz
│   │   │       ├── isaac_sim_example.rviz
│   │   │       ├── isaac_sim_people_segmentation_example.rviz
│   │   │       ├── multi_realsense_dynamics_example.rviz
│   │   │       ├── multi_realsense_people_detection_example.rviz
│   │   │       ├── multi_realsense_people_segmentation_example.rviz
│   │   │       ├── os1_dynamics_example.rviz
│   │   │       ├── os1_example.rviz
│   │   │       ├── realsense_dynamics_example.rviz
│   │   │       ├── realsense_example.rviz
│   │   │       ├── realsense_people_detection_example.rviz
│   │   │       ├── realsense_people_segmentation_example.rviz
│   │   │       └── zed_example.rviz
│   │   ├── launch/
│   │   │   ├── fuser_example.launch.py  [原样保留；未展开]
│   │   │   ├── isaac_sim_example.launch.py  [原样保留；未展开]
│   │   │   ├── navigation/
│   │   │   │   └── nvblox_carter_navigation.launch.py  [原样保留；未展开]
│   │   │   ├── os1_example.launch.py  [原样保留；未展开]
│   │   │   ├── perception/
│   │   │   │   ├── detection.launch.py  [原样保留；未展开]
│   │   │   │   ├── nvblox.launch.py  [本轮中文注释]
│   │   │   │   ├── segmentation.launch.py  [原样保留；未展开]
│   │   │   │   └── vslam.launch.py  [本轮中文注释]
│   │   │   ├── realsense_example.launch.py  [原样保留；未展开]
│   │   │   ├── sensors/
│   │   │   │   ├── realsense.launch.py  [原样保留；未展开]
│   │   │   │   ├── record_realsense.launch.py  [原样保留；未展开]
│   │   │   │   └── zed.launch.py  [原样保留；未展开]
│   │   │   ├── visualization/
│   │   │   │   ├── foxglove_bridge.launch.py  [原样保留；未展开]
│   │   │   │   ├── rviz.launch.py  [原样保留；未展开]
│   │   │   │   └── visualization.launch.py  [原样保留；未展开]
│   │   │   └── zed_example.launch.py  [原样保留；未展开]
│   │   ├── package.xml
│   │   └── test/
│   │       └── CMakeLists.txt
│   ├── nvblox_image_padding/
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── nvblox_image_padding/
│   │   │       └── image_padding_cropping_node.hpp  [原样保留；未展开]
│   │   ├── package.xml
│   │   └── src/
│   │       ├── image_padding_cropping_node.cpp  [原样保留；未展开]
│   │       └── image_padding_cropping_node_main.cpp  [原样保留；未展开]
│   ├── realsense_splitter/
│   │   ├── CMakeLists.txt
│   │   ├── COLCON_IGNORE
│   │   ├── config/
│   │   │   └── .gitkeep
│   │   ├── include/
│   │   │   └── realsense_splitter/
│   │   │       ├── qos.hpp  [原样保留；未展开]
│   │   │       └── realsense_splitter_node.hpp  [原样保留；未展开]
│   │   ├── launch/
│   │   │   └── realsense_splitter.launch.py  [原样保留；未展开]
│   │   ├── package.xml
│   │   └── src/
│   │       ├── realsense_splitter_node.cpp  [原样保留；未展开]
│   │       └── realsense_splitter_node_main.cpp  [原样保留；未展开]
│   └── semantic_label_conversion/
│       ├── CMakeLists.txt
│       ├── README.md
│       ├── launch/
│       │   └── semantic_label_conversion.launch.py  [原样保留；未展开]
│       ├── package.xml
│       ├── params/
│       │   └── semantic_label_conversion.yaml
│       └── semantic_label_conversion/
│           ├── __init__.py  [原样保留；未展开]
│           ├── semantic_label_converter.py  [原样保留；未展开]
│           └── semantic_label_stamper.py  [原样保留；未展开]
├── nvblox_foxglove/
│   ├── .eslintrc.yaml
│   ├── .npmignore
│   ├── .prettierrc.yaml
│   ├── CHANGELOG.md
│   ├── README.md
│   ├── package-lock.json
│   ├── package.json
│   ├── src/
│   │   └── index.ts
│   └── tsconfig.json
├── nvblox_message_adapters/
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── nvblox_message_adapters/
│   │       ├── nvblox_mesh_layer_adapter_node.hpp  [原样保留；未展开]
│   │       └── nvblox_voxel_layer_adapter_node.hpp  [原样保留；未展开]
│   ├── launch/
│   │   ├── mesh_serializer.launch.py  [原样保留；未展开]
│   │   └── voxel_serializer.launch.py  [原样保留；未展开]
│   ├── package.xml
│   └── src/
│       ├── nvblox_mesh_layer_adapter_main.cpp  [原样保留；未展开]
│       ├── nvblox_mesh_layer_adapter_node.cpp  [原样保留；未展开]
│       ├── nvblox_voxel_layer_adapter_main.cpp  [原样保留；未展开]
│       └── nvblox_voxel_layer_adapter_node.cpp  [原样保留；未展开]
├── nvblox_msgs/
│   ├── CMakeLists.txt
│   ├── msg/
│   │   ├── DistanceMapSlice.msg  [只读参考；未注释]
│   │   ├── Index3D.msg
│   │   ├── Mesh.msg
│   │   ├── MeshBlock.msg
│   │   ├── MeshSerialized.msg
│   │   ├── SemanticLabelsStamped.msg
│   │   ├── VoxelBlock.msg
│   │   ├── VoxelBlockLayer.msg
│   │   └── VoxelSerialized.msg
│   ├── package.xml
│   └── srv/
│       ├── EsdfAndGradients.srv  [只读参考；未注释]
│       └── FilePath.srv
├── nvblox_nav2/
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── nvblox_nav2/
│   │       └── nvblox_costmap_layer.hpp  [原样保留；未展开]
│   ├── nvblox_costmap_layer.xml
│   ├── package.xml
│   └── src/
│       └── nvblox_costmap_layer.cpp  [原样保留；未展开]
├── nvblox_ros/
│   ├── CMakeLists.txt  [只读参考；未注释]
│   ├── include/
│   │   └── nvblox_ros/
│   │       ├── camera_cache.hpp  [原样保留；未展开]
│   │       ├── conversions/
│   │       │   ├── esdf_and_gradients_conversions.hpp  [原样保留；未展开]
│   │       │   ├── esdf_slice_conversions.hpp  [原样保留；未展开]
│   │       │   ├── image_conversions.hpp  [原样保留；未展开]
│   │       │   ├── image_conversions_thrust.hpp  [原样保留；未展开]
│   │       │   ├── mesh_conversions.hpp  [原样保留；未展开]
│   │       │   ├── pointcloud_conversions.hpp  [原样保留；未展开]
│   │       │   └── transform_conversions.hpp  [原样保留；未展开]
│   │       ├── dataset_types.hpp  [原样保留；未展开]
│   │       ├── fuser_node.hpp  [原样保留；未展开]
│   │       ├── impl/
│   │       │   ├── mapper_initialization_impl.hpp  [原样保留；未展开]
│   │       │   ├── nvblox_node_impl.hpp  [本轮中文注释]
│   │       │   ├── rosbag_reading_impl.hpp  [原样保留；未展开]
│   │       │   └── service_request_task_impl.hpp  [原样保留；未展开]
│   │       ├── layer_publishing.hpp  [原样保留；未展开]
│   │       ├── mapper_initialization.hpp  [原样保留；未展开]
│   │       ├── node_params.hpp  [只读参考；未注释]
│   │       ├── nvblox_node.hpp  [本轮中文注释]
│   │       ├── rosbag_data_loader.hpp  [原样保留；未展开]
│   │       ├── rosbag_reading.hpp  [原样保留；未展开]
│   │       ├── service_request_task.hpp  [原样保留；未展开]
│   │       ├── terminal_reading.hpp  [原样保留；未展开]
│   │       ├── transformer.hpp  [本轮中文注释]
│   │       ├── utils.hpp  [原样保留；未展开]
│   │       └── visualization.hpp  [原样保留；未展开]
│   ├── nvblox_core  [外部 submodule @ 24eee4948768682fa1ffb969b881efee4fca29c2；内部未分析]
│   ├── package.xml  [只读参考；未注释]
│   ├── scripts/
│   │   ├── calculate_lidar_params.py  [原样保留；未展开]
│   │   ├── print_ros_params.sh  [原样保留；未展开]
│   │   └── run_nsys.sh  [原样保留；未展开]
│   ├── src/
│   │   ├── fuser_node_main.cpp  [原样保留；未展开]
│   │   ├── lib/
│   │   │   ├── camera_cache.cpp  [原样保留；未展开]
│   │   │   ├── conversions/
│   │   │   │   ├── esdf_and_gradients_conversions.cu  [原样保留；未展开]
│   │   │   │   ├── esdf_slice_conversions.cu  [原样保留；未展开]
│   │   │   │   ├── image_conversions.cpp  [本轮中文注释]
│   │   │   │   ├── image_conversions_thrust.cu  [只读参考；未注释]
│   │   │   │   ├── mesh_conversions.cpp  [原样保留；未展开]
│   │   │   │   ├── pointcloud_conversions.cu  [原样保留；未展开]
│   │   │   │   └── transform_conversions.cpp  [原样保留；未展开]
│   │   │   ├── fuser_node.cpp  [原样保留；未展开]
│   │   │   ├── layer_publishing.cpp  [原样保留；未展开]
│   │   │   ├── mapper_initialization.cpp  [原样保留；未展开]
│   │   │   ├── node_params.cpp  [只读参考；未注释]
│   │   │   ├── nvblox_node.cpp  [本轮中文注释]
│   │   │   ├── rosbag_data_loader.cpp  [原样保留；未展开]
│   │   │   ├── rosbag_reading.cpp  [原样保留；未展开]
│   │   │   ├── terminal_reading.cpp  [原样保留；未展开]
│   │   │   ├── transformer.cpp  [本轮中文注释]
│   │   │   ├── utils.cpp  [原样保留；未展开]
│   │   │   └── visualization.cpp  [原样保留；未展开]
│   │   └── nvblox_node_main.cpp  [原样保留；未展开]
│   └── test/
│       ├── integration_tests/
│       │   ├── CMakeLists.txt
│       │   ├── isaac_ros_nvblox_pol.py  [原样保留；未展开]
│       │   ├── isaac_ros_nvblox_srv_save_and_load_map.py  [原样保留；未展开]
│       │   ├── isaac_ros_nvblox_srv_save_ply.py  [原样保留；未展开]
│       │   ├── isaac_ros_nvblox_srv_save_timings_and_rates.py  [原样保留；未展开]
│       │   ├── test_cases/
│       │   │   └── rosbags/
│       │   │       └── nvblox_pol/
│       │   │           ├── metadata.yaml
│       │   │           └── rosbag2_2024_05_07-11_54_22_0.db3
│       │   └── test_utils/
│       │       └── __init__.py  [原样保留；未展开]
│       └── unit_tests/
│           ├── CMakeLists.txt
│           ├── test_esdf_and_gradient_conversions.cpp  [原样保留；未展开]
│           ├── test_node_params.cpp  [原样保留；未展开]
│           ├── test_rosbag_data_loader.cpp  [原样保留；未展开]
│           └── test_service_request_queue.cpp  [原样保留；未展开]
├── nvblox_ros_common/
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── nvblox_ros_common/
│   │       └── check_cuda_errors.hpp  [原样保留；未展开]
│   ├── package.xml
│   └── src/
│       └── check_cuda_errors.cpp  [原样保留；未展开]
├── nvblox_ros_python_utils/
│   ├── nvblox_ros_python_utils/
│   │   ├── __init__.py  [原样保留；未展开]
│   │   ├── nvblox_constants.py  [原样保留；未展开]
│   │   ├── nvblox_launch_test_utils.py  [原样保留；未展开]
│   │   └── nvblox_launch_utils.py  [只读参考；未注释]
│   ├── package.xml
│   ├── resource/
│   │   └── nvblox_ros_python_utils
│   ├── setup.cfg
│   ├── setup.py  [原样保留；未展开]
│   └── test/
│       ├── test_copyright.py  [原样保留；未展开]
│       ├── test_flake8.py  [原样保留；未展开]
│       └── test_pep257.py  [原样保留；未展开]
├── nvblox_rviz_plugin/
│   ├── CMakeLists.txt
│   ├── icons/
│   │   └── classes/
│   │       └── NvbloxMesh.png
│   ├── include/
│   │   └── nvblox_rviz_plugin/
│   │       ├── nvblox_hash_utils.hpp  [原样保留；未展开]
│   │       ├── nvblox_plugin_display.hpp  [原样保留；未展开]
│   │       └── nvblox_plugin_visual.hpp  [原样保留；未展开]
│   ├── materials/
│   │   ├── LICENSE.txt
│   │   ├── README.md
│   │   ├── glsl150/
│   │   │   ├── billboard.geom
│   │   │   ├── box.geom
│   │   │   ├── glsl150.program
│   │   │   └── pass_pos_color.vert
│   │   └── scripts150/
│   │       ├── point_cloud_box.material
│   │       ├── point_cloud_flat_square.material
│   │       ├── point_cloud_sphere.material
│   │       └── point_cloud_square.material
│   ├── package.xml
│   ├── plugins_description.xml
│   └── src/
│       ├── nvblox_plugin_display.cpp  [原样保留；未展开]
│       └── nvblox_plugin_visual.cpp  [原样保留；未展开]
├── nvblox_test/
│   ├── README.md
│   ├── nvblox_test/
│   │   └── __init__.py  [原样保留；未展开]
│   ├── package.xml
│   ├── pytest.ini
│   ├── setup.cfg
│   ├── setup.py  [原样保留；未展开]
│   └── test/
│       ├── __init__.py  [原样保留；未展开]
│       ├── isaac_sim_example_test.py  [原样保留；未展开]
│       ├── realsense_example_dynamics_test.py  [原样保留；未展开]
│       └── realsense_example_test.py  [原样保留；未展开]
├── nvblox_test_data/
│   ├── CMakeLists.txt
│   ├── asset_scripts/
│   │   └── quickstart.sh  [原样保留；未展开]
│   └── package.xml
├── resources/
│   └── nvblox_performance_setup.png
└── study/
    ├── analysis.md  [新增学习材料]
    ├── annotation_scope.json  [新增学习材料]
    ├── annotation_validation.json  [新增学习材料]
    ├── code_symbols.md  [新增学习材料]
    ├── code_tree.md  [新增学习材料]
    ├── generate_study_index.py  [新增学习材料]
    ├── repository.json  [新增学习材料]
    └── verify_annotations.py  [新增学习材料]
```

## 本轮阅读入口

| 文件 | 范围 | 阅读深度 |
|---|---|---|
| [.gitmodules](../.gitmodules) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [README.md](../README.md) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml](../nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_examples/nvblox_examples_bringup/config/nvblox/specializations/nvblox_multi_realsense.yaml](../nvblox_examples/nvblox_examples_bringup/config/nvblox/specializations/nvblox_multi_realsense.yaml) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_examples/nvblox_examples_bringup/config/nvblox/specializations/nvblox_realsense.yaml](../nvblox_examples/nvblox_examples_bringup/config/nvblox/specializations/nvblox_realsense.yaml) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_examples/nvblox_examples_bringup/launch/perception/nvblox.launch.py](../nvblox_examples/nvblox_examples_bringup/launch/perception/nvblox.launch.py) | 中文注释 | full |
| [nvblox_examples/nvblox_examples_bringup/launch/perception/vslam.launch.py](../nvblox_examples/nvblox_examples_bringup/launch/perception/vslam.launch.py) | 中文注释 | full |
| [nvblox_msgs/msg/DistanceMapSlice.msg](../nvblox_msgs/msg/DistanceMapSlice.msg) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_msgs/srv/EsdfAndGradients.srv](../nvblox_msgs/srv/EsdfAndGradients.srv) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_ros/CMakeLists.txt](../nvblox_ros/CMakeLists.txt) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_ros/include/nvblox_ros/impl/nvblox_node_impl.hpp](../nvblox_ros/include/nvblox_ros/impl/nvblox_node_impl.hpp) | 中文注释 | full |
| [nvblox_ros/include/nvblox_ros/node_params.hpp](../nvblox_ros/include/nvblox_ros/node_params.hpp) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_ros/include/nvblox_ros/nvblox_node.hpp](../nvblox_ros/include/nvblox_ros/nvblox_node.hpp) | 中文注释 | full |
| [nvblox_ros/include/nvblox_ros/transformer.hpp](../nvblox_ros/include/nvblox_ros/transformer.hpp) | 中文注释 | full |
| [nvblox_ros/package.xml](../nvblox_ros/package.xml) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_ros/src/lib/conversions/image_conversions.cpp](../nvblox_ros/src/lib/conversions/image_conversions.cpp) | 中文注释 | full |
| [nvblox_ros/src/lib/conversions/image_conversions_thrust.cu](../nvblox_ros/src/lib/conversions/image_conversions_thrust.cu) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_ros/src/lib/node_params.cpp](../nvblox_ros/src/lib/node_params.cpp) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [nvblox_ros/src/lib/nvblox_node.cpp](../nvblox_ros/src/lib/nvblox_node.cpp) | 中文注释 | full |
| [nvblox_ros/src/lib/transformer.cpp](../nvblox_ros/src/lib/transformer.cpp) | 中文注释 | full |
| [nvblox_ros_python_utils/nvblox_ros_python_utils/nvblox_launch_utils.py](../nvblox_ros_python_utils/nvblox_ros_python_utils/nvblox_launch_utils.py) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |

目录树和行号是定位材料，不代表所有代码已理解或运行。子模块只保留固定引用，内部文件不在本轮范围。

重新生成：`python study/generate_study_index.py`。再运行 `python study/verify_annotations.py --report study/annotation_validation.json` 审查源码保留情况；两个工具均不运行上游项目。
