import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ==========================================
    # 1. 声明总控参数 (默认开启本包下的定制 RViz)
    # ==========================================
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz2', default_value='true',
        description='是否启动总控定制版 RViz2'
    )
    use_rviz = LaunchConfiguration('use_rviz2')

    enable_stereo_ir_arg = DeclareLaunchArgument(
        'enable_stereo_ir', default_value='false',
        description=(
            '是否同时开启 Gemini 左右 IR 640x400@30Hz；仅供 cuVSLAM 旁路观察，'
            '默认关闭以保持当前 Nav2 资源占用不变'
        )
    )
    enable_stereo_ir = LaunchConfiguration('enable_stereo_ir')

    vslam_camera_profile_arg = DeclareLaunchArgument(
        'vslam_camera_profile',
        default_value='navigation',
        choices=['navigation', 'passive', 'interleaved'],
        description=(
            'Gemini 发射器档位：navigation=保持原导航激光深度；'
            'passive=关闭激光做双目排障；interleaved=激光开/关交替，'
            '深度仅保留激光帧、左右 IR 仅保留无激光帧'
        )
    )
    vslam_camera_profile = LaunchConfiguration('vslam_camera_profile')

    depth_scan_height_arg = DeclareLaunchArgument(
        'depth_scan_height', default_value='60',
        description=(
            '旧版中央行深度伪激光的垂直采样行数；仅在 legacy/shadow 模式使用。'
        )
    )
    depth_scan_height = LaunchConfiguration('depth_scan_height')

    depth_obstacle_mode_arg = DeclareLaunchArgument(
        'depth_obstacle_mode', default_value='legacy',
        choices=['legacy', 'shadow', 'height'],
        description=(
            '深度避障模式：legacy=当前中央60行；shadow=保持旧安全链并旁路发布'
            '/scan_camera_height；height=高度感知扫描正式接管 /scan_camera。'
        )
    )
    depth_obstacle_mode = LaunchConfiguration('depth_obstacle_mode')
    depth_min_height_arg = DeclareLaunchArgument(
        'depth_min_obstacle_height', default_value='0.12',
        description=(
            '相对 base_footprint 的最低深度障碍高度；默认略高于约0.11m的'
            '2D雷达扫描平面，由雷达覆盖与该平面相交的低位障碍。'
        )
    )
    depth_max_height_arg = DeclareLaunchArgument(
        'depth_max_obstacle_height', default_value='0.50',
        description='相对 base_footprint 的最高碰撞包络，覆盖相机上方约12 cm结构。'
    )
    depth_min_height = LaunchConfiguration('depth_min_obstacle_height')
    depth_max_height = LaunchConfiguration('depth_max_obstacle_height')

    # Gemini 336L rejects a mixed depth/IR profile.  The normal Nav2 profile
    # keeps the original 15 Hz depth stream; enabling the passive stereo pair
    # raises depth to the same 30 Hz as both IR streams so the camera driver
    # does not reset the complete pipeline.
    stereo_depth_fps = PythonExpression([
        "'30' if '", enable_stereo_ir, "'.lower() == 'true' else '15'"
    ])
    laser_enabled = PythonExpression([
        "'false' if '", vslam_camera_profile,
        "'.lower() == 'passive' else 'true'"
    ])
    interleave_enabled = PythonExpression([
        "'true' if '", vslam_camera_profile,
        "'.lower() == 'interleaved' else 'false'"
    ])

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel',
        description='底盘最终接收的速度话题；启用碰撞监控时使用 /cmd_vel_safe'
    )
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    desc_pkg_path = get_package_share_directory('my_robot_description')
    bringup_pkg_path = get_package_share_directory('my_robot_bringup')

    # ==========================================
    # 2. 导入第三方与底层 Launch 文件
    # ==========================================
    
    # 2.1 启动机器人状态发布 
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path, 'launch', 'display_robot.launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # 2.2 启动主雷达 A2M12
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m12_launch.py')
        )
    )

    # 2.3 启动 Gemini 相机 (极简模式)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'enable_point_cloud': 'false',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '15',
            'depth_width': '640',
            # Gemini 336L 内置 AMR 预设的推荐深度模式。
            # 实机验证比 640x480 + Default 的近场有效点更稳定。
            'depth_height': '400',
            'depth_fps': stereo_depth_fps,
            'depth_registration': 'false',
            'device_preset': 'AMR with IR-Pass',
            # The normal navigation profile keeps IR disabled.  The unified
            # system launch can enable the passive stereo pair without
            # starting a second camera process.
            'enable_left_ir': PythonExpression([
                "'", enable_stereo_ir, "'.lower() == 'true'"
            ]),
            'left_ir_width': '640',
            'left_ir_height': '400',
            'left_ir_fps': '30',
            'left_ir_format': 'ANY',
            'left_ir_qos': 'SENSOR_DATA',
            'enable_right_ir': PythonExpression([
                "'", enable_stereo_ir, "'.lower() == 'true'"
            ]),
            'right_ir_width': '640',
            'right_ir_height': '400',
            'right_ir_fps': '30',
            'right_ir_format': 'ANY',
            'right_ir_qos': 'SENSOR_DATA',
            # cuVSLAM cannot reliably track a projector pattern fixed to the
            # moving camera.  In the interleaved profile sequence 0 is the
            # laser/pattern frame and sequence 1 is the laser-off frame.
            # Depth keeps sequence 0 for obstacle sensing while both stereo
            # images keep sequence 1 for visual odometry.
            'enable_laser': laser_enabled,
            'interleave_ae_mode': 'laser',
            'interleave_frame_enable': interleave_enabled,
            'interleave_skip_enable': 'false',
            'enable_sequence_id_filter': interleave_enabled,
            'sequence_id_filter_id': '0',
            'enable_left_ir_sequence_id_filter': interleave_enabled,
            'left_ir_sequence_id_filter_id': '1',
            'enable_right_ir_sequence_id_filter': interleave_enabled,
            'right_ir_sequence_id_filter_id': '1',
            'show_fps_enable': 'false',
        }.items()
    )

    # Orbbec Laser On-Off interleave can keep publishing at the base 30 Hz
    # while repeating the previous filtered image stamp on alternate frames.
    # cuVSLAM requires strictly increasing synchronized stereo timestamps, so
    # gate all stereo profiles through one deterministic pair filter. Normal
    # passive streams pass at 30 Hz; interleaved streams become clean 15 Hz.
    vslam_stereo_gate = Node(
        package='my_robot_bringup',
        executable='vslam_stereo_monotonic_gate',
        name='vslam_stereo_monotonic_gate',
        output='screen',
        condition=IfCondition(enable_stereo_ir),
    )

    # ==========================================
    # 3. 启动所有独立节点 (Nodes)
    # ==========================================

    # 3.1 启动总控定制版 RViz2
    rviz_config_path = os.path.join(bringup_pkg_path, 'rviz_config', 'rviz2_config1.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    # 3.2 底盘驱动节点 (提供纯轮式 Odom)
    chassis_node = Node(
        package='chassis_driver',
        executable='chassis_node',
        name='chassis_driver_node',
        remappings=[('cmd_vel', cmd_vel_topic)],
        output='screen'
    )

    # 3.3 IMU 驱动节点
    imu_node = Node(
        package='imu_driver',
        executable='imu_driver_node',
        name='imu_driver_node',
        output='screen'
    )

    # 3.4 雷达盒子滤波节点 (剔除车身内部无效点云)
    box_filter_yaml = os.path.join(bringup_pkg_path, 'config', 'box_filter.yaml')
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        # Do not pass an explicit __node remap here. Humble's LaserScanBoxFilter
        # creates an internal helper node; a process-wide name remap renames
        # both objects to scan_to_scan_filter_chain and makes ros2 node list
        # report a false duplicate. The executable's default main-node name
        # already matches the box_filter.yaml root key.
        parameters=[box_filter_yaml],
        output='screen'
    )

    # 3.5 相机深度图转伪激光雷达节点
    depth_to_laser_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        condition=IfCondition(PythonExpression([
            "'", depth_obstacle_mode, "' != 'height'"
        ])),
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            # 🌟 核心修改 1：不直接发给 Nav2，而是输出为 raw 原始数据
            ('scan', '/scan_camera_raw') 
        ],
        parameters=[{
            'scan_height': ParameterValue(depth_scan_height, value_type=int),
            'range_min': 0.15,
            'range_max': 3.0,
            'output_frame': 'camera_depth_frame'
        }],
        output='screen'
    )

    # Height-aware projector. In shadow mode it publishes an independent
    # candidate scan for stationary A/B validation. In height mode it feeds
    # the existing filter and safety chain without changing topic contracts.
    depth_height_scan_node = Node(
        package='my_robot_bringup',
        executable='depth_height_scan',
        name='depth_height_scan',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", depth_obstacle_mode, "' != 'legacy'"
        ])),
        parameters=[{
            'output_topic': ParameterValue(
                PythonExpression([
                    "'/scan_camera_raw' if '", depth_obstacle_mode,
                    "' == 'height' else '/scan_camera_height_raw'"
                ]),
                value_type=str,
            ),
            'target_frame': 'base_footprint',
            'min_obstacle_height': ParameterValue(
                depth_min_height, value_type=float
            ),
            'max_obstacle_height': ParameterValue(
                depth_max_height, value_type=float
            ),
            'depth_min': 0.15,
            'depth_max': 3.0,
            'angle_min': -1.0,
            'angle_max': 1.0,
            'beam_count': 641,
            'row_stride': 2,
            'column_stride': 1,
            # Empty-scene artifact measured four same-depth samples, while
            # the real plate measured about 48; five cleanly separates them.
            'min_points_per_bin': 5,
            'support_range_tolerance': 0.03,
        }],
    )

    # Dual-channel camera safety. The range-clustered stream goes straight to
    # Collision Monitor. Costmaps receive only world-frame, multi-frame,
    # range-continuous returns so their 6/16 s memory cannot amplify speckle.
    camera_safety_filter_yaml = os.path.join(
        bringup_pkg_path, 'config', 'camera_safety_filter.yaml'
    )
    camera_safety_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='camera_scan_safety_filter_chain',
        parameters=[camera_safety_filter_yaml],
        remappings=[
            ('scan', '/scan_camera_raw'),
            ('scan_filtered', '/scan_camera_safety'),
        ],
        output='screen'
    )

    camera_confirmation_node = Node(
        package='my_robot_bringup',
        executable='scan_confirmation_filter',
        name='camera_scan_confirmation',
        output='screen',
        parameters=[{
            'input_topic': '/scan_camera_safety',
            'output_topic': '/scan_camera',
            'fixed_frame': 'odom',
            'confirmation_frames': 5,
            'history_duration': 0.50,
            'grid_resolution': 0.04,
            'neighbor_radius_cells': 1,
            'min_cluster_beams': 7,
            'cluster_range_tolerance': 0.08,
        }],
    )

    camera_height_filter_yaml = os.path.join(
        bringup_pkg_path, 'config', 'camera_height_filter.yaml'
    )
    camera_height_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='camera_height_scan_filter_chain',
        parameters=[camera_height_filter_yaml],
        remappings=[
            ('scan', '/scan_camera_height_raw'),
            ('scan_filtered', '/scan_camera_height'),
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", depth_obstacle_mode, "' == 'shadow'"
        ])),
    )

    # 3.7 核心 EKF 里程计融合节点
    ekf_config_path = os.path.join(bringup_pkg_path, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # ==========================================
    # 4. 返回完整启动描述符 (无 Nav2 版)
    # ==========================================
    return LaunchDescription([
        use_rviz_arg,
        enable_stereo_ir_arg,
        vslam_camera_profile_arg,
        depth_scan_height_arg,
        depth_obstacle_mode_arg,
        depth_min_height_arg,
        depth_max_height_arg,
        cmd_vel_topic_arg,
        robot_description_launch,
        rplidar_launch,
        camera_launch,
        vslam_stereo_gate,
        rviz_node,
        chassis_node,
        imu_node,
        laser_filter_node,
        depth_to_laser_node,
        depth_height_scan_node,
        camera_safety_filter_node,
        camera_confirmation_node,
        camera_height_filter_node,
        ekf_node
    ])
