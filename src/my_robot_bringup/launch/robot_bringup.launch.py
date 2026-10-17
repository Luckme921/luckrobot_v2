import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
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

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel',
        description='底盘最终接收的速度话题；启用碰撞监控时使用 /cmd_vel_safe'
    )
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    desc_pkg_path = get_package_share_directory('my_robot_description')
    bringup_pkg_path = get_package_share_directory('my_robot_bringup')
    nav2_pkg_path = get_package_share_directory('my_robot_navigation')

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

    # 2.3 启动 Gemini 相机
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
            'depth_height': '400',
            'depth_fps': '15',
            'depth_registration': 'false',
            'device_preset': 'AMR with IR-Pass'
        }.items()
    )

    # ==========================================
    # 3. 启动所有独立节点 (Nodes)
    # ==========================================

    rviz_config_path = os.path.join(bringup_pkg_path, 'rviz_config', 'rviz2_config1.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    chassis_node = Node(
        package='chassis_driver',
        executable='chassis_node',
        name='chassis_driver_node',
        remappings=[('cmd_vel', cmd_vel_topic)],
        output='screen'
    )

    imu_node = Node(
        package='imu_driver',
        executable='imu_driver_node',
        name='imu_driver_node',
        output='screen'
    )

    # 3.4 主雷达盒子滤波 (剔除车身内部)
    box_filter_yaml = os.path.join(bringup_pkg_path, 'config', 'box_filter.yaml')
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        # Keep the executable's default name. LaserScanBoxFilter owns an
        # internal helper node and Humble applies an explicit __node remap to
        # both, otherwise creating duplicate /scan_to_scan_filter_chain names.
        parameters=[box_filter_yaml],
        output='screen'
    )

    # 3.5 相机深度图转伪激光雷达节点
    # 🌟 修复流向：将原本直接给 Nav2 的 /scan_camera 改为发给中间商 /scan_camera_raw
    depth_to_laser_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan_camera_raw') # 🌟 输出原始未滤波数据
        ],
        parameters=[{
            # The camera is now ~0.37 m above the floor.  A 10-row slice can
            # miss a desk edge only a few centimetres above the optical
            # centre; 60 rows cover that height without reaching the floor
            # within the 3 m range limit for the calibrated horizontal mount.
            'scan_height': 60,
            'range_min': 0.15,
            'range_max': 3.0,
            'output_frame': 'camera_depth_frame'
        }],
        output='screen'
    )

    # 3.6 🌟 新增：相机的孤立点雷达滤波器 (剔除地面高反光噪点)
    camera_filter_yaml = os.path.join(bringup_pkg_path, 'config', 'camera_filter.yaml')
    camera_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='camera_scan_filter_chain',
        parameters=[camera_filter_yaml],
        remappings=[
            ('scan', '/scan_camera_raw'),      # 🌟 接收原始数据
            ('scan_filtered', '/scan_camera')  # 🌟 输出过滤后、极其干净的数据给 Nav2！
        ],
        output='screen'
    )

    ekf_config_path = os.path.join(bringup_pkg_path, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_path, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'log_level': 'info'
        }.items()
    )
    
    delayed_nav2_launch = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        use_rviz_arg,
        cmd_vel_topic_arg,
        robot_description_launch,
        rplidar_launch,
        camera_launch,
        rviz_node,
        chassis_node,
        imu_node,
        laser_filter_node,
        depth_to_laser_node,
        camera_filter_node, # 🌟 挂载相机专属滤波器
        ekf_node,
        delayed_nav2_launch
    ])
