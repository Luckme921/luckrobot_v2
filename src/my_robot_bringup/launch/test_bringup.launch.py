import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
            'depth_height': '480',
            'depth_fps': '15',
            'depth_registration': 'false'
        }.items()
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
    box_filter_yaml = os.path.expanduser('~/ros2_ws/src/nav_config/box_filter.yaml')
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[box_filter_yaml],
        output='screen'
    )

    # 3.5 相机深度图转伪激光雷达节点
    depth_to_laser_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            # 🌟 核心修改 1：不直接发给 Nav2，而是输出为 raw 原始数据
            ('scan', '/scan_camera_raw') 
        ],
        parameters=[{
            'scan_height': 10,
            'range_min': 0.15,
            'range_max': 3.0,
            'output_frame': 'camera_depth_frame'
        }],
        output='screen'
    )

    # 3.6 🌟 新增：相机的孤立点雷达滤波器 (剔除地面高反光噪点)
    camera_filter_yaml = os.path.expanduser('~/ros2_ws/src/nav_config/camera_filter.yaml')
    camera_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='camera_scan_filter_chain',
        parameters=[camera_filter_yaml],
        remappings=[
            # 🌟 核心修改 2：接收 raw 数据，过滤后发给正式的 /scan_camera 话题
            ('scan', '/scan_camera_raw'),      
            ('scan_filtered', '/scan_camera')  
        ],
        output='screen'
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
        robot_description_launch,
        rplidar_launch,
        camera_launch,
        rviz_node,
        chassis_node,
        imu_node,
        laser_filter_node,
        depth_to_laser_node,
        camera_filter_node, # 🌟 挂载相机专属强力去噪点滤波器
        ekf_node
    ])