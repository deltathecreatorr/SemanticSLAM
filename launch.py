import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    semantic_slam_dir = '/home/delta/pi/SemanticSLAM'
    ekf_config = os.path.join(semantic_slam_dir, 'ekf.yaml')
    slam_config = os.path.join(semantic_slam_dir, 'slam.yaml')
    nav2_config = os.path.join(semantic_slam_dir, 'slam_nav2.yaml')
    filter_config = os.path.join(semantic_slam_dir, 'laser_filter.yaml')
    explore_config = os.path.join(semantic_slam_dir, 'explore.yaml')

    use_sim_time_param = {'use_sim_time': False}

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_node',
        output='both',
        arguments=['serial', '--dev', '/dev/pico_robot'],
        emulate_tty = True
    )

    pico_link = ExecuteProcess(
        cmd = [os.path.join(semantic_slam_dir, 'build/slam')],
        output = 'both',
        emulate_tty = True
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ldlidar_stl_ros2'),
                'launch',
                'ld06.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=[
            '--x', '0.0', 
            '--y', '0.0', 
            '--z', '0.0', 
            '--yaw', '0.0', 
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'base_link', 
            '--child-frame-id', 'imu_link'
        ],
        parameters=[use_sim_time_param],
        output='both',
        emulate_tty = True
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=[
            '--x', '0.0', 
            '--y', '0.0', 
            '--z', '0.18', 
            '--yaw', '0.0', 
            '--pitch', '0.035',  
            '--roll', '0.0', 
            '--frame-id', 'base_link', 
            '--child-frame-id', 'base_laser'
        ],
        parameters=[use_sim_time_param],
        output='both',
        emulate_tty = True
    )

    madgwick_filter = Node(
        package = 'imu_filter_madgwick',
        executable = 'imu_filter_madgwick_node',
        parameters = [use_sim_time_param, {
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'orientation_stddev': 0.05,
        }],
        remappings=[('/imu/data_raw', '/imu/data_raw')],
        output = 'both',
        emulate_tty = True
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='both',
        parameters=[ekf_config, use_sim_time_param],
        remappings=[('/odometry/filtered', '/odom')],
        emulate_tty = True
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'false'
        }.items()
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_config,
            'use_sim_time': 'false'
        }.items()
    )

    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_node',
        parameters=[filter_config],
        output='both',
        emulate_tty = True
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=['/home/delta/pi/SemanticSLAM/explore.yaml', {'use_sim_time': False}],
        output='both',
        emulate_tty = True
    )

    yolo_node = Node(
        package = 'semantic_detector',
        executable = 'yolo_node',
        name = 'yolo_detector',
        prefix = ['taskset -c 3 '],
        parameters=[{
        'model_path': '/home/delta/pi/ros2_ws/src/semantic_detector/semantic_detector/yolo26n.onnx'
        }],
        output = 'both',
        emulate_tty = True
    )

    projector_node = Node(
        package = 'semantic_detector',
        executable = 'projector_node',
        name = 'semantic_projector',
        prefix = ['taskset -c 3 '],
        parameters=[{'camera_height': 0.07}],
        output = 'both',
        emulate_tty = True
    )

    heatmap_node = Node(
        package = 'semantic_detector',
        executable = 'heatmap_node',
        name = 'semantic_heatmap',
        prefix = ['taskset -c 3 '],
        parameters=[{'camera_height': 0.07}],
        output = 'both',
        emulate_tty = True

    )

    delayed_brain_bringup = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="=== IMU CALIBRATED! BOOTING EKF, SLAM, AND NAV2 ==="),
            ekf_node,
            slam_toolbox,
            nav2_bringup,
            laser_filter,
        ]
    )
    
    delayed_explore_launch = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="=== NAV2 BOOTED! STARTING EXPLORATION ==="),
            explore_node,
            yolo_node,
            projector_node,
            heatmap_node
        ]
    )

    return LaunchDescription([
        LogInfo(msg="=== CALIBRATING IMU FOR 10 SECONDS ==="),
        # micro_ros_agent
        pico_link,
        lidar_launch,
        static_transform,
        laser_tf,
        madgwick_filter,
        delayed_brain_bringup,
        delayed_explore_launch
    ])