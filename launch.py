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
    explore_config = os.path.join(semantic_slam_dir, 'explore.yaml')

    use_sim_time_param = {'use_sim_time': False}

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_node',
        output='both',
        arguments=['serial', '--dev', '/dev/pico_robot'],
        emulate_tty = True,
        respawn=True,
        respawn_delay=2.0
    )

    pico_link = ExecuteProcess(
        cmd = ['taskset', '-c', '0,1', os.path.join(semantic_slam_dir, 'build/slam')],
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

    imu_transform = Node(
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

    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=[
            '--x', '0.06',
            '--y', '0.0',
            '--z', '0.11', 
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera'
        ],
        parameters=[use_sim_time_param],
        output='both',
        emulate_tty = True
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        prefix=['taskset -c 0,1 '],
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
            'use_sim_time': 'false',
            'use_velocity_smoother': 'True',
            'use_collision_monitor': 'True'
        }.items()
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        prefix=['taskset -c 0,1 '],
        parameters=['/home/delta/pi/SemanticSLAM/explore.yaml', {'use_sim_time': False}],
        output='both',
        emulate_tty = True
    )

    pi_camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='pi_camera',
        parameters=[{
            'width': 320,
            'height': 240,
            'format': 'RGB888',
            'framerate': 30.0
        }],
        remappings=[
            ('/pi_camera/image_raw', '/image_raw') 
        ],
        output='both'
    )

    semantic_obstacle_node = Node(
        package='semantic_detector',
        executable='semantic_obstacle_node',
        name='semantic_obstacle_publisher',
        output="both"
    )

    semantic_node = Node(
        package='semantic_detector',
        executable='semantic_node',
        name='semantic_node',
        prefix=['nice -n 10 taskset -c 2,3 '],
        parameters=[use_sim_time_param],
        output='both',
        emulate_tty = True

    )

    delayed_brain_bringup = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="=== IMU CALIBRATED! BOOTING EKF, SLAM, AND NAV2 ==="),
            ekf_node,
            slam_toolbox,
            nav2_bringup,
        ]
    )
    
    delayed_yolo_launch = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg="=== NAV2 BOOTED! STARTING YOLO ==="),
            semantic_node,
            semantic_obstacle_node
        ]
    )

    delayed_explore_launch = TimerAction(
        period=100.0,
        actions=[
            LogInfo(msg="=== YOLO BOOTED! STARTING EXPLORATION ==="),
            explore_node
        ]
    )

    return LaunchDescription([
        LogInfo(msg="=== CALIBRATING IMU FOR 10 SECONDS ==="),
        micro_ros_agent,
        pico_link,
        lidar_launch,
        imu_transform,
        camera_transform,
        pi_camera_node,
        delayed_brain_bringup,
        delayed_yolo_launch,
        delayed_explore_launch
    ])