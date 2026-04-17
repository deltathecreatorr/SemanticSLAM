import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    semantic_slam_dir = '/home/delta/pi/SemanticSLAM'
    ekf_config = os.path.join(semantic_slam_dir, 'ekf.yaml')
    slam_config = os.path.join(semantic_slam_dir, 'slam.yaml')
    nav2_config = os.path.join(semantic_slam_dir, 'slam_nav2.yaml')

    pico_link = ExecuteProcess(
        cmd = [os.path.join(semantic_slam_dir, 'build/slam')],
        output = 'screen'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ldlidar_stl_ros2'),
                'launch',
                'ld06.launch.py'
            )
        ])
    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'base_laser']
    )

    madgwick_filter = Node(
        package = 'imu_filter_madgwick',
        executable = 'imu_filter_madgwick_node',
        parameters = [{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
        }],
        remappings=[('/imu/data_raw', '/imu/data_raw')]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom')]
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

    explore_lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        pico_link,
        lidar_launch,
        static_transform,
        laser_tf,
        madgwick_filter,
        ekf_node,
        slam_toolbox,
        nav2_bringup,
        explore_lite
    ])