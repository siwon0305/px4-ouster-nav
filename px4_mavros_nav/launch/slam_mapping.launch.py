from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_params = PathJoinSubstitution([
        FindPackageShare('px4_mavros_nav'),
        'config',
        'px4_ouster_slam_toolbox.yaml'
    ])

    slam_toolbox_launch = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('px4_mavros_nav'),
        'rviz',
        'slam_mapping.rviz'
    ])

    pose_to_odom = Node(
        package='px4_mavros_nav',
        executable='pose_to_odom',
        name='pose_to_odom',
        parameters=[{
            'use_sim_time': True,
            'pose_topic': '/mavros/local_position/pose',
            'odom_topic': '/odom',
            'parent_frame': 'odom',
            'child_frame': 'base_link',
            'min_time_step_ns': 1000000,
        }],
        output='screen',
    )

    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_os_lidar_tf',
        arguments=[
            '0', '0', '0.10',
            '0', '0', '0',
            'base_link', 'os_lidar'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/ouster/points'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'use_sim_time': True,
            'min_height': -0.50,
            'max_height': 0.50,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,
            'scan_time': 0.1,
            'range_min': 0.7,
            'range_max': 12.0,
            'use_inf': True,
            'queue_size': 1,
        }],
        output='screen',
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        pose_to_odom,
        base_to_lidar_tf,

        TimerAction(
            period=4.0,
            actions=[
                pointcloud_to_laserscan,
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[
                slam_toolbox,
            ],
        ),

        TimerAction(
            period=10.0,
            actions=[
                rviz,
            ],
        ),
    ])