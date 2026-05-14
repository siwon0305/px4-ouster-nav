from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    octomap_path = LaunchConfiguration('octomap_path')
    execute = LaunchConfiguration('execute')
    goal_z = LaunchConfiguration('goal_z')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('px4_mavros_nav'),
        'rviz',
        'octomap_3d_astar.rviz'
    ])

    pose_to_tf = Node(
        package='px4_mavros_nav',
        executable='pose_to_tf',
        name='pose_to_tf',
        parameters=[{
            'use_sim_time': True,
            'parent_frame': 'map',
            'child_frame': 'base_link',
        }],
        output='screen',
    )

    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_os_lidar_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.10',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'os_lidar',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'map',
            'resolution': 0.15,
            'octomap_path': octomap_path,
            'occupancy_min_z': 0.20,
            'occupancy_max_z': 2.80,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_octomap_3d_astar',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    set_max_depth_after_rviz = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set',
            '/octomap_server',
            'max_depth',
            '16'
        ],
        output='screen',
    )

    astar_3d = Node(
        package='px4_mavros_nav',
        executable='astar_goal_follower',
        name='astar_goal_follower',
        parameters=[{
            'use_sim_time': True,
            'execute': ParameterValue(execute, value_type=bool),
            'cloud_topic': '/octomap_point_cloud_centers',
            'grid_resolution': 0.20,
            'min_z': 0.20,
            'max_z': 2.80,
            'goal_z': ParameterValue(goal_z, value_type=float),
            'inflation_radius': 0.35,
            'waypoint_spacing': 0.60,
            'reach_threshold': 0.35,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'octomap_path',
            default_value='',
            description='Path to saved OctoMap .bt file'
        ),

        DeclareLaunchArgument(
            'execute',
            default_value='false',
            description='If true, 3D A* sends MAVROS position setpoints'
        ),

        DeclareLaunchArgument(
            'goal_z',
            default_value='1.5',
            description='Default target z value for RViz 2D Goal Pose'
        ),

        pose_to_tf,
        base_to_lidar_tf,
        octomap_server,

        TimerAction(
            period=3.0,
            actions=[
                rviz,
            ],
        ),

        RegisterEventHandler(
            OnProcessStart(
                target_action=rviz,
                on_start=[
                    TimerAction(
                        period=4.0,
                        actions=[
                            set_max_depth_after_rviz,
                        ],
                    ),
                    TimerAction(
                        period=6.0,
                        actions=[
                            astar_3d,
                        ],
                    ),
                ],
            )
        ),
    ])