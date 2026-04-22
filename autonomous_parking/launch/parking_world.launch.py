import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    pkg_dir = get_package_share_directory('autonomous_parking')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_dir, 'worlds', 'parking_lot.world')

    # TurtleBot3 model
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', f'turtlebot3_{turtlebot3_model}', 'model.sdf'
    )

    # Robot spawn position (entrance of parking lot)
    spawn_x = '-12.0'
    spawn_y = '0.0'
    spawn_z = '0.01'
    spawn_yaw = '0.0'

    return LaunchDescription([
        # Launch Gazebo with parking lot world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(
                    os.path.join(
                        get_package_share_directory('turtlebot3_description'),
                        'urdf',
                        f'turtlebot3_{turtlebot3_model}.urdf'
                    )
                ).read(),
                'use_sim_time': True
            }],
        ),

        # Spawn TurtleBot3 at parking lot entrance
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', turtlebot3_model,
                '-file', urdf_file,
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
            output='screen',
        ),
    ])
