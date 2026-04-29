import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def _env_path(var_name, *paths):
    entries = [p for p in os.environ.get(var_name, '').split(os.pathsep) if p]
    for path in paths:
        if path and path not in entries:
            entries.append(path)
    return os.pathsep.join(entries)


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_parking')
    models_path = os.path.join(pkg_dir, 'models')
    world_file = os.path.join(pkg_dir, 'worlds', 'parking_lot.world')
    gazebo_model_path = '/usr/share/gazebo-11/models'
    gazebo_resource_path = '/usr/share/gazebo-11'

    return LaunchDescription([
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            _env_path('GAZEBO_MODEL_PATH', gazebo_model_path, models_path)
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            _env_path('GAZEBO_RESOURCE_PATH', gazebo_resource_path, pkg_dir)
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),
    ])
