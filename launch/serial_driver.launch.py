import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rm_serial = get_package_share_directory('rm_serial')

    config_file_path = os.path.join(pkg_rm_serial, 'config', 'serial_driver_params.yaml')

    print(f"--- Launch File Debug ---")
    print(f"Package share directory: {pkg_rm_serial}")
    print(f"Calculated config file path: {config_file_path}")
    if os.path.exists(config_file_path):
        print(f"Config file EXISTS at the calculated path.")
    else:
        print(f"Config file DOES NOT EXIST at the calculated path: {config_file_path}")
        print(f"Please check your CMakeLists.txt install rule for the config directory and the file name.")
    print(f"--- End Launch File Debug ---")

    return LaunchDescription([
        Node(
            package='rm_serial',
            executable='serialcopy',
            name='serial_driver',
            output='screen',
            parameters=[config_file_path]
        )
    ])
    