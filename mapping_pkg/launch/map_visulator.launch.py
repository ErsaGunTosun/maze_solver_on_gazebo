import os 

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription


def generate_launch_description():
    package_share_dir = get_package_share_directory("mapping_pkg")

    config_file_path = os.path.join(
        package_share_dir,
        'config',
        'slam.yaml'
    )

    slam_pkg = get_package_share_directory("slam_toolbox")

    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg,"launch","online_async_launch.py")    
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'slam_params_file': config_file_path,
            }.items()
    )
    

    return LaunchDescription([
        slam
    ])