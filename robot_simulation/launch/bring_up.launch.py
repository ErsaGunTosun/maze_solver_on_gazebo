import os 

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    difficulty_arg = DeclareLaunchArgument(
        'difficulty',
        default_value='easy',
        description='Difficulty level: easy, medium, or hard'
    )
    
    difficulty = LaunchConfiguration('difficulty')

    filename = [difficulty, TextSubstitution(text='.sdf')]

    package_share_dir = get_package_share_directory("robot_simulation")

    world_path = PathJoinSubstitution([
        package_share_dir,
        'worlds',
        filename  
    ])
    

    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    launch_path = os.path.join(
            get_package_share_directory("robot_simulation"),
            "launch"
    )

    gz_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")
            ),
            launch_arguments ={"gz_args": ["-s -r -v1 ", world_path]}.items()
    )

    gz_client = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")    
            ),
            launch_arguments = {"gz_args": ["-g -v1"]}.items()
    )
    

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"robot_state_publisher.launch.py")
             )
    )

    spawn_entity = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"spawn_entity.launch.py")
            )
    )



    return LaunchDescription([
        difficulty_arg,
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_entity,
        ])
