import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    spawn_entity = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments=[
            "-name","robot",
            "-topic","robot_description",
            "-x","0.0",
            "-y","0.0",
            "-z","0.01"
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output="screen"
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )



    return LaunchDescription([spawn_entity,bridge])