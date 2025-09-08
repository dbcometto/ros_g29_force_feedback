import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import  DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    params_file = "publishing.yaml"
    params = os.path.join(
        get_package_share_directory('ros_g29_force_feedback'),
        "config",
        params_file)
        
    g29_ff = Node(
            package="ros_g29_force_feedback",
            executable="g29_feedback_publisher",
            name="g29_feedback_publisher",
            output="screen",
            parameters=[params])
    
    delayed_node = TimerAction(
        period=3.0,
        actions=[g29_ff]
    )

    return LaunchDescription([
        delayed_node
    ])