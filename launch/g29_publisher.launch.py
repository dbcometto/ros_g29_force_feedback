import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import  DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    params_file = "convoy_steering.yaml"
    params = os.path.join(
        get_package_share_directory('ros_g29_force_feedback'),
        "config",
        params_file)
    
    ld = LaunchDescription()


    #=============== Launch Arguments ===============#

    # Ben: Steering wheel port
    steering_wheel_port = LaunchConfiguration("steering_wheel_port")
    steering_wheel_port_la = DeclareLaunchArgument(
        "steering_wheel_port", default_value="/dev/input/event7"
    )
    ld.add_action(steering_wheel_port_la)


    #=============== Nodes ===============#

        
    g29_ff = Node(
            package="ros_g29_force_feedback",
            executable="g29_feedback_publisher",
            name="g29_feedback_publisher",
            output="screen",
            parameters=[params,{"device_name":steering_wheel_port}])
    
    delayed_node = TimerAction(
        period=3.0,
        actions=[g29_ff]
    )
    ld.add_action(delayed_node)


    

    return ld