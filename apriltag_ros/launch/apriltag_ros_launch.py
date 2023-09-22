import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    image_topic = LaunchConfiguration('image_topic', default="/image_rect")
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value = image_topic)

    caminfo_topic = LaunchConfiguration('caminfo_topic', default="/camera_info")
    caminfo_topic_arg = DeclareLaunchArgument('caminfo_topic', default_value = caminfo_topic)


    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'config',
        'settings.yaml'
        )
    apriltag_ros_node=Node(
        package = 'apriltag_ros',
        name = 'apriltag_ros',
        executable = 'apriltag_ros_continuous_node',
        parameters = [config],
        remappings = [
                ('/image_rect',image_topic),
                ('/camera_info', caminfo_topic)
            ],
        emulate_tty=True,
    )
    ld.add_action(apriltag_ros_node)
    ld.add_action(image_topic_arg)
    ld.add_action(caminfo_topic_arg)
    return ld