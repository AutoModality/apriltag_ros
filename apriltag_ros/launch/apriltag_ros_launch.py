import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
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
                ('/image_rect','/image_rect'),
                ('/camera_info', '/camera_info')
            ],
        emulate_tty=True,
    )
    ld.add_action(apriltag_ros_node)
    return ld