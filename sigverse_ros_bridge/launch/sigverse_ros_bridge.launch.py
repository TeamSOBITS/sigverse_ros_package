import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sigverse_ros_bridge_port = LaunchConfiguration('sigverse_ros_bridge_port', default='50001')
    ros_bridge_port = LaunchConfiguration('ros_bridge_port', default='9090')

    sigverse_node = Node(
        package='sigverse_ros_bridge',
        executable='sigverse_ros_bridge',
        name='sigverse_ros_bridge',
        parameters=[{'port': sigverse_ros_bridge_port}]
    )

    rosbridge_launch_file = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rosbridge_server"),
            "launch",
            "rosbridge_websocket_launch.xml",
        ),
        launch_arguments={'port': ros_bridge_port}.items()
    )

    return LaunchDescription([
        sigverse_node,
        rosbridge_launch_file
    ])