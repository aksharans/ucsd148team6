import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    target_detection_package = 'aimbot_pkg'
    td_node_name = 'target_detection_node'

    nav_package = 'ucsd_robocar_nav2_pkg'
    all_components_launch = 'all_components.launch.py'

    ld = LaunchDescription()

    components_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(nav_package),
                    'launch',
                    all_components_launch)
            )
        )

    target_detection_node = Node(
        package=aimbot_pkg,
        executable=target_detection,
        output='screen',
    )

    ld.add_action(components_launch)
    ld.add_action(target_detection_node)

    return ld



