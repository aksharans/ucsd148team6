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

    aimbot_pkg = 'aimbot_pkg'
    td_node_name = 'target_detection_node'

    actuator_package = 'ucsd_robocar_actuator2_pkg'
    adafruit_servo_launch = 'adafruit_servo.launch.py'
    adafruit_twist_launch = 'adafruit_twist.launch.py'

    sensor_pkg = 'ucsd_robocar_sensor2_pkg'
    intel_launch = 'camera_intel.launch.py'

    ld = LaunchDescription()

    adafruit_servo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(actuator_package),
                    'launch',
                    adafruit_servo_launch)
            )
        )

    adafruit_twist_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(actuator_package),
                    'launch',
                    adafruit_twist_launch)
            )
        )

    intel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(sensor_pkg),
                'launch',
                intel_launch)
        )
    )

    target_detection_node = Node(
        package=aimbot_pkg,
        executable=td_node_name,
        output='screen',
    )


    ld.add_action(adafruit_servo_launch)
    ld.add_action(adafruit_twist_launch)
    ld.add_action(intel_launch)
    ld.add_action(target_detection_node)

    return ld



