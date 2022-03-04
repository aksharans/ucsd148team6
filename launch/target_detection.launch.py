from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    target_detection_package = 'ucsd148team6_aimbot_pkg'
    td_node_name = 'target_detection_node'

    intel_camera_package = ''
    cam_node_name = ''

    servo_package = ''
    servo_node_name = ''

    actuator_package = ''
    actuator_node_name = ''



    ld = LaunchDescription()

    return ld



