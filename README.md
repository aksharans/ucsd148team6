# UCSD ECE/MAE 148: Team 6 Aimbot Package
### Target Detection & Tracking
#### Branch for integration into https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2
By: Aksharan Saravanan, Hieu Luu, Katada Siraj

## About

Package developed for [our UCSD ECE/MAE 148 final project](https://guitar.ucsd.edu/maeece148/index.php/2022WinterTeam6). This ROS package allows an autonomous robot to detect a target of a specific color, and then continually follow and track that target.

## Calibration (config/target_detection.yaml)

```
Hue_low: lower hue 
Hue_high: higher hue 
Sat_low: lower saturation 
Sat_high: higher saturation
Val_low: lower value 
Val_high: higher value

image_midX: incoming sensor image center x coordinate
image_midY: incoming sensor image center y coordinate

throttle_neutral: neutral throttle
throttle_min: minimum throttle
throttle_max: maximum throttle
following_dist: following distance from target

steering_center: steering straight center
steering_maxleft: steering max left
steering_maxright: steering max right

servo_maxLeft: servo max left
servo_maxRight: steering max right
servo_center: servo straight center

area_min_threshold: minimum area of a contour to detect (filter out noise)

Kp_throttle: Kp for throttle
Kp_steering: Kp for steering
Kp_servo: Kp for the servo
Kp_depth_throttle: Kp for depth target throttle
```

## Workflow

1. Make sure the [ucsd_robocar docker image](https://hub.docker.com/r/djnighti/ucsd_robocar) is pulled and the docker container is running (following the instructions for the docker container as well as the [nav package](https://gitlab.com/ucsd_robocar2/ucsd_robocar_nav2_pkg))
3. Calibrate the values specified in the config file by launching the specific node (eg launching the adafruit_twist node) and publishing values directly using the command line.
4. Launch target detection: `ros2 launch aimbot_pkg target_detection.launch.py`


## Dependencies

n/a

## Nodes

#### target_detection_node

Associated File: aimbot_pkg/target_detection_node.py

Target Detection node interacts with the topics below and has 2 controllers: the steering controller and the throttle controller. The node recieves image data from a camera (in our case, the Intel RGBD camera, but can use with any webcam by changing some of the code). It then detects whether the image is within a certain HSV range and above a threshold area. If so, it will adjust its steering and servo (with the camera and a small laser mounted) to point at the center of the target. It also incorporates depth data by dynamically adjusting it's throttle depending on the target's distance (if the target is farther away, accelarate to max throttle and if the target gets closer, slow down until a full stop is made at around 1 meter).


## Topics 

| Topic | Node | Publishing To/Subscribing To | Message Type | Info |
| --- | ---- | ---- | ---- | ---- |
| /servo | adafruit_servo | publishing | std_msgs.msg.Float32 | .data for value |
| /cmd_vel | adafruit_twist | publishing | geometry_msgs.msg.Twist | linear.x for throttle, angular.z for steering |
| /camera/color/image_raw | camera_intel | subscribing | sensor_msgs.msg.Image | raw image in RGB |
| /camera/depth/image_rect_raw | camera_intel | subsribing | sensor_msgs.msg.Image | depth image in an array |


## Launch

#### target_detection

Associated File: launch/target_detection.launch.py

This file launches the target_detection node in this package. It also launches nodes in external packages: adafruit_twist node, adafruit_servo node, and camera_intel node.

`ros2 launch aimbot_pkg target_detection.launch.py`


## Demo Video

https://www.youtube.com/watch?v=tL3elf0cDe0

## Troubleshooting

- If the steering & throttle don't seem to be running, try re-calibrating the values by launching the twist node and publishing using the command line
- If the servo is not moving, make sure that the correct channel is specified and the values are calibrated by launching the servo node
- If the Intel camera is not running, try stopping and exiting the container, then restarting, and check if the wire is fully plugged in
- If the car is too slow and not as responsive, try changing the Kp values
- Start by testing the launched target detection node on the stand first to make sure the servo/steering is moving to track a target and also the throttle responds to the target depth 
