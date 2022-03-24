# UCSD ECE/MAE 148: Team 6 Aimbot Package
### Target Detection & Tracking
#### Branch for integration into https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2
By: Aksharan Saravanan, Hieu Luu, Katada Siraj

## About



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

# Steering (Twist angular.z)
steering_center: steering straight center
steering_maxleft: steering max left
steering_maxright: steering max right

# Servo (configure bus, port, max, min in servo calibration)
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




## Troubleshooting

TODO
