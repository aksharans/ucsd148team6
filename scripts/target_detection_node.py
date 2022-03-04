import rclpy
from rclpy.Node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np

NODE_NAME = 'target_detection_node'

# topics subscribed to
CAMERA_IMG_TOPIC_NAME = '/camera/color/image_raw'

# topics published to
SERVO_TOPIC_NAME = '/servo'
TWIST_TOPIC_NAME = '/cmd_vel'




class TargetDetection(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        ### Actuator constants ###

        # throttle values (Twist linear.x)
        self.throttle_neutral = 0.1
        self.throttle_forward = 0.125 # slow forward
        # self.throttle_forward = 0.2 # medium forward

        # steering values (Twist angular.z)
        # recalibrate these values
        self.steering_center = 0.15
        self.steering_maxleft = -0.3
        self.steering_maxright = 0.6

        # servo values
        # max left and max right for SERVO left: 180, right: 90, middle: 135
        # change adafruit_servo_calibration.yaml for max and min servo values 
        # (bus:  1, port 8, max: a, min: b)
        self.servo_maxLeft = 180
        self.servo_maxRight = 90
        self.servo_center = 135
        self.last_servo_pos = 135

        ### Publishers/Subscribers ###
           
        # change steering to channel 4 and throttle to channel 7 in adafruit_twist.py
        self.twist_publisher = self.create_publisher(Twist, TWIST_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        self.servo_publisher = self.create_publisher(Float32, SERVO_TOPIC_NAME, 10)
        self.servo = self.servo_center

        self.camera_subscriber = self.create_subscription(Image, CAMERA_IMG_TOPIC_NAME, self.controller, 10)

    
    def controller(self, data):

        # map servo value to steering value
        def servo_to_steering(servo):
            return servo/100 - 1.2

        # check whether servo is out of the max bounds, so the steering doesn't exceed it's max
        def check_servo(servo):
            if servo < self.servo_maxRight:
                return self.servo_maxRight
            elif servo > self.servo_maxLeft:
                return self.servo_maxLeft

        # get data (intel image) width
        _, width = data.shape[0:2]
        image_midX = width/2

        # cv2 image processing, data is the raw image from intel camera node
        hsv = cv2.cvtColor(data, cv2.COLOR_BGR2HSV)

        # (H, S, V)
        # blue object
        # calibrate these values with poster board
        lower = np.array([80, 155, 20]) 
        higher = np.array([130, 255, 255])

        # mask and find contours
        mask = cv2.inRange(hsv, lower, higher)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:

            # get max contour
            c = max(contours, key=cv2.contourArea)

            # draw a rectangle around c and get x position & width
            x, _, w, _ = cv2.boundingRect(c)

            # x coordinate of middle of the detected target
            target_midX = x + w/2

            # target x position minus image x position
            distance = target_midX - image_midX

            # evaluate servo adjustment with P controller
            turn_factor = (abs(distance)/image_midX)**2         # turn_amount decreases as target center
            angle_per_frame = 5                                 # is closer to image center
            turn_amount = angle_per_frame*turn_factor

            # Set throttle to forward
            self.twist_cmd.linear.x = self.throttle_neutral # neutral for now

            # center of detected object within small threshold of actual center, go straigt
            if abs(distance) < 90:   # calibrate this value with intel camera
                # servo
                self.servo = self.servo_center
                self.last_servo_pos = self.servo

                # steering
                self.twist_cmd.angular.z = self.steering_center


            # target x greater than image x, we need to turn right
            elif distance > 0: 

                # servo
                self.servo = check_servo(self.last_servo_pos - turn_amount)
                self.last_servo_pos = self.servo

                # steering
                self.twist_cmd.angular.z = servo_to_steering(self.servo)

            # target x less than image x, we need to turn left
            elif distance < 0:

                # servo
                self.servo = check_servo(self.last_servo_pos + turn_amount)
                self.last_servo_pos = self.servo

                # steering
                self.twist_cmd.angular.z = servo_to_steering(self.servo)

            # publish to the twist and servo topics with calculated values
            self.twist_publisher.publish(self.twist_cmd)
            self.servo_publisher.publish(self.servo)

        # if no target (rectangle), then stop -- no throttle, no steering
        else: 
            self.twist_cmd.linear.x = self.throttle_neutral
            self.twist_cmd.angular.z = self.steering_center
            self.servo = self.servo_center

            self.twist_publisher.publish(self.twist_cmd)
            self.servo_publisher.publish(self.servo)




def main(args=None):

    rclpy.init(args=args)
    target_detection = TargetDetection()

    try: 
        rclpy.spin(target_detection)
        target_detection.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print(f"Shutting down {NODE_NAME}")

        target_detection.twist_cmd.linear.x = 0.1
        target_detection.twist_publisher.publish(target_detection.twist_cmd)

        cv2.destroyAllWindows()

        target_detection.destroy_node()
        rclpy.shutdown()
        print(f"Successfully shut down {NODE_NAME}")

if __name__ == 'main':
    main()
