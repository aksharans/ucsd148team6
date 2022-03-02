from re import A
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
        # neutral: 0.1, medium forward: 0.2, slow forward: 0.125
        self.throttle_neutral = 0.1
        self.throttle_forward = 0.125

        # steering values (Twist angular.z)
            # max-left (not actual max, but to be safe) -0.3
            # max-right (not actual max, but to be safe) 0.6
            # center 0.15
        self.steering_center = 0.15

        # servo values
        # max left and max right for SERVO left: 180, right: 90, middle: 135
        # change adafruit_servo_calibration.yaml for max and min servo values (max: a, min: b)
        self.servo_center = 135


        ### Publishers/Subscribers ###
           
        # change steering to channel 4 and throttle to channel 7 in adafruit_twist.py
        self.twist_publisher = self.create_publisher(Twist, TWIST_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        self.servo_publisher = self.create_publisher(Float32, SERVO_TOPIC_NAME, 10)
        self.servo = self.servo_center

        self.camera_subscriber = self.create_subscription(Image, CAMERA_IMG_TOPIC_NAME, self.controller, 10)

    
    def controller(self, data):


        # cv2 image processing, data is the raw image from intel camera node
        hsv = cv2.cvtColor(data, cv2.COLOR_BGR2HSV)

        # (H, S, V)
        # green object
        lower = np.array([40, 50, 20])
        higher = np.array([75, 255, 255])

        mask = cv2.inRange(hsv, lower, higher)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:

            x = 1
            # if center is within ~30px of actual middle, go straight


        else: 
            # if no image (no rectangle found), then stop -- no throttle, no steering
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

        video.release()
        cv2.destroyAllWindows()

        target_detection.destroy_node()
        rclpy.shutdown()
        print(f"Successfully shut down {NODE_NAME}")

if __name__ == 'main':
    main()