import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge
import numpy as np
import time

NODE_NAME = 'target_detection_node'

# topics subscribed to
CAMERA_IMG_TOPIC_NAME = '/camera/color/image_raw'
DEPTH_TOPIC_NAME = '/camera/depth/image_rect_raw'

# topics published to
SERVO_TOPIC_NAME = '/servo'
TWIST_TOPIC_NAME = '/cmd_vel'


# change included sensors in car_config.yaml
# change adafruit_servo_calibration.yaml for max and min servo values 
# change steering to channel X and throttle to channel Y in adafruit_twist.py,
# where X and Y are the channels they're plugged into in the PWM


class TargetDetection(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        ### Target and Image Info ###
        self.target_found = False
        self.image_midX = 320 # this resolution is for both rgb data and depth data
        self.image_midY = 240
        self.target_midX = 320 # width
        self.target_midY = 240 # height
        
        ### Actuator constants ###

        # throttle values (Twist linear.x)
        self.throttle_neutral = 0.0
        self.last_throttle = self.throttle_neutral
        self.throttle_min = 0.0
        self.throttle_max = .14
        self.following_dist = 1  #meters

        # steering values (Twist angular.z)
        # recalibrate these values
        self.steering_center = 0.2
        self.steering_maxleft = -0.3
        self.steering_maxright = 0.7

        # servo values
        self.servo_maxLeft = 170.0
        self.servo_maxRight = 80.0
        self.servo_center = 125.0
        self.last_servo_pos = self.servo_center

        # bridge for camera
        self.bridge = CvBridge()

        ### Publishers/Subscribers ###
           
        self.twist_publisher = self.create_publisher(Twist, TWIST_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        self.servo_publisher = self.create_publisher(Float32, SERVO_TOPIC_NAME, 10)
        self.servo = Float32()

        self.camera_subscriber = self.create_subscription(Image, CAMERA_IMG_TOPIC_NAME, self.servo_steering_controller, 10)
        self.depth_subscriber = self.create_subscription(Image, DEPTH_TOPIC_NAME, self.throttle_controller, 10)
    
    
    ''' Utility Functions '''

    def pid(self, attribute, current, target):
        constants = {'throttle': 5, 'steering': 25, 'servo': 25}
        K = constants[attribute]
        t = current*(K-1)/K + target/K
        print(f" Caclulated {attribute}: {t}")
        return t

    # map servo value to steering value
    def servo_to_steering(self, servo):
        return ((250-servo)-125)/90 + .2

    # check whether servo is out of the max bounds, so the steering doesn't exceed it's max
    def check_servo(self, servo):
        if servo < self.servo_maxRight:
            return self.servo_maxRight
        elif servo > self.servo_maxLeft:
            return self.servo_maxLeft
        else:
            return float(servo)
    

    '''Controller Functions'''
    
    def servo_steering_controller(self, data):

        # get image from data and convert to RGB
        frame = self.bridge.imgmsg_to_cv2(data)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # cv2 image processing, data is the raw image from intel camera node
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # (H, S, V)
        # blue object
        # calibrate these values with poster board
        blue_lower = np.array([80, 155, 20])
        blue_higher = np.array([130, 255, 255])
        purpl_lower = np.array([120, 90, 20])
        purpl_higher = np.array([160, 255, 255])

        # mask and find contours
        # mask = cv2.inRange(hsv, blue_lower, blue_higher)
        mask = cv2.inRange(hsv, purpl_lower, purpl_higher)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # get max contour if there is one and get it's area
        area = 0
        if len(contours) != 0:
            # get max contour
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
        print(f"Area: {area}")

        area_min_threshold = 100.0

        # if area greater than a certain threshold
        if area > area_min_threshold:
            print("Contour found")
            self.target_found = True
            # draw a rectangle around c and get x position & width
            x, y, w, h = cv2.boundingRect(c)

            # x and y coordinate of middle of the detected target
            self.target_midX = x + w/2
            self.target_midY = y + h/2
            
            # get distance of target x position minus image x position
            distance = self.target_midX - self.image_midX

            # evaluate servo adjustment with P controller
            # turn_amount decreases as target center
            turn_factor = (abs(distance)/self.image_midX)**2
            angle_per_frame = 20                                # is closer to image center
            turn_amount = angle_per_frame*turn_factor


            # target x greater than image x, we need to turn right
            if distance > 0: 

                # servo
                self.servo.data = self.check_servo(self.last_servo_pos - turn_amount)
                self.last_servo_pos = self.servo.data

                # steering
                self.twist_cmd.angular.z = self.servo_to_steering(self.servo.data)

            # target x less than image x, we need to turn left
            elif distance < 0:

                # servo
                self.servo.data = self.check_servo(self.last_servo_pos + turn_amount)
                self.last_servo_pos = self.servo.data

                # steering
                self.twist_cmd.angular.z = self.servo_to_steering(self.servo.data)

            print('Publishing throttle, steering, and servo')
            # publish to the twist and servo topics with calculated values
            self.twist_publisher.publish(self.twist_cmd)
            self.servo_publisher.publish(self.servo)

        # if no target (rectangle), then stop -- no throttle, no steering
        else: 
            print("NO CONTOUR FOUND!!!!")
            self.target_found = False
            steering = self.pid('steering', self.servo_to_steering(self.last_servo_pos), self.steering_center)
            servo = self.pid('servo', self.last_servo_pos, self.servo_center)

            self.twist_cmd.angular.z = steering
            
            self.servo.data = float(servo)
            self.last_servo_pos = self.servo.data

            print("Publishing neutral throttle, steering, and servo")
            self.twist_publisher.publish(self.twist_cmd)
            self.servo_publisher.publish(self.servo)



    def throttle_controller(self, data):
        if self.target_found:
            image = self.bridge.imgmsg_to_cv2(data)
            pixel = (int(self.target_midY), int(self.target_midX))
            depth = image[pixel[0], pixel[1]]
            print('\n' + f'wanted depth: {self.following_dist} m' + '\n')
            print(f'depth at target location {pixel} is {depth/1000} m')
            m2mm_factor = 1000
            error = depth - self.following_dist * m2mm_factor
            Kp = .001
            if error > 0:
                print('publishing forward throttle')
                control = min(Kp * error + self.throttle_neutral, self.throttle_max)
                throttle = self.pid('throttle', self.last_throttle, control)
            else:
                if depth == 0:
                    throttle = self.last_throttle
                else:
                    print('TOO CLOSE!!! ---publishing neutral throttle')
                    throttle = self.throttle_neutral
            self.twist_cmd.linear.x = throttle
            self.last_throttle = throttle
        else:
            print("setting neutral throttle cuz no target found")
            throttle = self.pid('throttle', self.last_throttle, self.throttle_neutral)
            self.last_throttle = throttle


def main(args=None):

    rclpy.init(args=args)
    target_detection = TargetDetection()

    try: 
        rclpy.spin(target_detection)
        target_detection.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print(f"Shutting down {NODE_NAME}")

        target_detection.twist_cmd.linear.x = target_detection.throttle_neutral
        target_detection.twist_publisher.publish(target_detection.twist_cmd)

        time.sleep(1)
        cv2.destroyAllWindows()
        target_detection.destroy_node()
        rclpy.shutdown()
        
        print(f"Successfully shut down {NODE_NAME}")

if __name__ == '__main__':
    main()