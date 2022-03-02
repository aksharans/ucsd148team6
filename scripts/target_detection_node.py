import rclpy
from rclpy.Node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Image
from geometry_msgs.msg import Twist


NODE_NAME = 'target_detection_node'

# topics subscribed to
CAMERA_IMG_TOPIC_NAME = '/camera/color/image_raw'

# topics published to
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'
SERVO_TOPIC_NAME = '/servo'
ADAFRUIT_TOPIC_NAME = '/cmd_vel'

class TargetDetection(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
    
        self.servo_publisher = self.create_publisher(Float32, SERVO_TOPIC_NAME, 10)

        self.adafruit_publisher = self.create_publisher(Twist, ADAFRUIT_TOPIC_NAME, 10)
        # steering: channel x, throttle: channel y
        # self.steering_publisher = self.create_publisher(Float32, STEERING_TOPIC_NAME, 10)
        # self.throttle_publisher = self.create_publisher(Float32, THROTTLE_TOPIC_NAME, 10)

        self.camera_subscriber = self.create_subscription(Image, CAMERA_IMG_TOPIC_NAME, self.controller, 10)
    
    def controller(self, data):


        # cv2 image processing
        # publish to steering & constant throttle

    
        # change steering to channel 4 and throttle to channel 7 in adafruit_twist.py

        # throttle:
        # linear.x: 0.1 (netural), 0.2 (medium forward)

        # steering:
        # angular.z: max-left (not actual max, but to be safe) -0.3
        # angular.z: max-right (not actual max, but to be safe) 0.6
        # angular.z: center (not actual max, but to be safe) 0.15



        # max left and max right for SERVO
        # left: 180, right: 90

        # if no image (no mid), then stop, no throttle, no steering

        # if center is within ~30px of actual middle, go straight



        self.steering_publisher.publish()
        self.throttle_publisher.publish(0.175)
        self.servo_publisher.publish()

def main(args=None):
    rclpy.init(args=args)
    target_detection = TargetDetection()
    rclpy.spin(target_detection)
    target_detection.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()