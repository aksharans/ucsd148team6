import rclpy
from rclpy.Node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Image

NODE_NAME = 'target_detection_node'

# topics subscribed to
CAMERA_IMG_TOPIC_NAME = '/camera/color/image_raw'

# topics published to
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'
SERVO_TOPIC_NAME = '/servo'


class TargetDetection(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
    
        self.steering_publisher = self.create_publisher(Float32, STEERING_TOPIC_NAME, 10)
        self.throttle_publisher = self.create_publisher(Float32, THROTTLE_TOPIC_NAME, 10)
        self.servo_publisher = self.create_publisher(Float32, SERVO_TOPIC_NAME, 10)

        self.camera_subscriber = self.create_subscription(Image, CAMERA_IMG_TOPIC_NAME, self.controller, 10)
    
    def controller(self, data):
        # cv2 image processing
        # publish to steering & constant throttle
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