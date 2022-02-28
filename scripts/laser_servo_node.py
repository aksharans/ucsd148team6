import rclpy
from rclpy.Node import Node

NODE_NAME = 'laser_servo_node'

# topics subscribed to
LASER_TOPIC_NAME = '/laser'

# topics published to
SERVO_TOPIC_NAME = '/servo'

# might not need if target_detection can do all the cv2 work and publish to multiple

class LaserServo(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
    
        self.servo_publisher = self.create_publisher(Float32, SERVO_TOPIC_NAME, 10)

        self.laser_subscriber = self.create_subscription(Float32, LASER_TOPIC_NAME, self.servo_aiming, 10)

    def servo_aiming(self, data):
        # angle detection using data
        x = 0

def main(args=None):
    rclpy.init(args=args)
    laser_servo = LaserServo()
    rclpy.spin(laser_servo)
    laser_servo.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()