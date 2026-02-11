import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class BaseSubscriber(Node):
    def __init__(self):
        super().__init__("We_now_know")
        self.subscription = self.create_subscription(String,'topic_1',self.listener_callback,10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info("Received: '%s'" % msg.data)

def main(args=None):
    rclpy.init(args=args)

    base_subscriber = BaseSubscriber()

    rclpy.spin(base_subscriber) #check to see if the order matters
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
