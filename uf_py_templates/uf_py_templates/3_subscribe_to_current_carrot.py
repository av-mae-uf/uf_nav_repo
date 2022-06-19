import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from uf_interfaces.msg import CurrentCarrot

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sub_to_current_carrot')
        self.subscription = self.create_subscription(
            CurrentCarrot,
            'current_carrot',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('sub_to_current_carrot heard x = "%lf"' % msg.current_goal_pose.position.x)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
