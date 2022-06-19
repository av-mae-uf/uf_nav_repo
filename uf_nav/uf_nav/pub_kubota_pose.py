import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import String
from geometry_msgs.msg import Pose

D2R = math.pi/180.0
R2D = 180.0/math.pi

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publish_kubota_pose')
        self.publisher_ = self.create_publisher(Pose, 'kubota_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        msg.position.x = -5.0  #self.i+.1
        msg.position.y =  5.0  #2*(self.i+.1)
        msg.position.z = 0.0
        msg.orientation.w = math.cos(-30.0*D2R) # math.cos(D2R*self.i/2.0)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(-30.0*D2R) # math.sin(D2R*self.i/2.0)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
