import rclpy
from rclpy.node import Node

import math

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from uf_interfaces.msg import CurrentCarrot

D2R = math.pi/180.0
R2D = 180.0/math.pi

class ufPubAfterSub(Node):

    def __init__(self):
        super().__init__('pub_after_sub_template')
        self.subscription = self.create_subscription(
            Pose,
            'kubota_pose',
            self.pub_after_sub_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(CurrentCarrot, 'current_carrot', 10)
        self.i = 0.0

    def pub_after_sub_callback(self, msg):
        self.get_logger().info('I heard: "%lf"' % msg.position.x)

        # got a Pose message in, now send a CurrentCarrot message out
        out_msg = CurrentCarrot()
        out_msg.current_goal_pose.position.x = self.i
        out_msg.current_goal_pose.position.y = 2*(self.i)
        out_msg.current_goal_pose.position.z = 3*(self.i)
        out_msg.current_goal_pose.orientation.w = math.cos(D2R * self.i/2.0)
        out_msg.current_goal_pose.orientation.x = 0.0
        out_msg.current_goal_pose.orientation.y = 0.0
        out_msg.current_goal_pose.orientation.z = math.sin(D2R * self.i/2.0)
        out_msg.speed = 23.3
        out_msg.state = 3

        self.publisher.publish(out_msg)

        self.i += 0.1


def main(args=None):
    rclpy.init(args=args)

    my_pub_after_sub = ufPubAfterSub()

    rclpy.spin(my_pub_after_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_pub_after_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
