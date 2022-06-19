import rclpy
from rclpy.node import Node

import math

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from uf_interfaces.msg import CurrentCarrot
from uf_interfaces.msg import VehicleCommand

D2R = math.pi/180.0
R2D = 180.0/math.pi

class ufPubAfterTwoSubs(Node):

    def __init__(self):
        super().__init__('pub_after_two_subs_template')
        self.subscription1 = self.create_subscription(
            Pose,
            'kubota_pose',
            self.kubota_pose_callback,
            10)
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            VehicleCommand,
            'vehicle_command',
            self.vehicle_command_callback,
            10)
        self.subscription2  # prevent unused variable warning

        self.publisher = self.create_publisher(CurrentCarrot, 'current_carrot', 10)
        self.i = 0.0

        # define the variables that will store the data from the two message inputs
        self.radius_of_curvature = 0.0
        self.speed = 0.0
        self.pt_x = 0.0
        self.pt_y = 0.0
        self.pt_z = 0.0
        self.orientation_w = 0.0
        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0

        # will only publish after each of the subscribes has occured at least once
        self.kubota_pose_sub_ok = False
        self.vehicle_command_sub_ok = False
        self.ok_to_pub = False


    def kubota_pose_callback(self, msg):
        self.get_logger().info('I heard: "%lf"' % msg.position.x)

        self.pt_x = msg.position.x
        self.pt_y = msg.position.y
        self.pt_z = msg.position.z
        self.orientation_w = msg.orientation.w
        self.orientation_x = msg.orientation.x
        self.orientation_y = msg.orientation.y
        self.orientation_z = msg.orientation.z

        self.kubota_pose_sub_ok = True
        if(self.vehicle_command_sub_ok):
            self.ok_to_pub = True

        # got a Pose message in, now send a CurrentCarrot message out
        if(self.ok_to_pub):
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

    def vehicle_command_callback(self, msg):
        self.get_logger().info('I heard rad_of_curvature: "%lf"' % msg.radius_of_curvature)

        self.radius_of_curvature = msg.radius_of_curvature
        self.speed = msg.speed

        self.vehicle_command_sub_ok = True
        if(self.kubota_pose_sub_ok):
            self.ok_to_pub = True

        # got a VehicleCommand message in, now send a CurrentCarrot message out
        if(self.ok_to_pub):
            out_msg = CurrentCarrot()
            out_msg.current_goal_pose.position.x = self.i
            out_msg.current_goal_pose.position.y = 2*(self.i)
            out_msg.current_goal_pose.position.z = 3*(self.i)
            out_msg.current_goal_pose.orientation.w = math.cos(D2R * self.i/2.0)
            out_msg.current_goal_pose.orientation.x = 0.0
            out_msg.current_goal_pose.orientation.y = 0.0
            out_msg.current_goal_pose.orientation.z = math.sin(D2R * self.i/2.0)
            out_msg.speed = 33.3
            out_msg.state = 6
            self.publisher.publish(out_msg)

        self.i += 0.1

def main(args=None):
    rclpy.init(args=args)

    my_pub_after_two_subs = ufPubAfterTwoSubs()

    rclpy.spin(my_pub_after_two_subs)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_pub_after_two_subs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
