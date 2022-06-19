import rclpy
from rclpy.node import Node

import math
import numpy as np

from uf_nav.uf_nav_support import get_rad_of_curvature_to_carrot

from geometry_msgs.msg import Pose
from uf_interfaces.msg import CurrentCarrot
from uf_interfaces.msg import VehicleCommand

D2R = math.pi/180.0
R2D = 180.0/math.pi

class VehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')
        self.subscription1 = self.create_subscription(
            Pose,
            'kubota_pose',
            self.kubota_pose_callback,
            10)
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            CurrentCarrot,
            'current_carrot',
            self.current_carrot_callback,
            10)
        self.subscription2  # prevent unused variable warning

        self.publisher = self.create_publisher(VehicleCommand, 'kubota_command', 10)

        # define the variables that will store the data from the two message inputs
        self.carrot_point = np.array([0.0, 0.0, 0.0])
        self.carrot_heading_rad = 0.0
        self.speed = 0.0
        self.state = 0.0

        self.pt_x = 0.0
        self.pt_y = 0.0
        self.pt_z = 0.0
        self.orientation_w = 0.0
        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0

        # will only publish after each of the subscribes has occured at least once
        self.kubota_pose_sub_ok = False
        self.current_carrot_sub_ok = False
        self.ok_to_pub = False


    def kubota_pose_callback(self, msg):
        #self.get_logger().info('I heard kubota pose: "%lf"' % msg.position.x)

        self.pt_x = msg.position.x
        self.pt_y = msg.position.y
        self.pt_z = msg.position.z
        self.orientation_w = msg.orientation.w
        self.orientation_x = msg.orientation.x
        self.orientation_y = msg.orientation.y
        self.orientation_z = msg.orientation.z

        self.kubota_pose_sub_ok = True
        if(self.current_carrot_sub_ok):
            self.ok_to_pub = True

        # got a Pose message in, now send a VehicleCommand message out
        if(self.ok_to_pub):
            vehicle_point = np.array([self.pt_x, self.pt_y, self.pt_z])
            vehicle_heading_rad = 2.0 * math.atan2(self.orientation_z, self.orientation_w)
            p1_ratio = 0.25
            
            ans = get_rad_of_curvature_to_carrot(vehicle_point, vehicle_heading_rad, \
                                           self.carrot_point, self.carrot_heading_rad, p1_ratio)

            radius_of_curvature = ans[0]
            driveme             = ans[1]

            out_msg = VehicleCommand()
            
            out_msg.radius_of_curvature = radius_of_curvature
            out_msg.speed = self.speed
            self.publisher.publish(out_msg)

    def current_carrot_callback(self, msg):
        #self.get_logger().info('I heard current carrot state: "%lf"' % msg.state)

        self.carrot_point = np.array([msg.current_goal_pose.position.x, \
                                      msg.current_goal_pose.position.y, \
                                      msg.current_goal_pose.position.z])
        self.carrot_heading_rad = 2.0 * math.atan2(msg.current_goal_pose.orientation.z, msg.current_goal_pose.orientation.w)
        self.speed = msg.speed
        self.state = msg.state

        self.vehicle_command_sub_ok = True
        if(self.kubota_pose_sub_ok):
            self.ok_to_pub = True

        # got a CurrentCarrot message in, now send a VehicleCommand message out
        if(self.ok_to_pub):
            vehicle_point = np.array([self.pt_x, self.pt_y, self.pt_z])
            vehicle_heading_rad = 2.0 * math.atan2(self.orientation_z, self.orientation_w)
            p1_ratio = 0.25
            
            ans = get_rad_of_curvature_to_carrot(vehicle_point, vehicle_heading_rad, \
                                           self.carrot_point, self.carrot_heading_rad, p1_ratio)

            radius_of_curvature = ans[0]
            driveme             = ans[1]

            out_msg = VehicleCommand()
            
            out_msg.radius_of_curvature = radius_of_curvature
            out_msg.speed = self.speed
            self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    my_vehicle_controller = VehicleController()

    rclpy.spin(my_vehicle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
