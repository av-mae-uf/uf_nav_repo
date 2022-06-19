import rclpy
from rclpy.node import Node

import numpy as np
import math

from uf_nav.uf_nav_support import update_vehicle_pose

from geometry_msgs.msg import Pose
from uf_interfaces.msg import VehicleCommand

D2R = math.pi/180.0
R2D = 180.0/math.pi

class VehicleSimulator(Node):

    def __init__(self):
        super().__init__('vehicle_simulator')
        self.subscription = self.create_subscription(
            VehicleCommand,
            'kubota_command',
            self.vehicle_simulator_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Pose, 'kubota_pose', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.old_position = np.array([-5.0, 5.0, 0.0])
        self.old_heading_deg = -60.0

        self.rad_of_curvature = 99999.0
        self.speed = 0.0
        self.i = 0

    def vehicle_simulator_callback(self, msg):
        #self.get_logger().info('I heard rad_of_curvature = "%lf"' % msg.radius_of_curvature)
        # just store the incoming values
        self.rad_of_curvature = msg.radius_of_curvature
        self.speed = msg.speed
        self.i +=1

    def timer_callback(self):
        # update the vehicle pose based on stored data

        ans = update_vehicle_pose(self.old_position, self.old_heading_deg, self.rad_of_curvature, self.speed)

        new_position    = ans[0]
        new_heading_deg = ans[1]

        msg = Pose()
        msg.position.x = new_position[0]
        msg.position.y = new_position[1]
        msg.position.z = new_position[2]
        msg.orientation.w = math.cos(D2R * new_heading_deg/2.0)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(D2R * new_heading_deg/2.0)
       
        self.publisher.publish(msg)

        self.old_position = new_position
        self.old_heading_deg = new_heading_deg


def main(args=None):
    rclpy.init(args=args)

    my_vehicle_simulator = VehicleSimulator()

    rclpy.spin(my_vehicle_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_vehicle_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
