from ament_index_python.packages import get_package_share_directory  # added so the pose_list.txt file can be found

from uf_interfaces.srv import GetRoutePoses     # CHANGED

import rclpy
from rclpy.node import Node

import math
import numpy as np

from uf_nav.uf_nav_support import *

D2R = math.pi/180.0
R2D = 180.0/math.pi

route_pose = []

class RoutePoseGenerator(Node):

    def __init__(self):
        super().__init__('route_pose_generator')
        self.srv = self.create_service(GetRoutePoses, 'get_route_poses', self.get_route_poses_callback)        # CHANGE
        self.declare_parameter('want_loop', 0)   # the default condition is false ; not to loop
        self.want_loop = self.get_parameter('want_loop')

    def get_route_poses_callback(self, request, response):
        self.get_logger().info('Incoming request for route points.\n' ) # CHANGE

        num_poses = len(route_pose)
        response.num_route_poses = num_poses

        response.want_loop = self.want_loop.value   # will be sending the value of the parameter (0 or 1)

        for i in range(num_poses):
            response.mypose[i].position.x = route_pose[i].pt[0]
            response.mypose[i].position.y = route_pose[i].pt[1]
            response.mypose[i].position.z = route_pose[i].pt[2]
            response.mypose[i].orientation.w = math.cos(route_pose[i].heading_rad/2.0)
            response.mypose[i].orientation.x = 0.0
            response.mypose[i].orientation.y = 0.0
            response.mypose[i].orientation.z = math.sin(route_pose[i].heading_rad/2.0)
            response.state[i] = route_pose[i].state

        return response

def main(args=None):
    rclpy.init(args=args)

    cnt = 0

    package_share_directory = get_package_share_directory('uf_nav')    
    fp = open(package_share_directory + '/my_data/pose_list.txt', "r")
    for x in fp:
        if x[0] == '#':
            continue
        else:
            cnt = cnt + 1
            a = x.split(',')
            myptx = float(a[0])
            mypty = float(a[1])
            myptz = 0.0
            myheadingrad = float(a[2])*D2R
            myw1 = 1.0
            myw2 = 1.0
            mystate = int(a[3])
            route_pose.append(route_pose_class(np.array([myptx,mypty, myptz]), myheadingrad, mystate, myw1, myw2))
            
    fp.close()

    route_pose_generator = RoutePoseGenerator()

    rclpy.spin(route_pose_generator)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
