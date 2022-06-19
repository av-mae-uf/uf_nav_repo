from uf_interfaces.srv import GetRoutePoses       # CHANGED
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from uf_interfaces.msg import CurrentCarrot
from visualization_msgs.msg import Marker

from tf2_ros import TransformBroadcaster

import sys
import rclpy
from rclpy.node import Node

import math
import numpy as np

from uf_nav.uf_nav_support import *

D2R = math.pi/180.0
R2D = 180.0/math.pi

look_ahead_dist = 8.0

route_poses = []
speed = 0.5  # meters per frame

class CarrotCreator(Node):

    def __init__(self):
        super().__init__('carrot_creator')
        self.cli = self.create_client(GetRoutePoses, 'get_route_poses')       # CHANGED
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetRoutePoses.Request()
        
        self.declare_parameter('send_to_rviz', 1)   # the default condition is false ; not to loop
        self.send_to_rviz = self.get_parameter('send_to_rviz')                                   # CHANGED

        # subscribe to 'kubota_pose' topic
        self.subscription = self.create_subscription(
            Pose,
            'kubota_pose',
            self.kubota_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        # prepare to publish 'current_carrot' topic
        self.publisher_carrot = self.create_publisher(CurrentCarrot, 'current_carrot', 10)

        print('sendtorvizvalue = ', self.send_to_rviz.value)
        if (self.send_to_rviz.value):
            print('going to rviz1')
            # prepare to publish 'route_points_to_rviz' topic
            self.publisher_marker = self.create_publisher(Marker, 'route_points_to_rviz', 10)
            timer_period = 3.0  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)

            # prepare to publish the 'closest_point' topic for rviz
            self.publisher_closest_point = self.create_publisher(PoseStamped, 'closest_point_to_rviz', 10)

            # prepare to publish the 'look_ahead_pose' topic for rviz
            self.publisher_look_ahead_pose = self.create_publisher(PoseStamped, 'look_ahead_pose_to_rviz', 10)

            # prepare to publish the 'vehicle_pose' topic for rviz
            self.publisher_vehicle_pose = self.create_publisher(PoseStamped, 'vehicle_pose_to_rviz', 10)

            # create the line strip data to send to rviz
            self.line_strip = Marker()

            # Initialize the transform broadcaster
            self.br = TransformBroadcaster(self)

        # define variables
        self.route_segments = []
        self.look_ahead_pose = route_pose_class()
        self.my_closest_pt = np.array([0.0,0.0,0.0])
        self.current_seg_num = 0
        self.look_ahead_seg_num = 0
        self.stop_flag = False
        self.already_sent = False
        self.ready_to_process = False

    def send_request(self):  # no data is sent in the request
        self.future = self.cli.call_async(self.req)
    
    def timer_callback(self):
        self.publisher_marker.publish(self.line_strip)
        self.already_sent = True

    def kubota_pose_callback(self, msg):
        if (self.ready_to_process == True):
            #self.get_logger().info('sub_to_kubota_pose heard x = "%lf"' % msg.position.x)

            vehicle_pt = np.array([msg.position.x, msg.position.y, 0.0])

            #print('look_ahead_dist = ', look_ahead_dist,' vehicle_pt = ', vehicle_pt[0], ', ', vehicle_pt[1], ', ', vehicle_pt[2], \
            #      ', current_seg_num = ', self.current_seg_num, ', lenth of route segs = ', len(self.route_segments))
       
            ans = get_look_ahead_point(look_ahead_dist, vehicle_pt, self.route_segments, self.current_seg_num)

            self.look_ahead_pose = ans[0]
            self.my_closest_pt   = ans[1]
            self.current_seg_num = ans[2]
            self.look_ahead_seg_num = ans[3]
            self.stop_flag = ans[4]

            #print('current_seg_num = ', self.current_seg_num)
            #print('closest_pt = ', self.my_closest_pt)
            #print('look_ahead_pt = ', self.look_ahead_pose.pt)

            # publish the current_carrot topic message
            out_msg = CurrentCarrot()
            out_msg.current_goal_pose.position.x = self.look_ahead_pose.pt[0]
            out_msg.current_goal_pose.position.y = self.look_ahead_pose.pt[1]
            out_msg.current_goal_pose.position.z = self.look_ahead_pose.pt[2]
            out_msg.current_goal_pose.orientation.w = math.cos(self.look_ahead_pose.heading_rad/2.0)
            out_msg.current_goal_pose.orientation.x = 0.0
            out_msg.current_goal_pose.orientation.y = 0.0
            out_msg.current_goal_pose.orientation.z = math.sin(self.look_ahead_pose.heading_rad/2.0)
            out_msg.speed = speed
            out_msg.state  = int(self.route_segments[self.current_seg_num].state)
            if(self.route_segments[self.current_seg_num].state == myState.END_PLUS_ONE.value):
                out_msg.speed = 0.0

            self.publisher_carrot.publish(out_msg)

            if (self.send_to_rviz.value):
                print('going to rviz2')
                # publish the 'closet_point' topic to rviz
                out_msg2 = PoseStamped()
                out_msg2.header.frame_id = 'my_frame'
                out_msg2.header.stamp = self.get_clock().now().to_msg()
                out_msg2.pose.position.x = self.my_closest_pt[0]
                out_msg2.pose.position.y = self.my_closest_pt[1]
                out_msg2.pose.position.z = self.my_closest_pt[2]
                out_msg2.pose.orientation.w = 1.0
                out_msg2.pose.orientation.x = 0.0
                out_msg2.pose.orientation.y = 0.0
                out_msg2.pose.orientation.z = 0.0
                self.publisher_closest_point.publish(out_msg2)

                # publish the 'look_ahead_pose' topic to rviz
                out_msg3 = PoseStamped()
                out_msg3.header.frame_id = 'my_frame'
                out_msg3.header.stamp = self.get_clock().now().to_msg()
                out_msg3.pose.position.x = self.look_ahead_pose.pt[0]
                out_msg3.pose.position.y = self.look_ahead_pose.pt[1]
                out_msg3.pose.position.z = self.look_ahead_pose.pt[2]
                out_msg3.pose.orientation.w = math.cos(self.look_ahead_pose.heading_rad/2.0)
                out_msg3.pose.orientation.x = 0.0
                out_msg3.pose.orientation.y = 0.0
                out_msg3.pose.orientation.z = math.sin(self.look_ahead_pose.heading_rad/2.0)
                self.publisher_look_ahead_pose.publish(out_msg3)

                # publish the 'vehicle_pose' topic to rviz
                out_msg4 = PoseStamped()
                out_msg4.header.frame_id = 'my_frame'
                out_msg4.header.stamp = self.get_clock().now().to_msg()
                out_msg4.pose.position.x = msg.position.x
                out_msg4.pose.position.y = msg.position.y
                out_msg4.pose.position.z = msg.position.z
                out_msg4.pose.orientation.w = msg.orientation.w
                out_msg4.pose.orientation.x = msg.orientation.x
                out_msg4.pose.orientation.y = msg.orientation.y
                out_msg4.pose.orientation.z = msg.orientation.z
                self.publisher_vehicle_pose.publish(out_msg4)

                # send out the tf2 for the vehicle pose
                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'my_frame'
                t.child_frame_id = 'kubota_frame'

                t.transform.translation.x = msg.position.x
                t.transform.translation.y = msg.position.y
                t.transform.translation.z = msg.position.z

                t.transform.rotation.x = msg.orientation.x
                t.transform.rotation.y = msg.orientation.y
                t.transform.rotation.z = msg.orientation.z
                t.transform.rotation.w = msg.orientation.w

                # Send the transformation
                self.br.sendTransform(t)

    def create_line_strip(self):
        self.line_strip.header.frame_id = "/my_frame"
        self.line_strip.header.stamp = self.get_clock().now().to_msg()
        self.line_strip.ns = 'complete_route'
        self.line_strip.pose.orientation.w = 1.0
        self.line_strip.id = 1
        self.line_strip.type = self.line_strip.LINE_STRIP

        self.line_strip.scale.x = 0.25
        self.line_strip.color.r = 1.0
        self.line_strip.color.g = 0.0
        self.line_strip.color.b = 0.0
        self.line_strip.color.a = 1.0

        
        
        if (not self.already_sent):
            for i in range(len(self.route_segments)):
                for j in range(50):
                    p = Point()
                    pt = get_point_on_route(self.route_segments[i], j/50.)
                    p.x = pt[0]
                    p.y = pt[1]
                    p.z = pt[2]
                    
                    self.line_strip.points.append(p)
                    #print('p i=',i, ', j=', j, '\t', p.x, ', ', p.y, ', ', p.z)

def main(args=None):
    rclpy.init(args=args)

    carrot_creator = CarrotCreator()
    carrot_creator.send_request()

    while rclpy.ok():
        rclpy.spin_once(carrot_creator)

        if carrot_creator.future.done():
            try:
                response = carrot_creator.future.result()
            except Exception as e:
                carrot_creator.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                num_poses = response.num_route_poses
                carrot_creator.get_logger().info(
                    'Carrot creator received %d poses.  Last pt = %lf, %lf, %lf' % (num_poses,\
                    response.mypose[num_poses-1].position.x,\
                    response.mypose[num_poses-1].position.y,\
                    response.mypose[num_poses-1].position.z))
                
                # create the route_poses array
                for i in range(num_poses):
                    ptx = response.mypose[i].position.x
                    pty = response.mypose[i].position.y
                    ptz = response.mypose[i].position.z
                    qw  = response.mypose[i].orientation.w
                    qx  = response.mypose[i].orientation.x
                    qy  = response.mypose[i].orientation.y
                    qz  = response.mypose[i].orientation.z
                    mystate = response.state[i]
                    myheadingrad = 2.0*math.atan2(qz, qw)
                    myw1 = 1.0
                    myw2 = 1.0
                    route_poses.append(route_pose_class(np.array([ptx,pty, ptz]), myheadingrad, mystate, myw1, myw2))

                # create the route_segments array
                want_loop = response.want_loop
                carrot_creator.route_segments = create_route_segments(route_poses, want_loop)
                carrot_creator.get_logger().info('Carrot creator made %d route segments.' % len(carrot_creator.route_segments))

                #print_out_route_segments for debugging
                #fp = open("/home/cimar/dev_ws_uf/src/uf_nav/my_data/output/route_segments.txt", "w")
                #print('p0x, p0y, p0z, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z, w1, w2, length, state', file=fp)
                #for i in range(len(carrot_creator.route_segments)):
                #    print(carrot_creator.route_segments[i].p0[0], ',', carrot_creator.route_segments[i].p0[1], ',', carrot_creator.route_segments[i].p0[2], ',',\
                #          carrot_creator.route_segments[i].p1[0], ',', carrot_creator.route_segments[i].p1[1], ',', carrot_creator.route_segments[i].p1[2], ',',\
                #          carrot_creator.route_segments[i].p2[0], ',', carrot_creator.route_segments[i].p2[1], ',', carrot_creator.route_segments[i].p2[2], ',',\
                #          carrot_creator.route_segments[i].p3[0], ',', carrot_creator.route_segments[i].p3[1], ',', carrot_creator.route_segments[i].p3[2], ',',\
                #          carrot_creator.route_segments[i].w1,    ',', carrot_creator.route_segments[i].w2,    ',', carrot_creator.route_segments[i].length, ',',\
                #          carrot_creator.route_segments[i].state, file=fp)

                #fp.close()

                # create the line strip data to send to rviz
                carrot_creator.create_line_strip()
                carrot_creator.ready_to_process = True

            break

    rclpy.spin(carrot_creator)

    carrot_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
