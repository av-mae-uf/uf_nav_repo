import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import String
from uf_interfaces.msg import VehicleCommand

D2R = math.pi/180.0
R2D = 180.0/math.pi

class PublishVehicleCommand(Node):

    def __init__(self):
        super().__init__('publish_vehicle_command')
        self.publisher_ = self.create_publisher(VehicleCommand, 'vehicle_command', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        msg = VehicleCommand()
        msg.radius_of_curvature = self.i
        msg.speed = 23.3
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%lf"' % self.i)
        self.i += 0.1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PublishVehicleCommand()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
