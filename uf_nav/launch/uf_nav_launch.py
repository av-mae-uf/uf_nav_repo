import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
config_dir = get_package_share_directory("uf_nav")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uf_nav',
            namespace='uf_nav',
            executable='route_pose_generator',
            name='route_pose_generator',
            parameters = [
                {'want_loop': 1}
            ]
        ),
        Node(
            package='uf_nav',
            namespace='uf_nav',
            executable='carrot_creator',
            name='carrot_creator',
            parameters = [
                {'send_to_rviz': 1}
            ]
        ),
        Node(
            package='uf_nav',
            namespace='uf_nav',
            executable='vehicle_controller',
            name='vehicle_controller',
            output='screen' 
        ),
        Node(
            package='uf_nav',
            namespace='uf_nav',
            executable='vehicle_simulator',
            name='vehicle_simulator',
            output='screen' 
        ),
        Node(
            package='tf2_ros',
            namespace='tf2_ros',
            executable='static_transform_publisher',
            name='cdc_tf',
            arguments=["0", "0", "0", "0", "0", "0", "map", "my_frame"]
        ),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
	        arguments=["-d", config_dir + "/rviz/depot_park_rviz_launch.rviz"]
        )
    ])
