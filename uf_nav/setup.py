from setuptools import setup
from glob import glob  # change for being able to read a file from a standard location

package_name = 'uf_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch*.py')),   # added so that launch files will be copied globally
        ('share/' + package_name+ '/my_data', glob('my_data/pose_list.txt')),  # change for opening the data file
        ('share/' + package_name+ '/rviz', glob('rviz/depot_park_rviz_launch.rviz')),  # change for opening the rviz file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carl Crane',
    maintainer_email='carl.crane@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_pose_generator = uf_nav.route_pose_generator:main',
            'carrot_creator = uf_nav.carrot_creator:main',
            'vehicle_simulator = uf_nav.vehicle_simulator:main',
            'vehicle_controller = uf_nav.vehicle_controller:main',
            'pub_kubota_pose = uf_nav.pub_kubota_pose:main',
        ],
    },
)
