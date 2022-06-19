from setuptools import setup

package_name = 'uf_py_templates'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            '1_talker = uf_py_templates.1_publisher_member_function:main',
            '1_listener = uf_py_templates.1_subscriber_member_function:main',
            '2_service = uf_py_templates.2_service_member_function:main',
            '2_client = uf_py_templates.2_client_member_function:main',
            '3_pub_after_sub = uf_py_templates.3_pub_after_sub:main',
            '4_pub_sub = uf_py_templates.4_pub_sub:main',
            '3_pub_kubota_pose = uf_py_templates.3_publish_kubota_pose:main',
            '3_sub_current_carrot = uf_py_templates.3_subscribe_to_current_carrot:main',
            '5_pub_vehicle_command = uf_py_templates.5_pub_vehicle_command:main',
            '5_pub_after_two_subs = uf_py_templates.5_pub_after_two_subs:main',
        ],
    },
)
