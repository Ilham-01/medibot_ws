from setuptools import find_packages
from setuptools import setup

package_name = 'medibot_example'

setup(
    name=package_name,
    version='2.1.5',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # To be added
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'medibot_interactive_marker.launch.py'))),
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'medibot_obstacle_detection.launch.py'))),
        # ('share/' + package_name + '/rviz', glob.glob(os.path.join('rviz',
        #                                               'medibot_interactive_marker.rviz'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
 
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Examples of Different Medibot Usage.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            # To be added
            # 'medibot_interactive_marker = \
            #   medibot_example.medibot_interactive_marker.main:main',
            'medibot_obstacle_detection = \
                medibot_example.medibot_obstacle_detection.main:main',
            'medibot_patrol_client = \
                medibot_example.medibot_patrol_client.main:main',
            'medibot_patrol_server = \
                medibot_example.medibot_patrol_server.main:main',
            'medibot_position_control = \
                medibot_example.medibot_position_control.main:main',
        ],
    },
)
