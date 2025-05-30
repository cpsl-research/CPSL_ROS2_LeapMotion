from setuptools import setup, find_packages

package_name = 'leap_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pytest', 'ohrc_msgs'],
    zip_safe=True,
    maintainer='masaki',
    maintainer_email='masaki312.saito@keio.jp',
    description='leapmotion sample package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'hands_publisher = leap_node.hands_publisher:main',
            'position_detector = leap_node.position_detector:main',
            'posture_detector = leap_node.posture_detector:main',
            'gesture_detector = leap_node.gesture_detector:main',
            'finger_detector = leap_node.finger_detector:main',
            'all_detector = leap_node.all_detector:main',
            'visualizer = leap_node.visualizer:main',
            'rviz_visualizer = leap_node.rviz_visualizer:main',
            'test_rviz = leap_node.test_rviz:main',
        ],
    },
)