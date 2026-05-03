from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'vision_yolo_3d_tracker'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name,
        ]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rakesh Suthar',
    maintainer_email='rakeshsuthar6322@gmail.com',
    description='Vision-based YOLOv8 3D detections + Kalman multi-object tracking (ROS 2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = vision_yolo_3d_tracker.yolo_detector_node:main',
            'kalman_tracker_node = vision_yolo_3d_tracker.kalman_tracker_node:main',
            'visualization_node = vision_yolo_3d_tracker.visualization_node:main',
        ],
    },
)
