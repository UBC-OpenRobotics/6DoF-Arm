from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'realsense_node = arm_perception.realsense_node:main',
            'yolo_detector_node = arm_perception.yolo_detector_node:main',
            'localization_3d_node = arm_perception.localization_3d_node:main',
            'webcam_demo_node = arm_perception.webcam_demo_node:main',
        ],
    },
)
