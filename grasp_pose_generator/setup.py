from setuptools import setup
import os
from glob import glob

package_name = 'grasp_pose_generator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for generating 6D grasp poses from GGCNN output and point cloud data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_pose_generator_node = grasp_pose_generator.grasp_pose_generator_node:main'
        ],
    },
)