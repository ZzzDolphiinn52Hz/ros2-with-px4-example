from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_uavcup_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='dolphiinn',
    maintainer_email='anh.nguyenvantuan54@hcmut.edu.vn',
    description='PX4 SITL + Gazebo LiDAR bridge for slam_toolbox',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'px4_odom_tf = px4_uavcup_slam.px4_odom_tf:main',
            'gz_lidar_bridge = px4_uavcup_slam.gz_lidar_bridge:main',
        ],
    },
)
