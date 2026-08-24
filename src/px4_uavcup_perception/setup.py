from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'px4_uavcup_perception'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='dolphiinn',
    maintainer_email='anh.nguyenvantuan54@hcmut.edu.vn',
    description='Jetson TensorRT camera perception for the Urban UAV Cup',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_calibration_publisher = '
            'px4_uavcup_perception.camera_calibration_publisher:main',
            'jetson_depth_node = '
            'px4_uavcup_perception.jetson_depth_node:main',
            'zipdepth_node = '
            'px4_uavcup_perception.zipdepth_node:main',
            'aruco_detector_node = '
            'px4_uavcup_perception.aruco_detector_node:main',
            'aruco_to_px4_landing_target = '
            'px4_uavcup_perception.aruco_to_px4_landing_target:main',
            'v4l2_camera_node = '
            'px4_uavcup_perception.v4l2_camera_node:main',
            'aruco_landing_pid_node = '
            'px4_uavcup_perception.aruco_landing_pid_node:main',
        ],
    },
)
