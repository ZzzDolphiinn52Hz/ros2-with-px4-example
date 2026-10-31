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
        (os.path.join('share', package_name, 'calibration'),
         glob('calibration/*.*')),
        (os.path.join('share', package_name, 'calibration', 'aruco_5x5_50'),
         glob('calibration/aruco_5x5_50/*.*')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='dolphiinn',
    maintainer_email='anh.nguyenvantuan54@hcmut.edu.vn',
    description='Raspberry Pi 5 ArUco and ZipDepth perception',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'zipdepth_node = '
            'px4_uavcup_perception.depth.zipdepth_node:main',
            'aruco_detector_node = '
            'px4_uavcup_perception.aruco.detector_node:main',
            'picamera2_socket_camera_node = '
            'px4_uavcup_perception.cameras.picamera2_socket_node:main',
        ],
    },
)
