from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'px4_uavcup_control'


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
    description='Local obstacle and ArUco landing controllers',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_controller_shadow = '
            'px4_uavcup_control.obstacle.local_controller_shadow:main',
            'corridor_controller_shadow = '
            'px4_uavcup_control.obstacle.corridor_controller_shadow:main',
            'aruco_landing_pid_node = '
            'px4_uavcup_control.landing.aruco_pid_node:main',
        ],
    },
)
