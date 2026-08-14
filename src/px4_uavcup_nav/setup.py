from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'px4_uavcup_nav'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dolphiinn',
    maintainer_email='anh.nguyenvantuan54@hcmut.edu.vn',
    description='Nav2/AMCL bringup for the PX4 UAV Cup drone',
    license='Apache-2.0',
)
