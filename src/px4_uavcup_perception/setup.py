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
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='dolphiinn',
    maintainer_email='anh.nguyenvantuan54@hcmut.edu.vn',
    description='Simulation and hardware camera perception nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gz_image_bridge = '
            'px4_uavcup_perception.gz_image_bridge:main',
            'depth_anything_node = '
            'px4_uavcup_perception.depth_anything_node:main',
            'free_space_node = '
            'px4_uavcup_perception.free_space_node:main',
            'jetson_depth_node = '
            'px4_uavcup_perception.jetson_depth_node:main',
        ],
    },
)
