from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'px4_uavcup_simulation'


def asset_files(root: str):
    """Install every regular file below an asset directory."""
    entries = []
    for directory, _, filenames in os.walk(root):
        files = [os.path.join(directory, name) for name in filenames]
        if files:
            entries.append((
                os.path.join('share', package_name, directory),
                files,
            ))
    return entries


data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]
data_files.extend(asset_files('models'))
data_files.extend(asset_files('worlds'))


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dolphiinn',
    maintainer_email='anh.nguyenvantuan54@hcmut.edu.vn',
    description='PC-only Gazebo and PX4 SITL assets for ArUco testing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gz_camera_bridge = '
            'px4_uavcup_simulation.gz_camera_bridge:main',
            'landing_readiness = '
            'px4_uavcup_simulation.landing_readiness:main',
        ],
    },
)
