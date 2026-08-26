from setuptools import find_packages, setup


package_name = 'px4_uavcup_px4_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='dolphiinn',
    maintainer_email='anh.nguyenvantuan54@hcmut.edu.vn',
    description='PX4 Offboard and landing-target bridges',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_px4 = px4_uavcup_px4_bridge.cmd_vel_to_px4:main',
            'aruco_to_px4_landing_target = '
            'px4_uavcup_px4_bridge.aruco_landing_target:main',
        ],
    },
)
