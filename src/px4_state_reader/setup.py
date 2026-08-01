from setuptools import find_packages, setup

package_name = 'px4_state_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dolphiinn',
    maintainer_email='anh.nguyenvantuan54@hcmut.edu.vn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_reader = px4_state_reader.state_reader:main'
        ],
    },
)
