from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ani',
    maintainer_email='yelluriani@gmail.com',
    description='Caterpillar rover nodes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_node = rover.odom_node:main',
            'mode_manager = rover.mode_manager:main',
            'velocity_bridge = rover.velocity_bridge:main',
        ],
    },
)
