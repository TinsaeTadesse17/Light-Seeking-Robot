from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'light_seeking_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sura-itana',
    maintainer_email='suraitana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'light_seeking_node = light_seeking_robot.light_seeking_node:main',
            'obstacle_avoidance = light_seeking_robot.obstacle_avoidance_node:main',
        ],
    },
)