from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_serving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyuna',
    maintainer_email='sjajmh6612@naver.com',
    description='A package for managing and displaying the serving robot screen interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_monitor = turtle_serving.server_monitor:main',
            'gazebo_sub = turtle_serving.gazebo_sub:main',
        ],
    },
)
