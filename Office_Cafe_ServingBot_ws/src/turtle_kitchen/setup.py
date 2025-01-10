from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_kitchen'

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
    description='A package for managing kitchen displays and coordinating serving robot deliveries',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'main_gui = turtle_kitchen.main_gui:main',
            'sub_data = turtle_kitchen.sub_data:main',
            'check = turtle_kitchen.check_order:main',
            'go_table = turtle_kitchen.send_goal:main'
        ],
    },
)
