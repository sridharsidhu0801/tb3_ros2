import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tb_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
	(os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
        # config files
        (os.path.join('share', package_name, 'config'),
        glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tb_1',
    maintainer_email='sridhar0801ofcl@gmail.com',
    description='TurtleBot3 system bringup with watchdog and velocity smoother',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
