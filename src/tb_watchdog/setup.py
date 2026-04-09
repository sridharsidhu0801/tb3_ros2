from setuptools import find_packages, setup

package_name = 'tb_watchdog'

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
    maintainer='tb_1',
    maintainer_email='tb_1@todo.todo',
    description='cmd_vel watchdog for TurtleBot3',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_watchdog = tb_watchdog.cmd_vel_watchdog:main',
        ],
    },
)
