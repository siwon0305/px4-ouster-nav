from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'px4_mavros_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsw',
    maintainer_email='lsw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_to_tf = px4_mavros_nav.pose_to_tf:main',
            'pose_to_odom = px4_mavros_nav.pose_to_odom:main',
            'cmd_vel_offboard = px4_mavros_nav.cmd_vel_offboard:main',
            'astar_goal_follower = px4_mavros_nav.astar_goal_follower:main',
        ],
    },
)
