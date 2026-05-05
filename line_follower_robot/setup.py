from setuptools import setup
import os
from glob import glob

package_name = 'line_follower_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sathwik',
    maintainer_email='sathwik@todo.todo',
    description='Autonomous mobile robot with line tracking and obstacle avoidance',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_detector = line_follower_robot.line_detector:main',
            'unified_controller = line_follower_robot.unified_controller:main',
        ],
    },
)
