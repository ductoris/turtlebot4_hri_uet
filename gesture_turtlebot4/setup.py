from setuptools import setup
import os
from glob import glob

package_name = 'gesture_turtlebot4'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ductri',
    maintainer_email='ductri@todo.todo',
    description='Giao tiep phi ngon ngu va dieu khien TurtleBot 4 bang cu chi tay',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gesture_recognition_node = gesture_turtlebot4.gesture_recognition_node:main',
            'robot_behavior_node = gesture_turtlebot4.robot_behavior_node:main',
            'webcam_test = gesture_turtlebot4.scripts.webcam_test:main',
        ],
    },
)
