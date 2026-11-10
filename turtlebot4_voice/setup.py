from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot4_voice'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [f for f in glob('launch/*.launch.py') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config'), [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'maps'), [f for f in glob('maps/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'scripts'), [f for f in glob('scripts/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'report'), [f for f in glob('report/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ductri',
    maintainer_email='ductri@todo.todo',
    description='Phan vung ban do va dieu khien TurtleBot 4 bang giong noi',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'speech_node = turtlebot4_voice.speech_node:main',
            'voice_nav_node = turtlebot4_voice.voice_nav_node:main',
            'regions_marker_node = turtlebot4_voice.regions_marker_node:main',
            'initial_pose_publisher = turtlebot4_voice.initial_pose_publisher:main',
            'voice_dashboard_gui = turtlebot4_voice.voice_dashboard_gui:main',
            'dummy_hardware_services = turtlebot4_voice.dummy_hardware_services:main',
            'sim_ground_truth_odom = turtlebot4_voice.sim_ground_truth_odom:main',
        ],
    },
)
