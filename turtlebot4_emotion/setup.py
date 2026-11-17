from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot4_emotion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ductri',
    maintainer_email='ductri@todo.todo',
    description='Nhận diện biểu cảm khuôn mặt qua webcam và điều khiển TurtleBot 4 trong mô phỏng',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'emotion_tb4_node = turtlebot4_emotion.emotion_tb4_node:main',
        ],
    },
)
