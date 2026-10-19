from glob import glob

from setuptools import setup

package_name = 'tb4_autonomy_yolo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='TurtleBot 4 localization, Nav2 waypoint navigation and YOLOv8 detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_node = tb4_autonomy_yolo.nav_node:main',
            'yolo_node = tb4_autonomy_yolo.yolo_node:main',
            'relay_node = tb4_autonomy_yolo.relay_node:main',
        ],
    },
)
