from setuptools import setup
import os
from glob import glob

package_name = 'capstone_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Launch files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # RViz config files
        (os.path.join('share', package_name, 'models/ground'), glob('models/ground/*')), # Ground model files
        (os.path.join('share', package_name, 'models/obstacles'), glob('models/obstacles/*')), # Obstacles model files
        (os.path.join('share', package_name, 'models/traffic_sign'), glob('models/traffic_sign/*')), # Traffic sign model files
        (os.path.join('share', package_name, 'models/walls'), glob('models/walls/*')), # Walls model files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')), # World files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinmay Samak' 'Tanmay Samak',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu',
    description='Assignment 3B for the course AuE-8230 "Autonomy: Science and Systems" at CU-ICAR (Spring 2023)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_following = capstone_project.wall_following:main', # Python script
            'lane_keeping = capstone_project.lane_keeping:main', # Python script
            'lane_following = capstone_project.lane_following:main', # Python script
            'apriltag_tracking = capstone_project.apriltag_tracking:main', # Python script
        ],
    },
)
