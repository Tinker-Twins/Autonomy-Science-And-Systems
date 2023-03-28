from setuptools import setup
import os
from glob import glob

package_name = 'assignment_3b'

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
            'lane_keeping = assignment_3b.lane_keeping:main', # Python script
            'lane_following = assignment_3b.lane_following:main', # Python script
            'apriltag_tracking = assignment_3b.apriltag_tracking:main', # Python script
        ],
    },
)
