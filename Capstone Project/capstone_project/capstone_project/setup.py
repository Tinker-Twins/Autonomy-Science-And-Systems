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
            'wall_following_sim = capstone_project.wall_following_sim:main', # Python script
            'wall_following_real = capstone_project.wall_following_real:main', # Python script
            'obstacle_avoidance_sim = capstone_project.obstacle_avoidance_sim:main', # Python script
            'obstacle_avoidance_real = capstone_project.obstacle_avoidance_real:main', # Python script
        ],
    },
)
