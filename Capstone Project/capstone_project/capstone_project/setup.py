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
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # RViz configuration files
        (os.path.join('share', package_name, 'models/ground'), glob('models/ground/*')), # Ground model files
        (os.path.join('share', package_name, 'models/obstacles'), glob('models/obstacles/*')), # Obstacles model files
        (os.path.join('share', package_name, 'models/traffic_sign'), glob('models/traffic_sign/*')), # Traffic sign model files
        (os.path.join('share', package_name, 'models/walls'), glob('models/walls/*')), # Walls model files
        (os.path.join('share', package_name, 'models/marker'), glob('models/marker/*')), # Marker model files
        (os.path.join('share', package_name, 'models/apriltag'), glob('models/apriltag/*')), # AprilTag (robot) model files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')), # World files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinmay Samak' 'Tanmay Samak',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu',
    description='Capstone Project for the course AuE-8230 "Autonomy: Science and Systems" at CU-ICAR (Spring 2023)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capstone_project_sim = capstone_project.capstone_project_sim:main', # Python script
            'capstone_project_real = capstone_project.capstone_project_real:main', # Python script
            'wall_following_sim = capstone_project.wall_following_sim:main', # Python script
            'wall_following_real = capstone_project.wall_following_real:main', # Python script
            'obstacle_avoidance_sim = capstone_project.obstacle_avoidance_sim:main', # Python script
            'obstacle_avoidance_real = capstone_project.obstacle_avoidance_real:main', # Python script
            'line_following_sim = capstone_project.line_following_sim:main', # Python script
            'line_following_real = capstone_project.line_following_real:main', # Python script
            'stop_sign_detection_sim = capstone_project.stop_sign_detection_sim:main', # Python script
            'stop_sign_detection_real = capstone_project.stop_sign_detection_real:main', # Python script
            'apriltag_tracking_sim = capstone_project.apriltag_tracking_sim:main', # Python script
            'apriltag_tracking_real = capstone_project.apriltag_tracking_real:main', # Python script
            'apriltag_teleop = capstone_project.apriltag_teleop:main', # Python script
        ],
    },
)
