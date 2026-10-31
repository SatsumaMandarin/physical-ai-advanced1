import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'patrol_robot'


def tree(directory):
    """directory 以下のファイルを、フォルダ構成を保ったまま share にインストールする"""
    entries = []
    for root, _, files in os.walk(directory):
        if files:
            entries.append((
                os.path.join('share', package_name, root),
                [os.path.join(root, f) for f in files],
            ))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GitHubのユーザー名',
    maintainer_email='数字+ユーザー名@users.noreply.github.com',
    description='Patrol and inspection robot with YOLO on Gazebo',
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [
            'patrol_node = patrol_robot.patrol_node:main',
            'detector_recorder_node = patrol_robot.detector_recorder_node:main',
        ],
    },
)
