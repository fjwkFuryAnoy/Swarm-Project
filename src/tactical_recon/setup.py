from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tactical_recon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fury',
    maintainer_email='fury@swarm-recon.local',
    description='Cooperative UAV-UGV reconnaissance and terrain-aware navigation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uav_patrol_node = tactical_recon.uav_patrol_node:main',
            'terrain_mapper_node = tactical_recon.terrain_mapper_node:main',
            'global_planner_node = tactical_recon.global_planner_node:main',
            'ugv_controller_node = tactical_recon.ugv_controller_node:main',
        ],
    },
)
