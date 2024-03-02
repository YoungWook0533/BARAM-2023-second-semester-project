from setuptools import find_packages, setup
from glob       import glob

import os

package_name = 'lidar_cluster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youngwook',
    maintainer_email='youngwook@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lidar_cluster = lidar_cluster.lidar_cluster:main",
            "goal_update = lidar_cluster.goal_update:main",
            "beagle0_core = lidar_cluster.beagle0_core:main",
            "beagle0_moved_path = lidar_cluster.beagle0_moved_path:main",
            "filtered_lidar0 = lidar_cluster.filtered_lidar0:main",
            "subscribe_map0 = lidar_cluster.subscribe_map0:main",
            "beagle1_core = lidar_cluster.beagle1_core:main",
            "beagle1_moved_path = lidar_cluster.beagle1_moved_path:main",
            "filtered_lidar1 = lidar_cluster.filtered_lidar1:main",
            "subscribe_map1 = lidar_cluster.subscribe_map1:main",             
        ],
    },
)
