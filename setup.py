from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'holoocean_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'scenarios'), glob('config/scenarios/*.json')),
        (os.path.join('share', package_name, 'config', 'slam'), glob('config/slam/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wyumhn',
    maintainer_email='joualinzong@gmail.com',
    description='Converts HoloOcean sensor topics into RViz2-friendly messages',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_viz = holoocean_viz.imu_viz:main',
            'lidar_bridge = holoocean_viz.lidar_bridge:main',
        ],
        # Extra sensor encoders picked up by holoocean_main at startup
        'holoocean_main.encoders': [
            'holoocean_viz = holoocean_viz.encoders:PLUGIN',
        ],
    },
)
