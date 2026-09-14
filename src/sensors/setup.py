import os
from glob import glob

from setuptools import setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dfl-perception',
    maintainer_email='bjonguk@gmail.com',
    description='Sensor-level processing (RealSense-D435-like depth point cloud) '
                'for the ERC 2026 head RGBD camera.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_cloud = sensors.depth_to_cloud_node:main',
        ],
    },
)
