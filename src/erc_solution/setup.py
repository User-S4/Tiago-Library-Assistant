from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'erc_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'book_colour_detector = erc_solution.book_colour_detector:main',
            'shelf_column_detector = erc_solution.shelf_column_detector:main',
            'column_navigator = erc_solution.column_navigator:main',
            'grasp_controller = erc_solution.manipulation.grasp_controller:main',
        ],
    },
)
