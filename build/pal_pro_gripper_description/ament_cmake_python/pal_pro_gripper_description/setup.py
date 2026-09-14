from setuptools import find_packages
from setuptools import setup

setup(
    name='pal_pro_gripper_description',
    version='1.12.5',
    packages=find_packages(
        include=('pal_pro_gripper_description', 'pal_pro_gripper_description.*')),
)
