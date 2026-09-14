from setuptools import find_packages
from setuptools import setup

setup(
    name='pal_sea_arm_description',
    version='2.8.4',
    packages=find_packages(
        include=('pal_sea_arm_description', 'pal_sea_arm_description.*')),
)
