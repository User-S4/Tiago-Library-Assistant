from setuptools import find_packages
from setuptools import setup

setup(
    name='tiago_pro_description',
    version='2.5.0',
    packages=find_packages(
        include=('tiago_pro_description', 'tiago_pro_description.*')),
)
