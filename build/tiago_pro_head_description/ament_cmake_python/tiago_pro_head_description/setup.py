from setuptools import find_packages
from setuptools import setup

setup(
    name='tiago_pro_head_description',
    version='1.12.0',
    packages=find_packages(
        include=('tiago_pro_head_description', 'tiago_pro_head_description.*')),
)
