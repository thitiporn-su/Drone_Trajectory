from setuptools import find_packages
from setuptools import setup

setup(
    name='uav_description',
    version='0.0.0',
    packages=find_packages(
        include=('uav_description', 'uav_description.*')),
)
