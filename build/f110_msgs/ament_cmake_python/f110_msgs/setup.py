from setuptools import find_packages
from setuptools import setup

setup(
    name='f110_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('f110_msgs', 'f110_msgs.*')),
)
