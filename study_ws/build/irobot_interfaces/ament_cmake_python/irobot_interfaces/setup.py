from setuptools import find_packages
from setuptools import setup

setup(
    name='irobot_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('irobot_interfaces', 'irobot_interfaces.*')),
)
