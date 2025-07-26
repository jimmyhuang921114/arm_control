from setuptools import find_packages
from setuptools import setup

setup(
    name='tm_robot_if',
    version='0.0.0',
    packages=find_packages(
        include=('tm_robot_if', 'tm_robot_if.*')),
)
