from setuptools import find_packages
from setuptools import setup

setup(
    name='my_first_packgae_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('my_first_packgae_msgs', 'my_first_packgae_msgs.*')),
)
