from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_firebase_bridge',
    version='0.0.1',
    packages=find_packages(
        include=('ros2_firebase_bridge', 'ros2_firebase_bridge.*')),
)
