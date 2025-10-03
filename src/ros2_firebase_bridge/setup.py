from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_firebase_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config ファイルのインストール
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'firebase-admin>=6.0.0',
        'google-cloud-firestore>=2.11.0',
        'PyYAML>=6.0',
    ],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@mobility.com',
    description='Firebase-ROS2 Bridge for Personal Mobility Platform',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'firebase_bridge = ros2_firebase_bridge.firebase_bridge_node:main',
            'coordinate_test = ros2_firebase_bridge.test_coordinates:main',
        ],
    },
)