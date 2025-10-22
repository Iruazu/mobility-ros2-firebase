from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_firebase_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ★★★ 以下を追加 ★★★

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),

        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # Map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.pgm') + glob('maps/*.yaml')),

        # Models (yoto7.dae)
        (os.path.join('share', package_name, 'models', 'yoto_campus'),
            glob('models/yoto_campus/*')),

        # RViz configs
        (os.path.join('share', package_name, 'config', 'rviz'),
            glob('config/rviz/*.rviz')),
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
            'test_tools = ros2_firebase_bridge.test_tools:main',  # 追加
        ],
    },
)