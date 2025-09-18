from setuptools import setup

package_name = 'ros2_firebase_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # find_packages() → [package_name] に変更
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/firebase_config.yaml']),
    ],
    install_requires=[
        'setuptools',
        'firebase-admin',
        'google-cloud-firestore',
        'tf-transformations',
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