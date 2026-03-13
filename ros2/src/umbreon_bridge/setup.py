from setuptools import setup

package_name = 'umbreon_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/umbreon_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Umbreon Team',
    maintainer_email='umbreon@example.com',
    description='ROS2 bridge for Umbreon roborace car',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = umbreon_bridge.bridge_node:main',
        ],
    },
)
