import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'forklift_driver'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages() automatically discovers the 'can_interface' folder as long as it has an __init__.py
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # ADD THIS: Copies your YAML file into the ROS 2 install space
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    # Added python-can and pyyaml so rosdep knows about them
    install_requires=['setuptools', 'python-can', 'pyyaml'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Driver layer for MBV15 forklift hardware via CANopen',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ADD THIS: Exposes your node as an executable command
            'driver_node = forklift_driver.driver_node:main',
        ],
    },
)