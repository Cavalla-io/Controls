from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'forklift_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line ensures any launch files you create get installed properly
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Teleoperation node for mapping standard joy to custom forklift commands',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # format: 'executable_name = package_name.script_name:main_function'
            'teleop_node = forklift_teleop.teleop_node:main',
        ],
    },
)