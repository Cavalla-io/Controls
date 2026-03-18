from setuptools import find_packages, setup

package_name = 'forklift_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'toml'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Shared configuration loader and schema for controls stack',
    license='TODO: License declaration',
    entry_points={},
)
