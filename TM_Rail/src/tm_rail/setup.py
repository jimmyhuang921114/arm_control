from setuptools import find_packages, setup
import os

package_name = 'tm_rail'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/' + file for file in os.listdir('launch')]),
    ],
    py_modules=[
        'tm_rail.tm_rail_communication',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_dev',
    maintainer_email='ros_dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rail_node=tm_rail.rail_node:main'
            'tm_mark=tm_rail.tm_mark:main'
        ],
    },
)
