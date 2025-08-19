import os
from setuptools import find_packages, setup

package_name = 'visuial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='work',
    maintainer_email='work@todo.todo',
    description='Visual perception package with YOLO and Kinect support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense = visuial.realsense:main',
            'aruco_check = visuial.aruco_check:main',
            'medicine_check_service = visuial.medicine_check_service:main',
            'socket_receive = visuial.socket_receive:main',
            'double_check = visuial.double_check:main',
            'second_camera = visuial.second_camera:main',
            'ocr = visuial.ocr:main',
        ],
    },
)
