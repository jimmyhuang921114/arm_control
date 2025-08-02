import os
from setuptools import find_packages, setup

package_name = 'visuial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 必須存在 resource 索引檔案（可空但要有）
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        # 安裝 package.xml
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
            'paddleocr = visuial.paddleocr:main',
            'yolo_mask = visuial.yolo_mask:main',
            'realsense = visuial.realsense:main',
            'aruco_check = visuial.aruco_check:main',
            'groundsam2_client = visuial.groundsam2_client:main',
            'groundsam2_service = visuial.groundsam2_service:main',
            'medicine_check_service = visuial.medicine_check_service:main',
            'medicine_check_test = visuial.medicine_check_test:main',
            'pic_combine = visuial.pic_combine:main',
            'socket_receive = visuial.socket_receive:main',
            'double_check = visuial.double_check:main',
        ],
    },
)
