from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tm_robot_main'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 安裝資源索引
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # 安裝 package.xml
        ('share/' + package_name, ['package.xml']),

        # 安裝 launch 檔案（所有 .py 檔）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='work',
    maintainer_email='jimmyjimmyhuang1114@gmail.com',
    description='Main control package for TM robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_control = tm_robot_main.main_control:main',
            'slider_simple = tm_robot_main.slider_simple:main',
            'find_med_info = tm_robot_main.find_med_info:main',
            'tm_flow_node = tm_robot_main.tm_flow_mode:main',
            'order_process = tm_robot_main.order_process:main',
        ],
    },
)
